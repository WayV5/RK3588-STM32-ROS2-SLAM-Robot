/*
 * can_protocol.c — CAN frame encoder/decoder and ring-buffer RX dispatch
 *
 * Architecture:
 *   TX path:   tasks.c (periodic) → can_send_xxx() → HAL_CAN_AddTxMessage()
 *   RX path:   CAN1_RX0_IRQ → HAL_CAN_IRQHandler()
 *              → HAL_CAN_RxFifo0MsgPendingCallback() → ringbuf push (ISR)
 *              → can_command_process() → ringbuf pop → dispatch (main loop)
 *
 * The ring buffer decouples the ISR from the main loop: the ISR only does
 * the minimum (copy frame from hardware FIFO to software ringbuf, <5µs),
 * while decoding and action happen in a cooperative scheduling slot.
 */

#include "can_protocol.h"
#include "motor.h"
#include "motor_protocol.h"
#include "imu.h"
#include "pid.h"
#include <string.h>
#include <math.h>
#include "SEGGER_RTT.h"

// ---------------------------------------------------------------------------
// Static state
// ---------------------------------------------------------------------------
static CanRingBuf rbuf;
static uint8_t g_estop_active;		// 1 = estop engaged, suppress motor output
static uint8_t g_sys_fault_code;	// last fault code

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Encode int16 into little-endian [2] bytes
static inline void put_i16(uint8_t *buf, int16_t v)
{
	buf[0] = v & 0xFF;
	buf[1] = (v >> 8) & 0xFF;
}

// Decode little-endian [2] bytes into int16
static inline int16_t get_i16(const uint8_t *buf)
{
	return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

// Decode little-endian [4] bytes into int32 (for PID float value)
static inline int32_t get_i32(const uint8_t *buf)
{
	return (int32_t)((uint32_t)buf[0]
		| ((uint32_t)buf[1] << 8)
		| ((uint32_t)buf[2] << 16)
		| ((uint32_t)buf[3] << 24));
}

// Send one CAN frame. Non-blocking. Aborts stuck mailboxes from BUS-OFF recovery.
static int can_send_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
	CAN_TxHeaderTypeDef tx = {0};
	uint32_t mb;

	tx.StdId = std_id;
	tx.IDE = CAN_ID_STD;
	tx.RTR = CAN_RTR_DATA;
	tx.DLC = dlc;

	// Abort any mailbox stuck in pending (BUS-OFF recovery leaves them)
	uint32_t tsr = READ_REG(hcan1.Instance->TSR);
	for (int m = 0; m < 3; m++) {
		if (!(tsr & (CAN_TSR_TME0 << m))           // not empty
		    && !(tsr & (CAN_TSR_ABRQ0 << m))) {    // abort not already pending
			hcan1.Instance->TSR |= (CAN_TSR_ABRQ0 << m);
		}
	}

	HAL_StatusTypeDef rc = HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t *)data, &mb);
	if (rc != HAL_OK) {
		static uint8_t tx_err_cnt;
		if (tx_err_cnt < 5) {
			SEGGER_RTT_printf(0, "CAN TX fail: ID=0x%03X rc=%d mb=%lu TSR=0x%08lX\n",
				std_id, rc, mb, (unsigned long)READ_REG(hcan1.Instance->TSR));
			tx_err_cnt++;
		}
		return -(int)rc;
	}
	return 0;
}

// ---------------------------------------------------------------------------
// Ring buffer (producer: ISR, consumer: main loop)
// ---------------------------------------------------------------------------

static inline void ringbuf_push(const CanRxFrame *f)
{
	uint32_t next = (rbuf.head + 1) % CAN_RX_RINGBUF_SIZE;
	if (next == rbuf.tail) return;	// full → drop (oldest safe, ISR never blocks)
	rbuf.buf[rbuf.head] = *f;
	rbuf.head = next;
}

static inline int ringbuf_pop(CanRxFrame *f)
{
	if (rbuf.tail == rbuf.head) return 0;	// empty
	*f = rbuf.buf[rbuf.tail];
	rbuf.tail = (rbuf.tail + 1) % CAN_RX_RINGBUF_SIZE;
	return 1;
}

int can_ringbuf_is_empty(void) { return rbuf.tail == rbuf.head; }
int can_ringbuf_is_full(void)
{
	return ((rbuf.head + 1) % CAN_RX_RINGBUF_SIZE) == rbuf.tail;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

void can_protocol_init(void)
{
	SEGGER_RTT_printf(0, "CAN init: LOOPBACK test...\n");

	// Zero ring buffer
	memset(&rbuf, 0, sizeof(rbuf));

	// --- LOOPBACK TEST: send known frame, verify self-receive ---
	HAL_CAN_Stop(&hcan1);
	hcan1.Init.Mode = CAN_MODE_LOOPBACK;
	HAL_CAN_Init(&hcan1);
	HAL_CAN_Start(&hcan1);
	{
		uint8_t tst[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
		CAN_TxHeaderTypeDef tx = {.StdId=0x7FF,.IDE=CAN_ID_STD,.RTR=CAN_RTR_DATA,.DLC=8};
		uint32_t mb;
		HAL_CAN_AddTxMessage(&hcan1, &tx, tst, &mb);
		HAL_Delay(10);
		CAN_RxHeaderTypeDef rx;
		uint8_t rxd[8] = {0};
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, rxd) == HAL_OK)
			SEGGER_RTT_printf(0, "CAN loopback OK: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				rxd[0],rxd[1],rxd[2],rxd[3],rxd[4],rxd[5],rxd[6],rxd[7]);
		else
			SEGGER_RTT_printf(0, "CAN loopback FAILED\n");
	}

	// Restore normal mode
	HAL_CAN_Stop(&hcan1);
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	HAL_CAN_Init(&hcan1);

	// --- CAN filter: accept 0x100–0x103 (motor cmd, estop, pid config) ---
	// Standard ID mask mode: FilterIdHigh bits [15:5] = STID[10:0].
	// Mask 0x1FC = 0b1_1111_1100 → match on ID bits 9:2, don't care on 1:0.
	// Accepts 0x100 (unused), 0x101 (motor cmd), 0x102 (estop), 0x103 (PID).
	CAN_FilterTypeDef f = {0};
	f.FilterBank = 0;
	f.FilterMode = CAN_FILTERMODE_IDMASK;
	f.FilterScale = CAN_FILTERSCALE_32BIT;
	f.FilterIdHigh = (0x100 << 5);
	f.FilterIdLow = 0;
	f.FilterMaskIdHigh = (0x1FC << 5);
	f.FilterMaskIdLow = 0;
	f.FilterFIFOAssignment = CAN_RX_FIFO0;
	f.FilterActivation = ENABLE;
	f.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
		SEGGER_RTT_printf(0, "CAN init: filter config FAILED\n");

	// Start CAN peripheral in normal mode
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
		SEGGER_RTT_printf(0, "CAN init: HAL_CAN_Start FAILED\n");

	// Enable RX FIFO 0 message pending interrupt
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		SEGGER_RTT_printf(0, "CAN init: ActivateNotification FAILED\n");

	g_estop_active = 0;
	g_sys_fault_code = 0;

	SEGGER_RTT_printf(0, "CAN init: done, Prescaler=%lu, Normal mode\n",
		hcan1.Init.Prescaler);
}

// ---------------------------------------------------------------------------
// TX — Motor telemetry (0x201 + 0x202) @ 100Hz
// ---------------------------------------------------------------------------

int can_send_motor_telemetry(void)
{
	static uint32_t call_cnt;
	int ret = 0;
	uint8_t buf[8];
	Motor *m;

	// Throttled init print
	if (call_cnt == 0)
		SEGGER_RTT_printf(0, "CAN: motor TX started\n");
	call_cnt++;

	// Frame 0x201: M1(LB) speed[0-1] + PWM[2-3], M2(LF) speed[4-5] + PWM[6-7]
	m = motor_get(MOTOR_M1_LB);
	put_i16(&buf[0], m->actual_speed);
	put_i16(&buf[2], (int16_t)m->pwm_output);
	m = motor_get(MOTOR_M2_LF);
	put_i16(&buf[4], m->actual_speed);
	put_i16(&buf[6], (int16_t)m->pwm_output);
	if (can_send_frame(CAN_ID_MOTOR_TELEM_1, buf, 8) != 0) ret = -1;

	// Frame 0x202: M3(RF) speed[0-1] + PWM[2-3], M4(RB) speed[4-5] + PWM[6-7]
	m = motor_get(MOTOR_M3_RF);
	put_i16(&buf[0], m->actual_speed);
	put_i16(&buf[2], (int16_t)m->pwm_output);
	m = motor_get(MOTOR_M4_RB);
	put_i16(&buf[4], m->actual_speed);
	put_i16(&buf[6], (int16_t)m->pwm_output);
	if (can_send_frame(CAN_ID_MOTOR_TELEM_2, buf, 8) != 0) ret = -1;

	return ret;
}

// ---------------------------------------------------------------------------
// TX — IMU (0x204 + 0x205 + 0x206) @ 200Hz
// ---------------------------------------------------------------------------

int can_send_imu(void)
{
	static uint32_t call_cnt;
	static uint8_t warned;
	if (!g_imu_ready) {
		if (!warned) {
			SEGGER_RTT_printf(0, "CAN: IMU not ready, skipping TX\n");
			warned = 1;
		}
		return -1;
	}
	if (call_cnt == 0)
		SEGGER_RTT_printf(0, "CAN: IMU TX started\n");
	call_cnt++;
	int ret = 0;
	static uint8_t buf[8];
	const ImuData *s = &g_imu_data;

	// Frame 0x204: AccelX/Y/Z (mg, int16) + GyroX (0.1°/s, int16)
	put_i16(&buf[0], (int16_t)(s->accel[0] * 1000.0f / 9.80665f));	// m/s² → mg
	put_i16(&buf[2], (int16_t)(s->accel[1] * 1000.0f / 9.80665f));
	put_i16(&buf[4], (int16_t)(s->accel[2] * 1000.0f / 9.80665f));
	put_i16(&buf[6], (int16_t)(s->gyro[0] * 57.29578f * 10.0f));	// rad/s → 0.1°/s
		{ static uint32_t tc; if (tc < 3) { uint8_t t[8] = {1,2,3,4,5,6,7,8}; can_send_frame(0x204, t, 8); tc++; } }
	if (can_send_frame(CAN_ID_IMU1, buf, 8) != 0) ret = -1;
	
	// Frame 0x205: GyroY/Z (0.1°/s, int16) + MagX/Y (µT, int16)
	put_i16(&buf[0], (int16_t)(s->gyro[1] * 57.29578f * 10.0f));
	put_i16(&buf[2], (int16_t)(s->gyro[2] * 57.29578f * 10.0f));
	put_i16(&buf[4], (int16_t)s->mag[0]);
	put_i16(&buf[6], (int16_t)s->mag[1]);
	if (can_send_frame(CAN_ID_IMU2, buf, 8) != 0) ret = -1;

	// Frame 0x206: MagZ (µT, int16) + Roll/Pitch (0.01°, int16 from accel) + Battery (mV)
	// Roll/Pitch from accelerometer only — quick orientation, no AHRS needed.
	float ax = s->accel[0], ay = s->accel[1], az = s->accel[2];
	float roll  = atan2f(ay, az) * 57.29578f;	// rad→deg
	float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;

	put_i16(&buf[0], (int16_t)s->mag[2]);
	put_i16(&buf[2], (int16_t)(roll * 100.0f));
	put_i16(&buf[4], (int16_t)(pitch * 100.0f));
	put_i16(&buf[6], 0);	// battery voltage — TODO: ADC read
	if (can_send_frame(CAN_ID_IMU3, buf, 8) != 0) ret = -1;

	return ret;
}

// ---------------------------------------------------------------------------
// TX — System status (0x207) @ 1Hz
// ---------------------------------------------------------------------------

int can_send_sys_status(uint8_t flags, uint8_t fault_code)
{
	uint8_t buf[8] = {0};
	buf[0] = flags;
	buf[1] = fault_code;
	return can_send_frame(CAN_ID_SYS_STATUS, buf, 8);
}

// ---------------------------------------------------------------------------
// TX — Emergency stop (0x102) — event-driven
// ---------------------------------------------------------------------------

int can_send_estop(uint8_t state)
{
	uint8_t buf[8] = {0};
	buf[0] = state;
	return can_send_frame(CAN_ID_ESTOP, buf, 8);
}

// ---------------------------------------------------------------------------
// TX — PID config (0x103) — event-driven
// ---------------------------------------------------------------------------

int can_send_pid_config(uint8_t motor_id, uint8_t param_type, float value)
{
	uint8_t buf[8] = {0};
	buf[0] = motor_id;
	buf[1] = param_type;
	// float → 4 bytes little-endian
	uint32_t raw;
	memcpy(&raw, &value, 4);
	buf[2] = raw & 0xFF;
	buf[3] = (raw >> 8) & 0xFF;
	buf[4] = (raw >> 16) & 0xFF;
	buf[5] = (raw >> 24) & 0xFF;
	return can_send_frame(CAN_ID_PID_CONFIG, buf, 8);
}

// ---------------------------------------------------------------------------
// RX — Interrupt callback (ISR context, <5µs)
// ---------------------------------------------------------------------------

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint32_t rx_cnt;
	CanRxFrame f;

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &f.header, f.data) != HAL_OK)
		return;
	ringbuf_push(&f);
	rx_cnt++;
	if (rx_cnt <= 5 || rx_cnt % 200 == 0)
		SEGGER_RTT_printf(0, "CAN RX: ID=0x%03X DLC=%d cnt=%lu\n",
			f.header.StdId, f.header.DLC, rx_cnt);
}

// ---------------------------------------------------------------------------
// RX — Command dispatch (main loop, cooperative slot)
// ---------------------------------------------------------------------------

void can_command_process(void)
{
	CanRxFrame f;

	while (ringbuf_pop(&f)) {
		switch (f.header.StdId) {

		case CAN_ID_MOTOR_CMD: {	// 0x101
			// Suppress motor command when estop is engaged
			if (g_estop_active) break;

			int16_t targets[4];
			targets[0] = get_i16(&f.data[0]);	// M1(LB)
			targets[1] = get_i16(&f.data[2]);	// M2(LF)
			targets[2] = get_i16(&f.data[4]);	// M3(RF)
			targets[3] = get_i16(&f.data[6]);	// M4(RB)
			for (int i = 0; i < 4; i++)
				motor_control_set_target((MotorID)i, targets[i]);
			break;
		}

		case CAN_ID_ESTOP: {		// 0x102
			if (f.data[0] == ESTOP_ENGAGE) {
				g_estop_active = 1;
				motor_control_stop_all();
			} else {
				g_estop_active = 0;
			}
			break;
		}

		case CAN_ID_PID_CONFIG: {	// 0x103
			uint8_t mid = f.data[0];
			if (mid >= MOTOR_COUNT) break;

			Motor *m = motor_get((MotorID)mid);
			if (!m) break;

			float val;
			int32_t raw = get_i32(&f.data[2]);
			memcpy(&val, &raw, 4);

			switch (f.data[1]) {
			case 0:	// Kp
				m->pid.Kp = val;
				break;
			case 1:	// Ki
				m->pid.Ki = val;
				break;
			case 2:	// Kd
				m->pid.Kd = val;
				break;
			case 3:	// Kf (feed-forward gain)
				motor_control_set_ff_gain((MotorID)mid, val);
				break;
			default:
				break;
			}
			break;
		}

		default:
			break;	// unknown ID — silently drop
		}
	}
}

// ---------------------------------------------------------------------------
// System status helpers (called from task context)
// ---------------------------------------------------------------------------

uint8_t can_get_status_flags(void)
{
	uint8_t f = 0;
	if (g_estop_active)    f |= SYS_FLAG_ESTOP;
	if (g_sys_fault_code)  f |= SYS_FLAG_FAULT;
	if (g_imu_ready)       f |= SYS_FLAG_IMU_READY;
	// TODO: read battery ADC → set SYS_FLAG_LOW_BATTERY when < 10.5V
	return f;
}

uint8_t can_get_fault_code(void) { return g_sys_fault_code; }
