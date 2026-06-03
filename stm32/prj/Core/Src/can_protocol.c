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

// Send one CAN frame (Standard ID, 8 bytes). Non-blocking: if all 3 mailboxes
// are full, return -1 immediately (frame dropped). Caller's responsibility to
// decide whether dropping a telemetry frame is acceptable.
static int can_send_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
	CAN_TxHeaderTypeDef tx = {0};
	uint32_t mb;

	tx.StdId = std_id;
	tx.IDE = CAN_ID_STD;
	tx.RTR = CAN_RTR_DATA;
	tx.DLC = dlc;

	if (HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t *)data, &mb) != HAL_OK)
		return -1;
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
	// Zero ring buffer
	memset(&rbuf, 0, sizeof(rbuf));

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
	HAL_CAN_ConfigFilter(&hcan1, &f);

	// Start CAN peripheral in normal mode
	HAL_CAN_Start(&hcan1);

	// Enable RX FIFO 0 message pending interrupt
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	g_estop_active = 0;
	g_sys_fault_code = 0;
}

// ---------------------------------------------------------------------------
// TX — Motor telemetry (0x201 + 0x202) @ 100Hz
// ---------------------------------------------------------------------------

int can_send_motor_telemetry(void)
{
	int ret = 0;
	uint8_t buf[8];
	Motor *m;

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
	if (!g_imu_ready) return -1;

	int ret = 0;
	uint8_t buf[8];
	const ImuData *s = &g_imu_data;

	// Frame 0x204: AccelX/Y/Z (mg, int16) + GyroX (0.1°/s, int16)
	put_i16(&buf[0], (int16_t)(s->accel[0] * 1000.0f / 9.80665f));	// m/s² → mg
	put_i16(&buf[2], (int16_t)(s->accel[1] * 1000.0f / 9.80665f));
	put_i16(&buf[4], (int16_t)(s->accel[2] * 1000.0f / 9.80665f));
	put_i16(&buf[6], (int16_t)(s->gyro[0] * 57.29578f * 10.0f));	// rad/s → 0.1°/s
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
	CanRxFrame f;

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &f.header, f.data) != HAL_OK)
		return;
	ringbuf_push(&f);
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
