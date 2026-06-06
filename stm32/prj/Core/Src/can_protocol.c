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

// Send one CAN frame. Non-blocking — drops if all 3 mailboxes are busy.
// Never aborts active mailboxes (that corrupts the frame on the bus).
static int can_send_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
	CAN_TxHeaderTypeDef tx = {0};
	uint32_t mb;

	tx.StdId = std_id;
	tx.IDE = CAN_ID_STD;
	tx.RTR = CAN_RTR_DATA;
	tx.DLC = dlc;

	// Clear any already-completed mailboxes to keep them available.
	// RQCP=1 means the transmission finished (TXOK, TERR, or aborted).
	// Writing 1 to RQCP frees the mailbox.
	uint32_t tsr = READ_REG(hcan1.Instance->TSR);
	for (int m = 0; m < 3; m++) {
		if (tsr & (CAN_TSR_RQCP0 << m))
			hcan1.Instance->TSR |= (CAN_TSR_RQCP0 << m);
	}

	HAL_StatusTypeDef rc = HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t *)data, &mb);
	if (rc != HAL_OK) {
		// No free mailbox — frame dropped. Normal when bus is saturated
		// or RK3588 is offline (all mailboxes stuck waiting for ACK).
		static uint32_t drop_cnt;
		drop_cnt++;
		if (drop_cnt == 1 || drop_cnt % 5000 == 0) {
			SEGGER_RTT_printf(0, "CAN drop: %lu frames (TSR=0x%08lX ESR=0x%08lX)\n",
				(unsigned long)drop_cnt,
				(unsigned long)READ_REG(hcan1.Instance->TSR),
				(unsigned long)READ_REG(hcan1.Instance->ESR));
		}
		return -1;
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

// Quick loopback test — bypasses transceiver, checks bxCAN hardware itself.
// Must be called after HAL_CAN_Init but before filter/start.
static void can_loopback_test(void)
{
	uint32_t t;
	int pass = 0;

	SEGGER_RTT_printf(0, "CAN loopback: entering init mode...\n");

	// Enter init mode to change LBKM
	CAN1->MCR |= CAN_MCR_INRQ;
	t = 100000;
	while (!(CAN1->MSR & CAN_MSR_INAK) && --t) {}
	if (!t) { SEGGER_RTT_printf(0, "CAN loopback: FAIL — cannot enter init\n"); return; }

	// Set loopback, clear silent
	CAN1->BTR |= CAN_BTR_LBKM;
	CAN1->BTR &= ~CAN_BTR_SILM;

	// Configure filter 0 to accept ALL standard IDs (mask=0 in 32-bit mask mode)
	CAN1->FMR |= CAN_FMR_FINIT; // init mode for filter config
	CAN1->FM1R &= ~CAN_FM1R_FBM0;  // bank 0 = mask mode
	CAN1->FS1R |= CAN_FS1R_FSC0;   // bank 0 = 32-bit
	CAN1->FFA1R &= ~CAN_FFA1R_FFA0; // assign to FIFO 0
	CAN1->FA1R |= CAN_FA1R_FACT0;   // activate bank 0
	CAN1->sFilterRegister[0].FR1 = 0; // STID=0, IDE=0, RTR=0
	CAN1->sFilterRegister[0].FR2 = 0; // MASK=0 → accept all
	CAN1->FMR &= ~CAN_FMR_FINIT; // leave filter init mode

	// Leave init mode → loopback mode active
	CAN1->MCR &= ~CAN_MCR_INRQ;
	t = 100000;
	while ((CAN1->MSR & CAN_MSR_INAK) && --t) {}
	if (!t) { SEGGER_RTT_printf(0, "CAN loopback: FAIL — cannot leave init\n"); return; }

	// Check for free mailbox
	t = 100000;
	while (!(CAN1->TSR & CAN_TSR_TME0) && --t) {}
	if (!t) {
		SEGGER_RTT_printf(0, "CAN loopback: no free mbox TSR=0x%08lX, aborting all\n",
			(unsigned long)CAN1->TSR);
		for (int i = 0; i < 3; i++) CAN1->TSR |= CAN_TSR_ABRQ0 << i;
		for (volatile int d = 0; d < 10000; d++) {}
		t = 100000;
		while (!(CAN1->TSR & CAN_TSR_TME0) && --t) {}
		if (!t) {
			SEGGER_RTT_printf(0, "CAN loopback: FAIL — mailboxes stuck after abort\n");
			return;
		}
	}

	// Send test frame via mailbox 0 (register-level, no HAL)
	// Data: {0xA5, 0x5A, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06}
	CAN1->sTxMailBox[0].TDTR = 8; // DLC=8
	CAN1->sTxMailBox[0].TDLR = 0x02015AA5; // bytes [3:0]
	CAN1->sTxMailBox[0].TDHR = 0x06050403; // bytes [7:4]
	CAN1->sTxMailBox[0].TIR = (0x555 << 21); // STD ID 0x555, data frame
	CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

	// Wait for TX done
	t = 1000000;
	while (!(CAN1->TSR & CAN_TSR_RQCP0) && --t) {}
	uint32_t tsr = CAN1->TSR;

	if (!(tsr & CAN_TSR_TXOK0)) {
		SEGGER_RTT_printf(0, "CAN loopback: TX FAIL TSR=0x%08lX (RQCP=%d TXOK=%d TERR=%d ALST=%d)\n",
			(unsigned long)tsr,
			!!(tsr & CAN_TSR_RQCP0),
			!!(tsr & CAN_TSR_TXOK0),
			!!(tsr & CAN_TSR_TERR0),
			!!(tsr & CAN_TSR_ALST0));
		return; // will RCC-reset below
	}

	// Check RX FIFO 0
	uint32_t rf0r = CAN1->RF0R;
	if ((rf0r & 0x03) == 0) {
		SEGGER_RTT_printf(0, "CAN loopback: TX OK but NO RX (RF0R=0x%08lX) — HW broken?\n",
			(unsigned long)rf0r);
		return;
	}

	// Read and verify
	uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
	uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;
	uint32_t rdtr = CAN1->sFIFOMailBox[0].RDTR;
	uint16_t rx_id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;
	uint8_t  rx_dlc = rdtr & 0x0F;

	CAN1->RF0R |= CAN_RF0R_RFOM0; // release FIFO

	if (rx_id == 0x555 && rx_dlc == 8 &&
	    (rdlr & 0xFF) == 0xA5 && ((rdlr >> 8) & 0xFF) == 0x5A) {
		SEGGER_RTT_printf(0, "CAN loopback: PASS ✓ (ID=0x555, data match)\n");
		pass = 1;
	} else {
		SEGGER_RTT_printf(0, "CAN loopback: DATA MISMATCH ID=0x%03X DLC=%d"
			" data=%02lX %02lX %02lX %02lX %02lX %02lX %02lX %02lX\n",
			rx_id, rx_dlc,
			(unsigned long)(rdlr & 0xFF), (unsigned long)((rdlr >> 8) & 0xFF),
			(unsigned long)((rdlr >> 16) & 0xFF), (unsigned long)((rdlr >> 24) & 0xFF),
			(unsigned long)(rdhr & 0xFF), (unsigned long)((rdhr >> 8) & 0xFF),
			(unsigned long)((rdhr >> 16) & 0xFF), (unsigned long)((rdhr >> 24) & 0xFF));
	}

	// RCC reset to get clean state for normal-mode init
	SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CAN1RST);
	__NOP(); __NOP();
	CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CAN1RST);

	SEGGER_RTT_printf(0, "CAN loopback: %s, RCC reset done\n", pass ? "PASS" : "FAIL");
}

void can_protocol_init(void)
{
	SEGGER_RTT_printf(0, "CAN init: starting...\n");

	// Zero ring buffer
	memset(&rbuf, 0, sizeof(rbuf));

	// --- RCC reset CAN1 to clear stuck mailboxes ---
	__HAL_RCC_CAN1_CLK_ENABLE();
	SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CAN1RST);
	__NOP(); __NOP();
	CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CAN1RST);

	// Re-init CAN after reset
	HAL_CAN_Init(&hcan1);

	// --- Loopback test (bypasses transceiver, tests bxCAN hardware) ---
	can_loopback_test();

	// Re-init CAN after loopback RCC reset
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
	call_cnt++;

	uint8_t buf[8];
	Motor *m;

	// Alternate 0x201 / 0x202 per tick — keeps ≤2 frames/tick with IMU TX
	if (call_cnt & 1) {
		// Frame 0x201: M1(LB) speed + PWM, M2(LF) speed + PWM
		m = motor_get(MOTOR_M1_LB);
		put_i16(&buf[0], m->actual_speed);
		put_i16(&buf[2], (int16_t)m->pwm_output);
		m = motor_get(MOTOR_M2_LF);
		put_i16(&buf[4], m->actual_speed);
		put_i16(&buf[6], (int16_t)m->pwm_output);
		return can_send_frame(CAN_ID_MOTOR_TELEM_1, buf, 8);
	} else {
		// Frame 0x202: M3(RF) speed + PWM, M4(RB) speed + PWM
		m = motor_get(MOTOR_M3_RF);
		put_i16(&buf[0], m->actual_speed);
		put_i16(&buf[2], (int16_t)m->pwm_output);
		m = motor_get(MOTOR_M4_RB);
		put_i16(&buf[4], m->actual_speed);
		put_i16(&buf[6], (int16_t)m->pwm_output);
		return can_send_frame(CAN_ID_MOTOR_TELEM_2, buf, 8);
	}
}

// ---------------------------------------------------------------------------
// TX — IMU (0x204 + 0x205 + 0x206) @ 200Hz
// IMU hardware is offline → send synthetic test data so RK3588 CAN gateway
// development can proceed.
// ---------------------------------------------------------------------------

int can_send_imu(void)
{
	static uint32_t call_cnt;
	call_cnt++;

	uint8_t buf[8];
	uint8_t seq = (uint8_t)(call_cnt & 0xFF);

	if (!g_imu_ready) return -1;

	// Convert SI → CAN units per protocol
	int16_t ax = (int16_t)(g_imu_data.accel[0] * 1000.0f / 9.80665f); // mg
	int16_t ay = (int16_t)(g_imu_data.accel[1] * 1000.0f / 9.80665f);
	int16_t az = (int16_t)(g_imu_data.accel[2] * 1000.0f / 9.80665f);
	int16_t gx = (int16_t)(g_imu_data.gyro[0] * 57.29578f * 10.0f);   // 0.1°/s
	int16_t gy = (int16_t)(g_imu_data.gyro[1] * 57.29578f * 10.0f);
	int16_t gz = (int16_t)(g_imu_data.gyro[2] * 57.29578f * 10.0f);
	int16_t mx = (int16_t)g_imu_data.mag[0];                          // µT
	int16_t my = (int16_t)g_imu_data.mag[1];
	int16_t mz = (int16_t)g_imu_data.mag[2];

	// Roll/pitch from accelerometer (Madgwick disabled; RK3588 does EKF)
	float a0=g_imu_data.accel[0], a1=g_imu_data.accel[1], a2=g_imu_data.accel[2];
	int16_t roll  = (int16_t)(atan2f(a1, a2) * 57.29578f * 100.0f);   // 0.01°
	int16_t pitch = (int16_t)(atan2f(-a0, sqrtf(a1*a1+a2*a2)) * 57.29578f * 100.0f);

	// Battery: TODO read ADC; hardcode 12.0V for now
	uint8_t batt = 120; // 0.1V/bit

	// Send 1 frame per tick, cycling IDs — keeps ≤2 frames/tick with motor TX
	switch (call_cnt % 3) {
	case 0:
		put_i16(&buf[0], ax);
		put_i16(&buf[2], ay);
		put_i16(&buf[4], az);
		put_i16(&buf[6], gx);
		return can_send_frame(CAN_ID_IMU1, buf, 8);
	case 1:
		put_i16(&buf[0], gy);
		put_i16(&buf[2], gz);
		put_i16(&buf[4], mx);
		put_i16(&buf[6], my);
		return can_send_frame(CAN_ID_IMU2, buf, 8);
	default: // case 2
		put_i16(&buf[0], mz);
		put_i16(&buf[2], roll);
		put_i16(&buf[4], pitch);
		buf[6] = batt;
		buf[7] = seq;
		return can_send_frame(CAN_ID_IMU3, buf, 8);
	}
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
// TX — Test frame (debug: verifies CAN TX + bus ACK)
// Period controlled by g_can_test_period_ms; default 500ms (2Hz).
// ---------------------------------------------------------------------------

uint32_t g_can_test_period_ms = 500;
int      g_can_mode          = 0;    // 0=normal telemetry, 1=test burst

int can_send_test(void)
{
	static uint32_t last_ms;
	static uint32_t print_count;
	static uint8_t  seq;
	uint32_t now = HAL_GetTick();

	if (now - last_ms < g_can_test_period_ms) return 0;
	last_ms = now;

	uint8_t buf[8] = {0};
	buf[0] = seq++;
	for (int i = 1; i < 8; i++) buf[i] = (uint8_t)i;

	uint32_t tsr = READ_REG(hcan1.Instance->TSR);
	uint32_t esr = READ_REG(hcan1.Instance->ESR);

	int rc = can_send_frame(0x201, buf, 8);

	// Print every 100 frames to avoid flooding RTT at high rates
	print_count++;
	if (print_count % 100 == 0 || rc != 0) {
		SEGGER_RTT_printf(0, "CAN test: seq=%u rc=%d TSR=0x%08lX ESR=0x%08lX period=%lums\n",
			(unsigned)(uint8_t)(seq - 1), rc, (unsigned long)tsr, (unsigned long)esr,
			(unsigned long)g_can_test_period_ms);
	}

	return rc;
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
