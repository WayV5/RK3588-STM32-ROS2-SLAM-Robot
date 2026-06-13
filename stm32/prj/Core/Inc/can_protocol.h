#ifndef __CAN_PROTOCOL_H__
#define __CAN_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "can.h"

// ---------------------------------------------------------------------------
// CAN message IDs
// 0x1xx = control (smaller ID → higher arbitration priority)
// 0x2xx = telemetry
// ---------------------------------------------------------------------------
#define CAN_ID_MOTOR_CMD	0x101	// RK3588→STM32: 4-motor speed targets (100Hz)
#define CAN_ID_ESTOP		0x102	// RK3588→STM32: emergency stop (event)
#define CAN_ID_PID_CONFIG	0x103	// RK3588→STM32: PID parameter tuning (event)
#define CAN_ID_MOTOR_SPEED	0x201	// STM32→RK3588: 4-wheel speeds, DLC=8 (125Hz)
#define CAN_ID_MOTOR_PWM	0x202	// STM32→RK3588: 4-wheel PWM, DLC=8 (10.417Hz)
#define CAN_ID_IMU_ACCEL	0x203	// STM32→RK3588: Accel raw ADC (body frame), DLC=6 (250Hz)
#define CAN_ID_IMU_GYRO	0x204	// STM32→RK3588: Gyro raw ADC (body frame), DLC=6 (250Hz)
#define CAN_ID_IMU_MAG		0x205	// STM32→RK3588: Mag raw ADC + Temp raw, DLC=8 (20.833Hz)
#define CAN_ID_SYS_STATUS	0x206	// STM32→RK3588: system status + battery, DLC=4 (1Hz)
// Scale: Accel/8192*9.80665=m/s², Gyro/65.536=°/s, Mag*0.15=µT, Temp/333.87+21=°C

// ---------------------------------------------------------------------------
// Estop command codes (0x102 data[0])
// ---------------------------------------------------------------------------
#define ESTOP_ENGAGE		0x01
#define ESTOP_RELEASE		0x00

// ---------------------------------------------------------------------------
// System status flags (0x206 data[0])
// ---------------------------------------------------------------------------
#define SYS_FLAG_ESTOP		(1 << 0)
#define SYS_FLAG_FAULT		(1 << 1)
#define SYS_FLAG_IMU_READY	(1 << 2)
#define SYS_FLAG_LOW_BATTERY	(1 << 3)

// ---------------------------------------------------------------------------
// RX ring buffer — decouples ISR from main loop
// Depth must outpace worst-case interrupt burst (3 frames × 200Hz burst = 600
// frames/sec, main loop drains at ~1000 Hz → 64 is generous).
// ---------------------------------------------------------------------------
#define CAN_RX_RINGBUF_SIZE	64

typedef struct {
	CAN_RxHeaderTypeDef	header;
	uint8_t			data[8];
} CanRxFrame;

typedef struct {
	CanRxFrame		buf[CAN_RX_RINGBUF_SIZE];
	volatile uint32_t	head;	// ISR writes
	uint32_t		tail;	// main loop reads
} CanRingBuf;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Helper: pack int16 into buf (little-endian)
static inline void put_i16(uint8_t *buf, int16_t v)
	{ buf[0] = (uint8_t)v; buf[1] = (uint8_t)(v >> 8); }

// One-time initialisation: start CAN peripheral, config filter, activate
// interrupt notification. Must be called after MX_CAN1_Init() and before
// any TX/RX.
void can_protocol_init(void);

// Send one CAN frame (non-blocking). Returns 0 on success, -1 if all
// mailboxes full (frame dropped). Clears completed mailboxes first.
int  can_send_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc);

// [event] Send emergency stop frame 0x102.
int  can_send_estop(uint8_t state);

// [event] Send PID config frame 0x103.
int  can_send_pid_config(uint8_t motor_id, uint8_t param_type, float value);

// [variable rate] Test frame 0x201 — single frame with counter, prints TSR/ESR.
// Period g_can_test_period_ms (default 500ms).  RTT: "can test <ms>"
extern uint32_t g_can_test_period_ms;
extern int      g_can_mode;  // 0=normal telemetry, 1=test burst
int  can_send_test(void);

// Called from main loop — pops frames from ring buffer, decodes and
// dispatches: 0x101→motor_control_set_target, 0x102→motor_control_stop_all,
// 0x103→motor_control_set_ff_gain / PID tuning.
void can_command_process(void);

// Interrupt callback — enqueues received frame into ring buffer.
// Called by HAL_CAN_IRQHandler() from CAN1_RX0_IRQHandler context.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

// Ring buffer helpers (also exposed for debug/diagnostic)
int  can_ringbuf_is_empty(void);
int  can_ringbuf_is_full(void);

// [1Hz] Send system status frame 0x206 (flags + fault_code + battery_mV).
int  can_send_sys_status(void);

// Build system status flags for 0x206
uint8_t can_get_status_flags(void);
uint8_t can_get_fault_code(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_PROTOCOL_H__ */
