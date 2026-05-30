#include "tasks.h"
#include "motor.h"
#include "rtt_console.h"

// ---------------------------------------------------------------------------
// Task scheduler: time-triggered cooperative multitasking
// SysTick @ 1kHz provides sys_tick_flag (extern from main.c);
// HAL_GetTick() for coarser period throttling.
// ---------------------------------------------------------------------------

extern volatile uint8_t sys_tick_flag;

// [1kHz] Motor PID + encoder — SysTick-gated
void task_motor_1khz(void)
{
	if (!sys_tick_flag) return;
	sys_tick_flag = 0;
	motor_control_update();
}

// [200Hz] IMU read + Madgwick AHRS (period = 5 ms)
// TODO: implement MPU9250 I2C driver and Madgwick filter
void task_imu_200hz(void)
{
	static uint32_t last_ms = 0;
	uint32_t now = HAL_GetTick();
	if (now - last_ms < 5) return;
	last_ms = now;

	// imu_read();           // I2C1@400kHz, 14 bytes, ~300us
	// madgwick_update();    // Roll/Pitch/Yaw, ~50us
}

// [200Hz] CAN TX: IMU frames 0x204/0x205/0x206
// Same rate as IMU sampling — every raw sample goes to RK3588 for EKF fusion.
// TODO: implement can_send_imu()
void task_can_tx_imu_200hz(void)
{
	static uint32_t last_ms = 0;
	uint32_t now = HAL_GetTick();
	if (now - last_ms < 5) return;
	last_ms = now;

	// can_send_imu();  // 3 frames, ~300us
}

// [100Hz] CAN TX: motor telemetry frames 0x201/0x202
// 10:1 down-sample from 1kHz PID loop — wheel inertia limits mechanical BW.
// TODO: implement can_send_motor_telemetry()
void task_can_tx_motor_100hz(void)
{
	static uint32_t last_ms = 0;
	uint32_t now = HAL_GetTick();
	if (now - last_ms < 10) return;
	last_ms = now;

	// can_send_motor_telemetry();  // 2 frames, ~200us
}

// [10Hz] RTT J-Scope 8-channel waveform (debug; removed in release via #ifdef)
void task_rtt_scope_10hz(void)
{
	rtt_scope_output();  // self-throttles at 10 ms internally
}

// [2Hz] RTT telemetry text (debug; removed in release via #ifdef)
void task_rtt_telemetry_2hz(void)
{
	rtt_telemetry_output();  // self-throttles at 500 ms internally
}

// [background] Non-blocking command input — reads RTT + CAN ringbuf
void task_command_poll(void)
{
	rtt_console_poll();
	// can_command_process();  // TODO: CAN 0x101 motion command consumer
}
