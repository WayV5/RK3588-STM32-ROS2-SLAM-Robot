#include "tasks.h"
#include "motor.h"
#include "rtt_console.h"
#include "imu.h"
#include "can_protocol.h"

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

// [200Hz] IMU data acquisition + chip→body transform (period = 5 ms)
// Attitude fusion is done by RK3588 EKF; STM32 only collects raw sensor data.
void task_imu_200hz(void)
{
	static uint32_t		last_ms = 0;
	static uint32_t		last_mag_ms = 0;

	uint32_t now = HAL_GetTick();
	if (now - last_ms < 5) return;
	last_ms = now;

	if (!g_imu_ready) return;

	// Read accel + gyro (14 bytes burst, ~300us)
	ImuRaw6Axis raw;
	if (imu_read_6axis(&raw) != 0) return;

	// Read mag at 20Hz (every 50ms) — skip if no magnetometer
	int	mag_valid = 0;
	int16_t	mag_raw[3] = {0};
	if (g_mag_available && now - last_mag_ms >= MAG_READ_MS) {
		ImuRawMag raw_mag;
		if (imu_read_mag(&raw_mag) == 0) {
			mag_raw[0] = raw_mag.mag[0];
			mag_raw[1] = raw_mag.mag[1];
			mag_raw[2] = raw_mag.mag[2];
			mag_valid = 1;
		}
		last_mag_ms = now;
	}

	// Convert raw → SI (chip→body axis correction)
	imu_raw_to_si(&raw, &g_imu_data, mag_valid, mag_raw);

#if 0 // Madgwick AHRS — deprecated; RK3588 EKF does the real fusion
	static Madgwick		madgwick;
	static uint8_t		madgwick_inited = 0;

	if (!madgwick_inited) {
		madgwick_init(&madgwick);
		madgwick_inited = 1;
	}
	madgwick_update(&madgwick,
			g_imu_data.gyro[0], g_imu_data.gyro[1], g_imu_data.gyro[2],
			g_imu_data.accel[0], g_imu_data.accel[1], g_imu_data.accel[2],
			g_imu_data.mag[0], g_imu_data.mag[1], g_imu_data.mag[2],
			0.005f);
	madgwick_get_attitude(&madgwick, &g_attitude);
#endif
}

// [200Hz] CAN TX: IMU frames 0x204/0x205/0x206
// Same rate as IMU sampling — every raw sample goes to RK3588 for EKF fusion.
void task_can_tx_imu_200hz(void)
{
	static uint32_t last_ms = 0;
	uint32_t now = HAL_GetTick();
	if (now - last_ms < 5) return;
	last_ms = now;

	can_send_imu();	// 3 frames, ~300µs
}

// [100Hz] CAN TX: motor telemetry frames 0x201/0x202
// 10:1 down-sample from 1kHz PID loop — wheel inertia limits mechanical BW.
void task_can_tx_motor_100hz(void)
{
	static uint32_t last_ms = 0;
	uint32_t now = HAL_GetTick();
	if (now - last_ms < 10) return;
	last_ms = now;

	can_send_motor_telemetry();	// 2 frames, ~200µs
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

// [background] Non-blocking command input — RTT console + CAN ringbuf drain
void task_command_poll(void)
{
	rtt_console_poll();
	can_command_process();
}
