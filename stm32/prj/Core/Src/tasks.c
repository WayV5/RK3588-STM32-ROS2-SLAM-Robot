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

	// Store raw (chip axes, no correction)
	// Conversion to CAN/SI is done by the consumer:
	//   can_send_imu: raw→CAN int16 (body frame, inline)
	//   cmd_imu:      raw→SI        (body frame, on demand)
	g_imu_data.accel[0] = raw.accel[0];
	g_imu_data.accel[1] = raw.accel[1];
	g_imu_data.accel[2] = raw.accel[2];
	g_imu_data.gyro[0]  = raw.gyro[0];
	g_imu_data.gyro[1]  = raw.gyro[1];
	g_imu_data.gyro[2]  = raw.gyro[2];
	g_imu_data.temp      = raw.temp;
	if (mag_valid) {
		g_imu_data.mag[0] = mag_raw[0];
		g_imu_data.mag[1] = mag_raw[1];
		g_imu_data.mag[2] = mag_raw[2];
	}

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

// [1kHz] CAN TX scheduler — 1 frame per tick, cnt%8 slots, non-blocking.
// Slot 0:0x201(M1+M2) 1:0x202(M3+M4) 2:0x203(accel) 3:0x204(gyro)
//       4:(skip) 5:(skip) 6:0x203(accel) 7:0x204(gyro)
// Rates: motor 125Hz, accel 250Hz, gyro 250Hz.  Mailbox full → skip slot.
void task_can_tx_scheduled(void)
{
	static uint32_t last_ms;
	static uint8_t  slot;
	uint32_t now = HAL_GetTick();
	if (now - last_ms < 1) return;
	last_ms = now;

	if (g_can_mode != 0) return;

	uint8_t buf[8];
	Motor *m;

	switch (slot) {
	case 0: // 0x201: M1(LB)+M2(LF)
		m = motor_get(MOTOR_M1_LB);
		put_i16(&buf[0], m->actual_speed);
		put_i16(&buf[2], (int16_t)m->pwm_output);
		m = motor_get(MOTOR_M2_LF);
		put_i16(&buf[4], m->actual_speed);
		put_i16(&buf[6], (int16_t)m->pwm_output);
		can_send_frame(CAN_ID_MOTOR_TELEM_1, buf, 8);
		break;
	case 1: // 0x202: M3(RF)+M4(RB)
		m = motor_get(MOTOR_M3_RF);
		put_i16(&buf[0], m->actual_speed);
		put_i16(&buf[2], (int16_t)m->pwm_output);
		m = motor_get(MOTOR_M4_RB);
		put_i16(&buf[4], m->actual_speed);
		put_i16(&buf[6], (int16_t)m->pwm_output);
		can_send_frame(CAN_ID_MOTOR_TELEM_2, buf, 8);
		break;
	case 2: case 6: // 0x203: AccelX/Y/Z (raw→body→mg, DLC=6)
		if (g_imu_ready) {
			put_i16(&buf[0], (int16_t)(-(int32_t)g_imu_data.accel[0]*1000/8192));
			put_i16(&buf[2], (int16_t)(-(int32_t)g_imu_data.accel[1]*1000/8192));
			put_i16(&buf[4], (int16_t)(-(int32_t)g_imu_data.accel[2]*1000/8192));
			can_send_frame(CAN_ID_IMU_ACCEL, buf, 6);
		}
		break;
	case 3: case 7: // 0x204: GyroX/Y/Z (raw→body→0.1°/s, DLC=6)
		if (g_imu_ready) {
			put_i16(&buf[0], (int16_t)(-(int32_t)g_imu_data.gyro[0]*10000/65536));
			put_i16(&buf[2], (int16_t)(-(int32_t)g_imu_data.gyro[1]*10000/65536));
			put_i16(&buf[4], (int16_t)(-(int32_t)g_imu_data.gyro[2]*10000/65536));
			can_send_frame(CAN_ID_IMU_GYRO, buf, 6);
		}
		break;
	default: break; // slots 4,5: empty
	}
	slot = (slot + 1) & 7;
}

// [20Hz] CAN TX: magnetometer (0x205) — MagX/Y/Z + Temperature
void task_can_tx_mag_20hz(void)
{
	static uint32_t last_ms;
	uint32_t now = HAL_GetTick();
	if (now - last_ms < 50) return;
	last_ms = now;

	if (g_can_mode != 0 || !g_imu_ready) return;

	uint8_t buf[8];
	// Mag: body_X=-chip_Y, body_Y=-chip_X, body_Z=+chip_Z, *15/100→µT
	put_i16(&buf[0], (int16_t)(-(int32_t)g_imu_data.mag[1]*15/100));
	put_i16(&buf[2], (int16_t)(-(int32_t)g_imu_data.mag[0]*15/100));
	put_i16(&buf[4], (int16_t)( +(int32_t)g_imu_data.mag[2]*15/100));
	// Temp: raw→0.1°C
	put_i16(&buf[6], (int16_t)(((float)g_imu_data.temp/333.87f+21.0f)*10.0f));
	can_send_frame(CAN_ID_IMU_MAG, buf, 8);
}

// [10Hz] RTT J-Scope 8-channel waveform (debug)
void task_rtt_scope_10hz(void)
{
	rtt_scope_output();
}

// [2Hz] RTT telemetry text (debug)
void task_rtt_telemetry_2hz(void)
{
	rtt_telemetry_output();
}

// [N/A] CAN TX test — enabled by RTT "can test <ms>", disabled by "can norm"
void task_can_test(void)
{
	if (g_can_mode != 1) return;
	can_send_test();
}

// [background] Non-blocking command input — RTT console + CAN ringbuf drain
void task_command_poll(void)
{
	rtt_console_poll();
	can_command_process();
}
