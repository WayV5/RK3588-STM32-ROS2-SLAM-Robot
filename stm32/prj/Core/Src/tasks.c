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

// [250Hz] IMU data acquisition (period = 4 ms) — matches CAN accel/gyro TX rate.
// Attitude fusion is done by RK3588 EKF; STM32 only collects raw sensor data.
void task_imu_250hz(void)
{
	static uint32_t		last_ms = 0;
	static uint32_t		last_mag_ms = 0;

	uint32_t now = HAL_GetTick();
	if (now - last_ms < 4) return;
	last_ms = now;

	if (!g_imu_ready) return;

	// Read accel + gyro (14 bytes burst, ~300us)
	ImuRaw6Axis raw;
	if (imu_read_6axis(&raw) != 0) return;

	// Chip→body correction: accel/gyro body = -chip
	g_imu_data.accel[0] = -raw.accel[0];
	g_imu_data.accel[1] = -raw.accel[1];
	g_imu_data.accel[2] = -raw.accel[2];
	g_imu_data.gyro[0]  = -raw.gyro[0];
	g_imu_data.gyro[1]  = -raw.gyro[1];
	g_imu_data.gyro[2]  = -raw.gyro[2];
	g_imu_data.temp      = raw.temp;

	// Mag @ 20Hz — body_X=-chip_Y, body_Y=-chip_X, body_Z=+chip_Z
	if (g_mag_available && now - last_mag_ms >= MAG_READ_MS) {
		ImuRawMag raw_mag;
		if (imu_read_mag(&raw_mag) == 0) {
			g_imu_data.mag[0] = -raw_mag.mag[1];
			g_imu_data.mag[1] = -raw_mag.mag[0];
			g_imu_data.mag[2] = +raw_mag.mag[2];
		}
		last_mag_ms = now;
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

// [1kHz] CAN TX slot scheduler — 1 frame per tick, cnt%8 slots, non-blocking.
//
// Slot layout (8 slots × 1ms = 8ms cycle):
//   0: 0x201  4-wheel speeds     every cycle             125.000 Hz
//   1: 0x202  4-wheel PWM        every 12 cycles          10.417 Hz
//   2: 0x203  Accel raw ADC                                250.000 Hz (×2)
//   3: 0x204  Gyro raw ADC                                 250.000 Hz (×2)
//   4: 0x205  Mag+Temp           every  6 cycles          20.833 Hz
//   5: 0x206  System status      every 125 cycles          1.000 Hz
//   6: 0x203  Accel raw ADC                                250.000 Hz
//   7: 0x204  Gyro raw ADC                                 250.000 Hz
//
// Sub-cycle slots (1,4,5) use a per-slot phase counter: on each slot hit,
// increment cnt; when cnt >= period, send and reset cnt to 0.
// Mailbox full → skip slot (non-blocking).
//
void task_can_tx_scheduled(void)
{
	static uint32_t last_ms;
	static uint8_t  slot;
	static uint8_t  cnt_slot1;   // 0x202 phase counter (period 12)
	static uint8_t  cnt_slot4;   // 0x205 phase counter (period 6)
	static uint16_t cnt_slot5;   // 0x206 phase counter (period 125)

	uint32_t now = HAL_GetTick();
	if (now - last_ms < 1) return;
	last_ms = now;

	if (g_can_mode != 0) return;

	uint8_t buf[8];
	Motor *m;

	switch (slot) {
	case 0: // 0x201: all 4 wheel speeds (int16 LE ×4) — 125Hz
		for (int i = 0; i < 4; i++) {
			m = motor_get((MotorID)i);
			put_i16(&buf[i * 2], m->actual_speed);
		}
		can_send_frame(CAN_ID_MOTOR_SPEED, buf, 8);
		break;

	case 1: // 0x202: all 4 PWM (int16 LE ×4) — 10.417Hz (every 12 cycles)
		cnt_slot1++;
		if (cnt_slot1 >= 12) {
			cnt_slot1 = 0;
			for (int i = 0; i < 4; i++) {
				m = motor_get((MotorID)i);
				put_i16(&buf[i * 2], (int16_t)m->pwm_output);
			}
			can_send_frame(CAN_ID_MOTOR_PWM, buf, 8);
		}
		break;

	case 2: case 6: // 0x203: Accel raw ADC (body frame) — 250Hz
		if (g_imu_ready) {
			put_i16(&buf[0], g_imu_data.accel[0]);
			put_i16(&buf[2], g_imu_data.accel[1]);
			put_i16(&buf[4], g_imu_data.accel[2]);
			can_send_frame(CAN_ID_IMU_ACCEL, buf, 6);
		}
		break;

	case 3: case 7: // 0x204: Gyro raw ADC (body frame) — 250Hz
		if (g_imu_ready) {
			put_i16(&buf[0], g_imu_data.gyro[0]);
			put_i16(&buf[2], g_imu_data.gyro[1]);
			put_i16(&buf[4], g_imu_data.gyro[2]);
			can_send_frame(CAN_ID_IMU_GYRO, buf, 6);
		}
		break;

	case 4: // 0x205: Mag+Temp raw ADC (body frame) — 20.833Hz (every 6 cycles)
		cnt_slot4++;
		if (cnt_slot4 >= 6) {
			cnt_slot4 = 0;
			if (g_imu_ready) {
				put_i16(&buf[0], g_imu_data.mag[0]);
				put_i16(&buf[2], g_imu_data.mag[1]);
				put_i16(&buf[4], g_imu_data.mag[2]);
				put_i16(&buf[6], g_imu_data.temp);
				can_send_frame(CAN_ID_IMU_MAG, buf, 8);
			}
		}
		break;

	case 5: // 0x206: System status — 1Hz (every 125 cycles)
		cnt_slot5++;
		if (cnt_slot5 >= 125) {
			cnt_slot5 = 0;
			put_i16(&buf[2], 0);	// battery_mV = 0 (not available yet)
			buf[0] = can_get_status_flags();
			buf[1] = can_get_fault_code();
			can_send_frame(CAN_ID_SYS_STATUS, buf, 4);
		}
		break;
	}
	slot = (slot + 1) & 7;
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
