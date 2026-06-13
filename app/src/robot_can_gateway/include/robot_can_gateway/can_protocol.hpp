#pragma once

#include <cstdint>
#include <linux/can.h>

namespace robot_can_gateway
{
namespace protocol
{

// ---- Protocol v3 decoded types ----

// 0x201: all 4 wheel speeds in one frame (125Hz)
struct FourMotorSpeeds {
	int16_t m1;	// LB, mm/s
	int16_t m2;	// LF, mm/s
	int16_t m3;	// RF, mm/s
	int16_t m4;	// RB, mm/s
};

// 0x202: all 4 motor PWM outputs (10.417Hz)
struct FourMotorPWM {
	int16_t m1;	// -1000..1000
	int16_t m2;
	int16_t m3;
	int16_t m4;
};

// 0x206: system status + fault + reserved (1Hz)
struct SysStatus {
	uint8_t flags;
	uint8_t fault_code;
	// [2-3] reserved
};

struct ImuAccelRaw {
	int16_t ax_raw;
	int16_t ay_raw;
	int16_t az_raw;
};

struct ImuGyroRaw {
	int16_t gx_raw;
	int16_t gy_raw;
	int16_t gz_raw;
};

struct ImuMagTempRaw {
	int16_t mx_raw;
	int16_t my_raw;
	int16_t mz_raw;
	int16_t temp_raw;
};

struct ImuAccelSI {
	float ax;	// m/s^2
	float ay;
	float az;
};

struct ImuGyroSI {
	float gx;	// deg/s
	float gy;
	float gz;
};

struct ImuMagTempSI {
	float mx;	// uT
	float my;
	float mz;
	float temp_c;	// degrees Celsius
};

struct MotorCommand {
	int16_t m1_speed_mms;	// M1 (LB)
	int16_t m2_speed_mms;	// M2 (LF)
	int16_t m3_speed_mms;	// M3 (RF)
	int16_t m4_speed_mms;	// M4 (RB)
};

// ---- Telemetry decoders (STM32 -> RK3588) ----

FourMotorSpeeds   decode_motor_speeds(const struct can_frame &frame);	// 0x201
FourMotorPWM      decode_motor_pwm(const struct can_frame &frame);	// 0x202
ImuAccelRaw       decode_imu_accel(const struct can_frame &frame);	// 0x203
ImuGyroRaw        decode_imu_gyro(const struct can_frame &frame);	// 0x204
ImuMagTempRaw     decode_imu_mag_temp(const struct can_frame &frame);	// 0x205
SysStatus         decode_sys_status(const struct can_frame &frame);	// 0x206

// ---- Raw-to-SI converters ----

ImuAccelSI    raw_to_si(const ImuAccelRaw &raw);
ImuGyroSI     raw_to_si(const ImuGyroRaw &raw);
ImuMagTempSI  raw_to_si(const ImuMagTempRaw &raw);

// ---- Command encoder (RK3588 -> STM32) ----

// Writes 4x int16 LE into data_out[0..7]. Returns DLC=8.
uint8_t encode_motor_command(uint8_t *data_out, const MotorCommand &cmd);

// ---- Helpers ----

bool is_known_telemetry_id(uint32_t can_id);
const char *can_id_name(uint32_t can_id);

} // namespace protocol
} // namespace robot_can_gateway
