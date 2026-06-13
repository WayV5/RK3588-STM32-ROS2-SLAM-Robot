#include "robot_can_gateway/can_protocol.hpp"

namespace robot_can_gateway
{
namespace protocol
{

// ---- Internal helpers ----

static inline int16_t get_i16(const uint8_t *buf)
{
	return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

static inline void put_i16(uint8_t *buf, int16_t v)
{
	buf[0] = (uint8_t)(v & 0xFF);
	buf[1] = (uint8_t)((v >> 8) & 0xFF);
}

static constexpr float ACCEL_SCALE = 8192.0f;
static constexpr float GYRO_SCALE  = 65.536f;
static constexpr float MAG_SCALE   = 0.15f;
static constexpr float GRAVITY     = 9.80665f;
static constexpr float TEMP_SLOPE  = 333.87f;
static constexpr float TEMP_OFFSET = 21.0f;

// ---- Protocol v3 decoders ----

FourMotorSpeeds decode_motor_speeds(const struct can_frame &frame)
{
	FourMotorSpeeds s = {};
	s.m1 = get_i16(&frame.data[0]);
	s.m2 = get_i16(&frame.data[2]);
	s.m3 = get_i16(&frame.data[4]);
	s.m4 = get_i16(&frame.data[6]);
	return s;
}

FourMotorPWM decode_motor_pwm(const struct can_frame &frame)
{
	FourMotorPWM p = {};
	p.m1 = get_i16(&frame.data[0]);
	p.m2 = get_i16(&frame.data[2]);
	p.m3 = get_i16(&frame.data[4]);
	p.m4 = get_i16(&frame.data[6]);
	return p;
}

ImuAccelRaw decode_imu_accel(const struct can_frame &frame)
{
	ImuAccelRaw r = {};
	r.ax_raw = get_i16(&frame.data[0]);
	r.ay_raw = get_i16(&frame.data[2]);
	r.az_raw = get_i16(&frame.data[4]);
	return r;
}

ImuGyroRaw decode_imu_gyro(const struct can_frame &frame)
{
	ImuGyroRaw r = {};
	r.gx_raw = get_i16(&frame.data[0]);
	r.gy_raw = get_i16(&frame.data[2]);
	r.gz_raw = get_i16(&frame.data[4]);
	return r;
}

ImuMagTempRaw decode_imu_mag_temp(const struct can_frame &frame)
{
	ImuMagTempRaw r = {};
	r.mx_raw   = get_i16(&frame.data[0]);
	r.my_raw   = get_i16(&frame.data[2]);
	r.mz_raw   = get_i16(&frame.data[4]);
	r.temp_raw = get_i16(&frame.data[6]);
	return r;
}

SysStatus decode_sys_status(const struct can_frame &frame)
{
	SysStatus s = {};
	s.flags       = frame.data[0];
	s.fault_code  = frame.data[1];
	return s;
}

// ---- Raw-to-SI converters ----

ImuAccelSI raw_to_si(const ImuAccelRaw &raw)
{
	ImuAccelSI s = {};
	s.ax = (float)raw.ax_raw / ACCEL_SCALE * GRAVITY;
	s.ay = (float)raw.ay_raw / ACCEL_SCALE * GRAVITY;
	s.az = (float)raw.az_raw / ACCEL_SCALE * GRAVITY;
	return s;
}

ImuGyroSI raw_to_si(const ImuGyroRaw &raw)
{
	ImuGyroSI s = {};
	s.gx = (float)raw.gx_raw / GYRO_SCALE;
	s.gy = (float)raw.gy_raw / GYRO_SCALE;
	s.gz = (float)raw.gz_raw / GYRO_SCALE;
	return s;
}

ImuMagTempSI raw_to_si(const ImuMagTempRaw &raw)
{
	ImuMagTempSI s = {};
	s.mx     = (float)raw.mx_raw * MAG_SCALE;
	s.my     = (float)raw.my_raw * MAG_SCALE;
	s.mz     = (float)raw.mz_raw * MAG_SCALE;
	s.temp_c = (float)raw.temp_raw / TEMP_SLOPE + TEMP_OFFSET;
	return s;
}

// ---- Command encoder ----

uint8_t encode_motor_command(uint8_t *data_out, const MotorCommand &cmd)
{
	put_i16(&data_out[0], cmd.m1_speed_mms);
	put_i16(&data_out[2], cmd.m2_speed_mms);
	put_i16(&data_out[4], cmd.m3_speed_mms);
	put_i16(&data_out[6], cmd.m4_speed_mms);
	return 8;
}

// ---- Helpers ----

bool is_known_telemetry_id(uint32_t can_id)
{
	switch (can_id & CAN_SFF_MASK) {
	case 0x201: case 0x202:
	case 0x203: case 0x204:
	case 0x205: case 0x206:
		return true;
	default:
		return false;
	}
}

const char *can_id_name(uint32_t can_id)
{
	switch (can_id & CAN_SFF_MASK) {
	case 0x101: return "0x101 MotorCmd";
	case 0x102: return "0x102 E-Stop";
	case 0x103: return "0x103 PID Config";
	case 0x201: return "0x201 Speeds";
	case 0x202: return "0x202 PWM";
	case 0x203: return "0x203 Accel";
	case 0x204: return "0x204 Gyro";
	case 0x205: return "0x205 Mag+Temp";
	case 0x206: return "0x206 Status";
	default:    return "unknown";
	}
}

} // namespace protocol
} // namespace robot_can_gateway
