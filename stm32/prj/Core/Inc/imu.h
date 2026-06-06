#ifndef __IMU_H__
#define __IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// I2C addresses (7-bit, HAL expects them left-shifted)
#define MPU9250_I2C_ADDR	(0x68 << 1)	// 0xD0
#define AK8963_I2C_ADDR		(0x0C << 1)	// 0x18

// MPU9250 registers
#define MPU9250_WHO_AM_I	0x75
#define MPU9250_PWR_MGMT_1	0x6B
#define MPU9250_SMPLRT_DIV	0x19
#define MPU9250_CONFIG		0x1A
#define MPU9250_GYRO_CONFIG	0x1B
#define MPU9250_ACCEL_CONFIG	0x1C
#define MPU9250_INT_PIN_CFG	0x37
#define MPU9250_ACCEL_XOUT_H	0x3B
#define MPU9250_GYRO_XOUT_H	0x43

// AK8963 registers (accessed via I2C bypass)
#define AK8963_WHO_AM_I		0x00
#define AK8963_CNTL1		0x0A
#define AK8963_HXL		0x03
#define AK8963_ST2		0x09

// Sensor scales — MPU6500 16-bit ADC (±32768 counts)
//
// Scale factor = 32768 / full_scale_range
// Smaller range → more LSB per physical unit → better resolution.
// For a slow indoor diff-drive robot (±500°/s, ±4g) gives 4x better
// resolution than the defaults (±2000°/s, ±16g) while keeping enough
// headroom (max turn ~200°/s, max shock <3g on bump).
//
// Range trade-off reference (16-bit ADC):
//   Accel:  ±2g=16384, ±4g=8192,  ±8g=4096,  ±16g=2048 LSB/g
//   Gyro:   ±250=131.1, ±500=65.5, ±1000=32.8, ±2000=16.4 LSB/(°/s)
#define ACCEL_SCALE_4G		8192.0f		// LSB/g at ±4g
#define GYRO_SCALE_500DPS	65.536f		// LSB/(°/s) at ±500°/s
#define MAG_SCALE_16BIT		0.15f		// µT/LSB (AK8963 fixed range)
#define GRAVITY_MSS		9.80665f	// m/s² per g
#define DEG2RAD			0.0174532925f	// PI/180

// Madgwick AHRS
#define MADGWICK_BETA		0.1f		// gyroscope noise weight
#define MADGWICK_SAMPLE_HZ	200.0f
#define MAG_READ_MS		50		// read magnetometer every 50ms (20Hz)

// Raw sensor data from register burst reads
typedef struct {
	int16_t	accel[3];	// raw ADC: ax, ay, az
	int16_t	temp;		// raw temperature
	int16_t	gyro[3];	// raw ADC: gx, gy, gz
} ImuRaw6Axis;

typedef struct {
	int16_t	mag[3];		// raw ADC: mx, my, mz
} ImuRawMag;

// Raw sensor data (chip axes, no correction applied)
// Conversion to CAN/SI units is done by the consumer (can_send_imu, cmd_imu).
typedef struct {
	int16_t	accel[3];	// raw ADC, MPU6500 chip axes
	int16_t	gyro[3];	// raw ADC, MPU6500 chip axes
	int16_t	mag[3];		// raw ADC, AK8963 chip axes (LE, 0.15µT/LSB)
	int16_t	temp;		// raw ADC
} ImuData;

// Euler angles in degrees
typedef struct {
	float	roll;		// X-axis rotation
	float	pitch;		// Y-axis rotation
	float	yaw;		// Z-axis rotation (drifting without GPS)
} Attitude;

// Madgwick filter state (quaternion)
typedef struct {
	float	q0, q1, q2, q3;
} Madgwick;

// --- Public API ---

// Initialise both chips: MPU9250 + AK8963. Returns 0 on success.
// Prints WHO_AM_I values to RTT for diagnostics.
int imu_init(void);

// Burst-read 14 bytes: accel(6) + temp(2) + gyro(6) from MPU9250
int imu_read_6axis(ImuRaw6Axis *raw);

// Burst-read 7 bytes from AK8963. Returns 0 on success.
// Only call this every 50ms (20Hz).
int imu_read_mag(ImuRawMag *raw);


// --- Madgwick AHRS ---

void madgwick_init(Madgwick *m);
void madgwick_update(Madgwick *m,
		     float gx, float gy, float gz,	// rad/s
		     float ax, float ay, float az,	// m/s²
		     float mx, float my, float mz,	// µT
		     float dt);				// seconds
void madgwick_get_attitude(const Madgwick *m, Attitude *a);

// --- Global state (for CAN telemetry) ---

extern ImuData	g_imu_data;
extern Attitude	g_attitude;
extern uint8_t	g_imu_ready;		// 1 after successful init + first read
extern uint8_t	g_mag_available;	// 1 if AK8963 magnetometer is present

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H__ */
