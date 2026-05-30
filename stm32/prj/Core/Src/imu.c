#include "imu.h"
#include "i2c.h"
#include "rtt_console.h"
#include <math.h>
#include <string.h>
#include "SEGGER_RTT.h"

extern I2C_HandleTypeDef hi2c1;

// --- Global state ---
ImuData		g_imu_data;
Attitude	g_attitude;
uint8_t		g_imu_ready;
uint8_t		g_mag_available;

// --- Static helpers ---

static float inv_sqrt(float x)
{
	return 1.0f / sqrtf(x);
}

static int reg_read(uint8_t dev_addr, uint8_t reg, uint8_t *val)
{
	return HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg,
				I2C_MEMADD_SIZE_8BIT, val, 1,
				HAL_MAX_DELAY);
}

static int reg_write(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
	return HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg,
				 I2C_MEMADD_SIZE_8BIT, &val, 1,
				 HAL_MAX_DELAY);
}

static int burst_read(uint8_t dev_addr, uint8_t start_reg, uint8_t *buf, uint8_t len)
{
	return HAL_I2C_Mem_Read(&hi2c1, dev_addr, start_reg,
				I2C_MEMADD_SIZE_8BIT, buf, len,
				HAL_MAX_DELAY);
}

// --- Public API ---

int imu_init(void)
{
	uint8_t who;
	HAL_StatusTypeDef status;

	// 1. Check WHO_AM_I (0x75): MPU9250=0x71, MPU6500=0x70
	status = reg_read(MPU9250_I2C_ADDR, MPU9250_WHO_AM_I, &who);
	if (status != HAL_OK || (who != 0x71 && who != 0x70)) {
		SEGGER_RTT_printf(0, "[IMU] WHO_AM_I fail: HAL=%d val=0x%02X (expected 0x71 or 0x70)\n",
				  status, who);
		return -1;
	}
	SEGGER_RTT_printf(0, "[IMU] WHO_AM_I OK (0x%02X, %s)\n", who,
			  who == 0x71 ? "MPU9250" : "MPU6500");

	// 2. Wake up: clear sleep bit
	reg_write(MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_1, 0x00);
	HAL_Delay(10);

	// 3. DLPF config: 92Hz bandwidth (gyro & accel)
	reg_write(MPU9250_I2C_ADDR, MPU9250_CONFIG, 0x02);

	// 4. Sample rate divider: 0 → internal sample rate = 1kHz
	reg_write(MPU9250_I2C_ADDR, MPU9250_SMPLRT_DIV, 0x00);

	// 5. Gyro ±2000°/s
	reg_write(MPU9250_I2C_ADDR, MPU9250_GYRO_CONFIG, 0x18);

	// 6. Accel ±16g
	reg_write(MPU9250_I2C_ADDR, MPU9250_ACCEL_CONFIG, 0x18);

	// 7. Enable I2C bypass to access AK8963
	reg_write(MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG, 0x02);
	HAL_Delay(10);

	// 8. Check AK8963 WHO_AM_I (0x00 → 0x48)
	//    Graceful degradation: if absent, still run 6-axis (accel+gyro only)
	status = reg_read(AK8963_I2C_ADDR, AK8963_WHO_AM_I, &who);
	if (status != HAL_OK || who != 0x48) {
		g_mag_available = 0;
		SEGGER_RTT_printf(0, "[IMU] AK8963 not found (HAL=%d val=0x%02X) — running 6-axis only\n",
				  status, who);
		SEGGER_RTT_printf(0, "[IMU] Init complete. Accel+Gyro only, no magnetometer.\n");
		return 0;
	}
	g_mag_available = 1;
	SEGGER_RTT_printf(0, "[IMU] AK8963 WHO_AM_I OK (0x%02X)\n", who);

	// 9. AK8963 continuous mode 2 (100Hz)
	reg_write(AK8963_I2C_ADDR, AK8963_CNTL1, 0x16);
	HAL_Delay(10);

	SEGGER_RTT_printf(0, "[IMU] Init complete. ±16g/±2000dps, AK8963 100Hz cont.\n");
	return 0;
}

int imu_read_6axis(ImuRaw6Axis *raw)
{
	uint8_t buf[14];
	HAL_StatusTypeDef status;

	status = burst_read(MPU9250_I2C_ADDR, MPU9250_ACCEL_XOUT_H, buf, 14);
	if (status != HAL_OK) return -1;

	// Parse big-endian: accel(6) + temp(2) + gyro(6)
	raw->accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
	raw->accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
	raw->accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
	raw->temp     = (int16_t)((buf[6] << 8) | buf[7]);
	raw->gyro[0]  = (int16_t)((buf[8] << 8) | buf[9]);
	raw->gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]);
	raw->gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]);

	return 0;
}

int imu_read_mag(ImuRawMag *raw)
{
	uint8_t buf[8];	// HXL..HZH + ST2
	uint8_t st1;
	HAL_StatusTypeDef status;

	// Read ST1 to check DRDY
	status = reg_read(AK8963_I2C_ADDR, 0x02, &st1);
	if (status != HAL_OK) return -1;
	if (!(st1 & 0x01)) return 1;	// data not ready

	// Burst read 8 bytes: HXL..HZH + ST2 (ST2 read clears DRDY)
	status = burst_read(AK8963_I2C_ADDR, AK8963_HXL, buf, 8);
	if (status != HAL_OK) return -1;

	// Magnetic sensor overflow check (ST2 bit 3)
	if (buf[7] & 0x08) return 2;

	// Parse little-endian (AK8963 is LE — unlike MPU6500 which is BE)
	raw->mag[0] = (int16_t)((buf[1] << 8) | buf[0]);	// X
	raw->mag[1] = (int16_t)((buf[3] << 8) | buf[2]);	// Y
	raw->mag[2] = (int16_t)((buf[5] << 8) | buf[4]);	// Z

	return 0;
}

void imu_6axis_to_si(const ImuRaw6Axis *raw, ImuData *out, int mag_valid,
		     const int16_t mag_raw[3])
{
	// Accel: LSB → m/s²  (RAW — no axis correction, chip-native output)
	out->accel[0] = (float)raw->accel[0] / ACCEL_SCALE_16G * GRAVITY_MSS;
	out->accel[1] = (float)raw->accel[1] / ACCEL_SCALE_16G * GRAVITY_MSS;
	out->accel[2] = (float)raw->accel[2] / ACCEL_SCALE_16G * GRAVITY_MSS;

	// Gyro: LSB → rad/s  (RAW — no axis correction)
	out->gyro[0] = (float)raw->gyro[0] / GYRO_SCALE_2000DPS * DEG2RAD;
	out->gyro[1] = (float)raw->gyro[1] / GYRO_SCALE_2000DPS * DEG2RAD;
	out->gyro[2] = (float)raw->gyro[2] / GYRO_SCALE_2000DPS * DEG2RAD;

	// Temperature: °C
	out->temp_c = (float)raw->temp / 333.87f + 21.0f;

	// Mag: LSB → µT (RAW — no axis correction, keep previous when no new data)
	if (mag_valid) {
		out->mag[0] = (float)mag_raw[0] * MAG_SCALE_16BIT;
		out->mag[1] = (float)mag_raw[1] * MAG_SCALE_16BIT;
		out->mag[2] = (float)mag_raw[2] * MAG_SCALE_16BIT;
	}
}

// --- Madgwick AHRS ---

void madgwick_init(Madgwick *m)
{
	m->q0 = 1.0f;
	m->q1 = 0.0f;
	m->q2 = 0.0f;
	m->q3 = 0.0f;
}

void madgwick_update(Madgwick *m,
		     float gx, float gy, float gz,
		     float ax, float ay, float az,
		     float mx, float my, float mz,
		     float dt)
{
	float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;
	float recip_norm;
	float s0, s1, s2, s3;
	float q_dot1, q_dot2, q_dot3, q_dot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
	float _2q0, _2q1, _2q2, _2q3;
	float _2q0q2, _2q2q3;
	float q0q0, q0q1, q0q2, q0q3;
	float q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Gyroscope: rate of change of quaternion
	q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	q_dot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
	q_dot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
	q_dot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

	// Accelerometer + magnetometer feedback (skip if no accel to avoid NaN)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer
		recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
		ax *= recip_norm;
		ay *= recip_norm;
		az *= recip_norm;

		// Normalise magnetometer (skip if zero)
		if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
			recip_norm = inv_sqrt(mx * mx + my * my + mz * mz);
			mx *= recip_norm;
			my *= recip_norm;
			mz *= recip_norm;
		}

		// Pre-compute repeated terms
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0   = 2.0f * q0;
		_2q1   = 2.0f * q1;
		_2q2   = 2.0f * q2;
		_2q3   = 2.0f * q3;
		_2q0q2 = _2q0 * q2;
		_2q2q3 = _2q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1
		     + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2
		     - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3
		       - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient descent corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
		     + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
		     - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3)
			+ _2bz * (q1q3 - q0q2) - mx)
		     + (-_2bx * q3 + _2bz * q1)
			* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
		     + _2bx * q2 * (_2bx * (q0q2 + q1q3)
			+ _2bz * (0.5f - q1q1 - q2q2) - mz);

		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
		     + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
		     - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
		     + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3)
			+ _2bz * (q1q3 - q0q2) - mx)
		     + (_2bx * q2 + _2bz * q0)
			* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
		     + (_2bx * q3 - _4bz * q1)
			* (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
		     + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
		     - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
		     + (-_4bx * q2 - _2bz * q0)
			* (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
		     + (_2bx * q1 + _2bz * q3)
			* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
		     + (_2bx * q0 - _4bz * q2)
			* (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
		     + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
		     + (-_4bx * q3 + _2bz * q1)
			* (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
		     + (-_2bx * q0 + _2bz * q2)
			* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
		     + _2bx * q1 * (_2bx * (q0q2 + q1q3)
			+ _2bz * (0.5f - q1q1 - q2q2) - mz);

		// Normalise gradient step
		recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		s0 *= recip_norm;
		s1 *= recip_norm;
		s2 *= recip_norm;
		s3 *= recip_norm;

		// Apply feedback
		q_dot1 -= MADGWICK_BETA * s0;
		q_dot2 -= MADGWICK_BETA * s1;
		q_dot3 -= MADGWICK_BETA * s2;
		q_dot4 -= MADGWICK_BETA * s3;
	}

	// Integrate quaternion
	q0 += q_dot1 * dt;
	q1 += q_dot2 * dt;
	q2 += q_dot3 * dt;
	q3 += q_dot4 * dt;

	// Normalise quaternion
	recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	m->q0 = q0 * recip_norm;
	m->q1 = q1 * recip_norm;
	m->q2 = q2 * recip_norm;
	m->q3 = q3 * recip_norm;
}

void madgwick_get_attitude(const Madgwick *m, Attitude *a)
{
	float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;
	a->roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
			  1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.2957795f;
	a->pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.2957795f;
	a->yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
			  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.2957795f;
}
