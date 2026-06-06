// can_decoder.c — read socketCAN, decode STM32 telemetry, print to terminal
// Build: gcc -o can_decoder can_decoder.c
// Usage: sudo ./can_decoder can0

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

// IMU sensitivity constants (match imu.h)
#define ACCEL_SCALE  8192.0f     // LSB/g at ±4g
#define GYRO_SCALE   65.536f     // LSB/(°/s) at ±500°/s
#define MAG_SCALE    0.15f       // µT/LSB
#define GRAVITY      9.80665f    // m/s² per g
#define TEMP_OFFSET  21.0f       // °C
#define TEMP_SLOPE   333.87f     // ADC/°C

static volatile int running = 1;

static void sigint(int sig) { running = 0; }

static int16_t get_i16(const uint8_t *buf)
{
	return (int16_t)((buf[1] << 8) | buf[0]);
}

static void print_frame(const struct can_frame *f, uint32_t *cnt)
{
	(*cnt)++;
	int16_t v0, v1, v2, v3;

	// Throttle to ~5Hz per ID — print if 200ms since last print of same ID
	static uint32_t last_ms[6];
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	uint32_t now = (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);

	int slot = f->can_id - 0x201; // 0..4
	if (slot >= 0 && slot <= 4 && now - last_ms[slot] < 200) {
		return; // same ID printed < 200ms ago → skip
	}
	if (slot >= 0 && slot <= 4) last_ms[slot] = now;

	switch (f->can_id) {
	case 0x201:
		v0 = get_i16(&f->data[0]); // M1 speed (mm/s)
		v1 = get_i16(&f->data[2]); // M1 PWM
		v2 = get_i16(&f->data[4]); // M2 speed
		v3 = get_i16(&f->data[6]); // M2 PWM
		printf("[%6u] 201 M1:%5d mm/s PWM:%4ld  M2:%5d mm/s PWM:%4ld\n",
		       *cnt, v0, (long)v1, v2, (long)v3);
		break;
	case 0x202:
		v0 = get_i16(&f->data[0]); // M3 speed
		v1 = get_i16(&f->data[2]); // M3 PWM
		v2 = get_i16(&f->data[4]); // M4 speed
		v3 = get_i16(&f->data[6]); // M4 PWM
		printf("[%6u] 202 M3:%5d mm/s PWM:%4ld  M4:%5d mm/s PWM:%4ld\n",
		       *cnt, v0, (long)v1, v2, (long)v3);
		break;
	case 0x203: { // Accel: raw ADC → m/s²
		v0 = get_i16(&f->data[0]);
		v1 = get_i16(&f->data[2]);
		v2 = get_i16(&f->data[4]);
		float ax = v0 / ACCEL_SCALE * GRAVITY;
		float ay = v1 / ACCEL_SCALE * GRAVITY;
		float az = v2 / ACCEL_SCALE * GRAVITY;
		printf("[%6u] 203 ACCEL  raw:%6d %6d %6d  (%.2f %.2f %.2f m/s²)\n",
		       *cnt, v0, v1, v2, ax, ay, az);
		} break;
	case 0x204: { // Gyro: raw ADC → °/s
		v0 = get_i16(&f->data[0]);
		v1 = get_i16(&f->data[2]);
		v2 = get_i16(&f->data[4]);
		float gx = v0 / GYRO_SCALE;
		float gy = v1 / GYRO_SCALE;
		float gz = v2 / GYRO_SCALE;
		printf("[%6u] 204 GYRO   raw:%6d %6d %6d  (%.2f %.2f %.2f °/s)\n",
		       *cnt, v0, v1, v2, gx, gy, gz);
		} break;
	case 0x205: { // Mag: raw ADC → µT; Temp: raw → °C
		v0 = get_i16(&f->data[0]);
		v1 = get_i16(&f->data[2]);
		v2 = get_i16(&f->data[4]);
		v3 = get_i16(&f->data[6]);
		float mx = v0 * MAG_SCALE, my = v1 * MAG_SCALE, mz = v2 * MAG_SCALE;
		float tc = v3 / TEMP_SLOPE + TEMP_OFFSET;
		printf("[%6u] 205 MAG    raw:%6d %6d %6d  (%.1f %.1f %.1f uT)  Temp raw:%d (%.1f C)\n",
		       *cnt, v0, v1, v2, mx, my, mz, v3, tc);
		} break;
		break;
	default:
		printf("[%6u] %03X  [%d]", *cnt, f->can_id, f->can_dlc);
		for (int i = 0; i < f->can_dlc; i++)
			printf(" %02X", f->data[i]);
		printf("\n");
		break;
	}
}

int main(int argc, char *argv[])
{
	if (argc < 2) {
		fprintf(stderr, "Usage: %s <can-iface>  (e.g. can0)\n", argv[0]);
		return 1;
	}

	int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s < 0) { perror("socket"); return 1; }

	struct ifreq ifr;
	strncpy(ifr.ifr_name, argv[1], IFNAMSIZ - 1);
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); return 1; }

	struct sockaddr_can addr = { .can_family = AF_CAN, .can_ifindex = ifr.ifr_ifindex };
	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }

	signal(SIGINT, sigint);

	printf("can_decoder: listening on %s (Ctrl+C to stop)\n", argv[1]);
	printf("  ID  | Type   | Decoded values\n");
	printf("------+--------+----------------------------\n");

	uint32_t cnt = 0;
	while (running) {
		struct can_frame frame;
		int n = read(s, &frame, sizeof(frame));
		if (n < 0) break;
		if ((size_t)n != sizeof(frame)) continue;
		print_frame(&frame, &cnt);
	}

	close(s);
	printf("\ncan_decoder: stopped.  Received %u frames.\n", cnt);
	return 0;
}
