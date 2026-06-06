// can_send_test.c — send CAN test frames to STM32 for RX validation
// Build: gcc -o can_send_test can_send_test.c -lm
// Usage:
//   sudo ./can_send_test can0 motor    # 0x101 @ 50Hz, triangle sweep ±300 mm/s
//   sudo ./can_send_test can0 motor 10 # 0x101 @ 10Hz
//   sudo ./can_send_test can0 estop    # 0x102 toggle test (every 2s)
//   sudo ./can_send_test can0 pid      # 0x103 PID config test

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

static volatile int running = 1;

static void sigint(int sig) { running = 0; }

// pack int16 → 2 bytes little-endian
static void put_i16(uint8_t *buf, int16_t v)
{
	buf[0] = (uint8_t)v;
	buf[1] = (uint8_t)(v >> 8);
}

// send one CAN frame, return 0 on success
static int can_send(int sock, uint32_t id, const uint8_t *data, uint8_t dlc)
{
	struct can_frame frame;
	memset(&frame, 0, sizeof(frame));
	frame.can_id = id;
	frame.can_dlc = dlc;
	memcpy(frame.data, data, dlc);
	return write(sock, &frame, sizeof(frame)) == sizeof(frame) ? 0 : -1;
}

static int open_can(const char *ifname)
{
	int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s < 0) { perror("socket"); return -1; }

	struct ifreq ifr;
	strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); close(s); return -1; }

	struct sockaddr_can addr = { .can_family = AF_CAN, .can_ifindex = ifr.ifr_ifindex };
	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); close(s); return -1; }

	return s;
}

// ─── 0x101 motor command ──────────────────────────────────────────
// Triangle sweep: -MAX → +MAX → -MAX, period ~4s
static void test_motor(int sock, int freq_hz)
{
	int interval_us = 1000000 / freq_hz;
	int16_t max_speed = 300;  // mm/s

	printf("Motor test: 0x101 @ %dHz, sweep ±%d mm/s\n", freq_hz, max_speed);
	printf("  ID  | M1(LB)  M2(LF)  M3(RF)  M4(RB)\n");
	printf("------+--------------------------------\n");

	uint32_t seq = 0;
	uint32_t last_print = 0;
	struct timespec ts;

	while (running) {
		// triangle wave: 0→MAX→0→-MAX→0, period = max_speed*4 steps
		int32_t phase = (seq * 4) % (max_speed * 4);
		int16_t speed;
		if (phase < max_speed * 2)
			speed = (int16_t)(phase - max_speed);           // -MAX → +MAX
		else
			speed = (int16_t)(3 * max_speed - phase);       // +MAX → -MAX

		// same speed for all 4 wheels
		uint8_t buf[8];
		put_i16(&buf[0], speed);  // M1(LB)
		put_i16(&buf[2], speed);  // M2(LF)
		put_i16(&buf[4], speed);  // M3(RF)
		put_i16(&buf[6], speed);  // M4(RB)

		int rc = can_send(sock, 0x101, buf, 8);

		clock_gettime(CLOCK_MONOTONIC, &ts);
		uint32_t now_ms = (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);

		if (now_ms - last_print >= 500) {
			printf(" 0x101 | % 5d   % 5d   % 5d   % 5d   %s\n",
			       speed, speed, speed, speed,
			       rc == 0 ? "OK" : "FAIL");
			last_print = now_ms;
		}

		seq++;
		usleep(interval_us);
	}
}

// ─── 0x102 emergency stop ─────────────────────────────────────────
#define ESTOP_ENGAGE  0x01
#define ESTOP_RELEASE 0x00

static void test_estop(int sock)
{
	printf("Estop test: 0x102 toggle every 2s\n");
	printf("  ID  | Data[0]  Meaning\n");
	printf("------+------------------\n");

	int toggle = 0;
	while (running) {
		uint8_t buf[8] = {0};
		buf[0] = toggle ? ESTOP_ENGAGE : ESTOP_RELEASE;
		can_send(sock, 0x102, buf, 8);
		printf(" 0x102 | 0x%02X     %s\n", buf[0],
		       toggle ? "ENGAGE (stop motors)" : "RELEASE (resume)");
		toggle = !toggle;
		sleep(2);
	}
}

// ─── 0x103 PID config ─────────────────────────────────────────────
static void test_pid(int sock)
{
	printf("PID config test: 0x103 — one-shot Kp/Ki/Kd/Kf for each motor\n");
	printf("  ID  | Motor  Param  Value\n");
	printf("------+---------------------\n");

	const char *param_names[] = {"Kp", "Ki", "Kd", "Kf"};
	float values[] = {5.0f, 6.0f, 0.02f, 1.3f};

	for (int motor = 0; motor < 4 && running; motor++) {
		for (int param = 0; param < 4 && running; param++) {
			uint8_t buf[8] = {0};
			buf[0] = (uint8_t)motor;       // motor ID
			buf[1] = (uint8_t)param;       // 0=Kp, 1=Ki, 2=Kd, 3=Kf

			// float → 4 bytes little-endian
			uint32_t raw;
			memcpy(&raw, &values[param], 4);
			buf[2] = raw & 0xFF;
			buf[3] = (raw >> 8) & 0xFF;
			buf[4] = (raw >> 16) & 0xFF;
			buf[5] = (raw >> 24) & 0xFF;

			int rc = can_send(sock, 0x103, buf, 8);
			printf(" 0x103 |   M%d   %s    %.3f  %s\n",
			       motor + 1, param_names[param],
			       (double)values[param],
			       rc == 0 ? "OK" : "FAIL");
			usleep(200000);  // 200ms between frames
		}
	}
	printf("PID test done.\n");
}

// ─── stress test: send as fast as possible, no delay ────────────────
static void test_stress(int sock)
{
	printf("Stress test: 0x101 max rate, 5s burst, Ctrl+C to stop early\n");
	printf("  Elapsed  | Frames  | Rate (fps) | Drops\n");
	printf("-----------+---------+------------+-------\n");

	uint8_t buf[8];
	memset(buf, 0, 8);  // speed=0 (motors idle, just stress CAN RX path)

	uint64_t sent = 0, drops = 0;
	struct timespec t0, t_now;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	uint32_t last_print_s = 0;

	while (running) {
		if (can_send(sock, 0x101, buf, 8) != 0)
			drops++;
		sent++;

		clock_gettime(CLOCK_MONOTONIC, &t_now);
		uint32_t elapsed_s = (uint32_t)(t_now.tv_sec - t0.tv_sec);

		if (elapsed_s != last_print_s) {
			double rate = (elapsed_s > 0) ? (double)sent / (double)elapsed_s : 0;
			printf("  %4us     | %6lu | %8.0f   | %lu\n",
			       elapsed_s, (unsigned long)sent, rate, (unsigned long)drops);
			last_print_s = elapsed_s;
		}

		if (elapsed_s >= 5) break;
	}

	clock_gettime(CLOCK_MONOTONIC, &t_now);
	double total_s = (double)(t_now.tv_sec - t0.tv_sec) +
	                 (double)(t_now.tv_nsec - t0.tv_nsec) / 1e9;
	double avg_rate = (total_s > 0) ? (double)sent / total_s : 0;

	printf("\nDone: %lu frames in %.2fs — avg %.0f fps, %lu drops\n",
	       (unsigned long)sent, total_s, avg_rate, (unsigned long)drops);
	printf("Theoretical max @ 500kbps: ~1900 fps (8-byte std frame, ~520us/frame)\n");
}

// ─── main ─────────────────────────────────────────────────────────

int main(int argc, char *argv[])
{
	if (argc < 3) {
		fprintf(stderr,
			"Usage: %s <can-iface> <mode> [freq_hz]\n"
			"  mode: motor | estop | pid | stress\n"
			"  freq_hz: motor send rate (default 50)\n"
			"\n"
			"Examples:\n"
			"  sudo %s can0 motor      # 0x101 @ 50Hz, sweep +/-300mm/s\n"
			"  sudo %s can0 motor 10   # 0x101 @ 10Hz\n"
			"  sudo %s can0 stress     # 0x101 max rate, 5s burst\n"
			"  sudo %s can0 estop      # 0x102 engage/release every 2s\n"
			"  sudo %s can0 pid        # 0x103 PID config (one-shot)\n",
			argv[0], argv[0], argv[0], argv[0], argv[0], argv[0]);
		return 1;
	}

	const char *ifname = argv[1];
	const char *mode   = argv[2];
	int freq = (argc >= 4) ? atoi(argv[3]) : 50;

	if (freq < 1 || freq > 1000) {
		fprintf(stderr, "freq must be 1-1000 Hz\n");
		return 1;
	}

	int sock = open_can(ifname);
	if (sock < 0) return 1;

	signal(SIGINT, sigint);
	printf("can_send_test: sending on %s (Ctrl+C to stop)\n\n", ifname);

	if (!strcmp(mode, "motor")) {
		test_motor(sock, freq);
	} else if (!strcmp(mode, "stress")) {
		test_stress(sock);
	} else if (!strcmp(mode, "estop")) {
		test_estop(sock);
	} else if (!strcmp(mode, "pid")) {
		test_pid(sock);
	} else {
		fprintf(stderr, "Unknown mode: %s\n", mode);
		close(sock);
		return 1;
	}

	close(sock);
	printf("\ncan_send_test: stopped.\n");
	return 0;
}
