#include "rtt_pid_debug.h"
#include "SEGGER_RTT.h"
#include <stdio.h>    // sscanf
#include <string.h>   // strncmp
#include <stdlib.h>   // atoi, atof

// RTT channel assignment
#define RTT_CH_TERMINAL   0   // J-LINK Viewer text commands + replies
#define RTT_CH_SCOPE      1   // J-Link Scope waveform data

// Scope output period (ms)
#define SCOPE_PERIOD_MS   10

// Gear ratio: motor shaft / wheel
#define GEAR_RATIO_F  30.0f

// Convert motor-shaft RPM (internal) to wheel RPM (display)
#define TO_WHEEL_RPM(v) ((int16_t)((float)(v) / GEAR_RATIO_F))

// Down-buffer command line buffer
#define CMD_BUF_SIZE      64

// J-Scope RTT data format: 8 channels of int16_t
// Channel order: M1_RPM M1_TGT M2_RPM M2_TGT M3_RPM M3_TGT M4_RPM M4_TGT
#define JSCOPE_CH_NAME    "JScope_I2I2I2I2I2I2I2I2"

// 8 * int16_t = 16 bytes per sample
#pragma pack(push, 1)
typedef struct {
    int16_t m1_rpm, m1_tgt;
    int16_t m2_rpm, m2_tgt;
    int16_t m3_rpm, m3_tgt;
    int16_t m4_rpm, m4_tgt;
} ScopeData;
#pragma pack(pop)

// Static buffer for J-Scope up-channel
static uint8_t g_scope_buffer[512];

void rtt_pid_debug_init(void)
{
    // Configure up-buffer 1 for J-Link Scope.
    // The channel name "JScope_I2..." tells J-Scope the binary format.
    SEGGER_RTT_ConfigUpBuffer(RTT_CH_SCOPE, JSCOPE_CH_NAME,
                              (char*)g_scope_buffer, sizeof(g_scope_buffer),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    SEGGER_RTT_printf(RTT_CH_TERMINAL,
        "\n"
        "========================================\n"
        "  STM32 Motor PID Debug Console\n"
        "  J-LINK RTT ready.\n"
        "========================================\n"
        "Commands:\n"
        "  m<N> <rpm>     e.g. m1 100  (set M1 target)\n"
        "  all <rpm>      e.g. all 50  (set all motors)\n"
        "  kp <N> <val>   e.g. kp 1 0.5\n"
        "  ki <N> <val>   e.g. ki 1 0.01\n"
        "  kd <N> <val>   e.g. kd 1 0.02\n"
        "  stop            stop all motors\n"
        "  status          print status\n"
        "========================================\n\n");
}

// --- Command parser ---

static void cmd_status(void)
{
    SEGGER_RTT_printf(RTT_CH_TERMINAL,
        "Motor  Target(RPM)  Actual(RPM)  Speed(mm/s)  PWM  Kp    Ki    Kd  (target/actual: wheel RPM)\n");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Motor *m = motor_get((MotorID)i);
        SEGGER_RTT_printf(RTT_CH_TERMINAL,
            "  M%d    %6d       %6d       %7.1f      %4ld   %.2f  %.3f  %.3f\n",
            i + 1,
            TO_WHEEL_RPM(m->target_rpm),
            TO_WHEEL_RPM(m->actual_rpm),
            m->encoder.wheel_speed,
            m->pwm_output,
            m->pid.Kp, m->pid.Ki, m->pid.Kd);
    }
}

static void cmd_motor(int motor_idx, int16_t rpm)
{
    motor_control_set_target((MotorID)motor_idx, rpm);
    SEGGER_RTT_printf(RTT_CH_TERMINAL, "M%d target -> %d RPM\n",
                      motor_idx + 1, rpm);
}

static void cmd_all(int16_t rpm)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_control_set_target((MotorID)i, rpm);
    }
    SEGGER_RTT_printf(RTT_CH_TERMINAL, "All motors target -> %d RPM\n", rpm);
}

static void cmd_stop(void)
{
    motor_control_stop_all();
    SEGGER_RTT_printf(RTT_CH_TERMINAL, "All motors stopped.\n");
}

static void cmd_pid_param(int motor_idx, char param_type, float val)
{
    Motor *m = motor_get((MotorID)motor_idx);
    if (!m) return;

    switch (param_type) {
    case 'p':
        pid_set_params(&m->pid, val, m->pid.Ki, m->pid.Kd);
        SEGGER_RTT_printf(RTT_CH_TERMINAL, "M%d Kp = %.3f\n", motor_idx + 1, val);
        break;
    case 'i':
        pid_set_params(&m->pid, m->pid.Kp, val, m->pid.Kd);
        SEGGER_RTT_printf(RTT_CH_TERMINAL, "M%d Ki = %.4f\n", motor_idx + 1, val);
        break;
    case 'd':
        pid_set_params(&m->pid, m->pid.Kp, m->pid.Ki, val);
        SEGGER_RTT_printf(RTT_CH_TERMINAL, "M%d Kd = %.3f\n", motor_idx + 1, val);
        break;
    }
}

static int parse_motor_idx(const char *s)
{
    int idx = atoi(s) - 1;  // user types 1-4
    if (idx < 0 || idx >= MOTOR_COUNT) return -1;
    return idx;
}

static void process_command(const char *line)
{
    // Skip leading whitespace
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    char cmd[16], arg1[16], arg2[16];
    int n = 0;
    sscanf(line, "%15s %15s %15s %n", cmd, arg1, arg2, &n);
    (void)n;

    if (strncmp(cmd, "status", 6) == 0) {
        cmd_status();
    } else if (strncmp(cmd, "stop", 4) == 0) {
        cmd_stop();
    } else if (strncmp(cmd, "all", 3) == 0) {
        cmd_all((int16_t)atoi(arg1));
    } else if (cmd[0] == 'm' && cmd[1] >= '1' && cmd[1] <= '4' && cmd[2] == '\0') {
        int idx = parse_motor_idx(&cmd[1]);
        if (idx >= 0) cmd_motor(idx, (int16_t)atoi(arg1));
    } else if (strncmp(cmd, "kp", 2) == 0) {
        int idx = parse_motor_idx(arg1);
        if (idx >= 0) cmd_pid_param(idx, 'p', (float)atof(arg2));
    } else if (strncmp(cmd, "ki", 2) == 0) {
        int idx = parse_motor_idx(arg1);
        if (idx >= 0) cmd_pid_param(idx, 'i', (float)atof(arg2));
    } else if (strncmp(cmd, "kd", 2) == 0) {
        int idx = parse_motor_idx(arg1);
        if (idx >= 0) cmd_pid_param(idx, 'd', (float)atof(arg2));
    } else {
        SEGGER_RTT_printf(RTT_CH_TERMINAL, "Unknown command: %s\n", cmd);
    }
}

void rtt_pid_debug_poll(void)
{
    static char buf[CMD_BUF_SIZE];
    static int  pos = 0;

    while (SEGGER_RTT_HasKey()) {
        int c = SEGGER_RTT_GetKey();
        if (c < 0) break;

        if (c == '\r' || c == '\n') {
            if (pos > 0) {
                buf[pos] = '\0';
                process_command(buf);
                pos = 0;
            }
        } else if (c == '\b' || c == 0x7f) {
            if (pos > 0) pos--;
        } else if (pos < CMD_BUF_SIZE - 1) {
            buf[pos++] = (char)c;
            SEGGER_RTT_Write(RTT_CH_TERMINAL, &c, 1);
        }
    }
}

void rtt_scope_output(void)
{
    static uint32_t last_ms = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_ms < SCOPE_PERIOD_MS) return;
    last_ms = now;

    // Pack motor data into J-Scope binary format
    ScopeData d;
    Motor *m0 = motor_get(MOTOR_M1_LR);
    Motor *m1 = motor_get(MOTOR_M2_LF);
    Motor *m2 = motor_get(MOTOR_M3_RF);
    Motor *m3 = motor_get(MOTOR_M4_RR);

    d.m1_rpm = TO_WHEEL_RPM(m0->actual_rpm); d.m1_tgt = TO_WHEEL_RPM(m0->target_rpm);
    d.m2_rpm = TO_WHEEL_RPM(m1->actual_rpm); d.m2_tgt = TO_WHEEL_RPM(m1->target_rpm);
    d.m3_rpm = TO_WHEEL_RPM(m2->actual_rpm); d.m3_tgt = TO_WHEEL_RPM(m2->target_rpm);
    d.m4_rpm = TO_WHEEL_RPM(m3->actual_rpm); d.m4_tgt = TO_WHEEL_RPM(m3->target_rpm);

    SEGGER_RTT_Write(RTT_CH_SCOPE, &d, sizeof(d));
}
