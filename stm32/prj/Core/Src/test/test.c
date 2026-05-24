#include "test.h"
#include "tim.h"
#include "encoder.h"
#include "SEGGER_RTT.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ------------------------------------------------------------------ */
/* Step 1: encoder test with RPM + wheel-speed                         */
/*   - 1kHz encoder_update() via SysTick flag                          */
/*   - RTT Viewer (ch0): CNT / wheel-RPM / wheel-speed every 500ms    */
/*   - J-Scope  (ch1):   wheel-speed (mm/s) @100Hz, 4×f4              */
/*   - wheel-speed calc is encapsulated in encoder.h/c                 */
/* ------------------------------------------------------------------ */

extern volatile uint8_t sys_tick_flag;

static uint8_t g_scope_buf[512];
#pragma pack(push, 1)
typedef struct { float v1, v2, v3, v4; } ScopeData;  // mm/s
#pragma pack(pop)

static Encoder g_enc[4];

void encoder_test(void)
{
    static uint8_t inited = 0;

    if (!inited) {
        // --- Direction: all forward ---
        HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M4_IN1_GPIO_Port, M4_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M4_IN2_GPIO_Port, M4_IN2_Pin, GPIO_PIN_RESET);

        // --- PWM 30% ---
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
        uint32_t duty = (uint32_t)(16799UL * 30 / 100);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty);

        // --- Encoder objects ---
        encoder_init(&g_enc[0], &htim2);
        encoder_init(&g_enc[1], &htim3);
        encoder_init(&g_enc[2], &htim4);
        encoder_init(&g_enc[3], &htim8);

        // --- J-Scope RTT up-buffer (ch1): 4×float32 = wheel-speed mm/s ---
        SEGGER_RTT_ConfigUpBuffer(1, "JScope_f4f4f4f4",
                                  (char*)g_scope_buf, sizeof(g_scope_buf),
                                  SEGGER_RTT_MODE_NO_BLOCK_SKIP);

        SEGGER_RTT_printf(0,
            "\n=== Encoder Test ===\n"
            "Dir: all forward, PWM 30%% (CCR=%lu)\n"
            "Ch0: text every 500ms | Ch1: J-Scope 4xf4 wheel-speed mm/s @100Hz\n\n",
            duty);
        inited = 1;
    }

    // ---- 1 kHz encoder update (SysTick) ----
    if (sys_tick_flag) {
        sys_tick_flag = 0;
        for (int i = 0; i < 4; i++) encoder_update(&g_enc[i]);
    }

    uint32_t now = HAL_GetTick();

    // ---- RTT Viewer text (ch0): every 500ms ----
    static uint32_t t_print = 0;
    if (now - t_print >= 500) {
        t_print = now;
        for (int i = 0; i < 4; i++) {
            SEGGER_RTT_printf(0,
                "M%d: CNT=%-6ld  wheel-RPM=%-5d  V=%-6d mm/s\r\n",
                i + 1,
                (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(g_enc[i].htim),
                (int)(g_enc[i].rpm / (float)GEAR_RATIO),
                (int)g_enc[i].wheel_speed);
        }
    }

    // ---- J-Scope binary (ch1): every 10ms ----
    static uint32_t t_scope = 0;
    if (now - t_scope >= 10) {
        t_scope = now;
        ScopeData d;
        d.v1 = g_enc[0].wheel_speed;
        d.v2 = g_enc[1].wheel_speed;
        d.v3 = g_enc[2].wheel_speed;
        d.v4 = g_enc[3].wheel_speed;
        SEGGER_RTT_Write(1, &d, sizeof(d));
    }
}

/* ------------------------------------------------------------------ */
/* old motor open-loop test (kept for reference)                       */
/* ------------------------------------------------------------------ */
void led_test(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(500);
}

/* ------------------------------------------------------------------ */
/* per-motor pin / PWM / encoder descriptor                           */
/* ------------------------------------------------------------------ */
typedef struct {
    GPIO_TypeDef *in1_port; uint16_t in1_pin;
    GPIO_TypeDef *in2_port; uint16_t in2_pin;
    uint32_t      pwm_ch;          // TIM_CHANNEL_x
    TIM_HandleTypeDef *htim_enc;
    const char   *name;
} motor_t;

static const motor_t motors[4] = {
    { M1_IN1_GPIO_Port, M1_IN1_Pin, M1_IN2_GPIO_Port, M1_IN2_Pin,
      TIM_CHANNEL_1, &htim2, "M1_LR" },
    { M2_IN1_GPIO_Port, M2_IN1_Pin, M2_IN2_GPIO_Port, M2_IN2_Pin,
      TIM_CHANNEL_2, &htim3, "M2_LF" },
    { M3_IN1_GPIO_Port, M3_IN1_Pin, M3_IN2_GPIO_Port, M3_IN2_Pin,
      TIM_CHANNEL_3, &htim4, "M3_RF" },
    { M4_IN1_GPIO_Port, M4_IN1_Pin, M4_IN2_GPIO_Port, M4_IN2_Pin,
      TIM_CHANNEL_4, &htim8, "M4_RR" },
};

#define PWM_DUTY_50   8399
#define DURATION_RUN  1000
#define DURATION_STOP 2000

typedef enum { MOTOR_FWD, MOTOR_STOP1, MOTOR_REV, MOTOR_STOP2 } motor_state_t;

static void motor_set_dir(const motor_t *m, uint8_t forward)
{
    if (forward) {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
    }
}

static void motor_brake(const motor_t *m)
{
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
}

static void motor_pwm(const motor_t *m, uint32_t pulse)
{
    __HAL_TIM_SET_COMPARE(&htim1, m->pwm_ch, pulse);
}

/* ------------------------------------------------------------------ */
/* Step 2: open-loop test — RTT commands to set PWM duty directly     */
/*   Commands (paste into RTT Viewer):                                 */
/*     m1 30   → M1 forward 30% duty                                   */
/*     m2 -25  → M2 reverse 25% duty                                   */
/*     all 40  → all motors forward 40%                                */
/*     stop    → all motors brake                                      */
/*     status  → print current state                                   */
/* ------------------------------------------------------------------ */

#define PWM_ARR         16799
#define PWM_MAX_DUTY    60      // percent, safety limit for TB6612
#define PWM_MIN_DUTY    15      // percent, below this motor won't move

#define CMD_BUF_SIZE    32

void open_loop_test(void)
{
    static uint8_t  inited = 0;
    static int8_t   duty_pct[4] = {0};  // per-motor duty, negative = reverse
    static Encoder  enc[4];

    if (!inited) {
        // Start PWM
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

        // Init encoders (encoder_init now starts encoder timer internally)
        encoder_init(&enc[0], &htim2);
        encoder_init(&enc[1], &htim3);
        encoder_init(&enc[2], &htim4);
        encoder_init(&enc[3], &htim8);

        // J-Scope up-buffer (ch1): 4×float32 wheel-speed mm/s
        SEGGER_RTT_ConfigUpBuffer(1, "JScope_f4f4f4f4",
                                  (char*)g_scope_buf, sizeof(g_scope_buf),
                                  SEGGER_RTT_MODE_NO_BLOCK_SKIP);

        SEGGER_RTT_printf(0,
            "\n=== Open-Loop Test ===\n"
            "PWM range: %d%% – %d%%   ARR=%d\n"
            "Commands: m<N> <duty> | all <duty> | stop | status\n"
            "  duty: -60..-15 or 15..60 (negative = reverse)\n\n",
            PWM_MIN_DUTY, PWM_MAX_DUTY, PWM_ARR);

        inited = 1;
    }

    // ---- 1kHz encoder update ----
    if (sys_tick_flag) {
        sys_tick_flag = 0;
        for (int i = 0; i < 4; i++) encoder_update(&enc[i]);
    }

    // ---- RTT command polling ----
    {
        static char cmd[CMD_BUF_SIZE];
        static int  pos = 0;

        while (SEGGER_RTT_HasKey()) {
            int c = SEGGER_RTT_GetKey();
            if (c < 0) break;

            if (c == '\r' || c == '\n') {
                if (pos > 0) {
                    cmd[pos] = '\0';
                    pos = 0;

                    // Parse: m<N> <duty> | all <duty> | stop | status
                    char tok[8]; int val;
                    int n = sscanf(cmd, "%7s %d", tok, &val);

                    if (n >= 1 && strcmp(tok, "stop") == 0) {
                        for (int i = 0; i < 4; i++) {
                            duty_pct[i] = 0;
                            motor_brake(&motors[i]);
                            motor_pwm(&motors[i], 0);
                        }
                        SEGGER_RTT_printf(0, "All stopped.\n");
                    } else if (n >= 1 && strcmp(tok, "status") == 0) {
                        SEGGER_RTT_printf(0,
                            "Motor  Duty%%  Dir     CCR   CNT     wheel-RPM  V(mm/s)\n");
                        for (int i = 0; i < 4; i++) {
                            const char *dir = duty_pct[i] > 0 ? "FWD" :
                                              duty_pct[i] < 0 ? "REV" : "STOP";
                            int d = duty_pct[i];
                            uint32_t ccr = d ? ((uint32_t)PWM_ARR * abs(d) / 100) : 0;
                            SEGGER_RTT_printf(0,
                                "  %s   %3d   %-4s  %5lu  %-6d  %-5d       %-6d\n",
                                motors[i].name, d, dir, ccr,
                                (int16_t)__HAL_TIM_GET_COUNTER(enc[i].htim),
                                (int)(enc[i].rpm / (float)GEAR_RATIO),
                                (int)enc[i].wheel_speed);
                        }
                    } else if (n >= 2 && strcmp(tok, "all") == 0) {
                        // Clamp duty
                        if (val > PWM_MAX_DUTY) val = PWM_MAX_DUTY;
                        if (val < -PWM_MAX_DUTY) val = -PWM_MAX_DUTY;
                        if (val > 0 && val < PWM_MIN_DUTY) val = PWM_MIN_DUTY;
                        if (val < 0 && val > -PWM_MIN_DUTY) val = -PWM_MIN_DUTY;
                        for (int i = 0; i < 4; i++) {
                            duty_pct[i] = (int8_t)val;
                            motor_set_dir(&motors[i], val > 0);
                            motor_pwm(&motors[i],
                                      (uint32_t)PWM_ARR * (val > 0 ? val : -val) / 100);
                        }
                        SEGGER_RTT_printf(0, "All motors → %d%%\n", val);
                    } else if (n >= 2 && tok[0] == 'm' && tok[1] >= '1' && tok[1] <= '4' && tok[2] == '\0') {
                        int idx = tok[1] - '0' - 1;
                        // Clamp duty
                        if (val > PWM_MAX_DUTY) val = PWM_MAX_DUTY;
                        if (val < -PWM_MAX_DUTY) val = -PWM_MAX_DUTY;
                        if (val > 0 && val < PWM_MIN_DUTY) val = PWM_MIN_DUTY;
                        if (val < 0 && val > -PWM_MIN_DUTY) val = -PWM_MIN_DUTY;
                        duty_pct[idx] = (int8_t)val;
                        if (val == 0) {
                            motor_brake(&motors[idx]);
                            motor_pwm(&motors[idx], 0);
                        } else {
                            motor_set_dir(&motors[idx], val > 0);
                            motor_pwm(&motors[idx],
                                      (uint32_t)PWM_ARR * (val > 0 ? val : -val) / 100);
                        }
                        SEGGER_RTT_printf(0, "%s → %d%%\n", motors[idx].name, val);
                    }
                }
            } else if (c == '\b' || c == 0x7f) {
                if (pos > 0) pos--;
            } else if (pos < CMD_BUF_SIZE - 1) {
                cmd[pos++] = (char)c;
                SEGGER_RTT_Write(0, &c, 1);  // echo
            }
        }
    }

    uint32_t now = HAL_GetTick();

    // ---- Telemetry every 500ms ----
    static uint32_t t_print = 0;
    if (now - t_print >= 500) {
        t_print = now;
        SEGGER_RTT_printf(0,
            "M1(LR):%3d%% CNT=%-6d rpm=%-5d V=%-6d | "
            "M2(LF):%3d%% CNT=%-6d rpm=%-5d V=%-6d | "
            "M3(RF):%3d%% CNT=%-6d rpm=%-5d V=%-6d | "
            "M4(RR):%3d%% CNT=%-6d rpm=%-5d V=%-6d\r\n",
            duty_pct[0],
            (int16_t)__HAL_TIM_GET_COUNTER(enc[0].htim),
            (int)(enc[0].rpm / (float)GEAR_RATIO),
            (int)enc[0].wheel_speed,
            duty_pct[1],
            (int16_t)__HAL_TIM_GET_COUNTER(enc[1].htim),
            (int)(enc[1].rpm / (float)GEAR_RATIO),
            (int)enc[1].wheel_speed,
            duty_pct[2],
            (int16_t)__HAL_TIM_GET_COUNTER(enc[2].htim),
            (int)(enc[2].rpm / (float)GEAR_RATIO),
            (int)enc[2].wheel_speed,
            duty_pct[3],
            (int16_t)__HAL_TIM_GET_COUNTER(enc[3].htim),
            (int)(enc[3].rpm / (float)GEAR_RATIO),
            (int)enc[3].wheel_speed);
    }

    // ---- J-Scope every 10ms ----
    static uint32_t t_scope = 0;
    if (now - t_scope >= 10) {
        t_scope = now;
        ScopeData d;
        d.v1 = enc[0].wheel_speed;
        d.v2 = enc[1].wheel_speed;
        d.v3 = enc[2].wheel_speed;
        d.v4 = enc[3].wheel_speed;
        SEGGER_RTT_Write(1, &d, sizeof(d));
    }
}

void motor_test(void)
{
    static motor_state_t state = MOTOR_FWD;
    static uint32_t      t0    = 0;
    static uint8_t       inited = 0;

    if (!inited) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
        HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
        inited = 1;
    }

    uint32_t now = HAL_GetTick();
    if (t0 == 0) t0 = now;

    switch (state) {
    case MOTOR_FWD:
        for (int i = 0; i < 4; i++) {
            motor_set_dir(&motors[i], 1);
            motor_pwm(&motors[i], PWM_DUTY_50);
        }
        if (now - t0 >= DURATION_RUN) { state = MOTOR_STOP1; t0 = now; }
        break;
    case MOTOR_STOP1:
        for (int i = 0; i < 4; i++) {
            motor_brake(&motors[i]);
            motor_pwm(&motors[i], 0);
        }
        if (now - t0 >= DURATION_STOP) { state = MOTOR_REV; t0 = now; }
        break;
    case MOTOR_REV:
        for (int i = 0; i < 4; i++) {
            motor_set_dir(&motors[i], 0);
            motor_pwm(&motors[i], PWM_DUTY_50);
        }
        if (now - t0 >= DURATION_RUN) { state = MOTOR_STOP2; t0 = now; }
        break;
    case MOTOR_STOP2:
        for (int i = 0; i < 4; i++) {
            motor_brake(&motors[i]);
            motor_pwm(&motors[i], 0);
        }
        if (now - t0 >= DURATION_STOP) { state = MOTOR_FWD; t0 = now; }
        break;
    }

    // print encoder every 500ms
    static uint32_t t_print = 0;
    if (now - t_print >= 500) {
        t_print = now;
        SEGGER_RTT_printf(0,
            "enc: M1=%-6d M2=%-6d M3=%-6d M4=%-6d  state=%d\r\n",
            (int16_t)__HAL_TIM_GET_COUNTER(&htim2),
            (int16_t)__HAL_TIM_GET_COUNTER(&htim3),
            (int16_t)__HAL_TIM_GET_COUNTER(&htim4),
            (int16_t)__HAL_TIM_GET_COUNTER(&htim8),
            state);
    }
}
