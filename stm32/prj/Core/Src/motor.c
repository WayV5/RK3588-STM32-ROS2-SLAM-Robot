#include "motor.h"
#include "tim.h"
#include <stdlib.h>  // abs

// TIM1 PWM period (10 kHz @ 168 MHz)
#define PWM_PERIOD      16799
// PWM duty limits to protect TB6612 and motor
#define PWM_MAX_DUTY    10079   // 60% of period (16799 * 0.6)
#define PWM_MIN_DUTY     2520   // 15% of period (16799 * 0.15), below this motor won't move
#define PWM_OUT_MAX     1000

// Default PID gains (conservative, tune via RTT)
#define MOTOR_DEFAULT_KP  0.5f
#define MOTOR_DEFAULT_KI  0.05f
#define MOTOR_DEFAULT_KD  0.02f

#define DT                0.001f   // 1 ms

// --- Static motor pool ---
static Motor g_motors[MOTOR_COUNT];

// --- Hardware descriptor table ---
static const struct {
    MotorID       id;
    GPIO_TypeDef *in1_port; uint16_t in1_pin;
    GPIO_TypeDef *in2_port; uint16_t in2_pin;
    uint32_t      pwm_ch;
    TIM_HandleTypeDef *htim_enc;
} motor_cfg[MOTOR_COUNT] = {
    { MOTOR_M1_LR, M1_IN1_GPIO_Port, M1_IN1_Pin, M1_IN2_GPIO_Port, M1_IN2_Pin,
      TIM_CHANNEL_1, &htim2 },
    { MOTOR_M2_LF, M2_IN1_GPIO_Port, M2_IN1_Pin, M2_IN2_GPIO_Port, M2_IN2_Pin,
      TIM_CHANNEL_2, &htim3 },
    { MOTOR_M3_RF, M3_IN1_GPIO_Port, M3_IN1_Pin, M3_IN2_GPIO_Port, M3_IN2_Pin,
      TIM_CHANNEL_3, &htim4 },
    { MOTOR_M4_RR, M4_IN1_GPIO_Port, M4_IN1_Pin, M4_IN2_GPIO_Port, M4_IN2_Pin,
      TIM_CHANNEL_4, &htim8 },
};

// --- Per-motor direction helpers ---
static inline void motor_set_direction(const Motor *m)
{
    if (m->pwm_output > 0) {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
    } else if (m->pwm_output < 0) {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
    }
}

static inline void motor_set_pwm(const Motor *m)
{
    uint32_t duty;
    if (m->pwm_output == 0) {
        duty = 0;
    } else {
        duty = (uint32_t)((int32_t)abs(m->pwm_output) * PWM_PERIOD / PWM_OUT_MAX);
        if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
        else if (duty < PWM_MIN_DUTY) duty = PWM_MIN_DUTY;
    }
    __HAL_TIM_SET_COMPARE(&htim1, m->pwm_channel, duty);
}

// --- Public API ---

void motor_control_init(void)
{
    // Start PWM on all 4 channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // Start encoder on all 4 timers
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        Motor *m = &g_motors[i];
        m->id          = motor_cfg[i].id;
        m->in1_port    = motor_cfg[i].in1_port;
        m->in1_pin     = motor_cfg[i].in1_pin;
        m->in2_port    = motor_cfg[i].in2_port;
        m->in2_pin     = motor_cfg[i].in2_pin;
        m->pwm_channel = motor_cfg[i].pwm_ch;
        m->target_rpm  = 0;
        m->actual_rpm  = 0;
        m->pwm_output  = 0;

        encoder_init(&m->encoder, motor_cfg[i].htim_enc);
        pid_init(&m->pid, MOTOR_DEFAULT_KP, MOTOR_DEFAULT_KI, MOTOR_DEFAULT_KD);

        motor_set_direction(m);
        motor_set_pwm(m);
    }
}

void motor_control_update(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Motor *m = &g_motors[i];

        // 1. Read encoder, compute RPM
        encoder_update(&m->encoder);
        m->actual_rpm = (int16_t)m->encoder.rpm;

        // 2. Run PID
        float pid_out = pid_update(&m->pid, (float)m->target_rpm,
                                   (float)m->actual_rpm, DT);
        m->pwm_output = (int32_t)pid_out;

        // 3. Apply output: direction + PWM
        motor_set_direction(m);
        motor_set_pwm(m);
    }
}

// target_rpm is stored as motor-shaft RPM internally.
// Public API accepts wheel RPM and converts via gear ratio.
void motor_control_set_target(MotorID id, int16_t wheel_rpm)
{
    if (id >= MOTOR_COUNT) return;
    Motor *m = &g_motors[id];
    m->target_rpm = (int16_t)((float)wheel_rpm * GEAR_RATIO);
    if (wheel_rpm == 0) {
        pid_reset(&m->pid);
        m->pwm_output = 0;
    }
}

void motor_control_stop(MotorID id)
{
    motor_control_set_target(id, 0);
}

void motor_control_stop_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_control_stop((MotorID)i);
    }
}

Motor* motor_get(MotorID id)
{
    if (id >= MOTOR_COUNT) return NULL;
    return &g_motors[id];
}
