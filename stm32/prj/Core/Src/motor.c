#include "motor.h"
#include "tim.h"
#include <stdlib.h>  // abs

// TIM1 PWM period (10 kHz @ 168 MHz)
#define PWM_PERIOD      16799
// PWM duty limits to protect TB6612 and motor
#define PWM_MAX_DUTY    10079   // 60% of period (16799 * 0.6)
#define PWM_MIN_DUTY     2520   // 15% of period (16799 * 0.15), below this motor won't move
#define PWM_OUT_MAX     1000

// Default PID gains for wheel-speed domain (mm/s)
// Feed-forward handles the bulk; PID corrects residual errors
// User tunes via RTT: kp/ki/kd/kf commands
#define MOTOR_DEFAULT_KP   4.0f
#define MOTOR_DEFAULT_KI   5.0f
#define MOTOR_DEFAULT_KD   0.01f
#define MOTOR_FF_GAIN      1.24f  // loaded calib 2026-05-26: pwm=1.235*V-16.8, R²=0.96

// Soft ramp: mm/s change per 1ms tick (500 mm/s^2 accel, 1000 mm/s^2 decel)
#define MOTOR_RAMP_UP      0.5f
#define MOTOR_RAMP_DOWN    1.0f

#define DT                 0.001f  // 1 ms

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

    // Start encoder on all 4 timers (encoder_init does this now, skip duplicates)
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Motor *m = &g_motors[i];
        m->id           = motor_cfg[i].id;
        m->in1_port     = motor_cfg[i].in1_port;
        m->in1_pin      = motor_cfg[i].in1_pin;
        m->in2_port     = motor_cfg[i].in2_port;
        m->in2_pin      = motor_cfg[i].in2_pin;
        m->pwm_channel  = motor_cfg[i].pwm_ch;
        m->target_speed = 0;
        m->ramp_target  = 0.0f;
        m->actual_speed = 0;
        m->pwm_output   = 0;
        m->ff_gain      = MOTOR_FF_GAIN;
        m->pending_target = 0;
        m->stopped      = 1;

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

        // 1. Read encoder
        encoder_update(&m->encoder);
        m->actual_speed = (int16_t)m->encoder.wheel_speed;

        // 2. Soft ramp: move ramp_target toward target_speed at controlled rate
        float target_f = (float)m->target_speed;
        if (m->ramp_target < target_f) {
            m->ramp_target += MOTOR_RAMP_UP;
            if (m->ramp_target > target_f) m->ramp_target = target_f;
        } else if (m->ramp_target > target_f) {
            m->ramp_target -= MOTOR_RAMP_DOWN;
            if (m->ramp_target < target_f) m->ramp_target = target_f;
        }

        // 3. When fully stopped (target=0, ramp=0): short-brake, hold firm
        if (m->target_speed == 0 && m->ramp_target == 0.0f) {
            if (!m->stopped) {
                pid_reset(&m->pid);
                m->pwm_output = 0;
                m->stopped = 1;
            }
            // If a direction reversal was requested, now apply the pending target
            if (m->pending_target != 0) {
                m->target_speed = m->pending_target;
                m->pending_target = 0;
                m->stopped = 0;   // re-enable PID for new direction
                // fall through to PID path below
            } else {
                // IN1=IN2=1 shorts motor windings via low-side FETs = dynamic brake
                HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
                __HAL_TIM_SET_COMPARE(&htim1, m->pwm_channel, 0);
                continue;
            }
        }

        m->stopped = 0;

        // 4. Feed-forward + PID correction
        // FF provides the bulk output based on target speed (derived from open-loop data)
        // PID only corrects residual errors → much faster response at all speeds
        float ff = m->ramp_target * m->ff_gain;
        float pid_out = pid_update(&m->pid, m->ramp_target,
                                   (float)m->actual_speed, DT);
        int32_t out = (int32_t)(ff + pid_out);
        if (out > PWM_OUT_MAX) out = PWM_OUT_MAX;
        if (out < -PWM_OUT_MAX) out = -PWM_OUT_MAX;
        m->pwm_output = out;

        // 5. Apply output: direction + PWM
        motor_set_direction(m);
        motor_set_pwm(m);
    }
}

void motor_control_set_target(MotorID id, int16_t speed_mms)
{
    if (id >= MOTOR_COUNT) return;
    Motor *m = &g_motors[id];

    // Direction reversal: force a full stop (ramp→0→short brake) before turning
    if (speed_mms != 0 && m->ramp_target != 0.0f &&
        (speed_mms > 0) != (m->ramp_target > 0.0f)) {
        m->pending_target = speed_mms;
        m->target_speed = 0;  // triggers ramp-down + short brake
        return;
    }

    m->target_speed = speed_mms;
    m->pending_target = 0;
    if (speed_mms != 0) {
        m->stopped = 0;
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

void motor_control_set_ff_gain(MotorID id, float gain)
{
    if (id >= MOTOR_COUNT) return;
    g_motors[id].ff_gain = gain;
}

Motor* motor_get(MotorID id)
{
    if (id >= MOTOR_COUNT) return NULL;
    return &g_motors[id];
}
