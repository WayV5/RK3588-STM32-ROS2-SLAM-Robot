#include "test.h"
#include "tim.h"
#include "SEGGER_RTT.h"

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
