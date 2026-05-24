#include "encoder.h"
#include <string.h>

// ENC_PPR = 11 lines * 4 (quadrature) = 44 counts per motor-shaft rev
#define ENC_CPR         44
#define ENC_WINDOW_MS   10    // 10 samples @1 kHz
// RPM = sum / CPR * (60*1000 / WINDOW_MS)
//     = sum * (60000 / (44 * 10)) = sum * 6000/44
#define ENC_RPM_SCALE   (6000.0f / (float)ENC_CPR)

void encoder_init(Encoder *e, TIM_HandleTypeDef *htim)
{
    memset(e, 0, sizeof(Encoder));
    e->htim     = htim;
    e->cnt_last = (int16_t)htim->Instance->CNT;
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void encoder_update(Encoder *e)
{
    int16_t cnt_now = (int16_t)__HAL_TIM_GET_COUNTER(e->htim);
    e->delta = cnt_now - e->cnt_last;
    e->cnt_last = cnt_now;

    // Sliding window
    e->sum -= e->history[e->idx];
    e->sum += e->delta;
    e->history[e->idx] = e->delta;
    e->idx = (e->idx + 1) % ENC_WINDOW_MS;

    // Motor-shaft RPM
    e->rpm = (float)e->sum * ENC_RPM_SCALE;
    // Wheel linear speed (mm/s): motor_rpm * circumference / (60 * gear_ratio)
    e->wheel_speed = e->rpm * WHEEL_CIRCUMFERENCE_MM / (60.0f * (float)GEAR_RATIO);
}

float encoder_get_rpm(const Encoder *e)
{
    return e->rpm;
}

float encoder_get_wheel_speed(const Encoder *e)
{
    return e->wheel_speed;
}

int32_t encoder_get_sum(const Encoder *e)
{
    return e->sum;
}

void encoder_reset(Encoder *e)
{
    e->sum = 0;
    e->idx = 0;
    e->rpm = 0.0f;
    memset(e->history, 0, sizeof(e->history));
    e->cnt_last = (int16_t)e->htim->Instance->CNT;
}
