#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "tim.h"

// Mechanical constants (must match actual hardware)
#define WHEEL_DIAMETER_MM       65.0f
#define WHEEL_CIRCUMFERENCE_MM  (3.14159265f * WHEEL_DIAMETER_MM)  // ~204.2 mm
#define GEAR_RATIO              30u     // motor shaft : wheel

typedef struct {
    TIM_HandleTypeDef *htim;
    int16_t  delta;             // raw delta this tick
    int16_t  history[10];       // 10 ms sliding window
    int32_t  sum;               // windowed delta sum
    int16_t  cnt_last;          // last CNT snapshot
    uint8_t  idx;               // ring buffer write index
    float    rpm;               // motor-shaft RPM (filtered)
    float    wheel_speed;       // wheel linear speed (mm/s)
} Encoder;

void    encoder_init(Encoder *e, TIM_HandleTypeDef *htim);
void    encoder_update(Encoder *e);      // call every 1 ms
float   encoder_get_rpm(const Encoder *e);
float   encoder_get_wheel_speed(const Encoder *e);
int32_t encoder_get_sum(const Encoder *e);
void    encoder_reset(Encoder *e);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
