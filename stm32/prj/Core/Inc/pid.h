#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float integral_limit;   // |error| > this -> reset integral (integral separation)
    float integral_max;     // clamp |integral| to this (anti-windup, = 0.8*out_max/Ki)
    float deadzone;         // |error| < this -> output 0
    float out_min, out_max; // output clamp (suggest +-1000)
} PID;

void pid_init(PID *pid, float Kp, float Ki, float Kd);
float pid_update(PID *pid, float setpoint, float measurement, float dt);
void pid_reset(PID *pid);
void pid_set_params(PID *pid, float Kp, float Ki, float Kd);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
