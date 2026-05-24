#include "pid.h"
#include <math.h>   // fabsf
#include <string.h> // memset

#define PID_DEFAULT_INTEGRAL_LIMIT  50.0f
#define PID_DEFAULT_DEADZONE         3.0f
#define PID_DEFAULT_OUT_MIN      -1000.0f
#define PID_DEFAULT_OUT_MAX       1000.0f

void pid_init(PID *pid, float Kp, float Ki, float Kd)
{
    memset(pid, 0, sizeof(PID));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral_limit = PID_DEFAULT_INTEGRAL_LIMIT;
    pid->deadzone       = PID_DEFAULT_DEADZONE;
    pid->out_min        = PID_DEFAULT_OUT_MIN;
    pid->out_max        = PID_DEFAULT_OUT_MAX;
}

void pid_set_params(PID *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

void pid_reset(PID *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(PID *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // Dead zone: output zero + reset integral to avoid windup
    if (fabsf(error) < pid->deadzone) {
        pid->integral   = 0.0f;
        pid->prev_error = error;
        return 0.0f;
    }

    // Integral separation: reset integral when error is large
    if (fabsf(error) > pid->integral_limit) {
        pid->integral = 0.0f;
    } else {
        pid->integral += error * dt;
    }

    float derivative = (error - pid->prev_error) / dt;
    float out = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // Output clamp with anti-windup
    if (out > pid->out_max) {
        out = pid->out_max;
        pid->integral -= error * dt;
    } else if (out < pid->out_min) {
        out = pid->out_min;
        pid->integral -= error * dt;
    }

    pid->prev_error = error;
    return out;
}
