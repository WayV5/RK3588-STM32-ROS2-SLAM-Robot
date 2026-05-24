#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "pid.h"
#include "encoder.h"
#include "motor_protocol.h"

// --- Motor ID enum ---
typedef enum {
    MOTOR_M1_LR = 0,  // Left Rear
    MOTOR_M2_LF,      // Left Front
    MOTOR_M3_RF,      // Right Front
    MOTOR_M4_RR,      // Right Rear
    MOTOR_COUNT
} MotorID;

// --- Per-motor descriptor ---
typedef struct {
    MotorID          id;
    GPIO_TypeDef    *in1_port;
    uint16_t         in1_pin;
    GPIO_TypeDef    *in2_port;
    uint16_t         in2_pin;
    uint32_t         pwm_channel;
    Encoder          encoder;
    PID              pid;
    int16_t          target_speed;  // mm/s — final target set by host
    float            ramp_target;   // mm/s — ramped setpoint fed to PID
    int16_t          actual_speed;  // mm/s
    int32_t          pwm_output;    // [-1000, 1000]
    float            ff_gain;       // feed-forward gain (pwm_out per mm/s)
    int16_t          pending_target; // stored target during direction reversal
    uint8_t          stopped;       // 1 = fully stopped, skip PID
} Motor;

// --- Public API ---
void    motor_control_init(void);
void    motor_control_update(void);                  // call at 1 kHz
void    motor_control_set_target(MotorID id, int16_t speed_mms);
void    motor_control_set_ff_gain(MotorID id, float gain);
void    motor_control_stop(MotorID id);
void    motor_control_stop_all(void);
Motor*  motor_get(MotorID id);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
