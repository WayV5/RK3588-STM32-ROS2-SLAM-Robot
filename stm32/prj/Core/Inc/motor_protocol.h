#ifndef __MOTOR_PROTOCOL_H__
#define __MOTOR_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// --- Shared data structures (link-layer agnostic) ---

// CAN ID 0x101: Host -> MCU motor command
typedef struct {
    int16_t target_rpm[4];  // [M1(LR), M2(LF), M3(RF), M4(RR)]
} MotorCommand;

// CAN ID 0x201: MCU -> Host motor feedback
typedef struct {
    int16_t actual_rpm[4];
    int32_t encoder_count[4];
} MotorTelemetry;

// CAN ID 0x103: Host -> MCU PID parameter
typedef struct {
    uint8_t motor_id;   // 0=M2(LF) 1=M1(LR) 2=M3(RF) 3=M4(RR)
    uint8_t param_type; // 0=Kp 1=Ki 2=Kd
    float   value;
} PidConfig;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PROTOCOL_H__ */
