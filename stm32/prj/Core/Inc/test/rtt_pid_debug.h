#ifndef __RTT_PID_DEBUG_H__
#define __RTT_PID_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor.h"

// Call once after RTT is ready. Prints a welcome banner.
void rtt_pid_debug_init(void);

// Non-blocking poll of down-channel 0 (J-LINK Viewer terminal input).
// Call as often as possible from the main loop.
void rtt_pid_debug_poll(void);

// Output one J-Scope binary packet on up-channel 1 (every 10 ms recommended).
// Format: 8 × int16_t  "JScope_I2I2I2I2I2I2I2I2"
//   M1_act  M1_tgt  M2_act  M2_tgt  M3_act  M3_tgt  M4_act  M4_tgt
//   (actual wheel speed mm/s, target wheel speed mm/s)
void rtt_scope_output(void);

// Periodic telemetry to RTT Viewer ch0 (call from main loop, auto-throttled to 500ms).
// Prints one compact line: target/actual speed + PWM for all 4 motors.
void rtt_telemetry_output(void);

#ifdef __cplusplus
}
#endif

#endif /* __RTT_PID_DEBUG_H__ */
