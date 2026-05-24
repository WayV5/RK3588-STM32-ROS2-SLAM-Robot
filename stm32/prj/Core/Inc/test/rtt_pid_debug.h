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

// Output one CSV line for J-Link Scope on up-channel 1.
// Format: M1_RPM;M1_TGT;M2_RPM;M2_TGT;M3_RPM;M3_TGT;M4_RPM;M4_TGT
// Call at a fixed rate (every 10 ms recommended).
void rtt_scope_output(void);

#ifdef __cplusplus
}
#endif

#endif /* __RTT_PID_DEBUG_H__ */
