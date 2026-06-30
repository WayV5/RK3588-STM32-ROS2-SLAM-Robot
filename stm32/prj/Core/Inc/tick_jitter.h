/*
 * tick_jitter.h — DWT cycle-counter based task period jitter measurement
 *
 * Cortex-M4 DWT @ 168 MHz → 1 tick = 5.952 ns, 1ms = 168000 cycles.
 */

#ifndef TICK_JITTER_H
#define TICK_JITTER_H

#include <stdint.h>

// Per-task jitter stats (ring-buffer of last N periods)
typedef struct {
	uint32_t last_ts;
	uint32_t period_ticks[4];       // [0]=min, [1]=max, [2]=last, [3]=sum
	uint32_t sample_count;
	uint32_t overruns;
	const char *name;
	uint32_t expected_us;
} JitterCtx;

// Initialize DWT cycle counter (call once at startup)
void tick_jitter_init(void);

// Sample one period. Call at the TOP of the task loop.
void tick_jitter_sample(JitterCtx *ctx, uint32_t expected_us);

// Get jitter stats as a human-readable string
const char *tick_jitter_report(const JitterCtx *ctx);

#endif /* TICK_JITTER_H */
