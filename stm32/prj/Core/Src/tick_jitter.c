/*
 * tick_jitter.c — DWT cycle-counter based task period jitter measurement
 *
 * DWT cycle counter @ 168 MHz → 1 tick = 5.952 ns
 *  1ms → 168000 cycles,  4ms → 672000 cycles
 */

#include "tick_jitter.h"
#include "rtt_debug.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

// ---------------------------------------------------------------------------
// DWT init — call once at startup
// ---------------------------------------------------------------------------
void tick_jitter_init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	__DSB();  // ARM requires DSB after DEMCR write before DWT access
	__ISB();
	DWT->CYCCNT = 0;
	DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

// ---------------------------------------------------------------------------
// Sample one period — call at TOP of each task loop iteration
// ---------------------------------------------------------------------------
void tick_jitter_sample(JitterCtx *ctx, uint32_t expected_us)
{
	uint32_t now = DWT->CYCCNT;
	uint32_t expected_cycles = expected_us * (SystemCoreClock / 1000000U);

	if (ctx->sample_count == 0) {
		ctx->last_ts = now;
		ctx->sample_count = 1;
		ctx->period_ticks[0] = UINT32_MAX;
		ctx->period_ticks[1] = 0;
		return;
	}

	uint32_t delta = now - ctx->last_ts;
	ctx->last_ts = now;

	if (delta > expected_cycles * 2) {
		ctx->overruns++;
		return;
	}

	if (ctx->sample_count == 1) {
		ctx->period_ticks[0] = delta;
		ctx->period_ticks[1] = delta;
		ctx->period_ticks[3] = delta;
	} else {
		if (delta < ctx->period_ticks[0]) ctx->period_ticks[0] = delta;
		if (delta > ctx->period_ticks[1]) ctx->period_ticks[1] = delta;
		ctx->period_ticks[3] += delta;
	}

	ctx->period_ticks[2] = delta;
	ctx->sample_count++;
}

// ---------------------------------------------------------------------------
// Format jitter report string
// ---------------------------------------------------------------------------
const char *tick_jitter_report(const JitterCtx *ctx)
{
	static char buf[128];

	if (ctx->sample_count < 2) {
		snprintf(buf, sizeof(buf), "%-6s no data", ctx->name);
		return buf;
	}

	uint32_t avg = (uint32_t)(ctx->period_ticks[3] / (ctx->sample_count - 1));
	uint32_t min = ctx->period_ticks[0];
	uint32_t max = ctx->period_ticks[1];

	float us_per_tick = 1.0f / ((float)SystemCoreClock / 1000000.0f);
	float exp_us    = (float)ctx->expected_us;
	float avg_us    = (float)avg * us_per_tick;
	float jitter_us = (float)(max - min) * us_per_tick;

	snprintf(buf, sizeof(buf),
		"%-6s avg=%lu.%01luus  jitter=+/-%lu.%01luus  max=%lu.%01luus  ovr=%lu  n=%lu",
		ctx->name,
		(unsigned long)(uint32_t)avg_us, (unsigned long)((uint32_t)(avg_us * 10) % 10),
		(unsigned long)(uint32_t)(jitter_us / 2.0f),
		(unsigned long)((uint32_t)(jitter_us * 5.0f) % 10),
		(unsigned long)(uint32_t)((float)max * us_per_tick),
		(unsigned long)((uint32_t)((float)max * us_per_tick * 10) % 10),
		(unsigned long)ctx->overruns,
		(unsigned long)(ctx->sample_count - 1));

	return buf;
}
