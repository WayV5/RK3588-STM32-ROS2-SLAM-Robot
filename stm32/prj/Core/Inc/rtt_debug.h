/*
 * rtt_debug.h — Compile-time RTT debug output gating
 *
 * Usage:
 *   Set RTT_DEBUG_LEVEL before including this header, or at compile time
 *   (-D RTT_DEBUG_LEVEL=3 in Keil → Options → C/C++ → Define).
 *   Default is INFO (2).
 *
 * Levels:
 *   0 = OFF    — no debug output
 *   1 = ERROR  — only critical failures (init fails, I2C stuck, etc.)
 *   2 = INFO   — init, status, stack watermark, CAN command summary (default)
 *   3 = TRACE  — per-frame CAN RX/TX, motor target details
 *
 * Macros:
 *   RTT_ERR(fmt, ...)   — critical errors, level >= 1
 *   RTT_INF(fmt, ...)   — init, status, stack, CAN commands — level >= 2
 *   RTT_DBG(fmt, ...)   — per-frame debug, CAN RX cnt, verbose — level >= 3
 *   RTT_CON(fmt, ...)   — RTT console interaction (channel 1), always on
 */

#ifndef RTT_DEBUG_H
#define RTT_DEBUG_H

#include "test/SEGGER_RTT.h"

// --- Compile-time debug level ---
#ifndef RTT_DEBUG_LEVEL
#define RTT_DEBUG_LEVEL  2
#endif

#define RTT_LVL_OFF    0
#define RTT_LVL_ERROR  1
#define RTT_LVL_INFO   2
#define RTT_LVL_TRACE  3

// Debug output to RTT channel 0 — gated by debug level
#define RTT_LOG(lvl, fmt, ...) \
	do { if ((lvl) <= RTT_DEBUG_LEVEL) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__); } while(0)

// RTT console output (channel 1, interactive) — always on
#define RTT_CON(fmt, ...) \
	SEGGER_RTT_printf(RTT_CH_TERMINAL, fmt, ##__VA_ARGS__)

// Shortcut macros — intention-revealing names
#define RTT_ERR(fmt, ...)  RTT_LOG(RTT_LVL_ERROR, fmt, ##__VA_ARGS__)
#define RTT_INF(fmt, ...)  RTT_LOG(RTT_LVL_INFO,  fmt, ##__VA_ARGS__)
#define RTT_DBG(fmt, ...)  RTT_LOG(RTT_LVL_TRACE, fmt, ##__VA_ARGS__)

#endif /* RTT_DEBUG_H */
