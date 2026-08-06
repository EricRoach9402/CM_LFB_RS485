/**
 * @file alarm_bridge.h
 * @brief Generic alarm monitor bridge – portable across projects.
 *
 * Responsibilities
 * ────────────────
 *  Owns one dedicated monitor thread that periodically calls
 *  alarm_engine_run() for every registered alarm_engine_ctx_t.
 *
 *  This file intentionally knows nothing about UPS, Modbus, or any
 *  other device type.  Device-specific concerns (alarm tables, read_fn,
 *  event_fn) live in project-specific files that register ctx objects
 *  before calling alarm_bridge_start().
 *
 * Lifecycle
 * ─────────
 *  1. Register one alarm_engine_ctx_t per monitored unit via
 *     alarm_bridge_register_ctx() – must be called before start().
 *  2. Call alarm_bridge_start() to spawn the monitor thread.
 *  3. Call alarm_bridge_stop() during shutdown, before releasing any
 *     resource that an active ctx still points to.
 *
 * Startup delay
 * ─────────────
 *  ALARM_BRIDGE_STARTUP_DELAY_MS suppresses all alarm evaluations for a
 *  fixed period after start().  This prevents false triggers caused by
 *  the internal pool being all-zero before the first successful device
 *  scan.  Set this value to at least the worst-case time required for
 *  all registered units to complete their first scan.
 *
 * Heartbeat
 * ─────────
 *  The monitor thread updates an internal monotonic timestamp on every
 *  loop iteration (including during the startup delay).  Callers can
 *  retrieve it via alarm_bridge_last_beat_ms() to verify the thread is
 *  still alive.  A value of 0 means the bridge has not been started.
 */

#ifndef ALARM_BRIDGE_H
#define ALARM_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#include "alarm_engine.h"

/* ── Capacity ─────────────────────────────────────────────────────────── */

/** Maximum number of alarm_engine_ctx_t pointers that can be registered. */
#define ALARM_BRIDGE_MAX_CTXS  16u

/* ── Public API ───────────────────────────────────────────────────────── */

/**
 * @brief Register one alarm engine context with the bridge.
 *
 * Must be called before alarm_bridge_start().  The pointer must remain
 * valid until alarm_bridge_stop() returns.
 *
 * @param ctx  Fully initialised alarm_engine_ctx_t.
 * @return 0 on success, -1 if the bridge is already running, ctx is NULL,
 *         or the maximum number of contexts has been reached.
 */
int alarm_bridge_register_ctx(alarm_engine_ctx_t *ctx);

/**
 * @brief Start the alarm monitor thread.
 *
 * All contexts must already be registered.  A startup delay
 * (ALARM_BRIDGE_STARTUP_DELAY_MS) is applied before the first evaluation.
 *
 * @return 0 on success, -1 on thread creation failure.
 */
int alarm_bridge_start(void);

/**
 * @brief Stop the alarm monitor thread and block until it exits.
 *
 * Safe to call even if alarm_bridge_start() was never called or failed.
 */
void alarm_bridge_stop(void);

/**
 * @brief Return the monotonic timestamp (milliseconds) of the last
 *        heartbeat update from the monitor thread.
 *
 * Returns 0 if the bridge has not been started yet.
 * Use this to check thread liveness:
 *
 *   uint64_t age_ms = get_monotonic_ms() - alarm_bridge_last_beat_ms();
 *   if (age_ms > EXPECTED_MAX_MS) → thread may be stuck
 */
uint64_t alarm_bridge_last_beat_ms(void);

#endif /* ALARM_BRIDGE_H */
