/**
 * @file alarm_bridge.c
 * @brief Generic alarm monitor bridge implementation.
 *
 * One monitor thread wakes every ALARM_BRIDGE_POLL_MS milliseconds,
 * updates the heartbeat timestamp, and (after the startup delay) calls
 * alarm_engine_run() for every registered context.
 *
 * No UPS-specific logic lives here.  All device knowledge is encapsulated
 * in the alarm_engine_ctx_t objects registered before start().
 */

#include "alarm_bridge.h"
#include "alarm_engine.h"
#include "log.h"

#include <pthread.h>
#include <unistd.h>
#include <stddef.h>
#include <time.h>

/* ── Tunable constants ────────────────────────────────────────────────── */

/** Interval between alarm evaluation passes. */
#define ALARM_BRIDGE_POLL_MS           100u

/**
 * Silence period after start() before the first evaluation pass.
 * Set to the worst-case time for all registered devices to complete
 * their first successful poll and populate the internal pool.
 * Heartbeat updates happen normally during this period.
 */
#define ALARM_BRIDGE_STARTUP_DELAY_MS  5000u

/* ── Module-level state ───────────────────────────────────────────────── */

static alarm_engine_ctx_t *g_ctxs[ALARM_BRIDGE_MAX_CTXS];
static size_t              g_ctx_count  = 0u;
static volatile int        g_running    = 0;
static volatile uint64_t   g_last_beat_ms = 0u;
static pthread_t           g_thread;

/* ── Internal helpers ─────────────────────────────────────────────────── */

/**
 * @brief Return the current monotonic clock value in milliseconds.
 */
static uint64_t get_monotonic_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

/* ── Monitor thread ───────────────────────────────────────────────────── */

static void *alarm_monitor_thread(void *arg)
{
    (void)arg;

    uint64_t start_ms = get_monotonic_ms();
    bool     warming  = true;

    LOG_INFO("[Alarm Bridge] monitor thread started "
             "(startup_delay=%u ms, poll=%u ms, contexts=%zu).",
             ALARM_BRIDGE_STARTUP_DELAY_MS,
             ALARM_BRIDGE_POLL_MS,
             g_ctx_count);

    while (g_running) {
        uint64_t now_ms = get_monotonic_ms();
        g_last_beat_ms = now_ms;

        if (warming) {
            if (now_ms - start_ms >= ALARM_BRIDGE_STARTUP_DELAY_MS) {
                warming = false;
                LOG_INFO("[Alarm Bridge] startup delay elapsed, "
                         "alarm evaluation active.");
            } else {
                usleep(ALARM_BRIDGE_POLL_MS * 1000u);
                continue;
            }
        }

        for (size_t i = 0u; i < g_ctx_count; i++) {
            alarm_engine_run(g_ctxs[i]);
        }

        usleep(ALARM_BRIDGE_POLL_MS * 1000u);
    }

    LOG_INFO("[Alarm Bridge] monitor thread stopped.");
    return NULL;
}

/* ── Public API ───────────────────────────────────────────────────────── */

int alarm_bridge_register_ctx(alarm_engine_ctx_t *ctx)
{
    if (g_running) {
        LOG_ERROR("[Alarm Bridge] register_ctx called after start – ignored.");
        return -1;
    }

    if (!ctx) {
        return -1;
    }

    if (g_ctx_count >= ALARM_BRIDGE_MAX_CTXS) {
        LOG_ERROR("[Alarm Bridge] register_ctx: capacity reached (%u).",
                  ALARM_BRIDGE_MAX_CTXS);
        return -1;
    }

    g_ctxs[g_ctx_count++] = ctx;
    return 0;
}

int alarm_bridge_start(void)
{
    if (g_ctx_count == 0u) {
        LOG_WARNING("[Alarm Bridge] start: no contexts registered, "
                    "monitor thread will run but evaluate nothing.");
    }

    g_last_beat_ms = get_monotonic_ms();
    g_running      = 1;

    if (pthread_create(&g_thread, NULL, alarm_monitor_thread, NULL) != 0) {
        LOG_ERROR("[Alarm Bridge] failed to create monitor thread.");
        g_running = 0;
        return -1;
    }

    LOG_INFO("[Alarm Bridge] started (%zu context(s)).", g_ctx_count);
    return 0;
}

void alarm_bridge_stop(void)
{
    if (!g_running) {
        return;
    }

    g_running = 0;
    pthread_join(g_thread, NULL);

    LOG_INFO("[Alarm Bridge] stopped.");
}

uint64_t alarm_bridge_last_beat_ms(void)
{
    return g_last_beat_ms;
}
