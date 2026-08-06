/**
 * @file inverter_alarm.c
 * @brief Inverter alarm tables and alarm bridge registration.
 *
 * Owns: the alarm tables (what to monitor), per-unit runtime state, and
 * the wiring to alarm_bridge (inverter_alarm_register_all).
 *
 * Does NOT own alarm behaviour – every event is forwarded to
 * alarm_manager_handle_event() with this module's sink (node + DB path).
 *
 * Table columns
 * ─────────────
 *  device_address  FC03 register address (must exist in inverter_map.c).
 *  condition       ALARM_COND_BITMASK / ALARM_COND_RANGE / ALARM_COND_CHANGE.
 *  lo_limit        Lower bound (RANGE) or ignored (BITMASK / CHANGE).
 *  hi_limit        Upper bound (RANGE) or bit mask (BITMASK) or ignored (CHANGE).
 *  error_code      Forwarded in the event; meaning defined in inverter_alarm.h.
 *  description     Human-readable label; forwarded in the event only.
 *
 * Condition quick reference
 * ─────────────────────────
 *  BITMASK  fires when (value & hi_limit) == hi_limit.
 *           Same register can appear N times for N independent bits.
 *
 *  RANGE    fires when value < lo_limit || value > hi_limit.
 *           Upper bound only: lo_limit = 0.
 *           Lower bound only: hi_limit = 0xFFFF.
 *
 *  CHANGE   fires when value != prev_value.
 *           No sticky suppression – every change is an independent event.
 *
 * Register address reference (from inverter_map.c)
 * ──────────────────────────────────────────────────
 *  0x0001  Fault_Code              0x0002  Warning_Code
 *  0x0010  AC_Output_Voltage       0x0015  AC_Output_Load_Percent
 *  0x0042  Battery_State_of_Charge 0x0043  Battery_Temperature
 *  0x0050  Heatsink_Temperature
 */

#include "inverter_alarm.h"
#include "alarm_manager.h"
#include "alarm_engine.h"
#include "alarm_bridge.h"
#include "device_register_map.h"
#include "config_loader.h"
#include "inverter/inverter_map.h"
#include "log.h"

#define ARRAY_SIZE(a)  (sizeof(a) / sizeof((a)[0]))

/* Source-owned sink – node and DB path are fixed here, not from config.json. */
static const alarm_manager_sink_t inverter_alarm_sink = {
    .log_dir  = "/home/cm/LFB_Feeder_Kit/Logs/inverter_alarm",
    .log_path = "/home/cm/LFB_Feeder_Kit/Logs/inverter_alarm/alarms.db",
    .log_node = "inverter_alarm_node",
};

/* ═══════════════════════════════════════════════════════════════════════
 * Shared alarm table — same hardware model for all Inverter units.
 * Per-unit runtime state is kept separate so sticky flags and prev_value
 * are not shared across units.
 *
 *  device_addr  condition            lo      hi      error_code                   description
 * ═══════════════════════════════════════════════════════════════════════ */
static const alarm_entry_t inverter_alarm_table[] = {
    { 0x2100, ALARM_COND_CHANGE,  0x0000, 0x0000, INV_ERR_FAULT_CODE, "fault code changed" },
};

/*
 * Per-unit mutable runtime state – one states slot per possible config
 * slot (indexed the same as global_config.inverter[]), so sticky flags
 * and prev_value are never shared across physical units even if several
 * of them use the same model/alarm table.
 */
static alarm_state_t inverter_alarm_states_pool[MAX_INVERTER_COUNT][ARRAY_SIZE(inverter_alarm_table)];

/* Stable ctx storage – lifetime must outlast alarm_bridge_stop(). */
static alarm_engine_ctx_t inverter_alarm_ctxs[MAX_INVERTER_COUNT];
static size_t             inverter_alarm_ctx_count = 0u;

/* ── alarm_read_fn ────────────────────────────────────────────────────── */

/**
 * @brief Read one register from the internal pool by device_address.
 * userdata is a const device_map_profile_t *.
 */
static bool alarm_read_register(uint16_t  device_address,
                                uint16_t *out_value,
                                void     *userdata)
{
    const device_map_profile_t *profile = (const device_map_profile_t *)userdata;
    return pool_read_by_device_addr(profile, device_address, out_value);
}

/* ── alarm_event_fn ───────────────────────────────────────────────────── */

/**
 * @brief Thin forwarder – all events go straight to the Alarm Manager.
 */
static void forward_to_manager(const alarm_entry_t *entry,
                               uint16_t              value,
                               alarm_event_t         event,
                               void                 *userdata)
{
    alarm_manager_handle_event(entry, value, event, userdata,
                               &inverter_alarm_sink);
}

/* ── Public API ───────────────────────────────────────────────────────── */

/**
 * @brief Register one alarm context per enabled Inverter unit in config.json.
 *
 * Iterates global_config.inverter[] directly (config-driven, no hardcoded
 * uid table).  Every unit uses inverter1_profile (the only Inverter
 * hardware model currently implemented) and gets its own states slot at
 * inverter_alarm_states_pool[j] so runtime state is never shared across
 * units.
 */
void inverter_alarm_register_all(void)
{
    inverter_alarm_ctx_count = 0u;

    for (int j = 0; j < global_config.inverter_count; j++) {
        module_config_t *cfg = &global_config.inverter[j];

        if (!cfg->enabled) {
            continue;
        }

        alarm_engine_ctx_t *ctx = &inverter_alarm_ctxs[inverter_alarm_ctx_count];

        ctx->table      = inverter_alarm_table;
        ctx->states     = inverter_alarm_states_pool[j];
        ctx->count      = ARRAY_SIZE(inverter_alarm_table);
        ctx->read_fn    = alarm_read_register;
        ctx->event_fn   = forward_to_manager;
        ctx->read_data  = (void *)&inverter1_profile;
        ctx->event_data = (void *)cfg;

        if (alarm_bridge_register_ctx(ctx) != 0) {
            LOG_ERROR("[Alarm] inverter_alarm_register_all: "
                      "failed to register ctx for %s.", cfg->name);
            continue;
        }

        inverter_alarm_ctx_count++;
        LOG_INFO("[Alarm] registered alarm context for %s.", cfg->name);
    }
}
