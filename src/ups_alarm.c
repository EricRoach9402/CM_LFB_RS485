/**
 * @file ups_alarm.c
 * @brief UPS alarm tables and alarm bridge registration.
 *
 * Owns: the alarm tables (what to monitor), per-unit runtime state, and
 * the wiring to alarm_bridge (ups_alarm_register_all).
 *
 * Does NOT own alarm behaviour – every event is forwarded to
 * alarm_manager_handle_event() with this module's sink (node + DB path).
 *
 * Table columns
 * ─────────────
 *  device_address  FC03 register address (must exist in ups_map.c).
 *  condition       ALARM_COND_BITMASK / ALARM_COND_RANGE / ALARM_COND_CHANGE.
 *  lo_limit        Lower bound (RANGE) or ignored (BITMASK / CHANGE).
 *  hi_limit        Upper bound (RANGE) or bit mask (BITMASK) or ignored (CHANGE).
 *  error_code      Forwarded in the event; meaning defined in ups_alarm.h.
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
 *  CHANGE   fires when value value != prev_value.
 *           No sticky suppression – every change is an independent event.
 */

#include "ups_alarm.h"
#include "alarm_manager.h"
#include "alarm_engine.h"
#include "alarm_bridge.h"
#include "device_register_map.h"
#include "config_loader.h"
#include "ups/ups_map.h"
#include "log.h"

#define ARRAY_SIZE(a)  (sizeof(a) / sizeof((a)[0]))

/* Source-owned sink – node and DB path are fixed here, not from config.json. */
static const alarm_manager_sink_t ups_alarm_sink = {
    .log_dir  = "/home/cm/LFB_Feeder_Kit/Logs/ups_alarm",
    .log_path = "/home/cm/LFB_Feeder_Kit/Logs/ups_alarm/alarms.db",
    .log_node = "ups_alarm_node",
};

/* ═══════════════════════════════════════════════════════════════════════
 * Shared alarm table — same hardware model for all UPS units.
 * Per-unit runtime state is kept separate so sticky flags and prev_value
 * are not shared across units.
 *
 *  device_addr  condition            lo      hi      error_code               description
 * ═══════════════════════════════════════════════════════════════════════ */
static const alarm_entry_t ups_alarm_table[] = {
    { 0x0000, ALARM_COND_CHANGE,  0x0000, 0x0000, UPS_ERR_WARNING_1,     "warning 1 value changed" },
    { 0x0001, ALARM_COND_CHANGE,  0x0000, 0x0000, UPS_ERR_WARNING_2,     "warning 2 value changed" },
    { 0x0002, ALARM_COND_CHANGE,  0x0000, 0x0000, UPS_ERR_WARNING_3,     "warning 3 value changed" },
    { 0x0003, ALARM_COND_CHANGE,  0x0000, 0x0000, UPS_ERR_WARNING_4,     "warning 4 value changed" },
    { 0x00D0, ALARM_COND_CHANGE,  0x0000, 0x0000, UPS_MODE_INFORMATION,  "UPS mode changed" },
    { 0x02A2, ALARM_COND_CHANGE,  0x0000, 0x0000, UPS_FAULT_INFORMATION, "UPS fault changed" },
};

/*
 * Per-unit mutable runtime state – one states slot per possible config
 * slot (indexed the same as global_config.ups[]), so sticky flags and
 * prev_value are never shared across physical units even if several of
 * them use the same model/alarm table.
 */
static alarm_state_t ups_alarm_states_pool[MAX_UPS_COUNT][ARRAY_SIZE(ups_alarm_table)];

/* Stable ctx storage – lifetime must outlast alarm_bridge_stop(). */
static alarm_engine_ctx_t ups_alarm_ctxs[MAX_UPS_COUNT];
static size_t             ups_alarm_ctx_count = 0u;

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
    alarm_manager_handle_event(entry, value, event, userdata, &ups_alarm_sink);
}

/* ── Public API ───────────────────────────────────────────────────────── */

/**
 * @brief Register one alarm context per enabled UPS unit in config.json.
 *
 * Iterates global_config.ups[] directly (config-driven, no hardcoded uid
 * table).  Every unit uses ups1_profile (the only UPS hardware model
 * currently implemented) and gets its own states slot at
 * ups_alarm_states_pool[j] so runtime state is never shared across units.
 */
void ups_alarm_register_all(void)
{
    ups_alarm_ctx_count = 0u;

    for (int j = 0; j < global_config.ups_count; j++) {
        module_config_t *cfg = &global_config.ups[j];

        if (!cfg->enabled) {
            continue;
        }

        alarm_engine_ctx_t *ctx = &ups_alarm_ctxs[ups_alarm_ctx_count];

        ctx->table      = ups_alarm_table;
        ctx->states     = ups_alarm_states_pool[j];
        ctx->count      = ARRAY_SIZE(ups_alarm_table);
        ctx->read_fn    = alarm_read_register;
        ctx->event_fn   = forward_to_manager;
        ctx->read_data  = (void *)&ups1_profile;
        ctx->event_data = (void *)cfg;

        if (alarm_bridge_register_ctx(ctx) != 0) {
            LOG_ERROR("[Alarm] ups_alarm_register_all: "
                      "failed to register ctx for %s.", cfg->name);
            continue;
        }

        ups_alarm_ctx_count++;
        LOG_INFO("[Alarm] registered alarm context for %s.", cfg->name);
    }
}
