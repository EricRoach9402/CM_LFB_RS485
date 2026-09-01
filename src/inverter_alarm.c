/**
 * @file inverter_alarm.c
 * @brief Inverter alarm registration.
 */

#include "inverter_alarm.h"
#include "alarm_bridge.h"
#include "alarm_engine.h"
#include "alarm_manager.h"
#include "config_loader.h"
#include "device_register_map.h"
#include "inverter/inverter_map.h"
#include "log.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static bool alarm_read_register(uint16_t device_address,
                                uint16_t *out_value,
                                void *userdata);
static void forward_to_manager(const alarm_entry_t *entry,
                               uint16_t value,
                               alarm_event_t event,
                               void *userdata);

static const alarm_manager_sink_t inverter_alarm_sink = {
    .log_dir = "/home/cm/LFB_Feeder_Kit/Logs/inverter_alarm",
    .log_path = "/home/cm/LFB_Feeder_Kit/Logs/inverter_alarm/alarms.db",
    .log_node = "inverter_alarm_node",
};

static const alarm_entry_t inverter_alarm_table[] = {
    { 0x2100, ALARM_COND_CHANGE, 0x0000, 0x0000, INV_ERR_FAULT_CODE, "fault code changed" },
};

static alarm_state_t inverter_alarm_states_pool[MAX_INVERTER_COUNT][ARRAY_SIZE(inverter_alarm_table)];

static alarm_engine_ctx_t inverter_alarm_ctxs[MAX_INVERTER_COUNT];
static size_t inverter_alarm_ctx_count = 0u;

/**
 * @brief Register alarm contexts for all enabled Inverter units.
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

        ctx->table = inverter_alarm_table;
        ctx->states = inverter_alarm_states_pool[j];
        ctx->count = ARRAY_SIZE(inverter_alarm_table);
        ctx->read_fn = alarm_read_register;
        ctx->event_fn = forward_to_manager;
        ctx->read_data = (void *)&inverter1_profile;
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

/**
 * @brief Read one register from the inverter pool.
 * @param device_address Device register address.
 * @param out_value Output value.
 * @param userdata device_map_profile_t pointer.
 * @return true on success.
 */
static bool alarm_read_register(uint16_t device_address,
                                uint16_t *out_value,
                                void *userdata)
{
    const device_map_profile_t *profile = (const device_map_profile_t *)userdata;
    return pool_read_by_device_addr(profile, device_address, out_value);
}

/**
 * @brief Forward one alarm event to alarm_manager.
 * @param entry Alarm table entry.
 * @param value Register value.
 * @param event Alarm event type.
 * @param userdata Caller context.
 */
static void forward_to_manager(const alarm_entry_t *entry,
                               uint16_t value,
                               alarm_event_t event,
                               void *userdata)
{
    alarm_manager_handle_event(entry, value, event, userdata,
                               &inverter_alarm_sink);
}
