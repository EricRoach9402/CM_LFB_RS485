/**
 * @file ups_alarm.c
 * @brief UPS alarm registration.
 */

#include "ups_alarm.h"
#include "alarm_bridge.h"
#include "alarm_engine.h"
#include "alarm_manager.h"
#include "config_loader.h"
#include "device_register_map.h"
#include "log.h"
#include "ups/ups_map.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static bool alarm_read_register(uint16_t device_address,
                                uint16_t *out_value,
                                void *userdata);
static void forward_to_manager(const alarm_entry_t *entry,
                               uint16_t value,
                               alarm_event_t event,
                               void *userdata);

static const alarm_manager_sink_t ups_alarm_sink = {
    .log_dir = "/home/cm/LFB_Feeder_Kit/Logs/ups_alarm",
    .log_path = "/home/cm/LFB_Feeder_Kit/Logs/ups_alarm/alarms.db",
    .log_node = "ups_alarm_node",
};

static const alarm_entry_t ups_alarm_table[] = {
    { 0x0000, ALARM_COND_CHANGE, 0x0000, 0x0000, UPS_ERR_WARNING_1, "warning 1 value changed" },
    { 0x0001, ALARM_COND_CHANGE, 0x0000, 0x0000, UPS_ERR_WARNING_2, "warning 2 value changed" },
    { 0x0002, ALARM_COND_CHANGE, 0x0000, 0x0000, UPS_ERR_WARNING_3, "warning 3 value changed" },
    { 0x0003, ALARM_COND_CHANGE, 0x0000, 0x0000, UPS_ERR_WARNING_4, "warning 4 value changed" },
    { 0x00D0, ALARM_COND_CHANGE, 0x0000, 0x0000, UPS_MODE_INFORMATION, "UPS mode changed" },
    { 0x02A2, ALARM_COND_CHANGE, 0x0000, 0x0000, UPS_FAULT_INFORMATION, "UPS fault changed" },
};

static alarm_state_t ups_alarm_states_pool[MAX_UPS_COUNT][ARRAY_SIZE(ups_alarm_table)];

static alarm_engine_ctx_t ups_alarm_ctxs[MAX_UPS_COUNT];
static size_t ups_alarm_ctx_count = 0u;

/**
 * @brief Register alarm contexts for all enabled UPS units.
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

        ctx->table = ups_alarm_table;
        ctx->states = ups_alarm_states_pool[j];
        ctx->count = ARRAY_SIZE(ups_alarm_table);
        ctx->read_fn = alarm_read_register;
        ctx->event_fn = forward_to_manager;
        ctx->read_data = (void *)&ups1_profile;
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

/**
 * @brief Read one register from the UPS pool.
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
    alarm_manager_handle_event(entry, value, event, userdata, &ups_alarm_sink);
}
