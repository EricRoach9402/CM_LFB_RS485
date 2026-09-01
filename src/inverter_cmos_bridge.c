/**
 * @file inverter_cmos_bridge.c
 * @brief Inverter CMOS subscriber and publisher.
 */

#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "cmos.h"
#include "config_loader.h"
#include "device_register_map.h"
#include "inverter/inverter_map.h"
#include "inverter_cmos_bridge.h"
#include "inverter_module.h"
#include "log.h"

#define BRIDGE_MASTER_IP "127.0.0.1"
#define BRIDGE_MASTER_PORT 10000
#define BRIDGE_PUB_PORT 13000
#define BRIDGE_NODE_NAME "inverter_node_pub"
#define BRIDGE_PUB_TOPIC "inverter"
#define BRIDGE_PUB_POLL_US 500000u

static pthread_t g_sub_thread;
static volatile sig_atomic_t g_pub_running = 0;

static bool resolve_target_uid(uint8_t *out_uid);
static int on_write(uint8_t uid, uint16_t addr, uint16_t val);
static void on_bypass_cmd(const char *topic, const char *value);
static void on_init_inverter_cmd(const char *topic, const char *value);
static void on_main_pump_duty_cmd(const char *topic, const char *value);
static void on_operation_cmd(const char *topic, const char *value);
static void on_frequency_write_cmd(const char *topic, const char *value);
static void on_fault_control_cmd(const char *topic, const char *value);
static void cleanup_sub_ctx(void *arg);
static void *cmos_sub_thread(void *arg);
static void publish_pool_register(const module_config_t *cfg, const char *type, const char *key, uint16_t pool_address);
static uint16_t duty_convert_frequency(uint16_t duty);
static uint16_t frequency_convert_duty(uint16_t frequency);
static void publish_all_pool_register(const module_config_t *cfg);
static void publish_additional_item(const module_config_t *cfg);
static void pub_signal_handler(int sig);
static void publish_frequency_cmd_duty(const module_config_t *cfg);
static void publish_frequency_out_duty(const module_config_t *cfg);
static void publish_fault_warning_code(const module_config_t *cfg);

/**
 * @brief Start the CMOS subscriber thread in the parent process.
 * @return 0 on success, -1 on failure.
 */
int inverter_cmos_bridge_start(void)
{
    if (pthread_create(&g_sub_thread, NULL, cmos_sub_thread, NULL) != 0) {
        LOG_ERROR("[Inverter] failed to start subscriber thread.");
        return -1;
    }

    LOG_INFO("[Inverter] subscriber started.");
    return 0;
}

/**
 * @brief Stop the CMOS subscriber thread.
 */
void inverter_cmos_bridge_stop(void)
{
    pthread_cancel(g_sub_thread);
    pthread_join(g_sub_thread, NULL);

    LOG_INFO("[Inverter] subscriber stopped.");
}

/**
 * @brief Publisher main loop for the Inverter child process.
 */
void inverter_cmos_pub_run(void)
{
    struct sigaction sa = {
        .sa_handler = pub_signal_handler,
        .sa_flags = 0,
    };
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (cmos_pub_init(BRIDGE_MASTER_IP, BRIDGE_MASTER_PORT,
                      BRIDGE_NODE_NAME, BRIDGE_PUB_TOPIC,
                      BRIDGE_PUB_PORT) != 0) {
        LOG_ERROR("[Inverter] publisher init failed "
                  "(master=%s:%d port=%d).",
                  BRIDGE_MASTER_IP, BRIDGE_MASTER_PORT, BRIDGE_PUB_PORT);
        return;
    }

    g_pub_running = 1;
    LOG_INFO("[Inverter] publisher process started, topic='%s' port=%d.",
             BRIDGE_PUB_TOPIC, BRIDGE_PUB_PORT);

    while (g_pub_running) {
        cmos_pub_poll();

        for (int i = 0; i < global_config.inverter_count; i++) {
            publish_all_pool_register(&global_config.inverter[i]);
            publish_additional_item(&global_config.inverter[i]);
        }

        usleep(BRIDGE_PUB_POLL_US);
    }

    cmos_pub_close();
    LOG_INFO("[Inverter] publisher process exiting.");
}

/**
 * @brief Resolve the running Inverter unit uid.
 * @param out_uid Output uid.
 * @return true if a unit is running.
 */
static bool resolve_target_uid(uint8_t *out_uid)
{
    if (inverter_get_primary_uid(out_uid) != 0) {
        LOG_WARNING("[Inverter] no running Inverter unit; command dropped.");
        return false;
    }

    return true;
}

/**
 * @brief Queue one register write to the inverter module.
 * @param uid Target modbus_uid.
 * @param addr Device register address.
 * @param val Register value.
 * @return 0 on success, -1 on failure.
 */
static int on_write(uint8_t uid, uint16_t addr, uint16_t val)
{
    const device_register_mapping_t *entry =
        device_find_slot(&inverter1_profile, addr);
    if (!entry) {
        LOG_WARNING("[Inverter] write: addr 0x%04X not mapped in profile "
                    "uid=%u", addr, uid);
        return -1;
    }

    if (entry->access == ACCESS_RO) {
        LOG_WARNING("[Inverter] write: addr 0x%04X is ACCESS_RO (uid=%u) "
                    "– rejected", addr, uid);
        return -1;
    }

    if (inverter_cmd_push(uid, addr, &val, 1, INVERTER_WRITE_MODE_AUTO) != 0) {
        return -1;
    }

    LOG_INFO("[Inverter] write queued: uid=%u addr=0x%04X val=%u",
              uid, addr, val);
    return 0;
}

/**
 * @brief Handle CMOS bypass command.
 * @param topic CMOS topic.
 * @param value Command value string.
 */
static void on_bypass_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter] SUB topic='%s' key='bypass' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t bypass_bool_val = (uint16_t)strtoul(value, NULL, 0);

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    if (bypass_bool_val == 1) {
        if (on_write(target_uid, dev_fault_control_cmd_reg,
                     INVERTER_FIRE_MODE_BIT) != 0) {
            return;
        }
        LOG_INFO("[Inverter] bypass: enable fire mode queued (uid=%u)",
                 target_uid);
    } else {
        if (on_write(target_uid, dev_fault_control_cmd_reg,
                     INVERTER_EF_BIT) != 0) {
            return;
        }
        LOG_INFO("[Inverter] bypass: cancel EF queued (uid=%u)", target_uid);
    }
}

/**
 * @brief Handle CMOS init command.
 * @param topic CMOS topic.
 * @param value Command value string.
 */
static void on_init_inverter_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter] SUB topic='%s' key='init' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t init_bool_val = (uint16_t)strtoul(value, NULL, 0);

    if (init_bool_val != 1) {
        LOG_WARNING("[Inverter] Received faulty init command: %u", init_bool_val);
        return;
    }

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    inverter_init_request(target_uid);
    LOG_INFO("[Inverter] Received init command: %u", init_bool_val);
}

/**
 * @brief Handle CMOS main pump duty command.
 * @param topic CMOS topic.
 * @param value Duty value string.
 */
static void on_main_pump_duty_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter] SUB topic='%s' key='main_pump' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t duty_val = (uint16_t)strtoul(value, NULL, 0);

    uint16_t frequency_val = duty_convert_frequency(duty_val);

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    if (duty_val == 0) {
        uint16_t operation_cmd_val = INVERTER_STOP_BIT | INVERTER_ENABLE_ACCELERATION_BIT;

        if (on_write(target_uid, dev_frequency_cmd_reg, duty_val) != 0) {
            return;
        }
        if (on_write(target_uid, dev_operation_cmd_reg, operation_cmd_val) != 0) {
            return;
        }
        LOG_INFO("[Inverter] main_pump: stop queued (duty=%u)", duty_val);
    } else {
        uint16_t operation_cmd_val = INVERTER_RUN_BIT | INVERTER_ENABLE_ACCELERATION_BIT;

        if (on_write(target_uid, dev_frequency_cmd_reg, frequency_val) != 0) {
            return;
        }
        if (on_write(target_uid, dev_operation_cmd_reg, operation_cmd_val) != 0) {
            return;
        }
        LOG_INFO("[Inverter] main_pump: run queued (duty=%u freq=%u)",
                 duty_val, frequency_val);
    }
}

/**
 * @brief Handle CMOS frequency write command.
 * @param topic CMOS topic.
 * @param value Frequency value string.
 */
static void on_frequency_write_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter] SUB topic='%s' key='inv_frequency_write_commands' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t frequency_val = (uint16_t)strtoul(value, NULL, 0);

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    if (frequency_val == 0) {
        uint16_t operation_cmd_val = INVERTER_STOP_BIT | INVERTER_ENABLE_ACCELERATION_BIT;

        if (on_write(target_uid, dev_frequency_cmd_reg, frequency_val) != 0) {
            return;
        }
        if (on_write(target_uid, dev_operation_cmd_reg, operation_cmd_val) != 0) {
            return;
        }
        LOG_INFO("[Inverter] frequency_write: stop queued (freq=%u)",
                 frequency_val);
    } else {
        uint16_t operation_cmd_val = INVERTER_RUN_BIT | INVERTER_ENABLE_ACCELERATION_BIT;

        if (on_write(target_uid, dev_frequency_cmd_reg, frequency_val) != 0) {
            return;
        }
        if (on_write(target_uid, dev_operation_cmd_reg, operation_cmd_val) != 0) {
            return;
        }
        LOG_INFO("[Inverter] frequency_write: run queued (freq=%u)",
                 frequency_val);
    }
}

/**
 * @brief Handle CMOS operation command.
 * @param topic CMOS topic.
 * @param value Command value string.
 */
static void on_operation_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter] SUB topic='%s' key='inv_operation_commands' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t val = (uint16_t)strtoul(value, NULL, 0);

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    if (on_write(target_uid, dev_operation_cmd_reg, val) != 0) {
        return;
    }

    LOG_INFO("[Inverter] operation_cmd: queued addr=0x%04X val=0x%04X",
             dev_operation_cmd_reg, val);
}

/**
 * @brief Handle CMOS fault control command.
 * @param topic CMOS topic.
 * @param value Command value string.
 */
static void on_fault_control_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter] SUB topic='%s' key='inv_fault_Control_commands' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t addr = dev_fault_control_cmd_reg;
    uint16_t val = (uint16_t)strtoul(value, NULL, 0);

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    if (on_write(target_uid, addr, val) != 0) {
        return;
    }

    LOG_INFO("[Inverter] fault_control_cmd: queued addr=0x%04X val=0x%04X",
             addr, val);
}

/**
 * @brief pthread_cleanup handler for subscriber context.
 * @param arg cmos_sub_ctx_t pointer.
 */
static void cleanup_sub_ctx(void *arg)
{
    cmos_sub_ctx_t *ctx = (cmos_sub_ctx_t *)arg;
    cmos_sub_destroy(ctx);
    LOG_INFO("[Inverter] subscriber context destroyed.");
}

/**
 * @brief CMOS subscriber thread entry point.
 * @param arg Unused.
 * @return NULL.
 */
static void *cmos_sub_thread(void *arg)
{
    (void)arg;

    cmos_sub_ctx_t *ctx = cmos_sub_create(BRIDGE_MASTER_IP,
                                           BRIDGE_MASTER_PORT,
                                           "inverter_sub");
    if (!ctx) {
        LOG_ERROR("[Inverter] failed to create subscriber context.");
        return NULL;
    }

    pthread_cleanup_push(cleanup_sub_ctx, ctx);

    cmos_sub_add(ctx, "control_output", NULL, "command", "inv_operation_commands", on_operation_cmd);
    cmos_sub_add(ctx, "control_output", NULL, "command", "inv_frequency_write_commands", on_frequency_write_cmd);
    cmos_sub_add(ctx, "control_output", NULL, "command", "inv_fault_Control_commands", on_fault_control_cmd);
    cmos_sub_add(ctx, "control_output", NULL, "ctrl", "main_pump", on_main_pump_duty_cmd);
    cmos_sub_add(ctx, "event", NULL, "initial", "request_init_status", on_init_inverter_cmd);
    cmos_sub_add(ctx, "control_output", NULL, "ctrl", "bypass", on_bypass_cmd);

    LOG_INFO("[Inverter] subscriber ready "
             "(topics: control_output, event).");

    cmos_sub_spin_ctx(ctx);

    pthread_cleanup_pop(1);
    return NULL;
}

/**
 * @brief Publish one pool register to CMOS.
 * @param cfg Module configuration.
 * @param type CMOS type field.
 * @param key CMOS key field.
 * @param pool_address Internal pool address.
 */
static void publish_pool_register(const module_config_t *cfg,
                                  const char *type,
                                  const char *key,
                                  uint16_t pool_address)
{
    uint16_t val = 0;

    const char *state =
        (shared_connection_state_get(cfg) == CONNECTION_CONNECTED)
            ? "alive"
            : "disconnect";

    if (!pool_read_register(pool_address, &val)) {
        LOG_WARNING("[Inverter] publish_pool_register: "
                    "pool_address 0x%04X out of range.", pool_address);
        return;
    }

    char val_str[8];
    snprintf(val_str, sizeof(val_str), "%u", val);

    LOG_VERBOSE("[Inverter] PUB topic='%s' state='%s' type='%s' key='%s' value='%s'",
                BRIDGE_PUB_TOPIC,
                state ? state : NULL,
                type ? type : NULL,
                key ? key : NULL,
                val_str);

    cmos_publish(state, type, key, val_str);
}

/**
 * @brief Publish all mapped registers for one unit.
 * @param cfg Module configuration.
 */
static void publish_all_pool_register(const module_config_t *cfg)
{
    if (!cfg || !cfg->enabled) {
        return;
    }

    for (size_t i = 0; i < inverter1_profile.table_count; i++) {
        publish_pool_register(cfg, NULL,
                              inverter1_profile.table[i].description,
                              inverter1_profile.table[i].pool_address);
    }
}

/**
 * @brief Publish derived and init-flag items for one unit.
 * @param cfg Module configuration.
 */
static void publish_additional_item(const module_config_t *cfg)
{
    if (!cfg || !cfg->enabled) {
        return;
    }

    publish_frequency_cmd_duty(cfg);
    publish_frequency_out_duty(cfg);
    publish_fault_warning_code(cfg);
    publish_pool_register(cfg, NULL, "initial_flag", int_inverter_init_flag_reg);
}

/**
 * @brief Stop publisher loop on signal.
 * @param sig Signal number.
 */
static void pub_signal_handler(int sig)
{
    (void)sig;
    g_pub_running = 0;
}

/**
 * @brief Convert duty percent to frequency register value.
 * @param duty Duty percent.
 * @return Frequency register value.
 */
static uint16_t duty_convert_frequency(uint16_t duty)
{
    uint16_t upper = 0;
    uint16_t lower = 0;

    pool_read_register(int_frequency_upper_limit_reg, &upper);
    pool_read_register(int_frequency_lower_limit_reg, &lower);

    if (upper <= lower) {
        return lower;
    }

    if (duty >= 100) {
        return upper;
    }

    return (uint16_t)(lower
                      + ((uint32_t)(upper - lower) * duty) / 100u);
}

/**
 * @brief Convert frequency register value to duty percent.
 * @param frequency Frequency register value.
 * @return Duty percent.
 */
static uint16_t frequency_convert_duty(uint16_t frequency)
{
    uint16_t upper = 0;
    uint16_t lower = 0;

    pool_read_register(int_frequency_upper_limit_reg, &upper);
    pool_read_register(int_frequency_lower_limit_reg, &lower);

    if (upper <= lower) {
        return 0;
    }

    if (frequency <= lower) {
        return 0;
    }

    if (frequency >= upper) {
        return 100;
    }

    uint32_t range = (uint32_t)(upper - lower);
    uint32_t delta = (uint32_t)(frequency - lower);
    return (uint16_t)((delta * 100u + range / 2u) / range);
}

/**
 * @brief Publish duty derived from frequency command.
 * @param cfg Module configuration.
 */
static void publish_frequency_cmd_duty(const module_config_t *cfg)
{
    uint16_t frequency_cmd = 0;

    pool_read_register(int_frequency_write_cmd_reg, &frequency_cmd);
    uint16_t duty_frequency_cmd = frequency_convert_duty(frequency_cmd);
    const char *state =
        (shared_connection_state_get(cfg) == CONNECTION_CONNECTED)
            ? "alive"
            : "disconnect";

    char val_str[8];
    snprintf(val_str, sizeof(val_str), "%u", duty_frequency_cmd);

    cmos_publish(state, NULL, "duty_command", val_str);
    LOG_VERBOSE("[Inverter] PUB topic='%s' state='%s' type='%s' key='%s' value='%s'",
                BRIDGE_PUB_TOPIC,
                state ? state : NULL,
                NULL,
                "duty_command",
                val_str);
}

/**
 * @brief Publish duty derived from output frequency.
 * @param cfg Module configuration.
 */
static void publish_frequency_out_duty(const module_config_t *cfg)
{
    uint16_t frequency_out = 0;

    pool_read_register(int_out_frequency_reg, &frequency_out);

    uint16_t duty_frequency_out = frequency_convert_duty(frequency_out);

    const char *state =
        (shared_connection_state_get(cfg) == CONNECTION_CONNECTED)
            ? "alive"
            : "disconnect";

    char val_str[8];
    snprintf(val_str, sizeof(val_str), "%u", duty_frequency_out);

    cmos_publish(state, NULL, "output_duty", val_str);
    LOG_VERBOSE("[Inverter] PUB topic='%s' state='%s' type='%s' key='%s' value='%s'",
                BRIDGE_PUB_TOPIC,
                state ? state : NULL,
                NULL,
                "output_duty",
                val_str);
}

/**
 * @brief Publish fault and warning code fields.
 * @param cfg Module configuration.
 */
static void publish_fault_warning_code(const module_config_t *cfg)
{
    uint16_t raw_value = 0;
    pool_read_register(int_fault_warning_code_reg, &raw_value);

    uint16_t fault_code_val = raw_value & 0xFF;
    uint16_t warning_code_val = (raw_value >> 8) & 0xFF;

    const char *state =
        (shared_connection_state_get(cfg) == CONNECTION_CONNECTED)
            ? "alive"
            : "disconnect";
    char fault_val_str[8];
    char warning_val_str[8];
    snprintf(fault_val_str, sizeof(fault_val_str), "%u", fault_code_val);
    snprintf(warning_val_str, sizeof(warning_val_str), "%u", warning_code_val);

    cmos_publish(state, NULL, "fault_code", fault_val_str);
    LOG_VERBOSE("[Inverter] PUB topic='%s' state='%s' type='%s' key='%s' value='%s'",
                BRIDGE_PUB_TOPIC,
                state ? state : NULL,
                NULL,
                "fault_code",
                fault_val_str);

    cmos_publish(state, NULL, "warning_code", warning_val_str);
    LOG_VERBOSE("[Inverter] PUB topic='%s' state='%s' type='%s' key='%s' value='%s'",
                BRIDGE_PUB_TOPIC,
                state ? state : NULL,
                NULL,
                "warning_code",
                warning_val_str);
}

