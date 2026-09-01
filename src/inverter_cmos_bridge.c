/**
 * @file inverter_cmos_bridge.c
 * @brief CMOS bridge for Inverter (subscriber in parent, publisher in child).
 *
 * Parent must not call cmos_publish(); write to internal_pool[] instead.
 * Subscriber thread stops via pthread_cancel() (epoll_wait cancellation point).
 */

#include "inverter_cmos_bridge.h"
#include "config_loader.h"
#include "inverter_module.h"
#include "cmos.h"
#include "device_register_map.h"
#include "inverter/inverter_map.h"
#include "log.h"

#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

#define BRIDGE_MASTER_IP        "127.0.0.1"
#define BRIDGE_MASTER_PORT      10000
#define BRIDGE_PUB_PORT         13000
#define BRIDGE_NODE_NAME        "inverter_node_pub"
#define BRIDGE_SUB_TOPIC        "control_output"
#define BRIDGE_PUB_TOPIC        "inverter"
#define BRIDGE_PUB_POLL_US      500000u

static pthread_t             g_sub_thread;
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
static void publish_all_pool_register(const module_config_t *cfg);
static void publish_additional_item(const module_config_t *cfg);
static void pub_signal_handler(int sig);
static void publish_frequency_cmd_duty(const module_config_t *cfg);
static void publish_frequency_out_duty(const module_config_t *cfg);
static void publish_fault_warning_code(const module_config_t *cfg);

int inverter_cmos_bridge_start(void)
{
    if (pthread_create(&g_sub_thread, NULL, cmos_sub_thread, NULL) != 0) {
        LOG_ERROR("[CMOS Bridge] failed to start subscriber thread.");
        return -1;
    }

    LOG_INFO("[CMOS Bridge] subscriber started.");
    return 0;
}

void inverter_cmos_bridge_stop(void)
{
    pthread_cancel(g_sub_thread);
    pthread_join(g_sub_thread, NULL);

    LOG_INFO("[CMOS Bridge] subscriber stopped.");
}

void inverter_cmos_pub_run(void)
{
    struct sigaction sa = {
        .sa_handler = pub_signal_handler,
        .sa_flags   = 0,
    };
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (cmos_pub_init(BRIDGE_MASTER_IP, BRIDGE_MASTER_PORT,
                      BRIDGE_NODE_NAME, BRIDGE_PUB_TOPIC,
                      BRIDGE_PUB_PORT) != 0) {
        LOG_ERROR("[CMOS Bridge] publisher init failed "
                  "(master=%s:%d port=%d).",
                  BRIDGE_MASTER_IP, BRIDGE_MASTER_PORT, BRIDGE_PUB_PORT);
        return;
    }

    g_pub_running = 1;
    LOG_INFO("[CMOS Bridge] publisher process started, topic='%s' port=%d.",
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
    LOG_INFO("[CMOS Bridge] publisher process exiting.");
}

/**
 * @brief Resolve the Inverter unit that an incoming HMI command targets.
 *
 * Queries the module registry instead of a config array index, so commands
 * are dropped rather than sent to a uid that was never started.
 *
 * @return true if a running unit was found.
 */
static bool resolve_target_uid(uint8_t *out_uid)
{
    if (inverter_get_primary_uid(out_uid) != 0) {
        LOG_WARNING("[CMOS Bridge] no running Inverter unit; command dropped.");
        return false;
    }

    return true;
}

static int on_write(uint8_t uid, uint16_t addr, uint16_t val)
{
    const device_register_mapping_t *entry =
        device_find_slot(&inverter1_profile, addr);
    if (!entry) {
        LOG_WARNING("[CMOS Bridge] write: addr 0x%04X not mapped in profile "
                    "uid=%u", addr, uid);
        return -1;
    }

    if (entry->access == ACCESS_RO) {
        LOG_WARNING("[CMOS Bridge] write: addr 0x%04X is ACCESS_RO (uid=%u) "
                    "– rejected", addr, uid);
        return -1;
    }

    if (inverter_cmd_push(uid, addr, &val, 1, INVERTER_WRITE_MODE_AUTO) != 0) {
        LOG_ERROR("[CMOS Bridge] write: command queue full for uid=%u "
                  "addr=0x%04X", uid, addr);
        return -1;
    }

    LOG_INFO("[CMOS Bridge] write queued: uid=%u addr=0x%04X val=%u",
              uid, addr, val);
    return 0;
}

static void on_bypass_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter CMOS] SUB topic='%s' key='bypass' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t bypass_bool_val = (uint16_t)strtoul(value, NULL, 0);

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    if ( bypass_bool_val == 1 ) {
        on_write(target_uid, dev_fault_control_cmd_reg, INVERTER_FIRE_MODE_BIT);
        LOG_INFO("[CMOS Bridge] Received enable bypass command : %u" , bypass_bool_val);
    } else {
        on_write(target_uid, dev_fault_control_cmd_reg, INVERTER_EF_BIT);
        LOG_INFO("[CMOS Bridge] Received cancel bypass command : %u" , bypass_bool_val);
    }
}
static void on_init_inverter_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter CMOS] SUB topic='%s' key='init' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t init_bool_val = (uint16_t)strtoul(value, NULL, 0);

    if ( init_bool_val != 1 ) {
        LOG_WARNING("[CMOS Bridge] Received faulty init command : %u" , init_bool_val);
        return;
    }

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    inverter_init_request(target_uid);
    LOG_INFO("[CMOS Bridge] Received init command : %u" , init_bool_val);
}

static void on_main_pump_duty_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter CMOS] SUB topic='%s' key='main_pump' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t duty_val = (uint16_t)strtoul(value, NULL, 0);

    uint16_t frequency_val = duty_convert_frequency(duty_val);

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    if ( duty_val == 0 ) {
        uint16_t operation_cmd_val = INVERTER_STOP_BIT | INVERTER_ENABLE_ACCELERATION_BIT;
        on_write(target_uid, dev_frequency_cmd_reg, duty_val);
        on_write(target_uid, dev_operation_cmd_reg, operation_cmd_val);
        LOG_INFO("[CMOS Bridge] Received stop command : %u" , duty_val);
    } else {
        uint16_t operation_cmd_val = INVERTER_RUN_BIT | INVERTER_ENABLE_ACCELERATION_BIT;
        on_write(target_uid, dev_frequency_cmd_reg, frequency_val);
        on_write(target_uid, dev_operation_cmd_reg, operation_cmd_val);
        LOG_INFO("[CMOS Bridge] Received run command : %u" , frequency_val);
    }
}

static void on_frequency_write_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter CMOS] SUB topic='%s' key='inv_frequency_write_commands' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t frequency_val  = (uint16_t)strtoul(value, NULL, 0);

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    if ( frequency_val == 0 ) {
        uint16_t operation_cmd_val = INVERTER_STOP_BIT | INVERTER_ENABLE_ACCELERATION_BIT;
        on_write(target_uid, dev_frequency_cmd_reg, frequency_val);
        on_write(target_uid, dev_operation_cmd_reg, operation_cmd_val);
        LOG_INFO("[CMOS Bridge] Received stop command : %u" , frequency_val);
    } else {
        uint16_t operation_cmd_val = INVERTER_RUN_BIT | INVERTER_ENABLE_ACCELERATION_BIT;
        on_write(target_uid, dev_frequency_cmd_reg, frequency_val);
        on_write(target_uid, dev_operation_cmd_reg, operation_cmd_val);
        LOG_INFO("[CMOS Bridge] Received run command : %u" , frequency_val);
    }
}

static void on_operation_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter CMOS] SUB topic='%s' key='inv_operation_commands' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t val  = (uint16_t)strtoul(value, NULL, 0);

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    on_write(target_uid, dev_operation_cmd_reg, val);
    LOG_INFO("[CMOS Bridge] Operation commands received register: '%4x' value:'%s'",dev_operation_cmd_reg, value);
}

static void on_fault_control_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[Inverter CMOS] SUB topic='%s' key='inv_fault_Control_commands' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t addr = dev_fault_control_cmd_reg;
    uint16_t val  = (uint16_t)strtoul(value, NULL, 0);

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    on_write(target_uid, addr, val);
    LOG_INFO("[CMOS Bridge] Operation commands received register: '%4x' value:'%s'",addr, value);
}

static void cleanup_sub_ctx(void *arg)
{
    cmos_sub_ctx_t *ctx = (cmos_sub_ctx_t *)arg;
    cmos_sub_destroy(ctx);
    LOG_INFO("[CMOS Bridge] subscriber context destroyed.");
}

static void *cmos_sub_thread(void *arg)
{
    (void)arg;

    cmos_sub_ctx_t *ctx = cmos_sub_create(BRIDGE_MASTER_IP,
                                           BRIDGE_MASTER_PORT,
                                           "inverter_sub");
    if (!ctx) {
        LOG_ERROR("[CMOS Bridge] failed to create subscriber context.");
        return NULL;
    }

    pthread_cleanup_push(cleanup_sub_ctx, ctx);

    cmos_sub_add(ctx, "control_output", NULL, "command", "inv_operation_commands", on_operation_cmd);
    cmos_sub_add(ctx, "control_output", NULL, "command", "inv_frequency_write_commands", on_frequency_write_cmd);
    cmos_sub_add(ctx, "control_output", NULL, "command", "inv_fault_Control_commands", on_fault_control_cmd);
    cmos_sub_add(ctx, "control_output", NULL, "ctrl",    "main_pump", on_main_pump_duty_cmd);
    cmos_sub_add(ctx, "event",          NULL, "initial", "request_init_status", on_init_inverter_cmd);
    cmos_sub_add(ctx, "control_output", NULL, "ctrl",    "bypass", on_bypass_cmd);

    LOG_INFO("[CMOS Bridge] subscriber thread ready, "
             "topic='%s' (write + read).", BRIDGE_SUB_TOPIC);

    cmos_sub_spin_ctx(ctx);

    pthread_cleanup_pop(1);
    return NULL;
}

static void publish_pool_register(const module_config_t *cfg,
                                  const char *type,
                                  const char *key,
                                  uint16_t    pool_address)
{
    uint16_t val = 0;

    const char *state =
        (shared_connection_state_get(cfg) == CONNECTION_CONNECTED)
            ? "alive"
            : "disconnect";

    if (!pool_read_register(pool_address, &val)) {
        LOG_WARNING("[CMOS Bridge] publish_pool_register: "
                    "pool_address 0x%04X out of range.", pool_address);
        return;
    }

    char val_str[8];
    snprintf(val_str, sizeof(val_str), "%u", val);

    LOG_VERBOSE("[Inverter CMOS] PUB topic='%s' state='%s' type='%s' key='%s' value='%s'",
                BRIDGE_PUB_TOPIC,
                state ? state : NULL,
                type ? type : NULL,
                key ? key : NULL,
                val_str);

    cmos_publish(state, type, key, val_str);
}

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

static void pub_signal_handler(int sig)
{
    (void)sig;
    g_pub_running = 0;
}

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
    LOG_VERBOSE("[Inverter CMOS] PUB topic='%s' state='%s' type='%s' key='%s' value='%s'",
                BRIDGE_PUB_TOPIC,
                state ? state : NULL,
                NULL,
                "duty_command",
                val_str);
}

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
    LOG_VERBOSE("[Inverter CMOS] PUB topic='%s' state='%s' type='%s' key='%s' value='%s'",
                BRIDGE_PUB_TOPIC,
                state ? state : NULL,
                NULL,
                "output_duty",
                val_str);
}

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
    LOG_VERBOSE("[Inverter CMOS] PUB topic='%s' state='%s' type='%s' key='%s' value='%s'",
        BRIDGE_PUB_TOPIC,
        state ? state : NULL,
        NULL,
        "fault_code",
        fault_val_str);

    cmos_publish(state, NULL, "warning_code", warning_val_str);
    LOG_VERBOSE("[Inverter CMOS] PUB topic='%s' state='%s' type='%s' key='%s' value='%s'",
                BRIDGE_PUB_TOPIC,
                state ? state : NULL,
                NULL,
                "warning_code",
                warning_val_str);
}

