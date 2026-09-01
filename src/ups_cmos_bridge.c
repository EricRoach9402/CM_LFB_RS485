/**
 * @file ups_cmos_bridge.c
 * @brief CMOS bridge for UPS (subscriber in parent, publisher in child).
 *
 * Parent must not call cmos_publish(); write to internal_pool[] instead.
 * Subscriber thread stops via pthread_cancel() (epoll_wait cancellation point).
 */

#include "ups_cmos_bridge.h"
#include "ups_module.h"
#include "cmos.h"
#include "device_register_map.h"
#include "ups/ups_map.h"
#include "log.h"

#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

#define BRIDGE_MASTER_IP        "127.0.0.1"
#define BRIDGE_MASTER_PORT      10000
#define BRIDGE_PUB_PORT         12000
#define BRIDGE_NODE_NAME        "ups_node"
#define BRIDGE_SUB_HMI_TOPIC    "hmi_ups"
#define BRIDGE_PUB_TOPIC        "ups"
#define BRIDGE_PUB_POLL_US      500000u

static pthread_t             g_sub_thread;
static volatile sig_atomic_t g_pub_running = 0;

static bool resolve_target_uid(uint8_t *out_uid);
static void on_init_ups_cmd(const char *topic, const char *value);
static void cleanup_sub_ctx(void *arg);
static void *cmos_sub_thread(void *arg);
static void publish_pool_register(const module_config_t *cfg,
                                    const char *type,
                                    const char *key,
                                    uint16_t    pool_address);
static void publish_all_pool_register(const module_config_t *cfg);
static void publish_additional_item(const module_config_t *cfg);
static void pub_signal_handler(int sig);

int ups_cmos_bridge_start(void)
{
    if (pthread_create(&g_sub_thread, NULL, cmos_sub_thread, NULL) != 0) {
        LOG_ERROR("[CMOS Bridge] failed to start subscriber thread.");
        return -1;
    }

    LOG_INFO("[CMOS Bridge] subscriber started.");
    return 0;
}

void ups_cmos_bridge_stop(void)
{
    pthread_cancel(g_sub_thread);
    pthread_join(g_sub_thread, NULL);

    LOG_INFO("[CMOS Bridge] subscriber stopped.");
}

void ups_cmos_pub_run(void)
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

        for (int i = 0; i < global_config.ups_count; i++) {
            publish_all_pool_register(&global_config.ups[i]);
            publish_additional_item(&global_config.ups[i]);
        }

        usleep(BRIDGE_PUB_POLL_US);
    }

    cmos_pub_close();
    LOG_INFO("[CMOS Bridge] publisher process exiting.");
}

/**
 * @brief Resolve the UPS unit that an incoming HMI command targets.
 *
 * Queries the module registry instead of a config array index, so commands
 * are dropped rather than sent to a uid that was never started.
 *
 * @return true if a running unit was found.
 */
static bool resolve_target_uid(uint8_t *out_uid)
{
    if (ups_get_primary_uid(out_uid) != 0) {
        LOG_WARNING("[CMOS Bridge] no running UPS unit; command dropped.");
        return false;
    }

    return true;
}

static void on_init_ups_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[UPS CMOS] SUB topic='%s' key='init' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t init_bool_val = (uint16_t)strtoul(value, NULL, 0);

    if (init_bool_val != 1) {
        LOG_WARNING("[CMOS Bridge] Received faulty init command : %u", init_bool_val);
        return;
    }

    uint8_t target_uid = 0;
    if (!resolve_target_uid(&target_uid)) {
        return;
    }

    ups_init_request(target_uid);
    LOG_INFO("[CMOS Bridge] Received init command : %u", init_bool_val);
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
                                           "ups_sub");
    if (!ctx) {
        LOG_ERROR("[CMOS Bridge] failed to create subscriber context.");
        return NULL;
    }

    pthread_cleanup_push(cleanup_sub_ctx, ctx);

    cmos_sub_add(ctx, "event", NULL, "initial", "request_init_status", on_init_ups_cmd);

    LOG_INFO("[CMOS Bridge] subscriber thread ready, "
             "topic='%s' (write + read).", BRIDGE_SUB_HMI_TOPIC);

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

    LOG_VERBOSE("[UPS CMOS] PUB topic='%s' state='%s' type='%s' key='%s' value='%s'",
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

    for (size_t i = 0; i < ups1_profile.table_count; i++) {
        publish_pool_register(cfg, NULL,
                              ups1_profile.table[i].description,
                              ups1_profile.table[i].pool_address);
    }
}

static void publish_additional_item(const module_config_t *cfg)
{
    if (!cfg || !cfg->enabled) {
        return;
    }

    publish_pool_register(cfg, NULL, "initial_flag", int_ups_init_flag_reg);
}

static void pub_signal_handler(int sig)
{
    (void)sig;
    g_pub_running = 0;
}
