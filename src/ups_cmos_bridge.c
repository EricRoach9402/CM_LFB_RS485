/**
 * @file ups_cmos_bridge.c
 * @brief CMOS ↔ UPS Modbus bridge.
 *
 * Parent process (ups_cmos_bridge_start):
 *  - cmos_sub_thread – blocking CMOS subscriber for HMI write commands.
 *
 * Child process (ups_cmos_pub_run, started from main):
 *  - Periodic publisher on BRIDGE_PUB_TOPIC; reads internal_pool[] and
 *    shared_connection_state_get() for Alive/Disconnect.
 *
 * cmos_sub_spin_ctx() cannot be stopped gracefully (it loops on while(1)).
 * pthread_cancel() is used; epoll_wait() is a POSIX cancellation point so the
 * thread exits cleanly, and the cleanup handler calls cmos_sub_destroy().
 */

#include "ups_cmos_bridge.h"
#include "ups_module.h"
#include "cmos.h"
#include "device_register_map.h"
#include "ups/ups_map.h"
#include "log.h"

#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

/* ── CMOS connection constants ────────────────────────────────────────── */
#define BRIDGE_MASTER_IP        "127.0.0.1"
#define BRIDGE_MASTER_PORT      10000
#define BRIDGE_PUB_PORT         12000       /* listen port for read responses  */
#define BRIDGE_NODE_NAME        "ups_node"  /* node name for master registration */
#define BRIDGE_SUB_HMI_TOPIC    "hmi_ups"   /* topic for write/read requests   */
#define BRIDGE_PUB_TOPIC        "ups"       /* topic for read responses        */
#define BRIDGE_PUB_POLL_US      500000u     /* 500 ms poll interval             */

/*
 * This project currently only wires up UPS#1, so this bridge always
 * targets the first configured UPS unit.  The uid itself is never
 * hardcoded – it is read straight from config.json at call time, so
 * re-wiring UPS#1 to a different modbus_uid (e.g. to share a bus with
 * another device) needs no source change.
 */
#define UPS_BRIDGE_DEFAULT_UID  ((uint8_t)global_config.ups[0].modbus_uid)

/* ── Module-level state ───────────────────────────────────────────────── */
static pthread_t             g_sub_thread;
static volatile sig_atomic_t g_pub_running = 0;

/* ── Static Function Prototypes ─────────────────────────────────────────── */
static int on_write(uint8_t uid, uint16_t addr, uint16_t val);
static void on_test_cmd(const char *topic, const char *value);
static void cleanup_sub_ctx(void *arg);
static void *cmos_sub_thread(void *arg);
static void publish_pool_register(const module_config_t *cfg,
                                    const char *type,
                                    const char *key,
                                    uint16_t    pool_address);
static void publish_all_pool_register(const module_config_t *cfg);
static void pub_signal_handler(int sig);

/* ── Public API ───────────────────────────────────────────────────────── */

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

/**
 * @brief CMOS publisher main loop for the UPS child process.
 *
 * Registers with the CMOS master, accepts subscribers, and publishes pool
 * values on BRIDGE_PUB_POLL_US intervals.  Exits on SIGTERM or SIGINT.
 */
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
        }

        usleep(BRIDGE_PUB_POLL_US);
    }

    cmos_pub_close();
    LOG_INFO("[CMOS Bridge] publisher process exiting.");
}

/* ── Register access core (shared by every on_xxx_cmd handler) ───────── */

/**
 * @brief Validate and enqueue a single-register write.
 *
 * Checks that addr is mapped in ups1_profile and not ACCESS_RO, then
 * pushes the write command via ups_cmd_push().
 *
 * Every on_xxx_cmd handler should call this instead of repeating the
 * validate/enqueue sequence itself.  Must stay non-blocking.
 *
 * @return 0 on success, -1 on validation failure or queue-full.
 */
static int on_write(uint8_t uid, uint16_t addr, uint16_t val)
{
    const device_register_mapping_t *entry =
        device_find_slot(&ups1_profile, addr);
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

    if (ups_cmd_push(uid, addr, &val, 1, UPS_WRITE_MODE_AUTO) != 0) {
        LOG_ERROR("[CMOS Bridge] write: command queue full for uid=%u "
                  "addr=0x%04X", uid, addr);
        return -1;
    }

    LOG_INFO("[CMOS Bridge] write queued: uid=%u addr=0x%04X val=%u",
              uid, addr, val);
    return 0;
}

/* ── CMOS callbacks ───────────────────────────────────────────────────── */

static void on_test_cmd(const char *topic, const char *value)
{
    LOG_VERBOSE("[UPS CMOS] SUB topic='%s' key='ups_test' value='%s'",
                topic ? topic : NULL, value ? value : "");

    uint16_t addr = 0x0012;
    uint16_t val  = (uint16_t)strtoul(value, NULL, 0);

    LOG_INFO("[CMOS Bridge] test command received: '%s'", value);

    on_write(UPS_BRIDGE_DEFAULT_UID, addr, val);
}
/* ── Thread cleanup handler ───────────────────────────────────────────── */

/**
 * @brief pthread cleanup handler – destroys the subscriber context on cancel.
 */
static void cleanup_sub_ctx(void *arg)
{
    cmos_sub_ctx_t *ctx = (cmos_sub_ctx_t *)arg;
    cmos_sub_destroy(ctx);
    LOG_INFO("[CMOS Bridge] subscriber context destroyed.");
}

/* ── Thread functions ─────────────────────────────────────────────────── */

/**
 * @brief CMOS subscriber thread.
 *
 * Subscribes to BRIDGE_SUB_HMI_TOPIC once per HMI command (type="command",
 * key=<cmd name>), each routed to its own on_xxx_cmd handler, then enters
 * cmos_sub_spin_ctx() which blocks indefinitely.  The thread is stopped via
 * pthread_cancel(); epoll_wait() inside spin_ctx is a cancellation point so
 * the thread exits cleanly.
 */
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

    /*
    cmos_sub_add(ctx, BRIDGE_SUB_HMI_TOPIC, NULL, "command", "ups_test",  on_test_cmd);
    cmos_sub_add(ctx, BRIDGE_SUB_HMI_TOPIC, NULL, "command", "ups_shutdown",  on_shutdown_cmd);
    cmos_sub_add(ctx, BRIDGE_SUB_HMI_TOPIC, NULL, "command", "on_time",  on_time_cmd);
    cmos_sub_add(ctx, BRIDGE_SUB_HMI_TOPIC, NULL, "command", "off_time",  on_off_time_cmd);
    cmos_sub_add(ctx, BRIDGE_SUB_HMI_TOPIC, NULL, "command", "trigger",  on_trigger_cmd);
    */

    LOG_INFO("[CMOS Bridge] subscriber thread ready, "
             "topic='%s' (write + read).", BRIDGE_SUB_HMI_TOPIC);

    cmos_sub_spin_ctx(ctx);  /* blocks – exited only by pthread_cancel() */

    pthread_cleanup_pop(1);
    return NULL;
}

/**
 * @brief Read one pool register and publish its value to BRIDGE_PUB_TOPIC.
 *
 * Converts the cached uint16_t pool value to a decimal string and forwards
 * it to cmos_publish().  Called from ups_cmos_pub_run().
 *
 * @param type  CMOS message type field.
 * @param key   CMOS message key field (e.g. register address string).
 * @param pool_address  Absolute index into internal_pool[].
 */
static void publish_pool_register(const module_config_t *cfg,
                                  const char *type,
                                  const char *key,
                                  uint16_t    pool_address)
{
    uint16_t val = 0;

    const char *state =
        (shared_connection_state_get(cfg) == CONNECTION_CONNECTED)
            ? "Alive"
            : "Disconnect";

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

/**
 * @brief Publish every mapped register of one unit's profile to CMOS.
 *
 * Drives the periodic status push directly from the unit's
 * device_map_profile_t table (see devices/ups/ups_map.c) instead of a
 * hand-written per-register list: for every row, table[i].description is
 * used as the CMOS key and table[i].pool_address selects the value.
 *
 * @param cfg  Module configuration for the unit being published (only
 *             used for its name and Alive/Disconnect connection_state;
 *             the register set is ups1_profile, the only UPS hardware
 *             model currently implemented).
 */
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
/**
 * @brief SIGTERM/SIGINT handler for the publisher child process.
 */
static void pub_signal_handler(int sig)
{
    (void)sig;
    g_pub_running = 0;
}