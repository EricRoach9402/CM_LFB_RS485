/**
 * @file ups_module.c
 * @brief UPS Modbus polling – RTU or TCP per config (one thread per unit).
 *
 * Uses ups1_profile only. Init sequence and int_ups_init_flag_reg are owned
 * here; the CMOS bridge calls ups_init_request().
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>

#include "ups_module.h"
#include "ups_cmos_bridge.h"
#include "device_register_map.h"
#include "bus_coord.h"
#include "modbus_tcp_client.h"
#include "modbus_rtu_client.h"
#include "ups/ups_map.h"
#include "log.h"

/* ── Tunable constants ────────────────────────────────────────────────── */
#define UPS_RECONNECT_DELAY_MS         5000u
#define UPS_COMM_FAIL_THRESHOLD        5
#define UPS_INTER_SEGMENT_DELAY_US     40000u   /* 40 ms between FC03 frames */
#define UPS_POST_WRITE_SETTLE_US       70000u   /* hold bus after writes before Inverter may TX */
#define UPS_SHUTDOWN_CHECK_INTERVAL_MS 100u     /* granularity for interruptible sleeps */

/* ── Command queue constants ──────────────────────────────────────────── */
#define UPS_CMD_QUEUE_CAPACITY         16u

/* ── Write command entry ──────────────────────────────────────────────── */

typedef struct {
    uint16_t addr;
    uint16_t values[MODBUS_MAX_WRITE_REGISTERS];
    uint16_t count;
    ups_write_mode_t mode;
} ups_write_cmd_t;

/* ── Per-unit command queue ───────────────────────────────────────────── */
typedef struct {
    ups_write_cmd_t entries[UPS_CMD_QUEUE_CAPACITY];
    unsigned int    head;
    unsigned int    tail;
    unsigned int    count;
    pthread_mutex_t lock;
} ups_cmd_queue_t;

/* ── Per-unit runtime state ───────────────────────────────────────────── */
typedef struct {
    module_config_t            *cfg;
    const device_map_profile_t *profile;
    mb_tcp_client_ctx_t         tcp_ctx;   /* used when cfg->format == MODBUS_FORMAT_TCP */
    mb_rtu_client_ctx_t         rtu_ctx;   /* used when cfg->format == MODBUS_FORMAT_RTU */
    module_callbacks_t          callbacks;
    pthread_t                   thread;
    volatile sig_atomic_t       running;
    int                         comm_fail_count;
    ups_cmd_queue_t             cmd_queue;
    atomic_bool                 init_requested;  /**< set by any caller, consumed by the polling thread */
} ups_unit_t;

static ups_unit_t  ups_units[MAX_UPS_COUNT];
static int         ups_unit_count = 0;

static int read_profile_to_pool(ups_unit_t *unit, bool track_comm_fail);
static void run_init_sequence(ups_unit_t *unit);

/* ── Queue helpers (static) ───────────────────────────────────────────── */

static void queue_init(ups_cmd_queue_t *q)
{
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->lock, NULL);
}

static void queue_destroy(ups_cmd_queue_t *q)
{
    pthread_mutex_destroy(&q->lock);
}

/**
 * @brief Push one write command onto the tail of the queue.
 *
 * Merge-allowed addresses (see ups_queue_merge_allowed) update an
 * existing pending single-register entry instead of consuming another slot.
 *
 * @return 0 on success, -1 if the queue is full or count is out of range.
 */
static int queue_push(ups_cmd_queue_t *q, uint16_t addr,
                      const uint16_t *values, uint16_t count,
                      ups_write_mode_t mode)
{
    if (!values || count == 0u || count > MODBUS_MAX_WRITE_REGISTERS) {
        return -1;
    }

    pthread_mutex_lock(&q->lock);

    if (count == 1u && ups_queue_merge_allowed(addr)) {
        unsigned int idx = q->head;
        for (unsigned int i = 0u; i < q->count; i++) {
            ups_write_cmd_t *entry = &q->entries[idx];

            if (entry->addr == addr && entry->count == 1u) {
                entry->mode = mode;
                entry->values[0] = values[0];
                pthread_mutex_unlock(&q->lock);
                return 0;
            }

            idx = (idx + 1u) % UPS_CMD_QUEUE_CAPACITY;
        }
    }

    if (q->count >= UPS_CMD_QUEUE_CAPACITY) {
        pthread_mutex_unlock(&q->lock);
        return -1;
    }

    ups_write_cmd_t *entry = &q->entries[q->tail];
    entry->addr  = addr;
    entry->count = count;
    entry->mode  = mode;
    memcpy(entry->values, values, count * sizeof(uint16_t));

    q->tail  = (q->tail + 1u) % UPS_CMD_QUEUE_CAPACITY;
    q->count++;

    pthread_mutex_unlock(&q->lock);
    return 0;
}

/**
 * @brief Pop one write command from the head of the queue.
 * @return 0 on success, -1 if the queue is empty.
 */
static int queue_pop(ups_cmd_queue_t *q, ups_write_cmd_t *out)
{
    pthread_mutex_lock(&q->lock);

    if (q->count == 0) {
        pthread_mutex_unlock(&q->lock);
        return -1;
    }

    *out    = q->entries[q->head];
    q->head = (q->head + 1u) % UPS_CMD_QUEUE_CAPACITY;
    q->count--;

    pthread_mutex_unlock(&q->lock);
    return 0;
}

/* ── Internal helpers ─────────────────────────────────────────────────── */

static ups_unit_t *ups_unit_from_config(const module_config_t *cfg)
{
    for (int i = 0; i < ups_unit_count; i++) {
        if (ups_units[i].cfg == cfg) {
            return &ups_units[i];
        }
    }
    return NULL;
}

static ups_unit_t *ups_unit_from_uid(uint8_t uid)
{
    for (int i = 0; i < ups_unit_count; i++) {
        if (ups_units[i].cfg &&
            (uint8_t)ups_units[i].cfg->modbus_uid == uid) {
            return &ups_units[i];
        }
    }
    return NULL;
}

/**
 * @brief Sleep for duration_ms, waking early if the unit is signalled to stop.
 */
static void interruptible_sleep_ms(const ups_unit_t *unit, uint32_t duration_ms)
{
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < duration_ms && unit->running) {
        usleep(UPS_SHUTDOWN_CHECK_INTERVAL_MS * 1000u);
        elapsed_ms += UPS_SHUTDOWN_CHECK_INTERVAL_MS;
    }
}

/* ── Transport dispatch (TCP / RTU) ───────────────────────────────────── *
 *
 * These are the only functions that know about mb_tcp_client_* versus
 * mb_rtu_client_*.  Every other function in this file (callbacks, queue
 * drain, segmented scan) is transport-neutral and simply calls into this
 * section based on unit->cfg->format.
 */

/**
 * @brief Serial path for shared-bus coordination, or NULL when not RTU.
 */
static const char *ups_bus_path(const ups_unit_t *unit)
{
    if (!unit || !unit->cfg) {
        return NULL;
    }
    if (unit->cfg->format != MODBUS_FORMAT_RTU) {
        return NULL;
    }
    return unit->cfg->path;
}

/**
 * @brief Log tag reflecting the active transport (TCP vs RTU).
 */
static const char *ups_log_tag(const ups_unit_t *unit)
{
    if (unit && unit->cfg && unit->cfg->format == MODBUS_FORMAT_TCP) {
        return "UPS TCP";
    }
    return "UPS RTU";
}

/**
 * @brief Connect the unit's transport (TCP socket or RTU serial port).
 *
 * RTU additionally probes the device with one FC03 read after opening the
 * serial port, since a successful open only proves the local device node
 * exists – it says nothing about whether a slave is actually present on
 * the bus.  TCP does not need this: a successful connect() already proves
 * a peer accepted the socket.
 *
 * @return 0 on success, -1 on failure.
 */
static int unit_connect(ups_unit_t *unit)
{
    module_config_t *cfg = unit->cfg;

    if (cfg->format == MODBUS_FORMAT_TCP) {
        mb_tcp_client_config_t tcp_cfg = {
            .remote_host          = cfg->ip,
            .port                 = (cfg->port > 0) ? (uint16_t)cfg->port : 502u,
            .unit_id              = (uint8_t)cfg->modbus_uid,
            .connect_timeout_sec  = 5,
            .response_timeout_ms  = 1000,
            .logv                 = NULL,
            .log_userdata         = NULL,
        };

        LOG_INFO("[UPS] %s: connecting to %s:%d uid=%d",
                 cfg->name, cfg->ip, cfg->port, cfg->modbus_uid);

        if (mb_tcp_client_connect(&unit->tcp_ctx, &tcp_cfg) != 0) {
            LOG_ERROR("[UPS] %s: TCP connection failed.", cfg->name);
            return -1;
        }
        return 0;
    }

    mb_rtu_client_config_t rtu_cfg = {
        .serial_path          = cfg->path,
        .baud_rate            = (uint32_t)cfg->baud_rate,
        .unit_id              = (uint8_t)cfg->modbus_uid,
        .response_timeout_ms  = 1000u,
    };

    LOG_INFO("[UPS] %s: opening %s @ %u baud uid=%d",
             cfg->name, cfg->path, cfg->baud_rate, cfg->modbus_uid);

    if (mb_rtu_client_connect(&unit->rtu_ctx, &rtu_cfg) != 0) {
        LOG_ERROR("[UPS] %s: failed to open serial port %s.",
                  cfg->name, cfg->path);
        return -1;
    }

    if (unit->profile->table_count > 0) {
        uint16_t    probe_addr = unit->profile->table[0].device_address;
        const char *path       = ups_bus_path(unit);

        bus_coord_acquire(path);
        int probe_rc = mb_rtu_client_probe_device(&unit->rtu_ctx,
                                                   probe_addr, 1);
        bus_coord_release(path);

        if (probe_rc != MB_RTU_CLIENT_OK) {
            LOG_ERROR("[UPS] %s: device not responding on %s (uid=%d).",
                      cfg->name, cfg->path, cfg->modbus_uid);
            mb_rtu_client_disconnect(&unit->rtu_ctx);
            return -1;
        }
    }

    return 0;
}

/**
 * @brief disconnect the unit's transport (TCP socket or RTU serial port).
 */
static void unit_disconnect(ups_unit_t *unit)
{
    if (unit->cfg->format == MODBUS_FORMAT_TCP) {
        mb_tcp_client_disconnect(&unit->tcp_ctx);
    } else {
        mb_rtu_client_disconnect(&unit->rtu_ctx);
    }
}

/**
 * @brief FC03 – read holding registers via whichever transport is active.
 *
 * @return 0 on success, a positive Modbus exception code, or a negative
 *         transport-specific error code (MB_TCP_CLIENT_ERR_* / MB_RTU_CLIENT_ERR_*).
 */
static int unit_read_holding_registers(ups_unit_t *unit, uint16_t addr,
                                       uint16_t count, uint16_t *out)
{
    if (unit->cfg->format == MODBUS_FORMAT_TCP) {
        return mb_tcp_client_read_holding_registers(&unit->tcp_ctx, addr, count, out);
    }

    const char *path = ups_bus_path(unit);
    int         result;

    bus_coord_acquire(path);
    result = mb_rtu_client_read_holding_registers(&unit->rtu_ctx, addr, count, out);
    bus_coord_release(path);
    return result;
}

/**
 * @brief Execute a Modbus write without acquiring bus_coord.
 *
 * "locked" means the caller must already hold the bus (queue drain) or
 * use write_registers_to_device() which wraps this with acquire / settle /
 * release.  Dispatches to TCP or RTU depending on unit->cfg->format.
 * On failure, logs and returns -1 (does NOT increment comm_fail_count –
 * a write failure is an operational error, not a connectivity loss).
 *
 * @return 0 on success, -1 on failure.
 */
static int write_registers_locked(ups_unit_t *unit,
                                  uint16_t    addr,
                                  const uint16_t *values,
                                  uint16_t    count,
                                  ups_write_mode_t mode)
{
    bool is_tcp = (unit->cfg->format == MODBUS_FORMAT_TCP);
    int  result;

    switch (mode) {
    case UPS_WRITE_MODE_FC06:
        if (count != 1) {
            LOG_ERROR("[%s] %s: FC06 requires count=1 (got %u) at 0x%04X.",
                      ups_log_tag(unit), unit->cfg->name, count, addr);
            return -1;
        }
        result = is_tcp
                 ? mb_tcp_client_write_single_register(&unit->tcp_ctx, addr, values[0])
                 : mb_rtu_client_write_single_register(&unit->rtu_ctx, addr, values[0]);
        break;
    case UPS_WRITE_MODE_FC16:
        result = is_tcp
                 ? mb_tcp_client_write_multiple_registers(&unit->tcp_ctx, addr, count, values)
                 : mb_rtu_client_write_multiple_registers(&unit->rtu_ctx, addr, count, values);
        break;
    case UPS_WRITE_MODE_AUTO:
    default:
        if (count == 1) {
            result = is_tcp
                     ? mb_tcp_client_write_single_register(&unit->tcp_ctx, addr, values[0])
                     : mb_rtu_client_write_single_register(&unit->rtu_ctx, addr, values[0]);
        } else {
            result = is_tcp
                     ? mb_tcp_client_write_multiple_registers(&unit->tcp_ctx, addr, count, values)
                     : mb_rtu_client_write_multiple_registers(&unit->rtu_ctx, addr, count, values);
        }
        break;
    }

    /* MB_TCP_CLIENT_OK and MB_RTU_CLIENT_OK are both 0. */
    if (result != 0) {
        LOG_ERROR("[%s] %s: write to 0x%04X failed (err %d).",
                  ups_log_tag(unit), unit->cfg->name, addr, result);
        return -1;
    }

    if (count == 1) {
        LOG_VERBOSE("[%s] %s: wrote register 0x%04X value=0x%04X.",
                    ups_log_tag(unit), unit->cfg->name, addr, values[0]);
    } else {
        char val_buf[160];
        size_t pos = 0;

        val_buf[0] = '\0';
        for (uint16_t i = 0; i < count; i++) {
            int n = snprintf(val_buf + pos, sizeof(val_buf) - pos,
                             "%s0x%04X", (i == 0) ? "" : " ", values[i]);
            if (n < 0 || (size_t)n >= sizeof(val_buf) - pos) {
                break;
            }
            pos += (size_t)n;
        }

        LOG_VERBOSE("[%s] %s: wrote %u register(s) at 0x%04X values=[%s].",
                    ups_log_tag(unit), unit->cfg->name, count, addr, val_buf);
    }
    return 0;
}

/**
 * @brief Execute a Modbus write with bus ownership and post-write settle.
 *
 * Used by msg_callback (single-shot path).  Queue drain holds the bus
 * itself and calls write_registers_locked() instead.  For TCP,
 * ups_bus_path() is NULL so acquire/release are no-ops.
 */
static int write_registers_to_device(ups_unit_t *unit,
                                     uint16_t    addr,
                                     const uint16_t *values,
                                     uint16_t    count,
                                     ups_write_mode_t mode)
{
    const char *path = ups_bus_path(unit);

    bus_coord_acquire(path);
    int result = write_registers_locked(unit, addr, values, count, mode);
    if (result == 0) {
        usleep(UPS_POST_WRITE_SETTLE_US);
    }
    bus_coord_release(path);
    return result;
}

/* ── Callbacks ────────────────────────────────────────────────────────── */

/**
 * @brief init_callback – connect the unit's transport (TCP or RTU).
 *
 * @param cfg  Module configuration.
 * @return 0 on success, -1 on failure.
 */
int ups_init_callback(module_config_t *cfg)
{
    if (!cfg) {
        LOG_ERROR("[UPS] Invalid configuration.");
        return -1;
    }

    ups_unit_t *unit = ups_unit_from_config(cfg);
    if (!unit) {
        LOG_ERROR("[UPS] %s: unit not found.", cfg->name);
        return -1;
    }

    if (unit_connect(unit) != 0) {
        return -1;
    }

    unit->comm_fail_count = 0;
    cfg->connection_state = CONNECTION_CONNECTED;
    shared_connection_state_set(cfg, CONNECTION_CONNECTED);

    LOG_INFO("[UPS] %s: connected.", cfg->name);
    return 0;
}

/**
 * @brief process_callback – drain write queue, then read all mapped registers.
 *
 * Phase 0: run a pending init request (full register refresh).
 *
 * Phase 1: drain the per-unit write queue under one bus_coord hold so a
 *          co-bus module cannot interleave until post-write settle completes.
 *          A write failure is logged but does not abort the read phase.
 *
 * Phase 2: consecutive device addresses are batched into one FC03 request.
 *          On any read failure the fail counter increments; after
 *          UPS_COMM_FAIL_THRESHOLD consecutive failures the callback returns
 *          -1 to trigger error_callback.
 *
 * @param cfg  Module configuration.
 * @return 0 on success, -1 on communication failure.
 */
int ups_process_callback(module_config_t *cfg)
{
    if (!cfg) {
        return -1;
    }

    ups_unit_t *unit = ups_unit_from_config(cfg);
    if (!unit) {
        return -1;
    }

    /* ── Phase 0: run a pending init request ────────────────────────── */
    if (atomic_exchange(&unit->init_requested, false)) {
        run_init_sequence(unit);
    }

    /* ── Phase 1: drain the write command queue ─────────────────────── */
    ups_write_cmd_t cmd;
    if (queue_pop(&unit->cmd_queue, &cmd) == 0) {
        const char *path = ups_bus_path(unit);

        bus_coord_acquire(path);

        do {
            if (write_registers_locked(unit, cmd.addr, cmd.values,
                                       cmd.count, cmd.mode) != 0) {
                LOG_WARNING("[%s] %s: queued write to 0x%04X failed, "
                            "continuing scan.",
                            ups_log_tag(unit), cfg->name, cmd.addr);
            } else {
                usleep(UPS_INTER_SEGMENT_DELAY_US);
            }
        } while (queue_pop(&unit->cmd_queue, &cmd) == 0);

        usleep(UPS_POST_WRITE_SETTLE_US);
        bus_coord_release(path);
    }

    /* ── Phase 2: segmented FC03 read scan ──────────────────────────── */
    if (read_profile_to_pool(unit, true) != 0) {
        return -1;
    }

    usleep((useconds_t)(cfg->rtu_poll_interval_ms * 1000u));
    return 0;
}

/**
 * @brief Read all mapped registers into the pool.
 *
 * @param unit             Target UPS unit.
 * @param track_comm_fail  When true, increment comm_fail_count on failure and
 *                         blank the unit's pool on threshold; when false (init
 *                         path), a single failure is returned without affecting
 *                         the connection state.
 * @return 0 on success, -1 on read failure.
 */
static int read_profile_to_pool(ups_unit_t *unit, bool track_comm_fail)
{
    module_config_t *cfg = unit->cfg;
    const device_map_profile_t *profile = unit->profile;

    if (profile->table_count == 0) {
        return 0;
    }

    uint16_t buf[MODBUS_MAX_READ_REGISTERS] = {0};

    size_t seg_start = 0;
    size_t seg_len   = 1;

    for (size_t i = 1; i <= profile->table_count; i++) {

        bool contiguous = (i < profile->table_count) &&
                          (profile->table[i].device_address ==
                           profile->table[i - 1].device_address + 1);
        bool chunk_full = (seg_len >= profile->read_chunk);

        if (contiguous && !chunk_full) {
            seg_len++;
            continue;
        }

        uint16_t start = profile->table[seg_start].device_address;
        uint16_t count = (uint16_t)seg_len;

        int result = unit_read_holding_registers(unit, start, count, buf);

        if (result != 0) {
            if (!track_comm_fail) {
                LOG_WARNING("[%s] %s init read 0x%04X len %u failed (err %d).",
                            ups_log_tag(unit), cfg->name, start, count, result);
                return -1;
            }

            unit->comm_fail_count++;
            LOG_WARNING("[%s] %s read 0x%04X len %u failed (err %d, fail %d/%d)",
                        ups_log_tag(unit), cfg->name, start, count, result,
                        unit->comm_fail_count, UPS_COMM_FAIL_THRESHOLD);

            if (unit->comm_fail_count >= UPS_COMM_FAIL_THRESHOLD) {
                for (size_t k = 0; k < profile->table_count; k++) {
                    pool_write_register(profile->table[k].pool_address, 0xFFFF);
                }
                return -1;
            }
        } else {
            if (track_comm_fail) {
                unit->comm_fail_count = 0;
            }
            device_map_read_to_pool(profile, buf, start, (int)count);
        }

        usleep(UPS_INTER_SEGMENT_DELAY_US);

        seg_start = i;
        seg_len   = 1;
    }

    return 0;
}

/**
 * @brief Run the operator-initiated init sequence and record the result.
 *
 * Refreshes all mapped registers from the device and owns int_ups_init_flag_reg
 * end to end: cleared on entry so a repeated request starts from 0, set to 1
 * only once every segment read succeeds.  The CMOS bridge just publishes that
 * slot; it never needs to know what init does.
 */
static void run_init_sequence(ups_unit_t *unit)
{
    pool_write_register(int_ups_init_flag_reg, 0);

    if (read_profile_to_pool(unit, false) != 0) {
        LOG_ERROR("[%s] %s: init sequence failed; init flag stays 0.",
                  ups_log_tag(unit), unit->cfg->name);
        return;
    }

    pool_write_register(int_ups_init_flag_reg, 1);
    LOG_INFO("[%s] %s: init sequence complete.",
             ups_log_tag(unit), unit->cfg->name);
}

/**
 * @brief error_callback – disconnect and wait before the next reconnect attempt.
 *
 * @param cfg               Module configuration.
 * @param connection_state  New connection state (CONNECTION_DISCONNECTED).
 * @return 0 (reconnect is managed by the thread loop).
 */
int ups_error_callback(module_config_t *cfg, int connection_state)
{
    if (!cfg) {
        return 0;
    }

    ups_unit_t *unit = ups_unit_from_config(cfg);
    if (unit) {
        unit_disconnect(unit);
        unit->comm_fail_count = 0;
    }

    cfg->connection_state = (connection_state_t)connection_state;
    shared_connection_state_set(cfg, (connection_state_t)connection_state);

    LOG_WARNING("[UPS] %s: disconnected (state=%d). Retrying in %u ms …",
                cfg->name, connection_state, UPS_RECONNECT_DELAY_MS);

    if (unit) {
        interruptible_sleep_ms(unit, UPS_RECONNECT_DELAY_MS);
    } else {
        usleep(UPS_RECONNECT_DELAY_MS * 1000u);
    }
    return 0;
}

/**
 * @brief msg_callback – execute a Modbus write on a UPS unit.
 *
 * Intended as a synchronous write path via the module_callbacks_t framework.
 * In this module, all writes are routed through ups_cmd_push() → queue drain
 * inside process_callback; msg_callback is registered but never invoked.
 *
 * Retained for framework completeness.  If a direct synchronous write path
 * is needed in the future, call this via unit->callbacks.msg_callback().
 *
 * @param cfg    Module configuration of the target UPS.
 * @param addr   Device register address to write.
 * @param values Values to write (host byte order).
 * @param count  Number of registers (1 → FC06, >1 → FC16).
 * @return 0 on success, -1 on failure.
 */
int ups_msg_callback(module_config_t *cfg,
                     uint16_t addr, uint16_t *values, size_t count)
{
    if (!cfg || !values || count == 0) {
        return -1;
    }

    ups_unit_t *unit = ups_unit_from_config(cfg);
    if (!unit) {
        LOG_ERROR("[UPS] %s: msg_callback – unit not found.", cfg->name);
        return -1;
    }

    return write_registers_to_device(unit, addr, values, (uint16_t)count, UPS_WRITE_MODE_AUTO);
}

/* ── Thread function ──────────────────────────────────────────────────── */

/**
 * @brief Poll thread for one UPS unit.
 *
 *  connect → segment reads → write pool → wait for next poll interval
 *          → on failure: disconnect → wait → reconnect
 */
static void *ups_thread(void *arg)
{
    ups_unit_t *unit = (ups_unit_t *)arg;
    module_config_t *cfg  = unit->cfg;

    LOG_INFO("[UPS] Thread started: %s (profile=%s uid=%d)",
             cfg->name, unit->profile->name, cfg->modbus_uid);

    while (unit->running) {

        /* init --------------------------------------------------------- */
        if (unit->callbacks.init_callback(cfg) != 0) {
            LOG_ERROR("[UPS] %s: init failed, will retry.", cfg->name);
            interruptible_sleep_ms(unit, UPS_RECONNECT_DELAY_MS);
            continue;
        }

        /* process loop ------------------------------------------------- */
        while (unit->running) {
            if (unit->callbacks.process_callback(cfg) != 0) {
                if (unit->callbacks.error_callback) {
                    unit->callbacks.error_callback(cfg, CONNECTION_DISCONNECTED);
                }
                break; /* break inner loop → re-init */
            }
        }
    }

    unit_disconnect(unit);
    LOG_INFO("[UPS] Thread stopped: %s", cfg->name);
    return NULL;
}

/* ── Public API ───────────────────────────────────────────────────────── */

/**
 * @brief Start all enabled UPS modules.
 *
 * Assigns ups1_profile (the only UPS hardware model currently
 * implemented) and callbacks to every enabled unit, spawns one polling
 * thread per unit, then starts the CMOS bridge.
 *
 * @param ups       Array of UPS configurations from global_config.
 * @param ups_count Number of entries in the array.
 * @return 0 on success, -1 if any unit fails to start.
 */
int start_ups_modules(module_config_t ups[], int ups_count)
{
    int status = 0;

    ups_unit_count = 0;

    for (int i = 0; i < ups_count; i++) {
        if (!ups[i].enabled) {
            LOG_INFO("[UPS] %s is disabled. Skipping.", ups[i].name);
            continue;
        }

        const device_map_profile_t *profile = &ups1_profile;

        if (device_register_map_register_profile(profile) != 0) {
            LOG_ERROR("[UPS] %s: profile '%s' rejected due to a "
                      "pool_address collision with another device's "
                      "profile; unit will not be started.",
                      ups[i].name, profile->name);
            status = -1;
            continue;
        }

        ups_unit_t *unit      = &ups_units[ups_unit_count];
        unit->cfg             = &ups[i];
        unit->profile         = profile;
        unit->running         = 1;
        unit->comm_fail_count = 0;
        memset(&unit->tcp_ctx, 0, sizeof(unit->tcp_ctx));
        memset(&unit->rtu_ctx, 0, sizeof(unit->rtu_ctx));
        unit->rtu_ctx.fd = -1;
        queue_init(&unit->cmd_queue);
        atomic_init(&unit->init_requested, false);

        unit->callbacks.init_callback    = ups_init_callback;
        unit->callbacks.process_callback = ups_process_callback;
        unit->callbacks.error_callback   = ups_error_callback;
        unit->callbacks.msg_callback     = ups_msg_callback;
        unit->callbacks.start_callback   = NULL;

        ups_unit_count++;

        LOG_INFO("[UPS] Starting %s (profile=%s uid=%d format=%s).",
                 unit->cfg->name,
                 unit->profile->name,
                 unit->cfg->modbus_uid,
                 (unit->cfg->format == MODBUS_FORMAT_TCP) ? "TCP" : "RTU");

        if (pthread_create(&unit->thread, NULL, ups_thread, unit) != 0) {
            LOG_ERROR("[UPS] Failed to create thread for %s.", unit->cfg->name);
            queue_destroy(&unit->cmd_queue);
            unit->running = 0;
            ups_unit_count--;
            status = -1;
        }
    }

    if (ups_cmos_bridge_start() != 0) {
        LOG_ERROR("[UPS] CMOS bridge failed to start.");
        status = -1;
    }

    return status;
}

/**
 * @brief Stop all running UPS modules and join their threads.
 *
 * Stops the CMOS bridge first, then signals all unit threads to exit and
 * waits for them.
 *
 * @return 0 on success.
 */
int stop_ups_modules(void)
{
    ups_cmos_bridge_stop();

    for (int i = 0; i < ups_unit_count; i++) {
        ups_units[i].running = 0;
    }

    for (int i = 0; i < ups_unit_count; i++) {
        pthread_join(ups_units[i].thread, NULL);
        queue_destroy(&ups_units[i].cmd_queue);
        LOG_INFO("[UPS] %s stopped.", ups_units[i].cfg->name);
    }

    ups_unit_count = 0;
    return 0;
}

/**
 * @brief Resolve the modbus_uid of the first running UPS unit.
 *
 * Read-only access to the registry filled by start_ups_modules(); the CMOS
 * bridge thread is the only caller and runs after registration.
 *
 * @return 0 on success, -1 if no unit is running.
 */
int ups_get_primary_uid(uint8_t *out_uid)
{
    if (!out_uid) {
        return -1;
    }

    for (int i = 0; i < ups_unit_count; i++) {
        if (!ups_units[i].cfg) {
            continue;
        }

        *out_uid = (uint8_t)ups_units[i].cfg->modbus_uid;
        return 0;
    }

    return -1;
}

/**
 * @brief Enqueue a write command for the UPS unit identified by uid.
 *
 * Called from the CMOS bridge thread.  Thread-safe via per-unit queue mutex.
 *
 * @return 0 on success, -1 if unit not found or queue is full.
 */
int ups_cmd_push(uint8_t uid, uint16_t addr,
                 const uint16_t *values, uint16_t count,
                 ups_write_mode_t mode)
{
    if (!values || count == 0 || count > MODBUS_MAX_WRITE_REGISTERS) {
        return -1;
    }

    ups_unit_t *unit = ups_unit_from_uid(uid);
    if (!unit) {
        LOG_WARNING("[UPS] ups_cmd_push: uid=%u not found.", uid);
        return -1;
    }

    return queue_push(&unit->cmd_queue, addr, values, count, mode);
}

/**
 * @brief Mark the init sequence as requested for one unit.
 *
 * Non-blocking; the sequence runs on the unit's polling thread.
 *
 * @return 0 on success, -1 if the unit is not found.
 */
int ups_init_request(uint8_t uid)
{
    ups_unit_t *unit = ups_unit_from_uid(uid);
    if (!unit) {
        LOG_WARNING("[UPS] ups_init_request: uid=%u not found.", uid);
        return -1;
    }

    atomic_store(&unit->init_requested, true);

    LOG_INFO("[UPS] init sequence requested for uid=%u.", uid);
    return 0;
}
