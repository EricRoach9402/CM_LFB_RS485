/**
 * @file inverter_module.c
 * @brief Inverter Modbus RTU polling (one thread per enabled unit).
 *
 * Uses inverter1_profile only. Init sequence and int_inverter_init_flag_reg
 * are owned here; the CMOS bridge calls inverter_init_request().
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>

#include "inverter_module.h"
#include "inverter_cmos_bridge.h"
#include "device_register_map.h"
#include "bus_coord.h"
#include "modbus_rtu_client.h"
#include "inverter/inverter_map.h"
#include "log.h"

/* ── Tunable constants ────────────────────────────────────────────────── */
#define INVERTER_RECONNECT_DELAY_MS         5000u
#define INVERTER_COMM_FAIL_THRESHOLD        5
#define INVERTER_INTER_SEGMENT_DELAY_US     40000u   /* 40 ms between FC03 frames */
#define INVERTER_POST_WRITE_SETTLE_US       70000u   /* hold bus after writes before UPS may TX */
#define INVERTER_SHUTDOWN_CHECK_INTERVAL_MS 100u     /* granularity for interruptible sleeps */

/* ── Command queue constants ──────────────────────────────────────────── */
#define INVERTER_CMD_QUEUE_CAPACITY         16u

/* ── Write command entry ──────────────────────────────────────────────── */

typedef struct {
    uint16_t              addr;
    uint16_t              values[MODBUS_MAX_WRITE_REGISTERS];
    uint16_t              count;
    inverter_write_mode_t mode;
} inverter_write_cmd_t;

/* ── Per-unit command queue ───────────────────────────────────────────── */
typedef struct {
    inverter_write_cmd_t entries[INVERTER_CMD_QUEUE_CAPACITY];
    unsigned int         head;
    unsigned int         tail;
    unsigned int         count;
    pthread_mutex_t      lock;
} inverter_cmd_queue_t;

/* ── Per-unit runtime state ───────────────────────────────────────────── */
typedef struct {
    module_config_t            *cfg;
    const device_map_profile_t *profile;
    mb_rtu_client_ctx_t         rtu_ctx;
    module_callbacks_t          callbacks;
    pthread_t                   thread;
    volatile sig_atomic_t       running;
    int                         comm_fail_count;
    inverter_cmd_queue_t        cmd_queue;
    atomic_bool                 init_requested;  /**< set by any caller, consumed by the polling thread */
} inverter_unit_t;

/* ── Static Function Prototypes ─────────────────────────────────────────── */
static int inverter_rtu_init_callback(module_config_t *cfg);
static int inverter_rtu_process_callback(module_config_t *cfg);
static int inverter_rtu_error_callback(module_config_t *cfg, int connection_state);
static int inverter_msg_callback(module_config_t *cfg, uint16_t addr, uint16_t *values, size_t count);
static void queue_init(inverter_cmd_queue_t *q);
static void queue_destroy(inverter_cmd_queue_t *q);
static int queue_push(inverter_cmd_queue_t *q, uint16_t addr, const uint16_t *values, uint16_t count, inverter_write_mode_t mode);
static int queue_pop(inverter_cmd_queue_t *q, inverter_write_cmd_t *out);
static void run_init_sequence(inverter_unit_t *unit);
static inverter_unit_t *inverter_unit_from_config(const module_config_t *cfg);
static inverter_unit_t *inverter_unit_from_uid(uint8_t uid);
static void interruptible_sleep_ms(const inverter_unit_t *unit, uint32_t duration_ms);
static int write_registers_locked(inverter_unit_t *unit, uint16_t addr, const uint16_t *values, uint16_t count, inverter_write_mode_t mode);
static int write_registers_to_device(inverter_unit_t *unit, uint16_t addr, const uint16_t *values, uint16_t count, inverter_write_mode_t mode);
static bool inverter_baud_convert(uint32_t host_baud, uint16_t *out_reg);
static void init_inverter_reg(inverter_unit_t *unit);
static void *inverter_rtu_thread(void *arg);

static inverter_unit_t  inverter_units[MAX_INVERTER_COUNT];
static int              inverter_unit_count = 0;

/* ── Public API ───────────────────────────────────────────────────────── */

/**
 * @brief Start all enabled Inverter modules.
 *
 * Assigns inverter1_profile (the only Inverter hardware model currently
 * implemented) and callbacks to every enabled unit, spawns one polling
 * thread per unit, then starts the CMOS bridge.
 *
 * @param inverters       Array of Inverter configurations from global_config.
 * @param inverter_count  Number of entries in the array.
 * @return 0 on success, -1 if any unit fails to start.
 */
int start_inverter_modules(module_config_t inverters[], int inverter_count)
{
    int status = 0;

    inverter_unit_count = 0;

    for (int i = 0; i < inverter_count; i++) {
        if (!inverters[i].enabled) {
            LOG_INFO("[Inverter] %s is disabled. Skipping.", inverters[i].name);
            continue;
        }

        const device_map_profile_t *profile = &inverter1_profile;

        if (device_register_map_register_profile(profile) != 0) {
            LOG_ERROR("[Inverter] %s: profile '%s' rejected due to a "
                      "pool_address collision with another device's "
                      "profile; unit will not be started.",
                      inverters[i].name, profile->name);
            status = -1;
            continue;
        }

        inverter_unit_t *unit      = &inverter_units[inverter_unit_count];
        unit->cfg                  = &inverters[i];
        unit->profile              = profile;
        unit->running              = 1;
        unit->comm_fail_count      = 0;
        memset(&unit->rtu_ctx, 0, sizeof(unit->rtu_ctx));
        unit->rtu_ctx.fd = -1;
        queue_init(&unit->cmd_queue);
        atomic_init(&unit->init_requested, false);

        unit->callbacks.init_callback    = inverter_rtu_init_callback;
        unit->callbacks.process_callback = inverter_rtu_process_callback;
        unit->callbacks.error_callback   = inverter_rtu_error_callback;
        unit->callbacks.msg_callback     = inverter_msg_callback;
        unit->callbacks.start_callback   = NULL;

        inverter_unit_count++;

        LOG_INFO("[Inverter] Starting %s (profile=%s uid=%d).",
                 unit->cfg->name,
                 unit->profile->name,
                 unit->cfg->modbus_uid);

        if (pthread_create(&unit->thread, NULL,
                           inverter_rtu_thread, unit) != 0) {
            LOG_ERROR("[Inverter] Failed to create thread for %s.",
                      unit->cfg->name);
            queue_destroy(&unit->cmd_queue);
            unit->running = 0;
            inverter_unit_count--;
            status = -1;
        }
    }

    if (inverter_cmos_bridge_start() != 0) {
        LOG_ERROR("[Inverter] CMOS bridge failed to start.");
        status = -1;
    }

    return status;
}

/**
 * @brief Stop all running Inverter modules and join their threads.
 *
 * Stops the CMOS bridge first, then signals all unit threads to exit and
 * waits for them.
 *
 * @return 0 on success.
 */
int stop_inverter_modules(void)
{
    inverter_cmos_bridge_stop();

    for (int i = 0; i < inverter_unit_count; i++) {
        inverter_units[i].running = 0;
    }

    for (int i = 0; i < inverter_unit_count; i++) {
        pthread_join(inverter_units[i].thread, NULL);
        queue_destroy(&inverter_units[i].cmd_queue);
        LOG_INFO("[Inverter] %s stopped.", inverter_units[i].cfg->name);
    }

    inverter_unit_count = 0;
    return 0;
}

/**
 * @brief Resolve the modbus_uid of the first running Inverter unit.
 *
 * Read-only access to the registry filled by start_inverter_modules(); the
 * CMOS bridge thread is the only caller and runs after registration.
 *
 * @return 0 on success, -1 if no unit is running.
 */
int inverter_get_primary_uid(uint8_t *out_uid)
{
    if (!out_uid) {
        return -1;
    }

    for (int i = 0; i < inverter_unit_count; i++) {
        if (!inverter_units[i].cfg) {
            continue;
        }

        *out_uid = (uint8_t)inverter_units[i].cfg->modbus_uid;
        return 0;
    }

    return -1;
}

/**
 * @brief Enqueue a write command for the Inverter unit identified by uid.
 *
 * Called from the CMOS bridge thread.  Thread-safe via per-unit queue mutex.
 *
 * @return 0 on success, -1 if unit not found or queue is full.
 */
int inverter_cmd_push(uint8_t uid, uint16_t addr,
                      const uint16_t *values, uint16_t count,
                      inverter_write_mode_t mode)
{
    if (!values || count == 0 || count > MODBUS_MAX_WRITE_REGISTERS) {
        return -1;
    }

    inverter_unit_t *unit = inverter_unit_from_uid(uid);
    if (!unit) {
        LOG_WARNING("[Inverter] inverter_cmd_push: uid=%u not found.", uid);
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
int inverter_init_request(uint8_t uid)
{
    inverter_unit_t *unit = inverter_unit_from_uid(uid);
    if (!unit) {
        LOG_WARNING("[Inverter] inverter_init_request: uid=%u not found.", uid);
        return -1;
    }

    atomic_store(&unit->init_requested, true);

    LOG_INFO("[Inverter] init sequence requested for uid=%u.", uid);
    return 0;
}

/* ── Callbacks ────────────────────────────────────────────────────────── */

/**
 * @brief init_callback – open and configure the serial port.
 *
 * @param cfg  Module configuration.
 * @return 0 on success, -1 on failure.
 */
static int inverter_rtu_init_callback(module_config_t *cfg)
{
    if (!cfg) {
        LOG_ERROR("[Inverter RTU] Invalid configuration.");
        return -1;
    }

    inverter_unit_t *unit = inverter_unit_from_config(cfg);
    if (!unit) {
        LOG_ERROR("[Inverter RTU] %s: unit not found.", cfg->name);
        return -1;
    }

    mb_rtu_client_config_t rtu_cfg = {
        .serial_path          = cfg->path,
        .baud_rate            = (uint32_t)cfg->baud_rate,
        .unit_id              = (uint8_t)cfg->modbus_uid,
        .response_timeout_ms  = 1000u,
    };

    LOG_INFO("[Inverter RTU] %s: opening %s @ %u baud uid=%d",
             cfg->name, cfg->path, cfg->baud_rate, cfg->modbus_uid);

    if (mb_rtu_client_connect(&unit->rtu_ctx, &rtu_cfg) != 0) {
        LOG_ERROR("[Inverter RTU] %s: failed to open serial port %s.",
                  cfg->name, cfg->path);
        return -1;
    }

    /*
     * Hold the shared RS-485 path across probe + one-time register
     * programming so another co-bus module cannot interleave frames.
     */
    bus_coord_acquire(cfg->path);

    /*
     * Opening the serial port only proves the local device node exists; it
     * says nothing about whether an Inverter slave is actually present on
     * the bus.  Probe a known register with FC03 so that an absent device
     * fails init_callback (and is retried by the thread loop) instead of
     * falling through to a process_callback scan against nothing.
     */
    if (unit->profile->table_count > 0) {
        uint16_t probe_addr = unit->profile->table[0].device_address;

        if (mb_rtu_client_probe_device(&unit->rtu_ctx, probe_addr, 1) !=
            MB_RTU_CLIENT_OK) {
            LOG_ERROR("[Inverter RTU] %s: device not responding on %s "
                      "(uid=%d).", cfg->name, cfg->path, cfg->modbus_uid);
            bus_coord_release(cfg->path);
            mb_rtu_client_disconnect(&unit->rtu_ctx);
            return -1;
        }
    }

    init_inverter_reg(unit);
    usleep(INVERTER_POST_WRITE_SETTLE_US);
    bus_coord_release(cfg->path);

    unit->comm_fail_count = 0;
    cfg->connection_state = CONNECTION_CONNECTED;
    shared_connection_state_set(cfg, CONNECTION_CONNECTED);

    LOG_INFO("[Inverter RTU] %s: serial port open, device responding.",
             cfg->name);

    return 0;
}

/**
 * @brief process_callback – run pending init, drain write queue, then read.
 *
 * Phase 0: execute a pending init request.  Taking the request clears it, so
 *          it runs exactly once per request.
 *
 * Phase 1: drain the per-unit write queue under one bus_coord hold so a
 *          co-bus module cannot interleave until post-write settle completes.
 *          A write failure is logged but does not abort the read phase.
 *
 * Phase 2: consecutive device addresses are batched into one FC03 request.
 *          On any read failure the fail counter increments; after
 *          INVERTER_COMM_FAIL_THRESHOLD consecutive failures the callback
 *          returns -1 to trigger error_callback.
 *
 * @param cfg  Module configuration.
 * @return 0 on success, -1 on communication failure.
 */
static int inverter_rtu_process_callback(module_config_t *cfg)
{
    if (!cfg) {
        return -1;
    }

    inverter_unit_t *unit = inverter_unit_from_config(cfg);
    if (!unit) {
        return -1;
    }

    /* ── Phase 0: run a pending init request ────────────────────────── */
    if (atomic_exchange(&unit->init_requested, false)) {
        run_init_sequence(unit);
    }

    /* ── Phase 1: drain the write command queue ─────────────────────── */
    inverter_write_cmd_t cmd;
    if (queue_pop(&unit->cmd_queue, &cmd) == 0) {
        bus_coord_acquire(cfg->path);

        do {
            if (write_registers_locked(unit, cmd.addr, cmd.values,
                                         cmd.count, cmd.mode) != 0) {
                LOG_WARNING("[Inverter RTU] %s: queued write to 0x%04X failed, "
                            "continuing scan.", cfg->name, cmd.addr);
            } else {
                usleep(INVERTER_INTER_SEGMENT_DELAY_US);
            }
        } while (queue_pop(&unit->cmd_queue, &cmd) == 0);

        usleep(INVERTER_POST_WRITE_SETTLE_US);
        bus_coord_release(cfg->path);
    }

    /* ── Phase 2: segmented FC03 read scan ──────────────────────────── */
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

        bus_coord_acquire(cfg->path);
        int result = mb_rtu_client_read_holding_registers(
                         &unit->rtu_ctx, start, count, buf);
        bus_coord_release(cfg->path);

        if (result != MB_RTU_CLIENT_OK) {
            unit->comm_fail_count++;
            LOG_WARNING("[Inverter RTU] %s read 0x%04X len %u failed "
                        "(err %d, fail %d/%d)",
                        cfg->name, start, count, result,
                        unit->comm_fail_count, INVERTER_COMM_FAIL_THRESHOLD);

            if (unit->comm_fail_count >= INVERTER_COMM_FAIL_THRESHOLD) {
                /* Only blank this unit's own mapped registers – never a
                 * hardcoded address range, which could span into another
                 * device family's pool region. */
                for (size_t k = 0; k < profile->table_count; k++) {
                    pool_write_register(profile->table[k].pool_address, 0xFFFF);
                }
                return -1;
            }
        } else {
            unit->comm_fail_count = 0;
            device_map_read_to_pool(profile, buf, start, (int)count);
        }

        usleep(INVERTER_INTER_SEGMENT_DELAY_US);

        seg_start = i;
        seg_len   = 1;
    }

    usleep((useconds_t)(cfg->rtu_poll_interval_ms * 1000u));
    return 0;
}

/**
 * @brief error_callback – close serial port and wait before the next reconnect.
 *
 * @param cfg               Module configuration.
 * @param connection_state  New connection state (CONNECTION_DISCONNECTED).
 * @return 0 (reconnect is managed by the thread loop).
 */
static int inverter_rtu_error_callback(module_config_t *cfg, int connection_state)
{
    if (!cfg) {
        return 0;
    }

    inverter_unit_t *unit = inverter_unit_from_config(cfg);
    if (unit) {
        mb_rtu_client_disconnect(&unit->rtu_ctx);
        unit->comm_fail_count = 0;
    }

    cfg->connection_state = (connection_state_t)connection_state;
    shared_connection_state_set(cfg, (connection_state_t)connection_state);

    LOG_WARNING("[Inverter RTU] %s: disconnected (state=%d). "
                "Retrying in %u ms …",
                cfg->name, connection_state, INVERTER_RECONNECT_DELAY_MS);

    if (unit) {
        interruptible_sleep_ms(unit, INVERTER_RECONNECT_DELAY_MS);
    } else {
        usleep(INVERTER_RECONNECT_DELAY_MS * 1000u);
    }
    return 0;
}

/**
 * @brief msg_callback – execute a Modbus write on an Inverter unit.
 *
 * Retained for framework completeness.  All writes are normally routed
 * through inverter_cmd_push() → queue drain inside process_callback.
 *
 * @param cfg    Module configuration of the target Inverter.
 * @param addr   Device register address to write.
 * @param values Values to write (host byte order).
 * @param count  Number of registers (1 → FC06, >1 → FC16).
 * @return 0 on success, -1 on failure.
 */
static int inverter_msg_callback(module_config_t *cfg,
                                  uint16_t addr, uint16_t *values, size_t count)
{
    if (!cfg || !values || count == 0) {
        return -1;
    }

    inverter_unit_t *unit = inverter_unit_from_config(cfg);
    if (!unit) {
        LOG_ERROR("[Inverter RTU] %s: msg_callback – unit not found.",
                  cfg->name);
        return -1;
    }

    return write_registers_to_device(unit, addr, values,
                                     (uint16_t)count,
                                     INVERTER_WRITE_MODE_AUTO);
}


/* ── Queue helpers (static) ───────────────────────────────────────────── */

static void queue_init(inverter_cmd_queue_t *q)
{
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->lock, NULL);
}

static void queue_destroy(inverter_cmd_queue_t *q)
{
    pthread_mutex_destroy(&q->lock);
}

/**
 * @brief Push one write command onto the tail of the queue.
 *
 * Merge-allowed addresses (see inverter_queue_merge_allowed) update an
 * existing pending single-register entry instead of consuming another slot.
 *
 * @return 0 on success, -1 if the queue is full or count is out of range.
 */
static int queue_push(inverter_cmd_queue_t *q, uint16_t addr,
                      const uint16_t *values, uint16_t count,
                      inverter_write_mode_t mode)
{
    if (!values || count == 0u || count > MODBUS_MAX_WRITE_REGISTERS) {
        return -1;
    }

    pthread_mutex_lock(&q->lock);

    if (count == 1u && inverter_queue_merge_allowed(addr)) {
        unsigned int idx = q->head;
        for (unsigned int i = 0u; i < q->count; i++) {
            inverter_write_cmd_t *entry = &q->entries[idx];

            if (entry->addr == addr && entry->count == 1u) {
                entry->mode = mode;
                entry->values[0] = values[0];
                pthread_mutex_unlock(&q->lock);
                return 0;
            }

            idx = (idx + 1u) % INVERTER_CMD_QUEUE_CAPACITY;
        }
    }

    if (q->count >= INVERTER_CMD_QUEUE_CAPACITY) {
        pthread_mutex_unlock(&q->lock);
        return -1;
    }

    inverter_write_cmd_t *entry = &q->entries[q->tail];
    entry->addr  = addr;
    entry->count = count;
    entry->mode  = mode;
    memcpy(entry->values, values, count * sizeof(uint16_t));

    q->tail  = (q->tail + 1u) % INVERTER_CMD_QUEUE_CAPACITY;
    q->count++;

    pthread_mutex_unlock(&q->lock);
    return 0;
}

/**
 * @brief Pop one write command from the head of the queue.
 * @return 0 on success, -1 if the queue is empty.
 */
static int queue_pop(inverter_cmd_queue_t *q, inverter_write_cmd_t *out)
{
    pthread_mutex_lock(&q->lock);

    if (q->count == 0) {
        pthread_mutex_unlock(&q->lock);
        return -1;
    }

    *out    = q->entries[q->head];
    q->head = (q->head + 1u) % INVERTER_CMD_QUEUE_CAPACITY;
    q->count--;

    pthread_mutex_unlock(&q->lock);
    return 0;
}

/* ── Internal helpers ─────────────────────────────────────────────────── */

/**
 * @brief Run the operator-initiated init sequence and record the result.
 *
 * Drives the inverter to a known idle state (zero frequency, stopped with
 * acceleration enabled) and owns int_inverter_init_flag_reg end to end: cleared on
 * entry so a repeated request starts from 0, set to 1 only once every write
 * has been acknowledged by the device.  The CMOS bridge just publishes that
 * slot; it never needs to know what init does.
 *
 * Runs on the unit's polling thread and holds the bus for the whole
 * sequence, so the writes cannot be interleaved by a co-bus module.
 */
static void run_init_sequence(inverter_unit_t *unit)
{
    uint16_t frequency_cmd_val = 0;
    uint16_t operation_cmd_val = INVERTER_STOP_BIT |
                                 INVERTER_ENABLE_ACCELERATION_BIT;
    bool writes_ok = true;

    struct {
        uint16_t addr;
        const uint16_t *value;
    } 
    
    writes[] = {
        { dev_frequency_cmd_reg, &frequency_cmd_val },
        { dev_operation_cmd_reg, &operation_cmd_val },
    };

    pool_write_register(int_inverter_init_flag_reg, 0);

    bus_coord_acquire(unit->cfg->path);

    for (size_t i = 0; i < sizeof(writes) / sizeof(writes[0]); i++) {
        if (write_registers_locked(unit, writes[i].addr, writes[i].value,
                                     1, INVERTER_WRITE_MODE_FC06) != 0) {
            writes_ok = false;
            break;
        }
        usleep(INVERTER_INTER_SEGMENT_DELAY_US);
    }

    usleep(INVERTER_POST_WRITE_SETTLE_US);
    bus_coord_release(unit->cfg->path);

    if (!writes_ok) {
        LOG_ERROR("[Inverter RTU] %s: init sequence failed; "
                  "init flag stays 0.", unit->cfg->name);
        return;
    }

    pool_write_register(int_inverter_init_flag_reg, 1);
    LOG_INFO("[Inverter RTU] %s: init sequence complete.", unit->cfg->name);
}

static inverter_unit_t *inverter_unit_from_config(const module_config_t *cfg)
{
    for (int i = 0; i < inverter_unit_count; i++) {
        if (inverter_units[i].cfg == cfg) {
            return &inverter_units[i];
        }
    }
    return NULL;
}

static inverter_unit_t *inverter_unit_from_uid(uint8_t uid)
{
    for (int i = 0; i < inverter_unit_count; i++) {
        if (inverter_units[i].cfg &&
            (uint8_t)inverter_units[i].cfg->modbus_uid == uid) {
            return &inverter_units[i];
        }
    }
    return NULL;
}

/**
 * @brief Sleep for duration_ms, waking early if the unit is signalled to stop.
 */
static void interruptible_sleep_ms(const inverter_unit_t *unit,
                                   uint32_t duration_ms)
{
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < duration_ms && unit->running) {
        usleep(INVERTER_SHUTDOWN_CHECK_INTERVAL_MS * 1000u);
        elapsed_ms += INVERTER_SHUTDOWN_CHECK_INTERVAL_MS;
    }
}

/**
 * @brief Execute a Modbus write without acquiring bus_coord.
 *
 * "unlocked" means this helper does not call bus_coord_acquire() /
 * bus_coord_release().  The caller must already hold the bus (queue drain,
 * init_inverter_reg) or use write_registers_to_device() which wraps this
 * with acquire / settle / release.
 *
 * @return 0 on success, -1 on failure.
 */
static int write_registers_locked(inverter_unit_t *unit,
                                    uint16_t         addr,
                                    const uint16_t  *values,
                                    uint16_t         count,
                                    inverter_write_mode_t mode)
{
    int result;

    switch (mode) {
    case INVERTER_WRITE_MODE_FC06:
        if (count != 1) {
            LOG_ERROR("[Inverter RTU] %s: FC06 requires count=1 (got %u) at 0x%04X.",
                      unit->cfg->name, count, addr);
            return -1;
        }
        result = mb_rtu_client_write_single_register(
                     &unit->rtu_ctx, addr, values[0]);
        break;
    case INVERTER_WRITE_MODE_FC16:
        result = mb_rtu_client_write_multiple_registers(
                     &unit->rtu_ctx, addr, count, values);
        break;
    case INVERTER_WRITE_MODE_AUTO:
    default:
        if (count == 1) {
            result = mb_rtu_client_write_single_register(
                         &unit->rtu_ctx, addr, values[0]);
        } else {
            result = mb_rtu_client_write_multiple_registers(
                         &unit->rtu_ctx, addr, count, values);
        }
        break;
    }

    if (result != MB_RTU_CLIENT_OK) {
        LOG_ERROR("[Inverter RTU] %s: write to 0x%04X failed (err %d).",
                  unit->cfg->name, addr, result);
        return -1;
    }

    if (count == 1) {
        LOG_VERBOSE("[Inverter RTU] %s: wrote register 0x%04X value=0x%04X.",
                    unit->cfg->name, addr, values[0]);
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

        LOG_VERBOSE("[Inverter RTU] %s: wrote %u register(s) at 0x%04X "
                    "values=[%s].",
                    unit->cfg->name, count, addr, val_buf);
    }
    return 0;
}

/**
 * @brief Execute a Modbus write with bus ownership and post-write settle.
 *
 * Used by msg_callback (single-shot path).  Queue drain and init hold the
 * bus themselves and call write_registers_locked() instead.
 */
static int write_registers_to_device(inverter_unit_t *unit,
                                     uint16_t         addr,
                                     const uint16_t  *values,
                                     uint16_t         count,
                                     inverter_write_mode_t mode)
{
    const char *path = unit->cfg->path;

    bus_coord_acquire(path);
    int result = write_registers_locked(unit, addr, values, count, mode);
    if (result == 0) {
        usleep(INVERTER_POST_WRITE_SETTLE_US);
    }
    bus_coord_release(path);
    return result;
}

/* ── Thread function ──────────────────────────────────────────────────── */

/**
 * @brief Poll thread for one Inverter unit.
 *
 *  open serial → segment reads → write pool → wait for next poll interval
 *              → on failure: close → wait → reopen
 */
static void *inverter_rtu_thread(void *arg)
{
    inverter_unit_t *unit = (inverter_unit_t *)arg;
    module_config_t *cfg  = unit->cfg;

    LOG_INFO("[Inverter RTU] Thread started: %s (profile=%s uid=%d)",
             cfg->name, unit->profile->name, cfg->modbus_uid);

    while (unit->running) {

        /* init --------------------------------------------------------- */
        if (unit->callbacks.init_callback(cfg) != 0) {
            LOG_ERROR("[Inverter RTU] %s: init failed, will retry.", cfg->name);
            interruptible_sleep_ms(unit, INVERTER_RECONNECT_DELAY_MS);
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

    mb_rtu_client_disconnect(&unit->rtu_ctx);
    LOG_INFO("[Inverter RTU] Thread stopped: %s", cfg->name);
    return NULL;
}

/**
 * @brief Map config.json host baud to the inverter register code.
 *
 * Device encoding: register value = host_baud / 100 (e.g. 9600 -> 0x0060).
 *
 * @return true if host_baud is a valid multiple of 100, false otherwise.
 */
static bool inverter_baud_convert(uint32_t host_baud, uint16_t *out_reg)
{
    if (!out_reg || host_baud < 4800u || host_baud > 115200u || (host_baud % 100u) != 0u) {
        return false;
    }

    uint32_t code = host_baud / 100u;
    if (code > 0xFFFFu) {
        return false;
    }

    *out_reg = (uint16_t)code;
    return true;
}

/**
 * @brief Inverter register programming on each connect (caller holds bus_coord).
 */
static void init_inverter_reg(inverter_unit_t *unit)
{
    module_config_t *cfg = unit->cfg;

    // system settings
    uint16_t frequency_souce_val = 0x0001; /* default RS-485 */
    uint16_t run_souce_val = 0x0002;       /* default RS-485 */
    uint16_t frequency_upper_val = 0x1766; /* default 5990 */
    uint16_t frequency_lower_val = 0x06F4; /* default 1780 */
    uint16_t acceleration_val = 0x03E8;    /* default 1000 */
    uint16_t deceleration_val = 0x03E8;    /* default 10 00*/
    uint16_t slave_id_val = (uint16_t)cfg->modbus_uid;
    uint16_t baud_rate_val = 0;
    bool baud_reg_valid = inverter_baud_convert(
                              (uint32_t)cfg->baud_rate, &baud_rate_val);
    uint16_t error_handle_val = 0x0001;    /* Slow down and stop */
    uint16_t serial_setting_val = 0x000C;  /* 8N1 */
    uint16_t s_speed_start_val = 0x0096;    /* default 150 */
    uint16_t s_speed_end_val = 0x0096;      /* default 150 */
    uint16_t s_deceleration_start_val = 0x0096; /* default 150 */
    uint16_t s_deceleration_end_val = 0x0096;   /* default 150 */

    // device settings
    uint16_t pump_duty_val = 0x0000;
    uint16_t operation_cmd_val = INVERTER_STOP_BIT | INVERTER_ENABLE_ACCELERATION_BIT;

    struct {
        uint16_t        addr;
        const uint16_t *value;
    } writes[] = {
        // system settings
        { dev_frequency_cmd_souce_reg,  &frequency_souce_val },
        { dev_run_cmd_souce_reg,        &run_souce_val },
        { dev_frequency_upper_limit_reg, &frequency_upper_val },
        { dev_frequency_lower_limit_reg, &frequency_lower_val },
        { dev_acceleration_reg,         &acceleration_val },
        { dev_deceleration_reg,         &deceleration_val },
        { dev_slave_id,                 &slave_id_val },
        { dev_baud_rate,                &baud_rate_val },
        { dev_mb_error_handle_reg,      &error_handle_val },
        { dev_mb_serial_setting_reg,    &serial_setting_val },
        { dev_s_speed_start_reg,        &s_speed_start_val },
        { dev_s_speed_end_reg,          &s_speed_end_val },
        { dev_s_deceleration_start_reg, &s_deceleration_start_val },
        { dev_s_deceleration_end_reg,   &s_deceleration_end_val },

        // device settings
        {dev_frequency_cmd_reg,         &pump_duty_val},
        {dev_operation_cmd_reg,         &operation_cmd_val},
    };

    if (!baud_reg_valid) {
        LOG_WARNING("[Inverter RTU] %s: unsupported baud_rate %d for device "
                    "register 0x%04X; skipping baud write.",
                    cfg->name, cfg->baud_rate, dev_baud_rate);
    }

    for (size_t i = 0; i < sizeof(writes) / sizeof(writes[0]); i++) {
        if (writes[i].addr == dev_baud_rate && !baud_reg_valid) {
            continue;
        }

        if (write_registers_locked(unit, writes[i].addr, writes[i].value,
                                     1, INVERTER_WRITE_MODE_FC06) == 0) {
            usleep(INVERTER_INTER_SEGMENT_DELAY_US);
        }
    }
}