/**
 * @file inverter_module.c
 * @brief Inverter Modbus RTU/TCP polling.
 */

#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "inverter_module.h"
#include "bus_coord.h"
#include "device_register_map.h"
#include "inverter/inverter_map.h"
#include "inverter_cmos_bridge.h"
#include "log.h"
#include "modbus_defaults.h"
#include "modbus_rtu_client.h"
#include "modbus_tcp_client.h"

typedef struct {
    uint16_t addr;
    uint16_t values[MODBUS_MAX_WRITE_REGISTERS];
    uint16_t count;
    inverter_write_mode_t mode;
} inverter_write_cmd_t;

typedef struct {
    inverter_write_cmd_t entries[MODBUS_DEFAULT_CMD_QUEUE_CAPACITY];
    unsigned int head;
    unsigned int tail;
    unsigned int count;
    pthread_mutex_t lock;
} inverter_cmd_queue_t;

typedef struct {
    module_config_t *cfg;
    const device_map_profile_t *profile;
    mb_tcp_client_ctx_t tcp_ctx;
    mb_rtu_client_ctx_t rtu_ctx;
    module_callbacks_t callbacks;
    pthread_t thread;
    volatile sig_atomic_t running;
    int comm_fail_count;
    inverter_cmd_queue_t cmd_queue;
    atomic_bool init_requested;
} inverter_unit_t;

static void queue_init(inverter_cmd_queue_t *q);
static void queue_destroy(inverter_cmd_queue_t *q);
static int queue_push(inverter_cmd_queue_t *q, uint16_t addr,
                      const uint16_t *values, uint16_t count,
                      inverter_write_mode_t mode);
static int queue_pop(inverter_cmd_queue_t *q, inverter_write_cmd_t *out);
static void run_init_sequence(inverter_unit_t *unit);
static inverter_unit_t *inverter_unit_from_uid(uint8_t uid);
static void interruptible_sleep_ms(const inverter_unit_t *unit, uint32_t duration_ms);
static const char *inverter_bus_path(const inverter_unit_t *unit);
static int unit_connect(inverter_unit_t *unit);
static void unit_disconnect(inverter_unit_t *unit);
static int read_device_value(inverter_unit_t *unit, uint16_t addr,
                                       uint16_t count, uint16_t *out);
static int write_registers_locked(inverter_unit_t *unit,
                                  uint16_t addr,
                                  const uint16_t *values,
                                  uint16_t count,
                                  inverter_write_mode_t mode);
static int write_registers_to_device(inverter_unit_t *unit,
                                     uint16_t addr,
                                     const uint16_t *values,
                                     uint16_t count,
                                     inverter_write_mode_t mode);
static int read_profile_to_pool(inverter_unit_t *unit, bool track_comm_fail);
static bool inverter_baud_convert(uint32_t host_baud, uint16_t *out_reg);
static void init_inverter_reg(inverter_unit_t *unit);
static int inverter_init_callback(void *arg);
static int inverter_process_callback(void *arg);
static int inverter_error_callback(void *arg, int connection_state);
static int inverter_msg_callback(void *arg, uint16_t addr,
                                 uint16_t *values, size_t count);
static void *inverter_thread(void *arg);

static inverter_unit_t inverter_units[MAX_INVERTER_COUNT];
static int inverter_unit_count = 0;

/**
 * @brief Start all enabled Inverter units.
 * @param inverters Config array from global_config.
 * @param inverter_count Number of entries in inverters.
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

        inverter_unit_t *unit = &inverter_units[inverter_unit_count];
        unit->cfg = &inverters[i];
        unit->profile = profile;
        unit->running = 1;
        unit->comm_fail_count = 0;
        memset(&unit->tcp_ctx, 0, sizeof(unit->tcp_ctx));
        memset(&unit->rtu_ctx, 0, sizeof(unit->rtu_ctx));
        unit->rtu_ctx.fd = -1;
        queue_init(&unit->cmd_queue);
        atomic_init(&unit->init_requested, false);

        unit->callbacks.init_callback = inverter_init_callback;
        unit->callbacks.process_callback = inverter_process_callback;
        unit->callbacks.error_callback = inverter_error_callback;
        unit->callbacks.msg_callback = inverter_msg_callback;
        unit->callbacks.start_callback = NULL;

        inverter_unit_count++;

        LOG_INFO("[Inverter] Starting %s (profile=%s uid=%d format=%s).",
                 unit->cfg->name,
                 unit->profile->name,
                 unit->cfg->modbus_uid,
                 (unit->cfg->format == MODBUS_FORMAT_TCP) ? "TCP" : "RTU");

        if (pthread_create(&unit->thread, NULL, inverter_thread, unit) != 0) {
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
 * @brief Stop all running Inverter units and join threads.
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
 * @brief Return modbus_uid of the first running Inverter unit.
 * @param out_uid Output uid.
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
 * @brief Enqueue a register write for one Inverter unit.
 * @param uid Target modbus_uid.
 * @param addr Device register address.
 * @param values Register values in host byte order.
 * @param count Number of registers.
 * @param mode FC06, FC16, or AUTO.
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

    if (queue_push(&unit->cmd_queue, addr, values, count, mode) != 0) {
        LOG_WARNING("[Inverter] inverter_cmd_push: command queue full "
                    "(uid=%u addr=0x%04X count=%u).", uid, addr, count);
        return -1;
    }

    return 0;
}

/**
 * @brief Request the init sequence for one Inverter unit.
 * @param uid Target modbus_uid.
 * @return 0 if accepted, -1 if unit not found.
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

/**
 * @brief init_callback – connect the unit transport.
 * @param arg inverter_unit_t pointer.
 * @return 0 on success, -1 on failure.
 */
static int inverter_init_callback(void *arg)
{
    inverter_unit_t *unit = (inverter_unit_t *)arg;

    if (!unit || !unit->cfg) {
        LOG_ERROR("[Inverter] Invalid unit.");
        return -1;
    }

    if (unit_connect(unit) != 0) {
        return -1;
    }

    unit->comm_fail_count = 0;
    shared_connection_state_set(unit->cfg, CONNECTION_CONNECTED);

    LOG_INFO("[Inverter] %s: connected.", unit->cfg->name);
    return 0;
}

/**
 * @brief process_callback – drain queue, then read mapped registers.
 * @param arg inverter_unit_t pointer.
 * @return 0 on success, -1 on communication failure.
 */
static int inverter_process_callback(void *arg)
{
    inverter_unit_t *unit = (inverter_unit_t *)arg;

    if (!unit || !unit->cfg) {
        return -1;
    }

    if (atomic_exchange(&unit->init_requested, false)) {
        run_init_sequence(unit);
    }

    inverter_write_cmd_t cmd;
    if (queue_pop(&unit->cmd_queue, &cmd) == 0) {
        const char *path = inverter_bus_path(unit);

        bus_coord_acquire(path);

        do {
            if (write_registers_locked(unit, cmd.addr, cmd.values,
                                       cmd.count, cmd.mode) != 0) {
                LOG_WARNING("[Inverter] %s: queued write to 0x%04X failed, "
                            "continuing scan.", unit->cfg->name, cmd.addr);
            } else {
                usleep(MODBUS_DEFAULT_INTER_SEGMENT_DELAY_US);
            }
        } while (queue_pop(&unit->cmd_queue, &cmd) == 0);

        usleep(MODBUS_DEFAULT_POST_WRITE_SETTLE_US);
        bus_coord_release(path);
    }
    if (unit->profile->table_count == 0) {
        return 0;
    }
    if (read_profile_to_pool(unit, true) != 0) {
        return -1;
    }

    usleep((useconds_t)(MODBUS_DEFAULT_POLL_CYCLE_INTERVAL_MS * 1000u));
    return 0;
}

/**
 * @brief Read all mapped registers into the pool.
 * @param unit Target unit.
 * @param track_comm_fail Update comm_fail_count when true.
 * @return 0 on success, -1 on read failure.
 */
static int read_profile_to_pool(inverter_unit_t *unit, bool track_comm_fail)
{
    module_config_t *cfg = unit->cfg;
    const device_map_profile_t *profile = unit->profile;

    if (profile->table_count == 0) {
        return 0;
    }

    uint16_t buf[MODBUS_MAX_READ_REGISTERS] = {0};

    size_t seg_start = 0;
    size_t seg_len = 1;

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

        int result = read_device_value(unit, start, count, buf);

        if (result != 0) {
            if (!track_comm_fail) {
                LOG_WARNING("[Inverter] %s init read 0x%04X len %u failed (err %d).",
                            cfg->name, start, count, result);
                return -1;
            }

            unit->comm_fail_count++;
            LOG_WARNING("[Inverter] %s read 0x%04X len %u failed (err %d, fail %d/%d)",
                        cfg->name, start, count, result,
                        unit->comm_fail_count, MODBUS_DEFAULT_COMM_FAIL_THRESHOLD);

            if (unit->comm_fail_count >= MODBUS_DEFAULT_COMM_FAIL_THRESHOLD) {
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

        usleep(MODBUS_DEFAULT_INTER_SEGMENT_DELAY_US);

        seg_start = i;
        seg_len = 1;
    }

    return 0;
}

/**
 * @brief error_callback – disconnect and wait before reconnect.
 * @param arg inverter_unit_t pointer.
 * @param connection_state New connection state.
 * @return 0.
 */
static int inverter_error_callback(void *arg, int connection_state)
{
    inverter_unit_t *unit = (inverter_unit_t *)arg;

    if (!unit || !unit->cfg) {
        return 0;
    }

    unit_disconnect(unit);
    unit->comm_fail_count = 0;

    shared_connection_state_set(unit->cfg, (connection_state_t)connection_state);

    LOG_WARNING("[Inverter] %s: disconnected (state=%d). "
                "Retrying in %u ms …",
                unit->cfg->name, connection_state,
                MODBUS_DEFAULT_RECONNECT_DELAY_MS);

    interruptible_sleep_ms(unit, MODBUS_DEFAULT_RECONNECT_DELAY_MS);
    return 0;
}

/**
 * @brief msg_callback – synchronous write path (unused; queue is preferred).
 * @param arg inverter_unit_t pointer.
 * @param addr Device register address.
 * @param values Values in host byte order.
 * @param count Register count.
 * @return 0 on success, -1 on failure.
 */
static int inverter_msg_callback(void *arg,
                                 uint16_t addr, uint16_t *values, size_t count)
{
    inverter_unit_t *unit = (inverter_unit_t *)arg;

    if (!unit || !unit->cfg || !values || count == 0) {
        return -1;
    }

    return write_registers_to_device(unit, addr, values,
                                     (uint16_t)count,
                                     INVERTER_WRITE_MODE_AUTO);
}

/**
 * @brief Initialize a command queue.
 * @param q Queue to initialize.
 */
static void queue_init(inverter_cmd_queue_t *q)
{
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->lock, NULL);
}

/**
 * @brief Destroy a command queue.
 * @param q Queue to destroy.
 */
static void queue_destroy(inverter_cmd_queue_t *q)
{
    pthread_mutex_destroy(&q->lock);
}

/**
 * @brief Push one write command onto the queue tail.
 * @param q Target queue.
 * @param addr Device register address.
 * @param values Register values.
 * @param count Number of registers.
 * @param mode FC selection mode.
 * @return 0 on success, -1 on failure.
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

            idx = (idx + 1u) % MODBUS_DEFAULT_CMD_QUEUE_CAPACITY;
        }
    }

    if (q->count >= MODBUS_DEFAULT_CMD_QUEUE_CAPACITY) {
        pthread_mutex_unlock(&q->lock);
        return -1;
    }

    inverter_write_cmd_t *entry = &q->entries[q->tail];
    entry->addr = addr;
    entry->count = count;
    entry->mode = mode;
    memcpy(entry->values, values, count * sizeof(uint16_t));

    q->tail = (q->tail + 1u) % MODBUS_DEFAULT_CMD_QUEUE_CAPACITY;
    q->count++;

    pthread_mutex_unlock(&q->lock);
    return 0;
}

/**
 * @brief Pop one write command from the queue head.
 * @param q Source queue.
 * @param out Output command.
 * @return 0 on success, -1 if empty.
 */
static int queue_pop(inverter_cmd_queue_t *q, inverter_write_cmd_t *out)
{
    pthread_mutex_lock(&q->lock);

    if (q->count == 0) {
        pthread_mutex_unlock(&q->lock);
        return -1;
    }

    *out = q->entries[q->head];
    q->head = (q->head + 1u) % MODBUS_DEFAULT_CMD_QUEUE_CAPACITY;
    q->count--;

    pthread_mutex_unlock(&q->lock);
    return 0;
}

/**
 * @brief Run init sequence and update int_inverter_init_flag_reg.
 * @param unit Target unit.
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
    } writes[] = {
        { dev_frequency_cmd_reg, &frequency_cmd_val },
        { dev_operation_cmd_reg, &operation_cmd_val },
    };

    pool_write_register(int_inverter_init_flag_reg, 0);

    const char *path = inverter_bus_path(unit);

    bus_coord_acquire(path);

    for (size_t i = 0; i < sizeof(writes) / sizeof(writes[0]); i++) {
        if (write_registers_locked(unit, writes[i].addr, writes[i].value,
                                   1, INVERTER_WRITE_MODE_FC06) != 0) {
            writes_ok = false;
            break;
        }
        usleep(MODBUS_DEFAULT_INTER_SEGMENT_DELAY_US);
    }

    usleep(MODBUS_DEFAULT_POST_WRITE_SETTLE_US);
    bus_coord_release(path);

    if (!writes_ok) {
        LOG_ERROR("[Inverter] %s: init sequence failed; "
                  "init flag stays 0.", unit->cfg->name);
        return;
    }

    pool_write_register(int_inverter_init_flag_reg, 1);
    LOG_INFO("[Inverter] %s: init sequence complete.", unit->cfg->name);
}

/**
 * @brief Find a running Inverter unit by modbus uid.
 * @param uid Target modbus_uid.
 * @return Unit pointer, or NULL if not found.
 */
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
 * @brief Sleep until duration_ms elapses or unit stops.
 * @param unit Unit used for running flag.
 * @param duration_ms Sleep duration in milliseconds.
 */
static void interruptible_sleep_ms(const inverter_unit_t *unit,
                                   uint32_t duration_ms)
{
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < duration_ms && unit->running) {
        usleep(MODBUS_DEFAULT_SHUTDOWN_CHECK_INTERVAL_MS * 1000u);
        elapsed_ms += MODBUS_DEFAULT_SHUTDOWN_CHECK_INTERVAL_MS;
    }
}

/**
 * @brief Return RS-485 serial path for bus_coord, or NULL for TCP.
 * @param unit Target unit.
 * @return Serial path for RTU units, NULL otherwise.
 */
static const char *inverter_bus_path(const inverter_unit_t *unit)
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
 * @brief Connect TCP or RTU transport for one unit.
 * @param unit Target unit.
 * @return 0 on success, -1 on failure.
 */
static int unit_connect(inverter_unit_t *unit)
{
    module_config_t *cfg = unit->cfg;

    if (cfg->format == MODBUS_FORMAT_TCP) {
        mb_tcp_client_config_t tcp_cfg = {
            .remote_host = cfg->ip,
            .port = (uint16_t)cfg->port,
            .unit_id = (uint8_t)cfg->modbus_uid,
            .connect_timeout_sec = 5,
            .response_timeout_ms = 1000,
            .logv = NULL,
            .log_userdata = NULL,
        };

        LOG_INFO("[Inverter] %s: connecting to %s:%d uid=%d",
                 cfg->name, cfg->ip, cfg->port, cfg->modbus_uid);

        if (mb_tcp_client_connect(&unit->tcp_ctx, &tcp_cfg) != 0) {
            LOG_ERROR("[Inverter] %s: TCP connection failed.", cfg->name);
            return -1;
        }
        return 0;
    }

    mb_rtu_client_config_t rtu_cfg = {
        .serial_path = cfg->path,
        .baud_rate = (uint32_t)cfg->baud_rate,
        .unit_id = (uint8_t)cfg->modbus_uid,
        .response_timeout_ms = 1000u,
    };

    LOG_INFO("[Inverter] %s: opening %s @ %u baud uid=%d",
             cfg->name, cfg->path, cfg->baud_rate, cfg->modbus_uid);

    if (mb_rtu_client_connect(&unit->rtu_ctx, &rtu_cfg) != 0) {
        LOG_ERROR("[Inverter] %s: failed to open serial port %s.",
                  cfg->name, cfg->path);
        return -1;
    }

    const char *path = inverter_bus_path(unit);

    bus_coord_acquire(path);

    if (unit->profile->table_count > 0) {
        uint16_t probe_addr = unit->profile->table[0].device_address;

        if (mb_rtu_client_probe_device(&unit->rtu_ctx, probe_addr, 1) !=
            MB_RTU_CLIENT_OK) {
            LOG_ERROR("[Inverter] %s: device not responding on %s (uid=%d).",
                      cfg->name, cfg->path, cfg->modbus_uid);
            bus_coord_release(path);
            mb_rtu_client_disconnect(&unit->rtu_ctx);
            return -1;
        }
    }

    init_inverter_reg(unit);
    usleep(MODBUS_DEFAULT_POST_WRITE_SETTLE_US);
    bus_coord_release(path);

    return 0;
}

/**
 * @brief Disconnect TCP or RTU transport for one unit.
 * @param unit Target unit.
 */
static void unit_disconnect(inverter_unit_t *unit)
{
    if (unit->cfg->format == MODBUS_FORMAT_TCP) {
        mb_tcp_client_disconnect(&unit->tcp_ctx);
    } else {
        mb_rtu_client_disconnect(&unit->rtu_ctx);
    }
}

/**
 * @brief FC03 read via active transport.
 * @param unit Target unit.
 * @param addr Start register address.
 * @param count Number of registers.
 * @param out Output buffer.
 * @return 0 on success, or a transport/Modbus error code.
 */
static int read_device_value(inverter_unit_t *unit, uint16_t addr,
                                       uint16_t count, uint16_t *out)
{
    if (unit->cfg->format == MODBUS_FORMAT_TCP) {
        return mb_tcp_client_read_holding_registers(
            &unit->tcp_ctx, addr, count, out);
    }

    const char *path = inverter_bus_path(unit);
    int result;

    bus_coord_acquire(path);
    result = mb_rtu_client_read_holding_registers(
        &unit->rtu_ctx, addr, count, out);
    bus_coord_release(path);
    return result;
}

/**
 * @brief Write registers without acquiring bus_coord.
 * @param unit Target unit.
 * @param addr Device register address.
 * @param values Register values.
 * @param count Number of registers.
 * @param mode FC selection mode.
 * @return 0 on success, -1 on failure.
 */
static int write_registers_locked(inverter_unit_t *unit,
                                  uint16_t addr,
                                  const uint16_t *values,
                                  uint16_t count,
                                  inverter_write_mode_t mode)
{
    bool is_tcp = (unit->cfg->format == MODBUS_FORMAT_TCP);
    int result;

    switch (mode) {
    case INVERTER_WRITE_MODE_FC06:
        if (count != 1) {
            LOG_ERROR("[Inverter] %s: FC06 requires count=1 (got %u) at 0x%04X.",
                      unit->cfg->name, count, addr);
            return -1;
        }
        result = is_tcp
                 ? mb_tcp_client_write_single_register(
                       &unit->tcp_ctx, addr, values[0])
                 : mb_rtu_client_write_single_register(
                       &unit->rtu_ctx, addr, values[0]);
        break;
    case INVERTER_WRITE_MODE_FC16:
        result = is_tcp
                 ? mb_tcp_client_write_multiple_registers(
                       &unit->tcp_ctx, addr, count, values)
                 : mb_rtu_client_write_multiple_registers(
                       &unit->rtu_ctx, addr, count, values);
        break;
    case INVERTER_WRITE_MODE_AUTO:
    default:
        if (count == 1) {
            result = is_tcp
                     ? mb_tcp_client_write_single_register(
                           &unit->tcp_ctx, addr, values[0])
                     : mb_rtu_client_write_single_register(
                           &unit->rtu_ctx, addr, values[0]);
        } else {
            result = is_tcp
                     ? mb_tcp_client_write_multiple_registers(
                           &unit->tcp_ctx, addr, count, values)
                     : mb_rtu_client_write_multiple_registers(
                           &unit->rtu_ctx, addr, count, values);
        }
        break;
    }

    if (is_tcp ? (result != MB_TCP_CLIENT_OK)
               : (result != MB_RTU_CLIENT_OK)) {
        LOG_ERROR("[Inverter] %s: write to 0x%04X failed (err %d).",
                  unit->cfg->name, addr, result);
        return -1;
    }

    if (count == 1) {
        LOG_VERBOSE("[Inverter] %s: wrote register 0x%04X value=0x%04X.",
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

        LOG_VERBOSE("[Inverter] %s: wrote %u register(s) at 0x%04X "
                    "values=[%s].",
                    unit->cfg->name, count, addr, val_buf);
    }
    return 0;
}

/**
 * @brief Write registers with bus acquire and post-write settle.
 * @param unit Target unit.
 * @param addr Device register address.
 * @param values Register values.
 * @param count Number of registers.
 * @param mode FC selection mode.
 * @return 0 on success, -1 on failure.
 */
static int write_registers_to_device(inverter_unit_t *unit,
                                     uint16_t addr,
                                     const uint16_t *values,
                                     uint16_t count,
                                     inverter_write_mode_t mode)
{
    const char *path = inverter_bus_path(unit);

    bus_coord_acquire(path);
    int result = write_registers_locked(unit, addr, values, count, mode);
    if (result == 0) {
        usleep(MODBUS_DEFAULT_POST_WRITE_SETTLE_US);
    }
    bus_coord_release(path);
    return result;
}

/**
 * @brief Poll thread for one Inverter unit.
 * @param arg inverter_unit_t pointer.
 * @return NULL.
 */
static void *inverter_thread(void *arg)
{
    inverter_unit_t *unit = (inverter_unit_t *)arg;
    module_config_t *cfg = unit->cfg;

    LOG_INFO("[Inverter] Thread started: %s (profile=%s uid=%d)",
             cfg->name, unit->profile->name, cfg->modbus_uid);

    while (unit->running) {
        if (unit->callbacks.init_callback(unit) != 0) {
            LOG_ERROR("[Inverter] %s: init failed, will retry.", cfg->name);
            interruptible_sleep_ms(unit, MODBUS_DEFAULT_RECONNECT_DELAY_MS);
            continue;
        }
        while (unit->running) {
            if (unit->callbacks.process_callback(unit) != 0) {
                if (unit->callbacks.error_callback) {
                    unit->callbacks.error_callback(unit,
                                                   CONNECTION_DISCONNECTED);
                }
                break;
            }
        }
    }

    unit_disconnect(unit);
    LOG_INFO("[Inverter] Thread stopped: %s", cfg->name);
    return NULL;
}

/**
 * @brief Map host baud rate to inverter register code.
 * @param host_baud Baud rate from config.
 * @param out_reg Output register value.
 * @return true if host_baud is valid.
 */
static bool inverter_baud_convert(uint32_t host_baud, uint16_t *out_reg)
{
    if (!out_reg || host_baud < 4800u || host_baud > 115200u ||
        (host_baud % 100u) != 0u) {
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
 * @brief Program inverter registers on RTU connect; caller holds bus.
 * @param unit Target unit.
 */
static void init_inverter_reg(inverter_unit_t *unit)
{
    module_config_t *cfg = unit->cfg;
    uint16_t frequency_source_val = 0x0001; /* default RS-485 */
    uint16_t run_source_val = 0x0002; /* default RS-485 */
    uint16_t frequency_upper_val = 0x1766; /* default 5990 */
    uint16_t frequency_lower_val = 0x06F4; /* default 1780 */
    uint16_t acceleration_val = 0x03E8; /* default 1000 */
    uint16_t deceleration_val = 0x03E8; /* default 10 00*/
    uint16_t slave_id_val = (uint16_t)cfg->modbus_uid;
    uint16_t baud_rate_val = 0;
    bool baud_reg_valid = inverter_baud_convert(
        (uint32_t)cfg->baud_rate, &baud_rate_val);
    uint16_t error_handle_val = 0x0001; /* Slow down and stop */
    uint16_t serial_setting_val = 0x000C; /* 8N1 */
    uint16_t s_speed_start_val = 0x0096; /* default 150 */
    uint16_t s_speed_end_val = 0x0096; /* default 150 */
    uint16_t s_deceleration_start_val = 0x0096; /* default 150 */
    uint16_t s_deceleration_end_val = 0x0096; /* default 150 */
    uint16_t pump_duty_val = 0x0000;
    uint16_t operation_cmd_val = INVERTER_STOP_BIT | INVERTER_ENABLE_ACCELERATION_BIT;

    struct {
        uint16_t addr;
        const uint16_t *value;
    } writes[] = {
        { dev_frequency_cmd_source_reg, &frequency_source_val },
        { dev_run_cmd_source_reg, &run_source_val },
        { dev_frequency_upper_limit_reg, &frequency_upper_val },
        { dev_frequency_lower_limit_reg, &frequency_lower_val },
        { dev_acceleration_reg, &acceleration_val },
        { dev_deceleration_reg, &deceleration_val },
        { dev_slave_id, &slave_id_val },
        { dev_baud_rate, &baud_rate_val },
        { dev_mb_error_handle_reg, &error_handle_val },
        { dev_mb_serial_setting_reg, &serial_setting_val },
        { dev_s_speed_start_reg, &s_speed_start_val },
        { dev_s_speed_end_reg, &s_speed_end_val },
        { dev_s_deceleration_start_reg, &s_deceleration_start_val },
        { dev_s_deceleration_end_reg, &s_deceleration_end_val },
        { dev_frequency_cmd_reg, &pump_duty_val },
        { dev_operation_cmd_reg, &operation_cmd_val },
    };

    if (!baud_reg_valid) {
        LOG_WARNING("[Inverter] %s: unsupported baud_rate %d for device "
                    "register 0x%04X; skipping baud write.",
                    cfg->name, cfg->baud_rate, dev_baud_rate);
    }

    for (size_t i = 0; i < sizeof(writes) / sizeof(writes[0]); i++) {
        if (writes[i].addr == dev_baud_rate && !baud_reg_valid) {
            continue;
        }

        if (write_registers_locked(unit, writes[i].addr, writes[i].value,
                                   1, INVERTER_WRITE_MODE_FC06) == 0) {
            usleep(MODBUS_DEFAULT_INTER_SEGMENT_DELAY_US);
        }
    }
}
