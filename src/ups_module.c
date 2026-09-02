/**
 * @file ups_module.c
 * @brief UPS Modbus RTU/TCP polling.
 */

#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ups_module.h"
#include "bus_coord.h"
#include "device_register_map.h"
#include "log.h"
#include "modbus_defaults.h"
#include "modbus_rtu_client.h"
#include "modbus_tcp_client.h"
#include "ups/ups_map.h"
#include "ups_cmos_bridge.h"

typedef struct {
    uint16_t addr;
    uint16_t values[MODBUS_MAX_WRITE_REGISTERS];
    uint16_t count;
    ups_write_mode_t mode;
} ups_write_cmd_t;

typedef struct {
    ups_write_cmd_t entries[MODBUS_DEFAULT_CMD_QUEUE_CAPACITY];
    unsigned int head;
    unsigned int tail;
    unsigned int count;
    pthread_mutex_t lock;
} ups_cmd_queue_t;

typedef struct {
    module_config_t *cfg;
    const device_map_profile_t *profile;
    mb_tcp_client_ctx_t tcp_ctx;
    mb_rtu_client_ctx_t rtu_ctx;
    module_callbacks_t callbacks;
    pthread_t thread;
    volatile sig_atomic_t running;
    int comm_fail_count;
    ups_cmd_queue_t cmd_queue;
    atomic_bool init_requested;
} ups_unit_t;

static void queue_init(ups_cmd_queue_t *q);
static void queue_destroy(ups_cmd_queue_t *q);
static int queue_push(ups_cmd_queue_t *q, uint16_t addr,
                      const uint16_t *values, uint16_t count,
                      ups_write_mode_t mode);
static int queue_pop(ups_cmd_queue_t *q, ups_write_cmd_t *out);
static ups_unit_t *ups_unit_from_uid(uint8_t uid);
static void interruptible_sleep_ms(const ups_unit_t *unit, uint32_t duration_ms);
static const char *ups_bus_path(const ups_unit_t *unit);
static int unit_connect(ups_unit_t *unit);
static void unit_disconnect(ups_unit_t *unit);
static int unit_read_holding_registers(ups_unit_t *unit, uint16_t addr,
                                       uint16_t count, uint16_t *out);
static int write_registers_locked(ups_unit_t *unit,
                                  uint16_t addr,
                                  const uint16_t *values,
                                  uint16_t count,
                                  ups_write_mode_t mode);
static int write_registers_to_device(ups_unit_t *unit,
                                     uint16_t addr,
                                     const uint16_t *values,
                                     uint16_t count,
                                     ups_write_mode_t mode);
static int ups_init_callback(void *arg);
static int ups_process_callback(void *arg);
static int read_profile_to_pool(ups_unit_t *unit, bool track_comm_fail);
static void run_init_sequence(ups_unit_t *unit);
static int ups_error_callback(void *arg, int connection_state);
static int ups_msg_callback(void *arg, uint16_t addr,
                            uint16_t *values, size_t count);
static void *ups_thread(void *arg);

static ups_unit_t ups_units[MAX_UPS_COUNT];
static int ups_unit_count = 0;

/**
 * @brief Start all enabled UPS units.
 * @param ups Config array from global_config.
 * @param ups_count Number of entries in ups.
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

        ups_unit_t *unit = &ups_units[ups_unit_count];
        unit->cfg = &ups[i];
        unit->profile = profile;
        unit->running = 1;
        unit->comm_fail_count = 0;
        memset(&unit->tcp_ctx, 0, sizeof(unit->tcp_ctx));
        memset(&unit->rtu_ctx, 0, sizeof(unit->rtu_ctx));
        unit->rtu_ctx.fd = -1;
        queue_init(&unit->cmd_queue);
        atomic_init(&unit->init_requested, false);

        unit->callbacks.init_callback = ups_init_callback;
        unit->callbacks.process_callback = ups_process_callback;
        unit->callbacks.error_callback = ups_error_callback;
        unit->callbacks.msg_callback = ups_msg_callback;
        unit->callbacks.start_callback = NULL;

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
 * @brief Stop all running UPS units and join threads.
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
 * @brief Return modbus_uid of the first running UPS unit.
 * @param out_uid Output uid.
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
 * @brief Enqueue a register write for one UPS unit.
 * @param uid Target modbus_uid.
 * @param addr Device register address.
 * @param values Register values in host byte order.
 * @param count Number of registers.
 * @param mode FC06, FC16, or AUTO.
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

    if (queue_push(&unit->cmd_queue, addr, values, count, mode) != 0) {
        LOG_WARNING("[UPS] ups_cmd_push: command queue full "
                    "(uid=%u addr=0x%04X count=%u).", uid, addr, count);
        return -1;
    }

    return 0;
}

/**
 * @brief Request the init sequence for one UPS unit.
 * @param uid Target modbus_uid.
 * @return 0 if accepted, -1 if unit not found.
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

/**
 * @brief init_callback – connect the unit transport.
 * @param arg ups_unit_t pointer.
 * @return 0 on success, -1 on failure.
 */
static int ups_init_callback(void *arg)
{
    ups_unit_t *unit = (ups_unit_t *)arg;

    if (!unit || !unit->cfg) {
        LOG_ERROR("[UPS] Invalid unit.");
        return -1;
    }

    if (unit_connect(unit) != 0) {
        return -1;
    }

    unit->comm_fail_count = 0;
    shared_connection_state_set(unit->cfg, CONNECTION_CONNECTED);

    LOG_INFO("[UPS] %s: connected.", unit->cfg->name);
    return 0;
}

/**
 * @brief process_callback – drain queue, then read mapped registers.
 * @param arg ups_unit_t pointer.
 * @return 0 on success, -1 on communication failure.
 */
static int ups_process_callback(void *arg)
{
    ups_unit_t *unit = (ups_unit_t *)arg;

    if (!unit || !unit->cfg) {
        return -1;
    }

    if (atomic_exchange(&unit->init_requested, false)) {
        run_init_sequence(unit);
    }
    ups_write_cmd_t cmd;
    if (queue_pop(&unit->cmd_queue, &cmd) == 0) {
        const char *path = ups_bus_path(unit);

        bus_coord_acquire(path);

        do {
            if (write_registers_locked(unit, cmd.addr, cmd.values,
                                       cmd.count, cmd.mode) != 0) {
                LOG_WARNING("[UPS] %s: queued write to 0x%04X failed, "
                            "continuing scan.",
                            unit->cfg->name, cmd.addr);
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
static int read_profile_to_pool(ups_unit_t *unit, bool track_comm_fail)
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

        int result = unit_read_holding_registers(unit, start, count, buf);

        if (result != 0) {
            if (!track_comm_fail) {
                LOG_WARNING("[UPS] %s init read 0x%04X len %u failed (err %d).",
                            cfg->name, start, count, result);
                return -1;
            }

            unit->comm_fail_count++;
            LOG_WARNING("[UPS] %s read 0x%04X len %u failed (err %d, fail %d/%d)",
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
 * @brief Run init sequence and update int_ups_init_flag_reg.
 * @param unit Target unit.
 */
static void run_init_sequence(ups_unit_t *unit)
{
    pool_write_register(int_ups_init_flag_reg, 0);

    if (read_profile_to_pool(unit, false) != 0) {
        LOG_ERROR("[UPS] %s: init sequence failed; init flag stays 0.",
                  unit->cfg->name);
        return;
    }

    pool_write_register(int_ups_init_flag_reg, 1);
    LOG_INFO("[UPS] %s: init sequence complete.",
             unit->cfg->name);
}

/**
 * @brief error_callback – disconnect and wait before reconnect.
 * @param arg ups_unit_t pointer.
 * @param connection_state New connection state.
 * @return 0.
 */
static int ups_error_callback(void *arg, int connection_state)
{
    ups_unit_t *unit = (ups_unit_t *)arg;

    if (!unit || !unit->cfg) {
        return 0;
    }

    unit_disconnect(unit);
    unit->comm_fail_count = 0;

    shared_connection_state_set(unit->cfg, (connection_state_t)connection_state);

    LOG_WARNING("[UPS] %s: disconnected (state=%d). Retrying in %u ms …",
                unit->cfg->name, connection_state,
                MODBUS_DEFAULT_RECONNECT_DELAY_MS);

    interruptible_sleep_ms(unit, MODBUS_DEFAULT_RECONNECT_DELAY_MS);
    return 0;
}

/**
 * @brief msg_callback – synchronous write path (unused; queue is preferred).
 * @param arg ups_unit_t pointer.
 * @param addr Device register address.
 * @param values Values in host byte order.
 * @param count Register count.
 * @return 0 on success, -1 on failure.
 */
static int ups_msg_callback(void *arg,
                            uint16_t addr, uint16_t *values, size_t count)
{
    ups_unit_t *unit = (ups_unit_t *)arg;

    if (!unit || !unit->cfg || !values || count == 0) {
        return -1;
    }

    return write_registers_to_device(unit, addr, values, (uint16_t)count,
                                     UPS_WRITE_MODE_AUTO);
}

/**
 * @brief Initialize a command queue.
 * @param q Queue to initialize.
 */
static void queue_init(ups_cmd_queue_t *q)
{
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->lock, NULL);
}

/**
 * @brief Destroy a command queue.
 * @param q Queue to destroy.
 */
static void queue_destroy(ups_cmd_queue_t *q)
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

            idx = (idx + 1u) % MODBUS_DEFAULT_CMD_QUEUE_CAPACITY;
        }
    }

    if (q->count >= MODBUS_DEFAULT_CMD_QUEUE_CAPACITY) {
        pthread_mutex_unlock(&q->lock);
        return -1;
    }

    ups_write_cmd_t *entry = &q->entries[q->tail];
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
static int queue_pop(ups_cmd_queue_t *q, ups_write_cmd_t *out)
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
 * @brief Find a running UPS unit by modbus uid.
 * @param uid Target modbus_uid.
 * @return Unit pointer, or NULL if not found.
 */
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
 * @brief Sleep until duration_ms elapses or unit stops.
 * @param unit Unit used for running flag.
 * @param duration_ms Sleep duration in milliseconds.
 */
static void interruptible_sleep_ms(const ups_unit_t *unit, uint32_t duration_ms)
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
 * @brief Connect TCP or RTU transport for one unit.
 * @param unit Target unit.
 * @return 0 on success, -1 on failure.
 */
static int unit_connect(ups_unit_t *unit)
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

        LOG_INFO("[UPS] %s: connecting to %s:%d uid=%d",
                 cfg->name, cfg->ip, cfg->port, cfg->modbus_uid);

        if (mb_tcp_client_connect(&unit->tcp_ctx, &tcp_cfg) != 0) {
            LOG_ERROR("[UPS] %s: TCP connection failed.", cfg->name);
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

    LOG_INFO("[UPS] %s: opening %s @ %u baud uid=%d",
             cfg->name, cfg->path, cfg->baud_rate, cfg->modbus_uid);

    if (mb_rtu_client_connect(&unit->rtu_ctx, &rtu_cfg) != 0) {
        LOG_ERROR("[UPS] %s: failed to open serial port %s.",
                  cfg->name, cfg->path);
        return -1;
    }

    if (unit->profile->table_count > 0) {
        uint16_t probe_addr = unit->profile->table[0].device_address;
        const char *path = ups_bus_path(unit);

        bus_coord_acquire(path);
        int probe_rc = mb_rtu_client_probe_device(
            &unit->rtu_ctx, probe_addr, 1);
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
 * @brief Disconnect TCP or RTU transport for one unit.
 * @param unit Target unit.
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
 * @brief FC03 read via active transport.
 * @param unit Target unit.
 * @param addr Start register address.
 * @param count Number of registers.
 * @param out Output buffer.
 * @return 0 on success, or a transport/Modbus error code.
 */
static int unit_read_holding_registers(ups_unit_t *unit, uint16_t addr,
                                       uint16_t count, uint16_t *out)
{
    if (unit->cfg->format == MODBUS_FORMAT_TCP) {
        return mb_tcp_client_read_holding_registers(&unit->tcp_ctx, addr, count, out);
    }

    const char *path = ups_bus_path(unit);
    int result;

    bus_coord_acquire(path);
    result = mb_rtu_client_read_holding_registers(&unit->rtu_ctx, addr, count, out);
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
static int write_registers_locked(ups_unit_t *unit,
                                  uint16_t addr,
                                  const uint16_t *values,
                                  uint16_t count,
                                  ups_write_mode_t mode)
{
    bool is_tcp = (unit->cfg->format == MODBUS_FORMAT_TCP);
    int result;

    switch (mode) {
    case UPS_WRITE_MODE_FC06:
        if (count != 1) {
            LOG_ERROR("[UPS] %s: FC06 requires count=1 (got %u) at 0x%04X.",
                      unit->cfg->name, count, addr);
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

    if (result != 0) {
        LOG_ERROR("[UPS] %s: write to 0x%04X failed (err %d).",
                  unit->cfg->name, addr, result);
        return -1;
    }

    if (count == 1) {
        LOG_VERBOSE("[UPS] %s: wrote register 0x%04X value=0x%04X.",
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

        LOG_VERBOSE("[UPS] %s: wrote %u register(s) at 0x%04X values=[%s].",
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
static int write_registers_to_device(ups_unit_t *unit,
                                     uint16_t addr,
                                     const uint16_t *values,
                                     uint16_t count,
                                     ups_write_mode_t mode)
{
    const char *path = ups_bus_path(unit);

    bus_coord_acquire(path);
    int result = write_registers_locked(unit, addr, values, count, mode);
    if (result == 0) {
        usleep(MODBUS_DEFAULT_POST_WRITE_SETTLE_US);
    }
    bus_coord_release(path);
    return result;
}

/**
 * @brief Poll thread for one UPS unit.
 * @param arg ups_unit_t pointer.
 * @return NULL.
 */
static void *ups_thread(void *arg)
{
    ups_unit_t *unit = (ups_unit_t *)arg;
    module_config_t *cfg = unit->cfg;

    LOG_INFO("[UPS] Thread started: %s (profile=%s uid=%d)",
             cfg->name, unit->profile->name, cfg->modbus_uid);

    while (unit->running) {
        if (unit->callbacks.init_callback(unit) != 0) {
            LOG_ERROR("[UPS] %s: init failed, will retry.", cfg->name);
            interruptible_sleep_ms(unit, MODBUS_DEFAULT_RECONNECT_DELAY_MS);
            continue;
        }
        while (unit->running) {
            if (unit->callbacks.process_callback(unit) != 0) {
                if (unit->callbacks.error_callback) {
                    unit->callbacks.error_callback(unit, CONNECTION_DISCONNECTED);
                }
                break;
            }
        }
    }

    unit_disconnect(unit);
    LOG_INFO("[UPS] Thread stopped: %s", cfg->name);
    return NULL;
}

