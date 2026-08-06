/**
 * @file modbus_rtu_client.c
 * @brief Modbus RTU client (master) – serial port implementation.
 *
 * Transport layer
 * ───────────────
 *  Communicates via a POSIX serial port (termios).  The port is opened in
 *  raw mode, 8N1, no hardware or software flow control.  Baud rate is set
 *  via cfsetispeed / cfsetospeed.
 *
 * Request / response cycle
 * ────────────────────────
 *  1. Build the RTU ADU in a local buffer (slave_addr + PDU + CRC).
 *  2. Flush pending input (tcflush) to discard stale data.
 *  3. Write the full ADU to the serial port.
 *  4. Wait for the response using select() with response_timeout_ms.
 *  5. Read the expected number of bytes, verify CRC.
 *  6. Parse the PDU and copy register values to the caller's buffer.
 *
 * CRC16
 * ─────
 *  Standard Modbus CRC16: polynomial 0xA001, initial value 0xFFFF.
 *  The two CRC bytes are appended LSB-first (little-endian).
 *
 * Thread safety
 * ─────────────
 *  All public functions acquire mb_rtu_client_ctx_t.lock before touching
 *  the serial file descriptor or ADU buffers.
 *
 *  Multiple contexts on the same serial_path share one open file descriptor,
 *  one bus mutex, and one handoff-timing record (see mb_rtu_shared_bus).
 */

#include "modbus_rtu_client.h"
#include "log.h"

#include <errno.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>

/* ── Internal constants ───────────────────────────────────────────────── */

/** Default response timeout when cfg.response_timeout_ms == 0. */
#define RTU_DEFAULT_TIMEOUT_MS   1000u

/**
 * Inter-frame silent interval between a request and its response on the
 * same transaction.  At 9600 baud, 3.5 character times ≈ 4 ms.  We use a
 * conservative 5 ms.
 */
#define RTU_INTER_FRAME_DELAY_US 5000u

/**
 * Minimum bus silence between consecutive transactions from contexts with
 * different unit_id values on the same serial_path.
 */
#define RTU_MIN_BUS_HANDOFF_SILENCE_MS 30u

/** Max distinct serial_path values that can share a bus lock at once. */
#define MB_RTU_MAX_SHARED_BUSES  8

static int configure_serial_port(int fd, uint32_t baud_rate, uint32_t timeout_ms);

/* ── Shared serial-bus lock registry ──────────────────────────────────── */

/**
 * @brief Per-serial_path shared port state.
 *
 * All mb_rtu_client_ctx_t instances on the same serial_path reference one
 * record here: a single open fd, reference count, bus mutex, and handoff
 * timing for inter-unit_id transactions.
 */
typedef struct mb_rtu_shared_bus {
    char            path[256];
    pthread_mutex_t mutex;
    bool            in_use;
    int             fd;
    unsigned int    refcnt;
    uint32_t        baud_rate;
    bool            has_last_transaction;
    uint8_t         last_unit_id;
    struct timespec last_release_time;
} mb_rtu_shared_bus_t;

static mb_rtu_shared_bus_t shared_buses[MB_RTU_MAX_SHARED_BUSES];
static pthread_mutex_t     shared_buses_registry_lock = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief Return the shared-bus record for a serial_path, creating it if needed.
 *
 * @param path  Serial device path; used verbatim as the lookup key (two
 *              different path strings for the same physical bus will NOT
 *              be recognized as the same bus).
 * @return Pointer to the shared-bus record, or NULL if the registry is full.
 */
static mb_rtu_shared_bus_t *acquire_shared_bus(const char *path)
{
    pthread_mutex_lock(&shared_buses_registry_lock);

    for (int i = 0; i < MB_RTU_MAX_SHARED_BUSES; i++) {
        if (shared_buses[i].in_use &&
            strcmp(shared_buses[i].path, path) == 0) {
            pthread_mutex_unlock(&shared_buses_registry_lock);
            return &shared_buses[i];
        }
    }

    for (int i = 0; i < MB_RTU_MAX_SHARED_BUSES; i++) {
        if (!shared_buses[i].in_use) {
            memset(&shared_buses[i], 0, sizeof(shared_buses[i]));
            strncpy(shared_buses[i].path, path, sizeof(shared_buses[i].path) - 1);
            pthread_mutex_init(&shared_buses[i].mutex, NULL);
            shared_buses[i].in_use = true;
            shared_buses[i].fd     = -1;
            pthread_mutex_unlock(&shared_buses_registry_lock);
            return &shared_buses[i];
        }
    }

    pthread_mutex_unlock(&shared_buses_registry_lock);
    return NULL;
}

/**
 * @brief Open or attach to the shared serial port for one bus record.
 *
 * The first context on a path opens and configures the port; later contexts
 * reuse the same fd and must use the same baud_rate.
 *
 * @return 0 on success, -1 on error.
 */
static int shared_bus_attach_port(mb_rtu_shared_bus_t *bus,
                                   uint32_t             baud_rate,
                                   uint32_t             timeout_ms,
                                   int                 *out_fd)
{
    if (!bus || !out_fd) {
        return -1;
    }

    pthread_mutex_lock(&bus->mutex);

    if (bus->fd < 0) {
        int fd = open(bus->path, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0) {
            pthread_mutex_unlock(&bus->mutex);
            return -1;
        }

        int flags = fcntl(fd, F_GETFL, 0);
        if (flags != -1) {
            fcntl(fd, F_SETFL, flags & ~O_NDELAY);
        }

        if (configure_serial_port(fd, baud_rate, timeout_ms) != 0) {
            close(fd);
            pthread_mutex_unlock(&bus->mutex);
            return -1;
        }

        bus->fd         = fd;
        bus->baud_rate  = baud_rate;
        bus->refcnt     = 0;
        bus->has_last_transaction = false;
    } else if (bus->baud_rate != baud_rate) {
        LOG_ERROR("[modbus_rtu] %s: baud_rate %u conflicts with open bus %u",
                  bus->path, baud_rate, bus->baud_rate);
        pthread_mutex_unlock(&bus->mutex);
        return -1;
    }

    bus->refcnt++;
    *out_fd = bus->fd;
    pthread_mutex_unlock(&bus->mutex);
    return 0;
}

/**
 * @brief Release one context's reference to the shared serial port.
 *
 * Closes the port when the last context detaches.
 */
static void shared_bus_detach_port(mb_rtu_shared_bus_t *bus)
{
    if (!bus) {
        return;
    }

    pthread_mutex_lock(&bus->mutex);

    if (bus->refcnt > 0) {
        bus->refcnt--;
    }

    if (bus->refcnt == 0 && bus->fd >= 0) {
        close(bus->fd);
        bus->fd = -1;
        bus->has_last_transaction = false;
    }

    pthread_mutex_unlock(&bus->mutex);
}

/**
 * @brief Wait for the remaining handoff silence, if required.
 *
 * Applies only when the previous transaction on this bus used a different
 * unit_id.
 */
static void enforce_bus_handoff_silence(mb_rtu_shared_bus_t *bus, uint8_t unit_id)
{
    if (!bus->has_last_transaction || bus->last_unit_id == unit_id) {
        return;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    long elapsed_ms = (now.tv_sec  - bus->last_release_time.tv_sec)  * 1000L +
                      (now.tv_nsec - bus->last_release_time.tv_nsec) / 1000000L;

    if (elapsed_ms >= 0 && elapsed_ms < (long)RTU_MIN_BUS_HANDOFF_SILENCE_MS) {
        usleep((useconds_t)(RTU_MIN_BUS_HANDOFF_SILENCE_MS - (uint32_t)elapsed_ms) * 1000u);
    }
}

/* ── CRC16 ─────────────────────────────────────────────────────────────── */

/**
 * @brief Compute Modbus CRC16 (polynomial 0xA001, init 0xFFFF).
 *
 * @param data    Byte array to compute CRC over.
 * @param length  Number of bytes.
 * @return 16-bit CRC value.
 */
uint16_t mb_rtu_compute_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;
    size_t   i;
    int      bit;

    for (i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (bit = 0; bit < 8; bit++) {
            if (crc & 0x0001u) {
                crc = (crc >> 1) ^ 0xA001u;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ── Baud rate mapping ────────────────────────────────────────────────── */

/**
 * @brief Map a numeric baud rate to the corresponding termios B* constant.
 *
 * @param baud_rate  Numeric baud rate (e.g. 9600).
 * @return Corresponding speed_t, or B9600 as default for unknown values.
 */
static speed_t map_baud_rate(uint32_t baud_rate)
{
    switch (baud_rate) {
    case 1200:   return B1200;
    case 2400:   return B2400;
    case 4800:   return B4800;
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B9600;
    }
}

/* ── Serial port helpers ──────────────────────────────────────────────── */

/**
 * @brief Configure the serial port for Modbus RTU (raw, 8N1, no flow ctrl).
 *
 * @param fd          Open file descriptor.
 * @param baud_rate   Desired baud rate.
 * @param timeout_ms  Read timeout in milliseconds (rounded to 100 ms steps
 *                    via VTIME; minimum 100 ms).
 * @return 0 on success, -1 on error.
 */
static int configure_serial_port(int fd, uint32_t baud_rate, uint32_t timeout_ms)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        return -1;
    }

    speed_t speed = map_baud_rate(baud_rate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    /* 8N1 – 8 data bits, no parity, 1 stop bit */
    tty.c_cflag &= ~(speed_t)PARENB;
    tty.c_cflag &= ~(speed_t)CSTOPB;
    tty.c_cflag &= ~(speed_t)CSIZE;
    tty.c_cflag |=  (speed_t)CS8;

    /* Disable hardware flow control */
    tty.c_cflag &= ~(speed_t)CRTSCTS;

    /* Enable receiver; ignore modem control lines */
    tty.c_cflag |= (speed_t)(CREAD | CLOCAL);

    /* Raw input – no canonical mode, no echo, no signals */
    tty.c_lflag &= ~(tcflag_t)(ICANON | ECHO | ECHOE | ISIG);

    /* Raw output – no post-processing */
    tty.c_oflag &= ~(tcflag_t)OPOST;

    /* Disable software flow control */
    tty.c_iflag &= ~(tcflag_t)(IXON | IXOFF | IXANY);

    /* Disable special byte mappings */
    tty.c_iflag &= ~(tcflag_t)(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    /*
     * VMIN=0, VTIME=N: non-blocking read, return as soon as data is available
     * or after N×100 ms.  We use select() for the timeout instead, so set
     * VTIME to a small value to avoid indefinite blocking inside read().
     */
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = (cc_t)((timeout_ms / 100u) + 1u);  /* deciseconds, min 1 */

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return -1;
    }

    return 0;
}

/**
 * @brief Wait until data is available to read, or the timeout expires.
 *
 * @param fd          File descriptor.
 * @param timeout_ms  Maximum wait time in milliseconds.
 * @return 1 if data available, 0 on timeout, -1 on error.
 */
static int wait_for_data(int fd, uint32_t timeout_ms)
{
    fd_set         read_fds;
    struct timeval tv;
    int            result;

    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    tv.tv_sec  = (time_t)(timeout_ms / 1000u);
    tv.tv_usec = (suseconds_t)((timeout_ms % 1000u) * 1000u);

    do {
        result = select(fd + 1, &read_fds, NULL, NULL, &tv);
    } while (result == -1 && errno == EINTR);

    return result;
}

/**
 * @brief Read exactly n bytes from the serial port within the timeout.
 *
 * @param fd            File descriptor.
 * @param buf           Destination buffer.
 * @param n             Exact number of bytes expected.
 * @param timeout_ms    Overall timeout in milliseconds.
 * @param out_received  Optional; set to the number of bytes actually
 *                       received so far, even on timeout/error – lets the
 *                       caller tell "nothing at all arrived" apart from
 *                       "a partial/short frame arrived" for diagnostics.
 * @return Number of bytes read on success, -1 on error, 0 on timeout.
 */
static int read_exact(int fd, uint8_t *buf, size_t n, uint32_t timeout_ms,
                       size_t *out_received)
{
    size_t  received = 0;
    ssize_t ret;

    while (received < n) {
        int ready = wait_for_data(fd, timeout_ms);
        if (ready == 0) {
            if (out_received) {
                *out_received = received;
            }
            return 0;   /* timeout */
        }
        if (ready < 0) {
            if (out_received) {
                *out_received = received;
            }
            return -1;  /* select error */
        }

        ret = read(fd, buf + received, n - received);
        if (ret <= 0) {
            if (out_received) {
                *out_received = received;
            }
            return -1;
        }
        received += (size_t)ret;
    }

    if (out_received) {
        *out_received = received;
    }
    return (int)received;
}

/* ── ADU helpers ──────────────────────────────────────────────────────── */

/**
 * @brief Append a 16-bit value big-endian into a buffer.
 */
static void append_uint16_be(uint8_t *buf, size_t *pos, uint16_t value)
{
    buf[(*pos)++] = (uint8_t)(value >> 8);
    buf[(*pos)++] = (uint8_t)(value & 0xFFu);
}

/**
 * @brief Append CRC16 (LSB first) to an RTU ADU.
 *
 * @param buf  Buffer containing the ADU (without CRC).
 * @param len  Current length of the ADU (will be incremented by 2).
 */
static void append_crc(uint8_t *buf, size_t *len)
{
    uint16_t crc = mb_rtu_compute_crc16(buf, *len);
    buf[(*len)++] = (uint8_t)(crc & 0xFFu);   /* CRC low byte */
    buf[(*len)++] = (uint8_t)(crc >> 8);       /* CRC high byte */
}

/**
 * @brief Verify the CRC at the end of a received RTU ADU.
 *
 * @param buf  Full response buffer including the two CRC bytes at the end.
 * @param len  Total buffer length (including CRC bytes).
 * @return true if CRC matches, false otherwise.
 */
static bool verify_response_crc(const uint8_t *buf, size_t len)
{
    if (len < MODBUS_RTU_CRC_LEN) {
        return false;
    }
    uint16_t computed = mb_rtu_compute_crc16(buf, len - MODBUS_RTU_CRC_LEN);
    uint16_t received = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
    return (computed == received);
}

/**
 * @brief Format bytes as a space-separated upper-case hex string.
 *
 * @param buf       Byte buffer to format.
 * @param len       Number of bytes to format.
 * @param out       Destination string buffer.
 * @param out_size  Size of the destination buffer, including the
 *                   terminator; the output is truncated if too small.
 */
static void format_hex_string(const uint8_t *buf, size_t len,
                               char *out, size_t out_size)
{
    size_t pos = 0;

    for (size_t i = 0; i < len && pos + 3u < out_size; i++) {
        pos += (size_t)snprintf(out + pos, out_size - pos, "%02X ", buf[i]);
    }

    out[pos] = '\0';
}

/* ── Core transaction ─────────────────────────────────────────────────── */

/**
 * @brief Execute one Modbus RTU request–response transaction.
 *
 * Caller must hold ctx->lock.
 *
 * @param ctx          Open client context.
 * @param req          Request ADU (with CRC already appended).
 * @param req_len      Length of the request ADU in bytes.
 * @param resp         Buffer to receive the response ADU (caller-supplied).
 * @param resp_len     Expected response length in bytes (including CRC).
 * @return MB_RTU_CLIENT_OK on success, or a negative MB_RTU_CLIENT_ERR_* code.
 */
static int do_transaction(mb_rtu_client_ctx_t *ctx,
                           const uint8_t *req,  size_t req_len,
                           uint8_t       *resp, size_t resp_len)
{
    /*
     * Serialize against every other context on this same serial_path
     * (other modules/units sharing this bus), on top of ctx->lock which
     * the caller already holds for this ctx alone.
     */
    if (ctx->shared_bus) {
        pthread_mutex_lock(&ctx->shared_bus->mutex);
        enforce_bus_handoff_silence(ctx->shared_bus, ctx->cfg.unit_id);
    }

    uint32_t timeout_ms = (ctx->cfg.response_timeout_ms > 0u)
                          ? ctx->cfg.response_timeout_ms
                          : RTU_DEFAULT_TIMEOUT_MS;

    int    result;
    size_t received_bytes = 0u;

    /* Flush stale input before sending a new request */
    tcflush(ctx->fd, TCIFLUSH);

    /* Transmit request */
    ssize_t written = write(ctx->fd, req, req_len);
    if (written != (ssize_t)req_len) {
        result = MB_RTU_CLIENT_ERR_TRANSPORT;
    } else {
        /* Brief inter-frame silence before reading */
        usleep(RTU_INTER_FRAME_DELAY_US);

        /* Receive response */
        int received = read_exact(ctx->fd, resp, resp_len, timeout_ms,
                                   &received_bytes);

        if (received == 0) {
            result = MB_RTU_CLIENT_ERR_TIMEOUT;
        } else if (received < 0) {
            result = MB_RTU_CLIENT_ERR_TRANSPORT;
        } else if (!verify_response_crc(resp, resp_len)) {
            result = MB_RTU_CLIENT_ERR_CRC;
        } else if (resp[0] != ctx->cfg.unit_id) {
            result = MB_RTU_CLIENT_ERR_FRAME;
        } else if (resp[1] & MODBUS_EXCEPTION_FLAG) {
            result = (resp_len >= 3u + MODBUS_RTU_CRC_LEN)
                     ? (int)resp[2]  /* positive exception code */
                     : MB_RTU_CLIENT_ERR_FRAME;
        } else {
            result = MB_RTU_CLIENT_OK;
        }
    }

    /*
     * Diagnostic hex dump on any failure – shows exactly what left the
     * transmitter and whether any bytes at all came back, which tells
     * "silent bus" (received_bytes == 0) apart from "corrupted/short
     * frame" (0 < received_bytes < resp_len) apart from "wrong slave
     * answered" (full frame, but resp[0] mismatch or bad CRC).
     */
    if (result != MB_RTU_CLIENT_OK) {
        char tx_hex[MODBUS_RTU_MAX_ADU_LENGTH * 3u];
        char rx_hex[MODBUS_RTU_MAX_ADU_LENGTH * 3u];

        format_hex_string(req, req_len, tx_hex, sizeof(tx_hex));
        format_hex_string(resp, received_bytes, rx_hex, sizeof(rx_hex));

        LOG_WARNING("uid=%d transaction failed (err=%d): tx[%zu]={%s} "
                    "rx[%zu/%zu]={%s}",
                    ctx->cfg.unit_id, result, req_len, tx_hex,
                    received_bytes, resp_len, rx_hex);
    }

    if (ctx->shared_bus) {
        clock_gettime(CLOCK_MONOTONIC, &ctx->shared_bus->last_release_time);
        ctx->shared_bus->last_unit_id           = ctx->cfg.unit_id;
        ctx->shared_bus->has_last_transaction   = true;
        pthread_mutex_unlock(&ctx->shared_bus->mutex);
    }

    return result;
}

/* ── Public API ───────────────────────────────────────────────────────── */

int mb_rtu_client_connect(mb_rtu_client_ctx_t          *ctx,
                           const mb_rtu_client_config_t *cfg)
{
    if (!ctx || !cfg || !cfg->serial_path) {
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->fd  = -1;
    ctx->cfg = *cfg;

    pthread_mutex_init(&ctx->lock, NULL);
    ctx->shared_bus = acquire_shared_bus(cfg->serial_path);
    if (!ctx->shared_bus) {
        pthread_mutex_destroy(&ctx->lock);
        return -1;
    }

    uint32_t timeout_ms = (cfg->response_timeout_ms > 0u)
                          ? cfg->response_timeout_ms
                          : RTU_DEFAULT_TIMEOUT_MS;

    if (shared_bus_attach_port(ctx->shared_bus, cfg->baud_rate,
                                timeout_ms, &ctx->fd) != 0) {
        ctx->shared_bus = NULL;
        pthread_mutex_destroy(&ctx->lock);
        return -1;
    }

    return 0;
}

void mb_rtu_client_disconnect(mb_rtu_client_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    pthread_mutex_lock(&ctx->lock);

    if (ctx->fd >= 0 && ctx->shared_bus) {
        shared_bus_detach_port(ctx->shared_bus);
        ctx->fd = -1;
    }

    pthread_mutex_unlock(&ctx->lock);
    pthread_mutex_destroy(&ctx->lock);
}

int mb_rtu_client_read_holding_registers(mb_rtu_client_ctx_t *ctx,
                                          uint16_t addr,
                                          uint16_t qty,
                                          uint16_t *out)
{
    if (!ctx || !out || qty == 0 || qty > MODBUS_MAX_READ_REGISTERS) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    pthread_mutex_lock(&ctx->lock);

    if (ctx->fd < 0) {
        pthread_mutex_unlock(&ctx->lock);
        return MB_RTU_CLIENT_ERR_NOT_OPEN;
    }

    /* Build FC03 request ADU */
    uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH];
    size_t  req_len = 0;

    req[req_len++] = ctx->cfg.unit_id;
    req[req_len++] = MODBUS_FUNC_READ_HOLDING_REGISTERS;
    append_uint16_be(req, &req_len, addr);
    append_uint16_be(req, &req_len, qty);
    append_crc(req, &req_len);

    /* Expected response: [addr(1)] [fc(1)] [byte_count(1)] [data(qty*2)] [CRC(2)] */
    size_t  resp_len = 1u + 1u + 1u + (size_t)qty * 2u + MODBUS_RTU_CRC_LEN;
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];

    int result = do_transaction(ctx, req, req_len, resp, resp_len);

    if (result == MB_RTU_CLIENT_OK) {
        /* Validate function code and byte count */
        if (resp[1] != MODBUS_FUNC_READ_HOLDING_REGISTERS ||
            resp[2] != (uint8_t)(qty * 2u)) {
            result = MB_RTU_CLIENT_ERR_FRAME;
        } else {
            /* Copy register values, converting from big-endian */
            for (uint16_t i = 0; i < qty; i++) {
                out[i] = ((uint16_t)resp[3 + i * 2u] << 8) |
                          (uint16_t)resp[3 + i * 2u + 1u];
            }
        }
    }

    pthread_mutex_unlock(&ctx->lock);
    return result;
}

int mb_rtu_client_write_single_register(mb_rtu_client_ctx_t *ctx,
                                         uint16_t addr,
                                         uint16_t value)
{
    if (!ctx) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    pthread_mutex_lock(&ctx->lock);

    if (ctx->fd < 0) {
        pthread_mutex_unlock(&ctx->lock);
        return MB_RTU_CLIENT_ERR_NOT_OPEN;
    }

    /* Build FC06 request ADU */
    uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH];
    size_t  req_len = 0;

    req[req_len++] = ctx->cfg.unit_id;
    req[req_len++] = MODBUS_FUNC_WRITE_SINGLE_REGISTER;
    append_uint16_be(req, &req_len, addr);
    append_uint16_be(req, &req_len, value);
    append_crc(req, &req_len);

    /* FC06 response echoes the request (8 bytes total with CRC) */
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];
    size_t  resp_len = req_len;  /* echo: same size as request */

    int result = do_transaction(ctx, req, req_len, resp, resp_len);

    if (result == MB_RTU_CLIENT_OK) {
        /* Verify FC06 echo */
        if (resp[1] != MODBUS_FUNC_WRITE_SINGLE_REGISTER) {
            result = MB_RTU_CLIENT_ERR_FRAME;
        }
    }

    pthread_mutex_unlock(&ctx->lock);
    return result;
}

int mb_rtu_client_probe_device(mb_rtu_client_ctx_t *ctx,
                                uint16_t addr, uint16_t qty)
{
    uint16_t probe_buf[MODBUS_MAX_READ_REGISTERS];

    if (qty == 0 || qty > MODBUS_MAX_READ_REGISTERS) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    return mb_rtu_client_read_holding_registers(ctx, addr, qty, probe_buf);
}

int mb_rtu_client_write_multiple_registers(mb_rtu_client_ctx_t *ctx,
                                            uint16_t addr,
                                            uint16_t qty,
                                            const uint16_t *data)
{
    if (!ctx || !data || qty == 0 || qty > MODBUS_MAX_WRITE_REGISTERS) {
        return MB_RTU_CLIENT_ERR_ARG;
    }

    pthread_mutex_lock(&ctx->lock);

    if (ctx->fd < 0) {
        pthread_mutex_unlock(&ctx->lock);
        return MB_RTU_CLIENT_ERR_NOT_OPEN;
    }

    /* Build FC16 request ADU */
    uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH];
    size_t  req_len = 0;

    req[req_len++] = ctx->cfg.unit_id;
    req[req_len++] = MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS;
    append_uint16_be(req, &req_len, addr);
    append_uint16_be(req, &req_len, qty);
    req[req_len++] = (uint8_t)(qty * 2u);  /* byte count */

    for (uint16_t i = 0; i < qty; i++) {
        append_uint16_be(req, &req_len, data[i]);
    }
    append_crc(req, &req_len);

    /*
     * FC16 response: [slave_addr(1)] [fc(1)] [start_addr(2)] [qty(2)] [CRC(2)]
     */
    uint8_t resp[MODBUS_RTU_MAX_ADU_LENGTH];
    size_t  resp_len = 1u + 1u + 2u + 2u + MODBUS_RTU_CRC_LEN;

    int result = do_transaction(ctx, req, req_len, resp, resp_len);

    if (result == MB_RTU_CLIENT_OK) {
        if (resp[1] != MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS) {
            result = MB_RTU_CLIENT_ERR_FRAME;
        }
    }

    pthread_mutex_unlock(&ctx->lock);
    return result;
}
