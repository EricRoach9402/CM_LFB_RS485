/**
 * @file modbus_rtu_client.h
 * @brief Modbus RTU client (master) application-facing API.
 *
 * Provides blocking, thread-safe read/write functions for accessing registers
 * on a remote Modbus RTU slave device over a serial port.
 *
 * Relationship to other layers:
 *   modbus_defines.h      -- shared Modbus protocol constants and limits
 *   modbus_rtu_client     -- THIS FILE: serial transport + application API
 *
 * Thread safety:
 *   Multiple threads may share one mb_rtu_client_ctx_t.  Requests are
 *   serialized internally by a mutex: only one request is in flight at a time.
 *
 *   Multiple independent contexts on the same serial_path (different
 *   unit_id values) share one open file descriptor and one process-wide bus
 *   record keyed by serial_path.  Transactions on that path are serialized
 *   across all contexts.  When consecutive transactions use different
 *   unit_id values, a minimum bus silence gap is enforced
 *   (RTU_MIN_BUS_HANDOFF_SILENCE_MS).  All contexts on the same path must
 *   use the same baud_rate.
 *
 * Serial framing:
 *   Each request is a raw Modbus RTU ADU:
 *     [slave_addr (1)] [fc (1)] [payload...] [CRC_lo (1)] [CRC_hi (1)]
 *   CRC16 uses the standard Modbus polynomial (0xA001, LSB first).
 */

#ifndef MODBUS_RTU_CLIENT_H
#define MODBUS_RTU_CLIENT_H

#include <pthread.h>
#include <stdint.h>
#include "modbus_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Return codes ──────────────────────────────────────────────────────── */

/**
 * Return values for all mb_rtu_client_* request functions.
 *
 *   0          – success
 *   > 0        – Modbus exception code (MODBUS_EX_* from modbus_defines.h)
 *   < 0        – transport or framing error (one of the codes below)
 */
#define MB_RTU_CLIENT_OK                 0
#define MB_RTU_CLIENT_ERR_ARG           (-1)  /**< Invalid argument.                  */
#define MB_RTU_CLIENT_ERR_NOT_OPEN      (-2)  /**< Serial port not open; call connect. */
#define MB_RTU_CLIENT_ERR_TRANSPORT     (-3)  /**< Serial write/read error.            */
#define MB_RTU_CLIENT_ERR_TIMEOUT       (-4)  /**< Response not received in time.      */
#define MB_RTU_CLIENT_ERR_FRAME         (-5)  /**< Malformed or unexpected response.   */
#define MB_RTU_CLIENT_ERR_CRC           (-6)  /**< CRC mismatch in response.           */

/* ── Configuration ─────────────────────────────────────────────────────── */

/**
 * @brief Client configuration.
 *
 * All pointer fields must remain valid until mb_rtu_client_disconnect() returns.
 */
typedef struct mb_rtu_client_config {
    const char  *serial_path;           /**< Serial device path (e.g. "/dev/ttyS3"). */
    uint32_t     baud_rate;             /**< Baud rate (e.g. 9600, 19200, 115200).   */
    uint8_t      unit_id;               /**< Modbus slave address for every request. */
    uint32_t     response_timeout_ms;   /**< Per-request timeout; 0 = default (1 s). */
} mb_rtu_client_config_t;

/* ── Runtime context ───────────────────────────────────────────────────── */

/** Opaque handle to this serial_path's shared port / bus-lock record. */
struct mb_rtu_shared_bus;

/**
 * @brief Client runtime context.
 *
 * Zero-initialize before calling mb_rtu_client_connect().
 * Do not modify fields directly after connect.
 *
 * ctx->fd is the shared port fd when several contexts use the same
 * serial_path; all of them read and write through that single descriptor.
 */
typedef struct mb_rtu_client_ctx {
    int                        fd;          /**< Shared serial port fd; -1 when detached. */
    pthread_mutex_t            lock;        /**< Serializes concurrent requests on this ctx. */
    struct mb_rtu_shared_bus  *shared_bus;  /**< Shared port + bus lock for serial_path. */
    mb_rtu_client_config_t     cfg;         /**< Copy of configuration.                      */
} mb_rtu_client_ctx_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

/**
 * @brief Open or attach to the serial port for this context.
 *
 * The first context on a given serial_path opens the device; additional
 * contexts on the same path reuse that fd (same baud_rate required).
 * Initializes the per-context mutex.
 *
 * @param ctx  Zero-initialized context; must remain valid until disconnect.
 * @param cfg  Configuration; pointer fields are used by reference.
 * @return 0 on success, -1 on error.
 */
int mb_rtu_client_connect(mb_rtu_client_ctx_t *ctx,
                           const mb_rtu_client_config_t *cfg);

/**
 * @brief Detach this context from the serial port and release resources.
 *
 * Decrements the shared-port reference count; the port is closed when the
 * last context on that serial_path disconnects.  Safe to call even if
 * connect failed or was never called.
 */
void mb_rtu_client_disconnect(mb_rtu_client_ctx_t *ctx);

/* ── CRC utility ───────────────────────────────────────────────────────── */

/**
 * @brief Compute Modbus CRC16 over an arbitrary byte buffer.
 *
 * Uses the standard Modbus polynomial (0xA001, initial value 0xFFFF).
 * The two CRC bytes in a Modbus RTU ADU are stored LSB-first.
 *
 * @param data    Byte array.
 * @param length  Number of bytes.
 * @return 16-bit CRC value.
 */
uint16_t mb_rtu_compute_crc16(const uint8_t *data, size_t length);

/* ── Register access ───────────────────────────────────────────────────── */

/**
 * @brief FC03 – Read Holding Registers.
 *
 * @param ctx   Open context.
 * @param addr  Starting register address (0-based).
 * @param qty   Number of registers to read (1 – MODBUS_MAX_READ_REGISTERS).
 * @param out   Caller-owned buffer; receives qty values in host byte order.
 * @return MB_RTU_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_RTU_CLIENT_ERR_* code.
 */
int mb_rtu_client_read_holding_registers(mb_rtu_client_ctx_t *ctx,
                                          uint16_t addr, uint16_t qty,
                                          uint16_t *out);

/**
 * @brief FC06 – Write Single Register.
 *
 * @param ctx    Open context.
 * @param addr   Register address (0-based).
 * @param value  Register value in host byte order.
 * @return MB_RTU_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_RTU_CLIENT_ERR_* code.
 */
int mb_rtu_client_write_single_register(mb_rtu_client_ctx_t *ctx,
                                         uint16_t addr, uint16_t value);

/**
 * @brief FC16 – Write Multiple Registers.
 *
 * @param ctx   Open context.
 * @param addr  Starting register address (0-based).
 * @param qty   Number of registers to write (1 – MODBUS_MAX_WRITE_REGISTERS).
 * @param data  Register values in host byte order.
 * @return MB_RTU_CLIENT_OK, a positive Modbus exception code, or a negative
 *         MB_RTU_CLIENT_ERR_* code.
 */
int mb_rtu_client_write_multiple_registers(mb_rtu_client_ctx_t *ctx,
                                            uint16_t addr, uint16_t qty,
                                            const uint16_t *data);

/* ── Device presence probe ─────────────────────────────────────────────── */

/**
 * @brief Probe whether a slave device is actually present and responding.
 *
 * Opening a serial port only proves the local device node exists; it says
 * nothing about whether a Modbus slave is present on the bus.  This issues
 * one FC03 read of qty registers starting at addr and reports whether a
 * valid response was received, so callers can distinguish "port open" from
 * "device online".
 *
 * @param ctx   Open context (mb_rtu_client_connect() must have succeeded).
 * @param addr  A register address known to exist in the device's map.
 * @param qty   Number of registers to read (typically 1).
 * @return MB_RTU_CLIENT_OK if the device responded, a positive Modbus
 *         exception code, or a negative MB_RTU_CLIENT_ERR_* code.
 */
int mb_rtu_client_probe_device(mb_rtu_client_ctx_t *ctx,
                                uint16_t addr, uint16_t qty);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_CLIENT_H */
