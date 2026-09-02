/**
 * @file modbus_defaults.h
 * @brief Application-level Modbus polling and timing defaults.
 *
 * Tuning for this daemon's device modules (inverter_module.c, ups_module.c, …).
 * Lives under include/, not lib/, because these values are deployment policy
 * for CM_LFB_RS485, not portable library constants.
 *
 * To override for one device family, #define the symbol before including
 * this header in that module's .c file.
 */

#ifndef MODBUS_DEFAULTS_H
#define MODBUS_DEFAULTS_H

/** Delay before reconnect after comm failure or disconnect. */
#define MODBUS_DEFAULT_RECONNECT_DELAY_MS           5000u

/** Consecutive read failures before marking pool slots as 0xFFFF. */
#define MODBUS_DEFAULT_COMM_FAIL_THRESHOLD          5

/** Pause between consecutive Modbus transactions on the same bus (µs). */
#define MODBUS_DEFAULT_INTER_SEGMENT_DELAY_US       40000u

/** Settle time after a write burst before the next read (µs). */
#define MODBUS_DEFAULT_POST_WRITE_SETTLE_US         70000u

/** Sleep slice when waiting for shutdown or reconnect (ms). */
#define MODBUS_DEFAULT_SHUTDOWN_CHECK_INTERVAL_MS   100u

/** Per-unit async write command queue depth. */
#define MODBUS_DEFAULT_CMD_QUEUE_CAPACITY           16u

/** Pause after a full poll cycle (queue drain + profile read) before the next round (ms). */
#define MODBUS_DEFAULT_POLL_CYCLE_INTERVAL_MS       200u

#endif /* MODBUS_DEFAULTS_H */
