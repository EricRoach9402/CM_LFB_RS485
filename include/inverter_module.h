/**
 * @file inverter_module.h
 * @brief Inverter module interface.
 *
 * All Inverter units (regardless of hardware model) are managed by a single
 * inverter_module.c.  Business logic, polling threads, and fault handling
 * live there.  main() only calls start_inverter_modules() and
 * stop_inverter_modules().
 */

#ifndef INVERTER_MODULE_H
#define INVERTER_MODULE_H

#include <stdint.h>

#include "config_loader.h"

/**
 * @brief Start all enabled Inverter modules.
 *
 * Assigns inverter1_profile (the only Inverter hardware model currently
 * implemented) to every enabled unit, and spawns one polling thread per
 * unit.
 *
 * @param inverters       Array of Inverter configurations from global_config.
 * @param inverter_count  Number of entries in the array.
 * @return 0 on success, -1 if any unit fails to start.
 */
int start_inverter_modules(module_config_t inverters[], int inverter_count);

/**
 * @brief Stop all running Inverter modules and release resources.
 *
 * Operates entirely on internal module state; no caller-supplied
 * array is required.
 *
 * @return 0 on success.
 */
int stop_inverter_modules(void);

/* ── CMOS bridge command interface ────────────────────────────────────── */

/**
 * @brief Resolve the modbus_uid of the first running Inverter unit.
 *
 * Reflects the units actually registered by start_inverter_modules(), so
 * entries that are disabled in config.json are never selected.
 *
 * Parent process only: the forked CMOS publisher child holds no unit
 * registry and always gets -1.
 *
 * @param out_uid  Destination for the resolved modbus_uid.
 * @return 0 on success, -1 if no unit is running.
 */
int inverter_get_primary_uid(uint8_t *out_uid);

/**
 * @brief Modbus write function-code selection for inverter_cmd_push().
 *
 *  INVERTER_WRITE_MODE_AUTO – count == 1 → FC06, count > 1 → FC16
 *  INVERTER_WRITE_MODE_FC06 – force FC06 (count must be 1)
 *  INVERTER_WRITE_MODE_FC16 – force FC16 (count >= 1)
 */
typedef enum {
    INVERTER_WRITE_MODE_AUTO,
    INVERTER_WRITE_MODE_FC06,
    INVERTER_WRITE_MODE_FC16,
} inverter_write_mode_t;

/**
 * @brief Enqueue a Modbus register-write command for an Inverter unit.
 *
 * Called from the CMOS bridge thread.  Thread-safe (uses a per-unit mutex).
 * The command is drained by process_callback on the next poll cycle.
 *
 * @param uid     modbus_uid of the target Inverter unit.
 * @param addr    Device register address (device_address in the mapping table).
 * @param values  Register values in host byte order.
 * @param count   Number of registers to write.
 * @param mode    FC selection: AUTO, FC06, or FC16.
 * @return 0 on success, -1 if the unit is not found or the queue is full.
 */
int inverter_cmd_push(uint8_t uid, uint16_t addr,
                      const uint16_t *values, uint16_t count,
                      inverter_write_mode_t mode);

/* ── Init sequence ────────────────────────────────────────────────────── */

/**
 * @brief Request the operator-initiated init sequence for one Inverter unit.
 *
 * Non-blocking: only marks the request.  The sequence itself runs on the
 * unit's polling thread, which owns the register writes and raises the
 * init-finished flag (see inverter_init_status.h).  Callers therefore never
 * need to know which registers init touches or when it completes.
 *
 * May be called repeatedly; a request pending when another arrives is
 * simply absorbed into the next run.
 *
 * @param uid  modbus_uid of the target Inverter unit.
 * @return 0 if the request was accepted, -1 if the unit is not found.
 */
int inverter_init_request(uint8_t uid);

#endif /* INVERTER_MODULE_H */
