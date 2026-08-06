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

#endif /* INVERTER_MODULE_H */
