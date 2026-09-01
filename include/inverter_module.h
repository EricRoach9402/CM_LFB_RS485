/**
 * @file inverter_module.h
 * @brief Inverter module public API.
 */

#ifndef INVERTER_MODULE_H
#define INVERTER_MODULE_H

#include <stdint.h>

#include "config_loader.h"

/**
 * @brief Start all enabled Inverter units.
 * @param inverters Config array from global_config.
 * @param inverter_count Number of entries in inverters.
 * @return 0 on success, -1 if any unit fails to start.
 */
int start_inverter_modules(module_config_t inverters[], int inverter_count);

/**
 * @brief Stop all running Inverter units and join threads.
 * @return 0 on success.
 */
int stop_inverter_modules(void);

/**
 * @brief Return modbus_uid of the first running Inverter unit.
 * @param out_uid Output uid; parent process only.
 * @return 0 on success, -1 if no unit is running.
 */
int inverter_get_primary_uid(uint8_t *out_uid);

/**
 * @brief Modbus write FC selection for inverter_cmd_push().
 */
typedef enum {
    INVERTER_WRITE_MODE_AUTO,
    INVERTER_WRITE_MODE_FC06,
    INVERTER_WRITE_MODE_FC16,
} inverter_write_mode_t;

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
                      inverter_write_mode_t mode);

/**
 * @brief Request the init sequence for one Inverter unit.
 * @param uid Target modbus_uid.
 * @return 0 if accepted, -1 if unit not found.
 */
int inverter_init_request(uint8_t uid);

#endif /* INVERTER_MODULE_H */
