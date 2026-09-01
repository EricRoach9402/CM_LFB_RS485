/**
 * @file ups_module.h
 * @brief UPS module public API.
 */

#ifndef UPS_MODULE_H
#define UPS_MODULE_H

#include <stdint.h>

#include "config_loader.h"

/**
 * @brief Start all enabled UPS units.
 * @param ups Config array from global_config.
 * @param ups_count Number of entries in ups.
 * @return 0 on success, -1 if any unit fails to start.
 */
int start_ups_modules(module_config_t ups[], int ups_count);

/**
 * @brief Stop all running UPS units and join threads.
 * @return 0 on success.
 */
int stop_ups_modules(void);

/**
 * @brief Return modbus_uid of the first running UPS unit.
 * @param out_uid Output uid; parent process only.
 * @return 0 on success, -1 if no unit is running.
 */
int ups_get_primary_uid(uint8_t *out_uid);

/**
 * @brief Modbus write FC selection for ups_cmd_push().
 */
typedef enum {
    UPS_WRITE_MODE_AUTO,
    UPS_WRITE_MODE_FC06,
    UPS_WRITE_MODE_FC16,
} ups_write_mode_t;

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
                 ups_write_mode_t mode);

/**
 * @brief Request the init sequence for one UPS unit.
 * @param uid Target modbus_uid.
 * @return 0 if accepted, -1 if unit not found.
 */
int ups_init_request(uint8_t uid);

#endif /* UPS_MODULE_H */
