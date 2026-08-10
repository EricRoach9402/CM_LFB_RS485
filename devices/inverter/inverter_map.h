/**
 * @file inverter_map.h
 * @brief Inverter device register profile.
 *
 * This project currently implements exactly one Inverter hardware model,
 * inverter1_profile below.  Because there is only one, inverter_module.c
 * (and the CMOS bridge / alarm registration) reference it directly – no
 * uid- or config-driven lookup/selection mechanism exists, and none is
 * needed until a second hardware model is actually introduced.
 *
 * modbus_uid (config.json) is purely a wiring/deployment detail – which
 * slave address a unit answers to on the bus – and never selects a
 * profile.
 *
 * Adding a second hardware model (only once actually needed)
 * ────────────────────────────────────────────────────────────
 *  1. Add a mapping table in inverter_map.c with a unique pool_address
 *     range, plus a new device_map_profile_t definition.
 *  2. Declare the new profile below (extern const device_map_profile_t …).
 *  3. Introduce a real selection mechanism where profiles are currently
 *     referenced directly: start_inverter_modules() in inverter_module.c,
 *     on_write()/publish_all_pool_register() in inverter_cmos_bridge.c,
 *     and inverter_alarm_register_all() in inverter_alarm.c.
 */

#ifndef INVERTER_MAP_H
#define INVERTER_MAP_H

#include "device_register_map.h"

/** Inverter1 model – pool region 0xB000-0xD1FF (see table for exact rows). */
extern const device_map_profile_t inverter1_profile;

// RW
extern const uint16_t int_frequency_cmd_souce_reg;
extern const uint16_t int_run_cmd_souce_reg;
extern const uint16_t int_frequency_upper_limit_reg;
extern const uint16_t int_frequency_lower_limit_reg;
extern const uint16_t int_acceleration_1_reg;
extern const uint16_t int_Deceleration_1_reg;
extern const uint16_t int_slave_id_reg;
extern const uint16_t int_baud_rate_reg;
extern const uint16_t int_modbust_error_handle_reg;
extern const uint16_t int_modbust_timeout_setting_reg;
extern const uint16_t int_modbus_serial_format_reg;
extern const uint16_t int_operation_cmd_reg;
extern const uint16_t int_frequency_write_cmd_reg;
extern const uint16_t int_fault_control_cmd_reg;


// RO
extern const uint16_t int_firmware_version_reg;
extern const uint16_t int_fault_warning_code_reg;
extern const uint16_t int_operation_status_reg;
extern const uint16_t int_frequency_read_cmd_reg;
extern const uint16_t int_out_frequency_reg;
extern const uint16_t int_out_current_reg;
extern const uint16_t int_dc_bus_voltage_reg;
extern const uint16_t int_motor_actual_speed_reg;
extern const uint16_t int_pid_feedback_value_reg;

//device
extern const uint16_t dev_frequency_cmd_souce_reg;
extern const uint16_t dev_run_cmd_souce_reg;

extern const uint16_t dev_frequency_upper_limit_reg;
extern const uint16_t dev_frequency_lower_limit_reg;
extern const uint16_t dev_acceleration_reg;
extern const uint16_t dev_deceleration_reg;

extern const uint16_t dev_s_speed_start_reg;
extern const uint16_t dev_s_speed_end_reg;
extern const uint16_t dev_s_deceleration_start_reg;
extern const uint16_t dev_s_deceleration_end_reg;

extern const uint16_t dev_slave_id;
extern const uint16_t dev_baud_rate;
extern const uint16_t dev_mb_error_handle_reg;
extern const uint16_t dev_mb_timeout_reg;
extern const uint16_t dev_mb_serial_setting_reg;

extern const uint16_t dev_operation_cmd_reg;
extern const uint16_t dev_frequency_cmd_reg;
extern const uint16_t dev_fault_constrol_cmd_reg;
#endif /* INVERTER_MAP_H */
