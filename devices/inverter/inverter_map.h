/**
 * @file inverter_map.h
 * @brief Inverter1 profile and register constants.
 */

#ifndef INVERTER_MAP_H
#define INVERTER_MAP_H

#include "device_register_map.h"

/* 0x2000: bit0 stop, bit1 run, bit12 enable acceleration */
#define INVERTER_STOP_BIT (1 << 0)
#define INVERTER_RUN_BIT (1 << 1)
#define INVERTER_ENABLE_ACCELERATION_BIT (1 << 12)

/* 0x2002: bit0 EF, bit1 reset, bit2 B.B, bit5 fire mode */
#define INVERTER_EF_BIT (1 << 0)
#define INVERTER_RESET_BIT (1 << 1)
#define INVERTER_BB_BIT (1 << 2)
#define INVERTER_FIRE_MODE_BIT (1 << 5)

extern const device_map_profile_t inverter1_profile;

extern const uint16_t int_frequency_cmd_source_reg;
extern const uint16_t int_run_cmd_source_reg;
extern const uint16_t int_frequency_upper_limit_reg;
extern const uint16_t int_frequency_lower_limit_reg;
extern const uint16_t int_acceleration_1_reg;
extern const uint16_t int_deceleration_1_reg;
extern const uint16_t int_slave_id_reg;
extern const uint16_t int_baud_rate_reg;
extern const uint16_t int_modbust_error_handle_reg;
extern const uint16_t int_modbust_timeout_setting_reg;
extern const uint16_t int_modbus_serial_format_reg;
extern const uint16_t int_operation_cmd_reg;
extern const uint16_t int_frequency_write_cmd_reg;
extern const uint16_t int_fault_control_cmd_reg;

extern const uint16_t int_firmware_version_reg;
extern const uint16_t int_fault_warning_code_reg;
extern const uint16_t int_operation_status_reg;
extern const uint16_t int_frequency_read_cmd_reg;
extern const uint16_t int_out_frequency_reg;
extern const uint16_t int_out_current_reg;
extern const uint16_t int_dc_bus_voltage_reg;
extern const uint16_t int_motor_actual_speed_reg;
extern const uint16_t int_pid_feedback_value_reg;

/* 0 = init pending, 1 = init complete (software pool slot). */
extern const uint16_t int_inverter_init_flag_reg;

extern const uint16_t dev_frequency_cmd_source_reg;
extern const uint16_t dev_run_cmd_source_reg;

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
extern const uint16_t dev_fault_control_cmd_reg;

/**
 * @brief Return true when queued writes to addr may be merged.
 * @param device_address Device register address.
 * @return true if merge is allowed.
 */
bool inverter_queue_merge_allowed(uint16_t device_address);

#endif /* INVERTER_MAP_H */
