/**
 * @file inverter_map.c
 * @brief Inverter1 register map (pool 0xA000); rows sorted by device_address.
 */

#include "inverter_map.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static const device_register_mapping_t inverter1_mapping_table[] = {
    { 0x0006, 0xA000, ACCESS_RO, "firmware_version" },

    { 0x0014, 0xA100, ACCESS_RW, "frequency_cmd_source" },
    { 0x0015, 0xA101, ACCESS_RW, "run_cmd_source" },

    { 0x010A, 0xA102, ACCESS_RW, "frequency_upper_limit" },
    { 0x010B, 0xA103, ACCESS_RW, "frequency_lower_limit" },
    { 0x010C, 0xA104, ACCESS_RW, "acceleration_1" },
    { 0x010D, 0xA105, ACCESS_RW, "deceleration_1" },

    { 0x0118, 0xA10E, ACCESS_RW, "s_speed_start" },
    { 0x0119, 0xA10F, ACCESS_RW, "s_speed_end" },
    { 0x011A, 0xA110, ACCESS_RW, "s_deceleration_start" },
    { 0x011B, 0xA111, ACCESS_RW, "s_deceleration_end" },

    { 0x0900, 0xA106, ACCESS_RW, "slave_id" },
    { 0x0901, 0xA107, ACCESS_RW, "baud_rate" },
    { 0x0902, 0xA108, ACCESS_RW, "modbust_error_handle" },
    { 0x0903, 0xA109, ACCESS_RW, "modbust_timeout_setting" },
    { 0x0904, 0xA10A, ACCESS_RW, "modbus_serial_format" },

    { 0x0D00, 0xA112, ACCESS_RW, "parameter_mode" },

    { 0x2000, 0xA10B, ACCESS_RW, "operation_commands" },
    { 0x2001, 0xA10C, ACCESS_RW, "frequency_write_commands" },
    { 0x2002, 0xA10D, ACCESS_RW, "fault_control_commands" },

    { 0x2100, 0xA002, ACCESS_RO, "fault_warning_code" },
    { 0x2101, 0xA003, ACCESS_RO, "inverter_operating_status" },
    { 0x2102, 0xA004, ACCESS_RO, "frequency_read_command" },
    { 0x2103, 0xA005, ACCESS_RO, "output_frequency" },
    { 0x2104, 0xA006, ACCESS_RO, "output_current" },
    { 0x2105, 0xA007, ACCESS_RO, "dc_bus_voltage" },

    { 0x210C, 0xA008, ACCESS_RO, "motor_actual_speed" },

    { 0x220A, 0xA009, ACCESS_RO, "pid_feedback_value" },

};

const device_map_profile_t inverter1_profile = {
    .name = "Inverter1",
    .table = inverter1_mapping_table,
    .table_count = ARRAY_SIZE(inverter1_mapping_table),
    .read_chunk = 20,
};

const uint16_t int_frequency_cmd_source_reg = 0xA100;
const uint16_t int_run_cmd_source_reg = 0xA101;

const uint16_t int_frequency_upper_limit_reg = 0xA102;
const uint16_t int_frequency_lower_limit_reg = 0xA103;
const uint16_t int_acceleration_1_reg = 0xA104;
const uint16_t int_deceleration_1_reg = 0xA105;

const uint16_t int_slave_id_reg = 0xA106;
const uint16_t int_baud_rate_reg = 0xA107;
const uint16_t int_modbust_error_handle_reg = 0xA108;
const uint16_t int_modbust_timeout_setting_reg = 0xA109;
const uint16_t int_modbus_serial_format_reg = 0xA10A;

const uint16_t int_operation_cmd_reg = 0xA10B;
const uint16_t int_frequency_write_cmd_reg = 0xA10C;
const uint16_t int_fault_control_cmd_reg = 0xA10D;

const uint16_t int_firmware_version_reg = 0xA000;

const uint16_t int_fault_warning_code_reg = 0xA002;
const uint16_t int_operation_status_reg = 0xA003;
const uint16_t int_frequency_read_cmd_reg = 0xA004;
const uint16_t int_out_frequency_reg = 0xA005;
const uint16_t int_out_current_reg = 0xA006;
const uint16_t int_dc_bus_voltage_reg = 0xA007;

const uint16_t int_motor_actual_speed_reg = 0xA008;
const uint16_t int_pid_feedback_value_reg = 0xA009;

const uint16_t int_inverter_init_flag_reg = 0xA0F0;

const uint16_t dev_frequency_cmd_source_reg = 0x0014;
const uint16_t dev_run_cmd_source_reg = 0x0015;

const uint16_t dev_frequency_upper_limit_reg = 0x010A;
const uint16_t dev_frequency_lower_limit_reg = 0x010B;
const uint16_t dev_acceleration_reg = 0x010C;
const uint16_t dev_deceleration_reg = 0x010D;

const uint16_t dev_s_speed_start_reg = 0x0118;
const uint16_t dev_s_speed_end_reg = 0x0119;
const uint16_t dev_s_deceleration_start_reg = 0x011A;
const uint16_t dev_s_deceleration_end_reg = 0x011B;

const uint16_t dev_slave_id = 0x0900;
const uint16_t dev_baud_rate = 0x0901;
const uint16_t dev_mb_error_handle_reg = 0x0902;
const uint16_t dev_mb_timeout_reg = 0x0903;
const uint16_t dev_mb_serial_setting_reg = 0x0904;

const uint16_t dev_operation_cmd_reg = 0x2000;
const uint16_t dev_frequency_cmd_reg = 0x2001;
const uint16_t dev_fault_control_cmd_reg = 0x2002;

static const uint16_t inverter_queue_merge_addrs[] = {
    dev_operation_cmd_reg,
    dev_frequency_cmd_reg,
};

/**
 * @brief Return true when queued writes to addr may be merged.
 * @param device_address Device register address.
 * @return true if merge is allowed.
 */
bool inverter_queue_merge_allowed(uint16_t device_address)
{
    for (size_t i = 0u; i < ARRAY_SIZE(inverter_queue_merge_addrs); i++) {
        if (inverter_queue_merge_addrs[i] == device_address) {
            return true;
        }
    }
    return false;
}
