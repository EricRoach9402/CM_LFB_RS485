/**
 * @file inverter_map.c
 * @brief Inverter register mapping tables – one independent table per physical unit.
 *
 * Table format:  { device_address, pool_address, access, description }
 *
 *  device_address  – FC03 register address on the hardware.
 *  pool_address    – Absolute position in internal_pool[].  Unique per unit;
 *                    derived from the unit's pool base + its sequential offset.
 *  access          – ACCESS_RO / ACCESS_RW / ACCESS_WO for external requests.
 *
 * Pool regions
 * ────────────
 *  Inverter1 model:  0xB000 – 0xB0FF-ish (see table below for exact rows)
 *
 * Pool stores raw register values.  No masking is applied here.
 * Consumers needing a masked view use pool_read_masked_by_device_addr().
 *
 * Rules
 * ─────
 *  • Rows MUST be sorted by device_address ascending (binary search).
 *  • pool_address values must be unique across ALL tables in the project.
 *  • Each table is self-contained; adding a unit never touches another table.
 *
 * Adding a new unit of the same hardware model
 * ────────────────────────────────────────────
 *  Nothing to do here – just add another entry to config.json.  Every
 *  enabled unit currently uses inverter1_profile directly, since this is
 *  the only Inverter hardware model implemented so far.
 *
 * Adding a second hardware model (only once actually needed)
 * ────────────────────────────────────────────────────────────
 *  1. Copy an existing table, choose a new non-overlapping pool base.
 *  2. Update all pool_address values (new_base + sequential_offset).
 *  3. Add a new device_map_profile_t definition.
 *  4. Declare it extern in inverter_map.h.
 *  5. Introduce a real selection mechanism where inverter1_profile is
 *     currently referenced directly (see inverter_map.h for the list).
 */

#include "inverter_map.h"

#define ARRAY_SIZE(a)  (sizeof(a) / sizeof((a)[0]))

/* ═══════════════════════════════════════════════════════════════════════════
 * Inverter1 model – pool base 0xB000
 *
 * Typical solar / grid inverter Modbus RTU register map.  The actual
 * modbus_uid a unit answers to is whatever config.json says; this table
 * only defines the register layout for the hardware model.
 *
 *  device addr   pool addr    access     description
 * ═══════════════════════════════════════════════════════════════════════════ */
static const device_register_mapping_t inverter1_mapping_table[] = {
    { 0x0006,  0xB006,  ACCESS_RO,  "firmware_version" },

    { 0x0014,  0xB014,  ACCESS_RW,  "frequency_cmd_souce" },
    { 0x0015,  0xB015,  ACCESS_RW,  "run_cmd_souce" },

    { 0x010A,  0xB10A,  ACCESS_RW,  "frequency_upper_limit" },
    { 0x010B,  0xB10B,  ACCESS_RW,  "frequency_lower_limit" },
    { 0x010C,  0xB10C,  ACCESS_RW,  "acceleration_1"},
    { 0x010D,  0xB10D,  ACCESS_RW,  "deceleration_1"},

    { 0x0900,  0xB900,  ACCESS_RW,  "slave_id"},
    { 0x0901,  0xB901,  ACCESS_RW,  "baud_rate"},
    { 0x0902,  0xB902,  ACCESS_RW,  "modbust_error_handle"},
    { 0x0903,  0xB903,  ACCESS_RW,  "modbust_timeout_setting"},
    { 0x0904,  0xB904,  ACCESS_RW,  "modbus_serial_format"},

    { 0x0D00,  0xD10A,  ACCESS_RO,  "parament_mode" },

    { 0x2000,  0xD000,  ACCESS_RW,  "operation_commands" },
    { 0x2001,  0xD001,  ACCESS_RW,  "frequency_write_commands" },
    { 0x2002,  0xD002,  ACCESS_RW,  "fault_control_commands" },

    { 0x2100,  0xD100,  ACCESS_RO,  "fault_warning_code" },
    { 0x2101,  0xD101,  ACCESS_RO,  "inverter_operating_status" },
    { 0x2102,  0xD102,  ACCESS_RO,  "frequency_read_command" },
    { 0x2103,  0xD103,  ACCESS_RO,  "output_frequency" },
    { 0x2104,  0xD104,  ACCESS_RO,  "output_current" },
    { 0x2105,  0xD105,  ACCESS_RO,  "dc_bus_voltage" },

    { 0x210C,  0xD10C,  ACCESS_RO,  "motor_actual_speed" },

    { 0x220A,  0xD10A,  ACCESS_RO,  "pid_feedback value" },

};

const device_map_profile_t inverter1_profile = {
    .name        = "Inverter1",
    .table       = inverter1_mapping_table,
    .table_count = ARRAY_SIZE(inverter1_mapping_table),
    .read_chunk  = 20,
};

// int RW
const uint16_t int_frequency_cmd_souce_reg = 0xB014; //0xB014
const uint16_t int_run_cmd_souce_reg = 0xB015; //0xB015

const uint16_t int_frequency_upper_limit_reg = 0xB10A; //0xB10A
const uint16_t int_frequency_lower_limit_reg = 0xB10B; //0xB10B
const uint16_t int_acceleration_1_reg = 0xB10C; //0xB10C
const uint16_t int_Deceleration_1_reg = 0xB10D; //0xB10D

const uint16_t int_slave_id_reg = 0xB900; //0xB900
const uint16_t int_baud_rate_reg = 0xB901; //0xB901
const uint16_t int_modbust_error_handle_reg = 0xB902; //0xB902
const uint16_t int_modbust_timeout_setting_reg = 0xB903; //0xB903
const uint16_t int_modbus_serial_format_reg = 0xB904; //0xB904

const uint16_t int_operation_cmd_reg = 0xD000; //0xD000
const uint16_t int_frequency_write_cmd_reg = 0xD001; //0xD001
const uint16_t int_fault_control_cmd_reg = 0xD002; //0xD002

// int RO
const uint16_t int_firmware_version_reg = 0xB006; //0xB006

const uint16_t int_fault_warning_code_reg = 0xD100; //0xD100
const uint16_t int_operation_status_reg = 0xD101; //0xD101
const uint16_t int_frequency_read_cmd_reg = 0xD102; //0xD102
const uint16_t int_out_frequency_reg = 0xD103; //0xD103
const uint16_t int_out_current_reg = 0xD104; //0xD104
const uint16_t int_dc_bus_voltage_reg = 0xD105; //0xD105

const uint16_t int_motor_actual_speed_reg = 0xD10C; //0xD10C
const uint16_t int_pid_feedback_value_reg = 0xD10A; //0xD10A

// device RW
const uint16_t dev_frequency_cmd_souce_reg = 0x0014; //0x0014
const uint16_t dev_run_cmd_souce_reg = 0x0015; //0x0015

const uint16_t dev_frequency_upper_limit_reg = 0x010A; //0x010A
const uint16_t dev_frequency_lower_limit_reg = 0x010B; //010B
const uint16_t dev_acceleration_reg = 0x010C; //0x010C
const uint16_t dev_deceleration_reg = 0x010D; //0x010D

const uint16_t dev_s_speed_start_reg = 0x0118; // 0x0118
const uint16_t dev_s_speed_end_reg = 0x0119; // 0x0119
const uint16_t dev_s_deceleration_start_reg = 0x011A; // 0x011A
const uint16_t dev_s_deceleration_end_reg = 0x011B; // 0x011B

const uint16_t dev_slave_id = 0x0900; //0x0900
const uint16_t dev_baud_rate = 0x0901; //0x0901
const uint16_t dev_mb_error_handle_reg = 0x0902; //0x0902
const uint16_t dev_mb_timeout_reg = 0x0903; //0x0903
const uint16_t dev_mb_serial_setting_reg = 0x0904; //0x0904

const uint16_t dev_operation_cmd_reg = 0x2000; //0x2000
const uint16_t dev_frequency_cmd_reg = 0x2001; //0x2001
const uint16_t dev_fault_constrol_cmd_reg = 0x2002; //0x2002
