/**
 * @file ups_map.c
 * @brief UPS1 register map (pool 0xA200); rows sorted by device_address.
 */

#include "ups_map.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static const device_register_mapping_t ups1_mapping_table[] = {
    { 0x0000, 0xA200, ACCESS_RO, "ups_warning_information_1" },
    { 0x0001, 0xA201, ACCESS_RO, "ups_warning_information_2" },
    { 0x0002, 0xA202, ACCESS_RO, "ups_warning_information_3" },
    { 0x0003, 0xA203, ACCESS_RO, "ups_warning_information_4" },
    { 0x00BC, 0xA204, ACCESS_RO, "ups_p_battery_voltage" },
    { 0x00BD, 0xA205, ACCESS_RO, "ups_p_battery_discharging_current" },
    { 0x00BE, 0xA206, ACCESS_RO, "ups_p_battery_charging_current" },
    { 0x00BF, 0xA207, ACCESS_RO, "ups_battery_capacity" },
    { 0x00C0, 0xA208, ACCESS_RO, "ups_battery_remain_time" },
    { 0x00C1, 0xA209, ACCESS_RO, "ups_n_battery_voltage" },
    { 0x00C3, 0xA20A, ACCESS_RO, "ups_n_battery_charging_current" },
    { 0x00CB, 0xA20B, ACCESS_RO, "ups_battery_temperature" },
    { 0x00CC, 0xA20C, ACCESS_RO, "ups_temperature_pfc" },
    { 0x00CD, 0xA20D, ACCESS_RO, "ups_temperature_inv" },
    { 0x00CE, 0xA20E, ACCESS_RO, "ups_temperature_bypass" },
    { 0x00CF, 0xA20F, ACCESS_RO, "ups_max_temperature" },
    { 0x00D0, 0xA210, ACCESS_RO, "ups_mode_information" },
    { 0x00DE, 0xA211, ACCESS_RO, "ups_over_temperature_fault_recovery_value" },
    { 0x00DF, 0xA212, ACCESS_RO, "ups_over_temperature_warning_trigger_value" },
    { 0x00E0, 0xA213, ACCESS_RO, "ups_over_temperature_warning_recovery_value" },
    { 0x02A2, 0xA214, ACCESS_RO, "ups_fault_information" },
    { 0x0364, 0xA215, ACCESS_RO, "ups_battery_shutdown_voltage" },
    { 0x036A, 0xA216, ACCESS_RO, "ups_battery_low_voltage" },
    { 0x03E1, 0xA217, ACCESS_RO, "ups_pfc_fw_version_1" },
    { 0x03E2, 0xA218, ACCESS_RO, "ups_pfc_fw_version_2" },
    { 0x03E3, 0xA219, ACCESS_RO, "ups_pfc_fw_version_3" },
    { 0x03E4, 0xA21A, ACCESS_RO, "ups_pfc_fw_version_4" },
    { 0x03E5, 0xA21B, ACCESS_RO, "ups_pfc_fw_version_5" },
    { 0x03E6, 0xA21C, ACCESS_RO, "ups_inv_fw_version_1" },
    { 0x03E7, 0xA21D, ACCESS_RO, "ups_inv_fw_version_2" },
    { 0x03E8, 0xA21E, ACCESS_RO, "ups_inv_fw_version_3" },
    { 0x03E9, 0xA21F, ACCESS_RO, "ups_inv_fw_version_4" },
    { 0x03EA, 0xA220, ACCESS_RO, "ups_inv_fw_version_5" },
    { 0x0403, 0xA221, ACCESS_RO, "ups_com_fw_version_1" },
    { 0x0404, 0xA222, ACCESS_RO, "ups_com_fw_version_2" },
    { 0x0405, 0xA223, ACCESS_RO, "ups_com_fw_version_3" },
    { 0x0406, 0xA224, ACCESS_RO, "ups_com_fw_version_4" },
    { 0x0407, 0xA225, ACCESS_RO, "ups_com_fw_version_5" },
    { 0x0408, 0xA226, ACCESS_RO, "ups_lcd_fw_version_1" },
    { 0x0409, 0xA227, ACCESS_RO, "ups_lcd_fw_version_2" },
    { 0x040A, 0xA228, ACCESS_RO, "ups_lcd_fw_version_3" },
    { 0x040B, 0xA229, ACCESS_RO, "ups_lcd_fw_version_4" },
    { 0x040C, 0xA22A, ACCESS_RO, "ups_lcd_fw_version_5" },
    { 0x0412, 0xA22B, ACCESS_RO, "ups_fbpn_1" },
    { 0x0413, 0xA22C, ACCESS_RO, "ups_fbpn_2" },
    { 0x0414, 0xA22D, ACCESS_RO, "ups_fbpn_3" },
    { 0x0415, 0xA22E, ACCESS_RO, "ups_fbpn_4" },
    { 0x0416, 0xA22F, ACCESS_RO, "ups_fbpn_5" },
    { 0x05B0, 0xA230, ACCESS_RO, "ups_battery_high_voltage" }

};

const device_map_profile_t ups1_profile = {
    .name = "UPS1",
    .table = ups1_mapping_table,
    .table_count = ARRAY_SIZE(ups1_mapping_table),
    .read_chunk = 50,
};

const uint16_t int_ups_init_flag_reg = 0xA2F0;

/**
 * @brief Return true when queued writes to addr may be merged.
 * @param device_address Device register address.
 * @return true if merge is allowed.
 */
bool ups_queue_merge_allowed(uint16_t device_address)
{
    (void)device_address;
    return false;
}
