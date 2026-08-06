/**
 * @file ups_map.c
 * @brief UPS register mapping tables – one independent table per physical unit.
 *
 * Table format:  { device_address, pool_address, access, description }
 *
 *  device_address  – FC03 register address on the hardware.
 *  pool_address    – Absolute position in internal_pool[].  Unique per unit;
 *                    derived from the unit's pool base + its sequential offset.
 *  access          – ACCESS_RO / ACCESS_RW / ACCESS_WO for external requests.
 *  description     – Human-readable label.  ALSO the CMOS/HMI publish key
 *                    (see ups_cmos_bridge.c: publish_all_pool_register()
 *                    publishes every row here, keyed by this string).
 *                    Treat it as a stable API contract, not just a comment:
 *                    renaming it renames the value the HMI subscribes to.
 *
 * Pool stores raw register values.  No masking is applied here.
 * Consumers needing a masked view use pool_read_masked_by_device_addr().
 *
 * Rules
 * ─────
 *  • Rows MUST be sorted by device_address ascending (binary search).
 *  • pool_address values must be unique across ALL tables in the project.
 *  • Each table is self-contained; adding a unit never touches another table.
 *  • description values are published to CMOS verbatim – keep them stable,
 *    typo-free, and unique within a table (duplicates or renames change/
 *    break the HMI key the same way an addr change would).
 *
 * Adding a new unit of the same hardware model
 * ────────────────────────────────────────────
 *  Nothing to do here – just add another entry to config.json.  Every
 *  enabled unit currently uses ups1_profile directly, since this is the
 *  only UPS hardware model implemented so far.
 *
 * Adding a second hardware model (only once actually needed)
 * ────────────────────────────────────────────────────────────
 *  1. Copy an existing table, choose a new non-overlapping pool base.
 *  2. Update all pool_address values (new_base + sequential_offset).
 *  3. Add a new device_map_profile_t definition.
 *  4. Declare it extern in ups_map.h.
 *  5. Introduce a real selection mechanism where ups1_profile is
 *     currently referenced directly (see ups_map.h for the list).
 */

#include "ups_map.h"

#define ARRAY_SIZE(a)  (sizeof(a) / sizeof((a)[0]))

/* ═══════════════════════════════════════════════════════════════════════════
 * UPS1 model – pool base 0xB000
 *
 * The actual modbus_uid a UPS1 unit answers to is whatever config.json
 * says (see "UPS" array); this table only defines the register layout for
 * the hardware model, so it stays valid no matter how the bus is wired.
 *
 *  device addr   pool addr    access     description
 * ═══════════════════════════════════════════════════════════════════════════ */
static const device_register_mapping_t ups1_mapping_table[] = {
    { 0x0000, 0xB000, ACCESS_RO, "ups_warning_information_1" },                      
    { 0x0001, 0xB001, ACCESS_RO, "ups_warning_information_2" },                      
    { 0x0002, 0xB002, ACCESS_RO, "ups_warning_information_3" },                      
    { 0x0003, 0xB003, ACCESS_RO, "ups_warning_information_4" },                      
    { 0x00BC, 0xB0BC, ACCESS_RO, "ups_p_battery_voltage" },                          
    { 0x00BD, 0xB0BD, ACCESS_RO, "ups_p_battery_discharging_current" },              
    { 0x00BE, 0xB0BE, ACCESS_RO, "ups_p_battery_charging_current" },                 
    { 0x00BF, 0xB0BF, ACCESS_RO, "ups_battery_capacity" },                           
    { 0x00C0, 0xB0C0, ACCESS_RO, "ups_battery_remain_time" },                
    { 0x00C1, 0xB0C1, ACCESS_RO, "ups_n_battery_voltage" },                          
    { 0x00C3, 0xB0C3, ACCESS_RO, "ups_n_battery_charging_current" },
    { 0x00CB, 0xB0CB, ACCESS_RO, "ups_battery_temperature" },
    { 0x00CC, 0xB0CC, ACCESS_RO, "ups_temperature_pfc" },                            
    { 0x00CD, 0xB0CD, ACCESS_RO, "ups_temperature_inv" },                            
    { 0x00CE, 0xB0CE, ACCESS_RO, "ups_temperature_bypass" },                         
    { 0x00CF, 0xB0CF, ACCESS_RO, "ups_max_temperature" }, 
    { 0x00D0, 0xB0D0, ACCESS_RO, "ups_mode_information" },                       
    { 0x00DE, 0xB0DE, ACCESS_RO, "ups_over_temperature_fault_recovery_value" },      
    { 0x00DF, 0xB0DF, ACCESS_RO, "ups_over_temperature_warning_trigger_value" },     
    { 0x00E0, 0xB0E0, ACCESS_RO, "ups_over_temperature_warning_recovery_value" },    
    { 0x02A2, 0xB2A2, ACCESS_RO, "ups_fault_information" },                                                   
    { 0x0364, 0xB364, ACCESS_RO, "ups_battery_shutdown_voltage" },                   
    { 0x036A, 0xB36A, ACCESS_RO, "ups_battery_low_voltage" },
    { 0x03E1, 0xB3E1, ACCESS_RO, "ups_pfc_fw_version_1" },
    { 0x03E2, 0xB3E2, ACCESS_RO, "ups_pfc_fw_version_2" },
    { 0x03E3, 0xB3E3, ACCESS_RO, "ups_pfc_fw_version_3" },
    { 0x03E4, 0xB3E4, ACCESS_RO, "ups_pfc_fw_version_4" },
    { 0x03E5, 0xB3E5, ACCESS_RO, "ups_pfc_fw_version_5" },
    { 0x03E6, 0xB3E6, ACCESS_RO, "ups_inv_fw_version_1" },
    { 0x03E7, 0xB3E7, ACCESS_RO, "ups_inv_fw_version_2" },
    { 0x03E8, 0xB3E8, ACCESS_RO, "ups_inv_fw_version_3" },
    { 0x03E9, 0xB3E9, ACCESS_RO, "ups_inv_fw_version_4" },
    { 0x03EA, 0xB3EA, ACCESS_RO, "ups_inv_fw_version_5" },
    { 0x0403, 0xB403, ACCESS_RO, "ups_com_fw_version_1" },
    { 0x0404, 0xB404, ACCESS_RO, "ups_com_fw_version_2" },
    { 0x0405, 0xB405, ACCESS_RO, "ups_com_fw_version_3" },
    { 0x0406, 0xB406, ACCESS_RO, "ups_com_fw_version_4" },
    { 0x0407, 0xB407, ACCESS_RO, "ups_com_fw_version_5" },
    { 0x0408, 0xB408, ACCESS_RO, "ups_lcd_fw_version_1" },
    { 0x0409, 0xB409, ACCESS_RO, "ups_lcd_fw_version_2" },
    { 0x040A, 0xB40A, ACCESS_RO, "ups_lcd_fw_version_3" },
    { 0x040B, 0xB40B, ACCESS_RO, "ups_lcd_fw_version_4" },
    { 0x040C, 0xB40C, ACCESS_RO, "ups_lcd_fw_version_5" },
    { 0x0412, 0xB412, ACCESS_RO, "ups_fbpn_1" },
    { 0x0413, 0xB413, ACCESS_RO, "ups_fbpn_2" },
    { 0x0414, 0xB414, ACCESS_RO, "ups_fbpn_3" },
    { 0x0415, 0xB415, ACCESS_RO, "ups_fbpn_4" },
    { 0x0416, 0xB416, ACCESS_RO, "ups_fbpn_5" },
    { 0x05B0, 0xB5B0, ACCESS_RO, "ups_battery_high_voltage" }

};

const device_map_profile_t ups1_profile = {
    .name        = "UPS1",
    .table       = ups1_mapping_table,
    .table_count = ARRAY_SIZE(ups1_mapping_table),
    .read_chunk  = 50,
};
