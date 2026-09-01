/**
 * @file inverter_alarm.h
 * @brief Inverter alarm registration API.
 */

#ifndef INVERTER_ALARM_H
#define INVERTER_ALARM_H

#define INV_ERR_FAULT_CODE 0x0001u
#define INV_ERR_WARNING_CODE 0x0002u
#define INV_ERR_OUTPUT_VOLT_HIGH 0x0010u
#define INV_ERR_OUTPUT_VOLT_LOW 0x0011u
#define INV_ERR_BATT_SOC_LOW 0x0020u
#define INV_ERR_BATT_TEMP_HIGH 0x0021u
#define INV_ERR_HEATSINK_TEMP_HIGH 0x0030u
#define INV_ERR_OVERLOAD 0x0040u

/**
 * @brief Register alarm contexts for all enabled Inverter units.
 */
void inverter_alarm_register_all(void);

#endif /* INVERTER_ALARM_H */
