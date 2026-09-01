/**
 * @file ups_alarm.h
 * @brief UPS alarm registration API.
 */

#ifndef UPS_ALARM_H
#define UPS_ALARM_H

#define UPS_ERR_WARNING_1 0x0001u
#define UPS_ERR_WARNING_2 0x0002u
#define UPS_ERR_WARNING_3 0x0003u
#define UPS_ERR_WARNING_4 0x0004u
#define UPS_MODE_INFORMATION 0x0010u
#define UPS_FAULT_INFORMATION 0x0020u

/**
 * @brief Register alarm contexts for all enabled UPS units.
 */
void ups_alarm_register_all(void);

#endif /* UPS_ALARM_H */
