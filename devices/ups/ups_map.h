/**
 * @file ups_map.h
 * @brief UPS1 profile and register address constants.
 *
 * Single hardware model; modbus_uid is set in config.json.
 */

#ifndef UPS_MAP_H
#define UPS_MAP_H

#include "device_register_map.h"

extern const device_map_profile_t ups1_profile;

/* 0 = init pending, 1 = init complete (software pool slot). */
extern const uint16_t int_ups_init_flag_reg;

bool ups_queue_merge_allowed(uint16_t device_address);

#endif /* UPS_MAP_H */
