/**
 * @file ups_map.h
 * @brief UPS1 profile and register constants.
 */

#ifndef UPS_MAP_H
#define UPS_MAP_H

#include "device_register_map.h"

extern const device_map_profile_t ups1_profile;

extern const uint16_t int_ups_init_flag_reg;

/**
 * @brief Return true when queued writes to addr may be merged.
 * @param device_address Device register address.
 * @return true if merge is allowed.
 */
bool ups_queue_merge_allowed(uint16_t device_address);

#endif /* UPS_MAP_H */
