/**
 * @file ups_map.h
 * @brief UPS device register profile.
 *
 * This project currently implements exactly one UPS hardware model,
 * ups1_profile below.  Because there is only one, ups_module.c (and the
 * CMOS bridge / alarm registration) reference it directly – no uid- or
 * config-driven lookup/selection mechanism exists, and none is needed
 * until a second hardware model is actually introduced.
 *
 * modbus_uid (config.json) is purely a wiring/deployment detail – which
 * slave address a unit answers to on the bus – and never selects a
 * profile.
 *
 * Adding a second hardware model (only once actually needed)
 * ────────────────────────────────────────────────────────────
 *  1. Add a mapping table in ups_map.c with a unique pool_address range,
 *     plus a new device_map_profile_t definition.
 *  2. Declare the new profile below (extern const device_map_profile_t …).
 *  3. Introduce a real selection mechanism where profiles are currently
 *     referenced directly: start_ups_modules() in ups_module.c,
 *     on_write()/publish_all_pool_register() in ups_cmos_bridge.c, and
 *     ups_alarm_register_all() in ups_alarm.c.
 */

#ifndef UPS_MAP_H
#define UPS_MAP_H

#include "device_register_map.h"

/** UPS1 model – pool region 0xB000-0xB5FF (see table for exact rows). */
extern const device_map_profile_t ups1_profile;

#endif /* UPS_MAP_H */
