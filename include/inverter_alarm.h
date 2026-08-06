/**
 * @file inverter_alarm.h
 * @brief Inverter alarm table definitions and alarm bridge registration.
 *
 * Responsibility
 * ──────────────
 *  This file owns: the alarm tables (what to monitor) and the wiring
 *  that connects them to alarm_bridge (how they get evaluated).
 *
 *  It does NOT own: what happens when an alarm fires.  That is
 *  alarm_manager.c's job.  This module does own the sink identity
 *  (log node + DB directory) passed into the shared manager.
 *
 * Error codes
 * ───────────
 *  Each alarm_entry_t carries a uint16_t error_code, defined below.
 *  Meaning and severity of each code are interpreted by the Alarm Manager.
 *
 * Adding a new alarm condition
 * ────────────────────────────
 *  1. Add an error code constant below (if new).
 *  2. Add one row to the appropriate alarm table in inverter_alarm.c.
 *  3. Verify the device_address is present in inverter_map.c for that unit.
 *  No changes to alarm_engine.*, alarm_bridge.*, or alarm_manager.*
 *  are needed.
 */

#ifndef INVERTER_ALARM_H
#define INVERTER_ALARM_H

/* ── Error codes ──────────────────────────────────────────────────────── */

#define INV_ERR_FAULT_CODE          0x0001u  /**< Fault code register changed        */
#define INV_ERR_WARNING_CODE        0x0002u  /**< Warning code register changed      */
#define INV_ERR_OUTPUT_VOLT_HIGH    0x0010u  /**< AC output voltage above limit      */
#define INV_ERR_OUTPUT_VOLT_LOW     0x0011u  /**< AC output voltage below limit      */
#define INV_ERR_BATT_SOC_LOW        0x0020u  /**< Battery state of charge below 20%  */
#define INV_ERR_BATT_TEMP_HIGH      0x0021u  /**< Battery temperature too high       */
#define INV_ERR_HEATSINK_TEMP_HIGH  0x0030u  /**< Heatsink temperature too high      */
#define INV_ERR_OVERLOAD            0x0040u  /**< Output load percentage too high    */

/* ── Public API ───────────────────────────────────────────────────────── */

/**
 * @brief Build one alarm_engine_ctx_t per enabled Inverter unit and register
 *        each with alarm_bridge via alarm_bridge_register_ctx().
 *
 * Must be called after load_json_config() and before alarm_bridge_start().
 */
void inverter_alarm_register_all(void);

#endif /* INVERTER_ALARM_H */
