/**
 * @file ups_alarm.h
 * @brief UPS alarm table definitions and alarm bridge registration.
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
 *  2. Add one row to the appropriate alarm table in ups_alarm.c.
 *  3. Verify the device_address is present in ups_map.c for that unit.
 *  No changes to alarm_engine.*, alarm_bridge.*, or alarm_manager.*
 *  are needed.
 */

#ifndef UPS_ALARM_H
#define UPS_ALARM_H

/* ── Error codes ──────────────────────────────────────────────────────── */

#define UPS_ERR_WARNING_1         0x0001u  /**< Warning register 1 bit event  */
#define UPS_ERR_WARNING_2         0x0002u  /**< Warning register 2 bit event  */
#define UPS_ERR_WARNING_3         0x0003u  /**< Warning register 1 bit event  */
#define UPS_ERR_WARNING_4         0x0004u  /**< Warning register 2 bit event  */
#define UPS_MODE_INFORMATION      0x0010u  /**< Battery capacity below limit  */
#define UPS_FAULT_INFORMATION     0x0020u  /**< Battery temperature too high  */

/* ── Public API ───────────────────────────────────────────────────────── */

/**
 * @brief Build one alarm_engine_ctx_t per enabled UPS unit and register
 *        each with alarm_bridge via alarm_bridge_register_ctx().
 *
 * Must be called after load_json_config() and before alarm_bridge_start().
 */
void ups_alarm_register_all(void);

#endif /* UPS_ALARM_H */
