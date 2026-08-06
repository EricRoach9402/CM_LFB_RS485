/**
 * @file alarm_manager.h
 * @brief Shared Alarm Manager – owns all business behaviour for alarm events.
 *
 * Responsibility boundary
 * ────────────────────────
 *  alarm_engine.c decides WHETHER an alarm is triggered or cleared.
 *  This Manager decides WHAT TO DO about it: logging, severity mapping,
 *  CMOS publish, MQTT, DB write, rate limiting, escalation, etc.
 *
 *  Device-specific identity (log node, DB directory) is NOT taken from
 *  config.json.  Each caller (inverter_alarm.c / ups_alarm.c) supplies an
 *  alarm_manager_sink_t that owns those constants.
 *
 * Adding a new action (e.g. CMOS publish on trigger)
 * ────────────────────────────────────────────────────
 *  Edit alarm_manager_handle_event() only.  No changes to alarm_engine.*
 *  or to inverter_alarm.c / ups_alarm.c are needed for shared behaviour.
 */

#ifndef ALARM_MANAGER_H
#define ALARM_MANAGER_H

#include "alarm_engine.h"
#include "config_loader.h"

/**
 * @brief Per-source sink identity – owned by the calling alarm module.
 *
 * log_dir / log_path keep Inverter and UPS databases in separate folders.
 * log_node is the fixed source tag written into the DB node column.
 */
typedef struct alarm_manager_sink {
    const char *log_dir;
    const char *log_path;
    const char *log_node;
} alarm_manager_sink_t;

/**
 * @brief Receive one alarm event and decide what action to take.
 *
 * Must stay fast and non-blocking where possible; called from the polling
 * thread.
 *
 * @param entry     The alarm table entry that produced the event.
 * @param value     Register value at the time of evaluation.
 * @param event     ALARM_EVENT_TRIGGER or ALARM_EVENT_CLEAR.
 * @param userdata  Passed verbatim from alarm_engine_ctx_t.event_data;
 *                  expected to be a const module_config_t * for unit name.
 * @param sink      Source-owned log identity (node + DB path). Must not be NULL.
 */
void alarm_manager_handle_event(const alarm_entry_t       *entry,
                                uint16_t                   value,
                                alarm_event_t              event,
                                void                      *userdata,
                                const alarm_manager_sink_t *sink);

/**
 * @brief Close all open SQLite log databases.
 *
 * Must be called once during application shutdown.  Safe to call even if
 * no alarm event ever opened a database.
 */
void alarm_manager_close(void);

#endif /* ALARM_MANAGER_H */
