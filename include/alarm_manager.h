/**
 * @file alarm_manager.h
 * @brief Shared alarm event handler.
 */

#ifndef ALARM_MANAGER_H
#define ALARM_MANAGER_H

#include "alarm_engine.h"
#include "config_loader.h"

/** Per-source SQLite log identity supplied by alarm modules. */
typedef struct alarm_manager_sink {
    const char *log_dir;
    const char *log_path;
    const char *log_node;
} alarm_manager_sink_t;

/**
 * @brief Handle one alarm trigger or clear event.
 * @param entry Alarm table entry.
 * @param value Register value at evaluation time.
 * @param event ALARM_EVENT_TRIGGER or ALARM_EVENT_CLEAR.
 * @param userdata Caller context, usually module_config_t *.
 * @param sink Log directory, DB path, and node tag.
 */
void alarm_manager_handle_event(const alarm_entry_t *entry,
                                uint16_t value,
                                alarm_event_t event,
                                void *userdata,
                                const alarm_manager_sink_t *sink);

/**
 * @brief Close all open alarm SQLite databases.
 */
void alarm_manager_close(void);

#endif /* ALARM_MANAGER_H */
