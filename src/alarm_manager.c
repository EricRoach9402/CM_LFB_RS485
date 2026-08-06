/**
 * @file alarm_manager.c
 * @brief Shared Alarm Manager implementation – currently logs only.
 *
 * This is the single place to extend when alarm behaviour grows beyond
 * logging (CMOS publish, MQTT, DB write, escalation policy, rate limiting).
 * alarm_engine.c and device alarm modules never need to change as this grows.
 *
 * Multiple sinks (Inverter / UPS) may open separate SQLite databases;
 * each sink keeps its own directory and node tag.
 */

#include <errno.h>
#include <sqlite3.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "alarm_manager.h"
#include "log.h"

#define LOG_MAX_ROWS           10000
#define ALARM_MANAGER_MAX_DBS  4

typedef struct {
    const char *path;
    sqlite3    *db;
    int         open_failed;
} alarm_db_slot_t;

static alarm_db_slot_t g_dbs[ALARM_MANAGER_MAX_DBS];
static size_t          g_db_count = 0u;

/* ── Static prototypes ────────────────────────────────────────────────── */

static void            ensure_log_dir(const char *log_dir);
static alarm_db_slot_t *find_or_open_db(const alarm_manager_sink_t *sink);
static void            write_log(alarm_db_slot_t *slot,
                                 const char      *node,
                                 const char      *event,
                                 const char      *detail);

/* ── Public API ───────────────────────────────────────────────────────── */

void alarm_manager_close(void)
{
    for (size_t i = 0u; i < g_db_count; i++) {
        if (g_dbs[i].db) {
            sqlite3_close(g_dbs[i].db);
            g_dbs[i].db = NULL;
        }
        g_dbs[i].path = NULL;
        g_dbs[i].open_failed = 0;
    }
    g_db_count = 0u;
}

void alarm_manager_handle_event(const alarm_entry_t       *entry,
                                uint16_t                   value,
                                alarm_event_t              event,
                                void                      *userdata,
                                const alarm_manager_sink_t *sink)
{
    if (!entry || !sink || !sink->log_path || !sink->log_node) {
        return;
    }

    const module_config_t *cfg = (const module_config_t *)userdata;
    const char *unit_name = cfg ? cfg->name : "unknown";

    alarm_db_slot_t *slot = find_or_open_db(sink);
    char detail[128];

    switch (event) {
    case ALARM_EVENT_TRIGGER:
        LOG_WARNING("[Alarm] %s | err=0x%04X | value=0x%04X | %s",
                    unit_name, entry->error_code, value, entry->description);

        snprintf(detail, sizeof(detail), "err=0x%04X | value=0x%04X | %s",
                 entry->error_code, value, entry->description);

        if (slot) {
            write_log(slot, sink->log_node, "ALARM_TRIGGER", detail);
        }
        break;

    case ALARM_EVENT_CLEAR:
        LOG_INFO("[Alarm] %s | CLEARED err=0x%04X | value=0x%04X | %s",
                 unit_name, entry->error_code, value, entry->description);

        snprintf(detail, sizeof(detail), "CLEARED err=0x%04X | value=0x%04X | %s",
                 entry->error_code, value, entry->description);

        if (slot) {
            write_log(slot, sink->log_node, "ALARM_CLEAR", detail);
        }
        break;
    }
}

/* ── Static helpers ───────────────────────────────────────────────────── */

static void ensure_log_dir(const char *log_dir)
{
    if (!log_dir) {
        return;
    }

    if (mkdir(log_dir, 0755) != 0 && errno != EEXIST) {
        LOG_ERROR("[Alarm DB] cannot create directory '%s': %s",
                  log_dir, strerror(errno));
    }
}

static alarm_db_slot_t *find_or_open_db(const alarm_manager_sink_t *sink)
{
    for (size_t i = 0u; i < g_db_count; i++) {
        if (g_dbs[i].path && strcmp(g_dbs[i].path, sink->log_path) == 0) {
            if (g_dbs[i].open_failed || !g_dbs[i].db) {
                return NULL;
            }
            return &g_dbs[i];
        }
    }

    if (g_db_count >= ALARM_MANAGER_MAX_DBS) {
        LOG_ERROR("[Alarm DB] no free slot for '%s'.", sink->log_path);
        return NULL;
    }

    alarm_db_slot_t *slot = &g_dbs[g_db_count];
    slot->path = sink->log_path;
    slot->db = NULL;
    slot->open_failed = 0;
    g_db_count++;

    ensure_log_dir(sink->log_dir);

    if (sqlite3_open(sink->log_path, &slot->db) != SQLITE_OK) {
        LOG_ERROR("[Alarm DB] failed to open '%s': %s",
                  sink->log_path, sqlite3_errmsg(slot->db));
        sqlite3_close(slot->db);
        slot->db = NULL;
        slot->open_failed = 1;
        return NULL;
    }

    char *errmsg = NULL;
    if (sqlite3_exec(slot->db,
            "CREATE TABLE IF NOT EXISTS logs ("
            "  id        INTEGER PRIMARY KEY,"
            "  timestamp TEXT NOT NULL,"
            "  node      TEXT NOT NULL,"
            "  event     TEXT NOT NULL,"
            "  detail    TEXT NOT NULL);",
            NULL, NULL, &errmsg) != SQLITE_OK) {
        LOG_ERROR("[Alarm DB] CREATE TABLE failed: %s",
                  errmsg ? errmsg : "unknown");
        sqlite3_free(errmsg);
        sqlite3_close(slot->db);
        slot->db = NULL;
        slot->open_failed = 1;
        return NULL;
    }

    LOG_INFO("[Alarm DB] opened '%s'.", sink->log_path);
    return slot;
}

static void write_log(alarm_db_slot_t *slot,
                      const char      *node,
                      const char      *event,
                      const char      *detail)
{
    if (!slot || !slot->db) {
        return;
    }

    time_t    now = time(NULL);
    struct tm tm_buf;
    char      ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", localtime_r(&now, &tm_buf));

    int count = 0;
    sqlite3_stmt *cnt = NULL;
    if (sqlite3_prepare_v2(slot->db, "SELECT COUNT(*) FROM logs;", -1, &cnt, NULL)
        == SQLITE_OK) {
        if (sqlite3_step(cnt) == SQLITE_ROW) {
            count = sqlite3_column_int(cnt, 0);
        }
        sqlite3_finalize(cnt);
    }

    const char *sql;
    if (count < LOG_MAX_ROWS) {
        sql = "INSERT INTO logs (timestamp, node, event, detail) VALUES (?, ?, ?, ?);";
    } else {
        sql = "UPDATE logs SET timestamp=?, node=?, event=?, detail=?"
              " WHERE id=(SELECT id FROM logs ORDER BY timestamp ASC, id ASC LIMIT 1);";
    }

    sqlite3_stmt *stmt = NULL;
    if (sqlite3_prepare_v2(slot->db, sql, -1, &stmt, NULL) != SQLITE_OK) {
        return;
    }

    sqlite3_bind_text(stmt, 1, ts,    -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, node,  -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 3, event, -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 4, detail, -1, SQLITE_STATIC);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
        LOG_ERROR("[Alarm DB] write failed: %s", sqlite3_errmsg(slot->db));
    }

    sqlite3_finalize(stmt);
}
