/**
 * @file alarm_engine.h
 * @brief Generic register alarm engine – portable across projects.
 *
 * Responsibility boundary
 * ────────────────────────
 *  This engine does exactly three things:
 *    1. Read a register value (via alarm_read_fn).
 *    2. Evaluate the trigger condition.
 *    3. Emit a trigger/clear event (via alarm_event_fn).
 *
 *  It does NOT log, publish, write to a pool, or perform any business action.
 *  alarm_event_fn is expected to be a thin forwarder into an Alarm Manager
 *  (see <project>_alarm_manager.h) that owns logging, MQTT, notifications,
 *  etc.  Keeping the engine this narrow is what keeps it reusable: business
 *  behaviour differs per project, but "read → evaluate → emit" does not.
 *
 * Porting to a new project
 * ────────────────────────
 *  1. Copy alarm_engine.h and alarm_engine.c verbatim – no edits needed.
 *  2. Write a project-specific alarm table (<device>_alarm_table.c) and a
 *     read_fn that knows how to fetch a register value.
 *  3. Write a project-specific Alarm Manager that receives events from
 *     alarm_event_fn and decides what to do with them.
 *  4. Call alarm_engine_run() from the polling thread after each scan.
 *
 * Trigger conditions
 * ──────────────────
 *  ALARM_COND_BITMASK  (value & mask) == mask
 *                      hi_limit is used as the mask; lo_limit is ignored.
 *
 *  ALARM_COND_RANGE    value < lo_limit || value > hi_limit
 *                      To monitor only an upper bound set lo_limit = 0.
 *                      To monitor only a lower bound set hi_limit = UINT16_MAX.
 *
 *  ALARM_COND_CHANGE   value != prev_value
 *                      Fires every time the register value changes.
 *                      Does NOT use sticky suppression – every change is an
 *                      independent event.  lo_limit and hi_limit are ignored.
 *
 * Sticky suppression
 * ──────────────────
 *  BITMASK and RANGE conditions are sticky: once triggered, the engine emits
 *  exactly one ALARM_EVENT_TRIGGER and suppresses further events until the
 *  value returns to normal, at which point it emits exactly one
 *  ALARM_EVENT_CLEAR.  CHANGE conditions bypass sticky entirely – every
 *  qualifying change emits its own ALARM_EVENT_TRIGGER.
 */

#ifndef ALARM_ENGINE_H
#define ALARM_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── Trigger condition ────────────────────────────────────────────────── */

typedef enum {
    ALARM_COND_BITMASK,  /**< (value & hi_limit) == hi_limit               */
    ALARM_COND_RANGE,    /**< value < lo_limit || value > hi_limit          */
    ALARM_COND_CHANGE,   /**< value != prev_value (no sticky)              */
} alarm_cond_t;

/* ── Event type ───────────────────────────────────────────────────────── */

/**
 * @brief What happened to an alarm entry on this evaluation pass.
 *
 * The engine only ever emits these two.  Everything else (severity mapping,
 * log formatting, downstream notification) is the Alarm Manager's job.
 */
typedef enum {
    ALARM_EVENT_TRIGGER,  /**< Condition newly true (or CHANGE fired)       */
    ALARM_EVENT_CLEAR,    /**< Condition returned to normal (sticky only)   */
} alarm_event_t;

/* ── Static table entry (const, never modified at runtime) ────────────── */

/**
 * @brief One row in the alarm monitoring table.
 *
 * Declare tables as:
 *   static const alarm_entry_t my_alarm_table[] = { ... };
 *
 * Fields
 * ──────
 *  device_address  Register address passed to alarm_read_fn.
 *                  Semantics are determined by the caller's read function
 *                  (pool_address, device_address, or any other key).
 *
 *  condition       Trigger condition type.
 *
 *  lo_limit        Lower bound for ALARM_COND_RANGE.
 *                  Ignored by ALARM_COND_BITMASK and ALARM_COND_CHANGE.
 *
 *  hi_limit        Upper bound for ALARM_COND_RANGE; mask for ALARM_COND_BITMASK.
 *                  Ignored by ALARM_COND_CHANGE.
 *
 *  error_code      Opaque code forwarded in the event; meaning and severity
 *                  are defined and interpreted entirely by the Alarm Manager.
 *
 *  description     Human-readable label; the engine never logs this itself,
 *                  it is only carried through to the event for the Manager.
 */
typedef struct {
    uint16_t      device_address;
    alarm_cond_t  condition;
    uint16_t      lo_limit;
    uint16_t      hi_limit;
    uint16_t      error_code;
    const char   *description;
} alarm_entry_t;

/* ── Runtime state (one slot per table entry, zeroed at init) ─────────── */

/**
 * @brief Mutable runtime state for one alarm_entry_t.
 *
 * Allocated and owned by the caller (typically a static array of the same
 * length as the alarm table).  Zeroed at startup; never reset by the engine
 * between calls to alarm_engine_run().
 */
typedef struct {
    bool     is_active;   /**< Sticky flag – set on trigger, cleared on clear */
    uint16_t prev_value;  /**< Last seen value, used by ALARM_COND_CHANGE     */
} alarm_state_t;

/* ── Callback types ───────────────────────────────────────────────────── */

/**
 * @brief Read one register value.
 *
 * @param device_address  Key from alarm_entry_t.device_address.
 * @param out_value       Destination for the register value.
 * @param userdata         Opaque pointer supplied by the caller (e.g. profile).
 * @return true on success, false if the register is unavailable.
 */
typedef bool (*alarm_read_fn)(uint16_t  device_address,
                              uint16_t *out_value,
                              void     *userdata);

/**
 * @brief Event callback – the engine's ONLY output channel.
 *
 * This must stay a thin forwarder.  It should do nothing more than hand the
 * event off to an Alarm Manager (queue, callback table, direct call) – no
 * logging, no I/O, no business logic here.  Keeping this function trivial is
 * what keeps the engine itself free of project-specific behaviour.
 *
 * @param entry      The table entry that produced this event.
 * @param value      Register value that triggered the evaluation.
 * @param event      ALARM_EVENT_TRIGGER or ALARM_EVENT_CLEAR.
 * @param userdata   Opaque pointer supplied by the caller.
 */
typedef void (*alarm_event_fn)(const alarm_entry_t *entry,
                               uint16_t              value,
                               alarm_event_t         event,
                               void                 *userdata);

/* ── Engine context ───────────────────────────────────────────────────── */

/**
 * @brief Bundles everything the engine needs for one scan pass.
 *
 * Create one context per monitored unit and pass it to alarm_engine_run()
 * on every poll cycle.
 *
 * Example initialisation:
 *
 *   static alarm_state_t ups1_alarm_states[ARRAY_SIZE(ups1_alarm_table)];
 *
 *   alarm_engine_ctx_t ctx = {
 *       .table       = ups1_alarm_table,
 *       .states      = ups1_alarm_states,
 *       .count       = ARRAY_SIZE(ups1_alarm_table),
 *       .read_fn     = my_read_by_device_addr,
 *       .event_fn    = my_forward_to_alarm_manager,
 *       .read_data   = profile_ptr,
 *       .event_data  = unit_ptr,
 *   };
 */
typedef struct {
    const alarm_entry_t *table;       /**< Static alarm table (const)         */
    alarm_state_t       *states;      /**< Runtime state array (same length)  */
    size_t                count;       /**< Number of entries in table/states  */
    alarm_read_fn         read_fn;     /**< Register read function             */
    alarm_event_fn        event_fn;    /**< Event emission – forward only      */
    void                 *read_data;   /**< Passed verbatim to read_fn         */
    void                 *event_data;  /**< Passed verbatim to event_fn        */
} alarm_engine_ctx_t;

/* ── Public API ───────────────────────────────────────────────────────── */

/**
 * @brief Scan every entry in the alarm table and evaluate trigger conditions.
 *
 * Must be called after each successful register poll (i.e. only when the
 * communication layer reports a good read).  Calling it during a comm failure
 * would produce spurious clears as stale values are evaluated.
 *
 * All entries are evaluated regardless of whether an earlier entry triggered;
 * the loop never exits early.
 *
 * @param ctx  Fully initialised engine context.
 */
void alarm_engine_run(alarm_engine_ctx_t *ctx);

#endif /* ALARM_ENGINE_H */
