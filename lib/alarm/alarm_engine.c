/**
 * @file alarm_engine.c
 * @brief Generic register alarm engine implementation.
 *
 * No project-specific logic, no I/O, no logging.  This file only reads
 * values, evaluates conditions, and calls event_fn.  Do not add business
 * behaviour here when porting – it belongs in the project's Alarm Manager.
 */

#include "alarm_engine.h"

/* ── Internal condition evaluators ───────────────────────────────────── */

/**
 * @brief Evaluate ALARM_COND_BITMASK.
 * @return true if all bits in the mask (hi_limit) are set in value.
 */
static bool eval_bitmask(const alarm_entry_t *entry, uint16_t value)
{
    return (value & entry->hi_limit) == entry->hi_limit;
}

/**
 * @brief Evaluate ALARM_COND_RANGE.
 * @return true if value is outside [lo_limit, hi_limit].
 */
static bool eval_range(const alarm_entry_t *entry, uint16_t value)
{
    return (value < entry->lo_limit) || (value > entry->hi_limit);
}

/**
 * @brief Evaluate ALARM_COND_CHANGE.
 * @return true if value differs from the previous value.
 */
static bool eval_change(const alarm_state_t *state, uint16_t value)
{
    return (value != state->prev_value);
}

/**
 * @brief Sticky state machine for BITMASK and RANGE conditions.
 *
 * Emits exactly one TRIGGER when the condition becomes true, suppresses
 * further events while it stays true, and emits exactly one CLEAR when
 * the condition returns to normal.
 */
static void apply_sticky(alarm_engine_ctx_t  *ctx,
                         const alarm_entry_t *entry,
                         alarm_state_t       *state,
                         uint16_t             value,
                         bool                 triggered)
{
    if (triggered && !state->is_active) {
        state->is_active = true;
        ctx->event_fn(entry, value, ALARM_EVENT_TRIGGER, ctx->event_data);
    } else if (!triggered && state->is_active) {
        state->is_active = false;
        ctx->event_fn(entry, value, ALARM_EVENT_CLEAR, ctx->event_data);
    }
}

/* ── Public API ───────────────────────────────────────────────────────── */

void alarm_engine_run(alarm_engine_ctx_t *ctx)
{
    if (!ctx || !ctx->table || !ctx->states ||
        !ctx->read_fn || !ctx->event_fn || ctx->count == 0u) {
        return;
    }

    for (size_t i = 0u; i < ctx->count; i++) {
        const alarm_entry_t *entry = &ctx->table[i];
        alarm_state_t       *state = &ctx->states[i];

        uint16_t value = 0u;

        if (!ctx->read_fn(entry->device_address, &value, ctx->read_data)) {
            /* Register unavailable – skip without touching sticky state. */
            continue;
        }

        switch (entry->condition) {
        case ALARM_COND_BITMASK:
            apply_sticky(ctx, entry, state, value,
                         eval_bitmask(entry, value));
            break;

        case ALARM_COND_RANGE:
            apply_sticky(ctx, entry, state, value,
                         eval_range(entry, value));
            break;

        case ALARM_COND_CHANGE: {
            bool changed = eval_change(state, value);
            state->prev_value = value;

            /*
             * CHANGE bypasses sticky entirely: every qualifying change is
             * its own independent TRIGGER event.  There is no CLEAR for
             * this condition type.
             */
            if (changed) {
                ctx->event_fn(entry, value, ALARM_EVENT_TRIGGER,
                              ctx->event_data);
            }
            break;
        }

        default:
            /* Unknown condition – skip entry until enum is extended. */
            break;
        }
    }
}
