/**
 * @file test_alarm_engine.c
 * @brief Unit tests for alarm_engine.c
 *
 * Covered scenarios
 * ─────────────────
 *  BITMASK  – fires TRIGGER when bits match; sticky suppresses duplicates;
 *              fires CLEAR when bits clear.
 *  RANGE    – fires TRIGGER when value goes out of range; sticky suppression;
 *              fires CLEAR when value returns to range.
 *  CHANGE   – fires TRIGGER on every qualifying change (different value);
 *              never fires CLEAR; no sticky.
 *  read_fn  – returning false (register unavailable) skips the entry silently.
 *  NULL ctx – alarm_engine_run() does not crash on invalid context.
 */

#include "unity/unity.h"
#include "alarm_engine.h"

/* ── Test doubles ─────────────────────────────────────────────────────── */

static uint16_t  g_mock_register_value = 0u;
static bool      g_mock_read_available = true;

static int       g_event_count    = 0;
static alarm_event_t g_last_event = ALARM_EVENT_CLEAR;
static uint16_t  g_last_value     = 0u;

static bool mock_read_fn(uint16_t device_address,
                         uint16_t *out_value,
                         void     *userdata)
{
    (void)device_address;
    (void)userdata;
    if (!g_mock_read_available) {
        return false;
    }
    *out_value = g_mock_register_value;
    return true;
}

static void mock_event_fn(const alarm_entry_t *entry,
                          uint16_t             value,
                          alarm_event_t        event,
                          void                *userdata)
{
    (void)entry;
    (void)userdata;
    g_event_count++;
    g_last_event = event;
    g_last_value = value;
}

/* ── Fixtures ─────────────────────────────────────────────────────────── */

void setUp(void)
{
    g_mock_register_value = 0u;
    g_mock_read_available = true;
    g_event_count         = 0;
    g_last_event          = ALARM_EVENT_CLEAR;
    g_last_value          = 0u;
}

void tearDown(void) {}

/* ── Helper: build a minimal ctx with one entry ───────────────────────── */

static void build_ctx(alarm_engine_ctx_t  *ctx,
                      alarm_entry_t       *entry,
                      alarm_state_t       *state,
                      alarm_cond_t         cond,
                      uint16_t             lo,
                      uint16_t             hi)
{
    entry->device_address = 0x0000u;
    entry->condition      = cond;
    entry->lo_limit       = lo;
    entry->hi_limit       = hi;
    entry->error_code     = 0x0001u;
    entry->description    = "test";

    state->is_active  = false;
    state->prev_value = 0u;

    ctx->table      = entry;
    ctx->states     = state;
    ctx->count      = 1u;
    ctx->read_fn    = mock_read_fn;
    ctx->event_fn   = mock_event_fn;
    ctx->read_data  = NULL;
    ctx->event_data = NULL;
}

/* ═══════════════════════════════════════════════════════════════════════
 * BITMASK tests
 * ═══════════════════════════════════════════════════════════════════════ */

void test_bitmask_triggers_when_bits_match(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_BITMASK, 0x0000u, 0x0100u);

    g_mock_register_value = 0x0100u;  /* mask matches */
    alarm_engine_run(&ctx);

    TEST_ASSERT_EQUAL_INT(1, g_event_count);
    TEST_ASSERT_EQUAL_INT(ALARM_EVENT_TRIGGER, g_last_event);
    TEST_ASSERT_EQUAL_UINT16(0x0100u, g_last_value);
}

void test_bitmask_no_event_when_bits_clear(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_BITMASK, 0x0000u, 0x0100u);

    g_mock_register_value = 0x0010u;  /* different bit, mask doesn't match */
    alarm_engine_run(&ctx);

    TEST_ASSERT_EQUAL_INT(0, g_event_count);
}

void test_bitmask_sticky_fires_only_once_while_active(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_BITMASK, 0x0000u, 0x0100u);

    g_mock_register_value = 0x0100u;
    alarm_engine_run(&ctx);  /* first call → TRIGGER */
    alarm_engine_run(&ctx);  /* still active → suppressed */
    alarm_engine_run(&ctx);  /* still active → suppressed */

    TEST_ASSERT_EQUAL_INT(1, g_event_count);
    TEST_ASSERT_EQUAL_INT(ALARM_EVENT_TRIGGER, g_last_event);
}

void test_bitmask_fires_clear_when_bits_cleared(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_BITMASK, 0x0000u, 0x0100u);

    g_mock_register_value = 0x0100u;
    alarm_engine_run(&ctx);  /* TRIGGER */

    g_mock_register_value = 0x0000u;
    alarm_engine_run(&ctx);  /* CLEAR */

    TEST_ASSERT_EQUAL_INT(2, g_event_count);
    TEST_ASSERT_EQUAL_INT(ALARM_EVENT_CLEAR, g_last_event);
}

void test_bitmask_retriggerable_after_clear(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_BITMASK, 0x0000u, 0x0100u);

    g_mock_register_value = 0x0100u;
    alarm_engine_run(&ctx);  /* TRIGGER */

    g_mock_register_value = 0x0000u;
    alarm_engine_run(&ctx);  /* CLEAR */

    g_mock_register_value = 0x0100u;
    alarm_engine_run(&ctx);  /* TRIGGER again */

    TEST_ASSERT_EQUAL_INT(3, g_event_count);
    TEST_ASSERT_EQUAL_INT(ALARM_EVENT_TRIGGER, g_last_event);
}

/* ═══════════════════════════════════════════════════════════════════════
 * RANGE tests
 * ═══════════════════════════════════════════════════════════════════════ */

void test_range_no_event_when_value_in_range(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_RANGE, 0x0014u, 0xFFFFu);

    g_mock_register_value = 0x0020u;  /* inside [0x14, 0xFFFF] */
    alarm_engine_run(&ctx);

    TEST_ASSERT_EQUAL_INT(0, g_event_count);
}

void test_range_triggers_when_value_below_lo(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_RANGE, 0x0014u, 0xFFFFu);

    g_mock_register_value = 0x000Au;  /* below lo_limit=0x14 */
    alarm_engine_run(&ctx);

    TEST_ASSERT_EQUAL_INT(1, g_event_count);
    TEST_ASSERT_EQUAL_INT(ALARM_EVENT_TRIGGER, g_last_event);
}

void test_range_triggers_when_value_above_hi(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    /* Range [0x0082, 0x00DC] – voltage 130–220 (×1) */
    build_ctx(&ctx, &entry, &state, ALARM_COND_RANGE, 0x0082u, 0x00DCu);

    g_mock_register_value = 0x00FFu;  /* above hi_limit */
    alarm_engine_run(&ctx);

    TEST_ASSERT_EQUAL_INT(1, g_event_count);
    TEST_ASSERT_EQUAL_INT(ALARM_EVENT_TRIGGER, g_last_event);
}

void test_range_sticky_suppresses_repeated_triggers(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_RANGE, 0x0082u, 0x00DCu);

    g_mock_register_value = 0x00FFu;
    alarm_engine_run(&ctx);  /* TRIGGER */
    alarm_engine_run(&ctx);  /* suppressed */
    alarm_engine_run(&ctx);  /* suppressed */

    TEST_ASSERT_EQUAL_INT(1, g_event_count);
}

void test_range_clears_when_value_returns_to_range(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_RANGE, 0x0082u, 0x00DCu);

    g_mock_register_value = 0x00FFu;
    alarm_engine_run(&ctx);  /* TRIGGER */

    g_mock_register_value = 0x00B4u;  /* inside range */
    alarm_engine_run(&ctx);  /* CLEAR */

    TEST_ASSERT_EQUAL_INT(2, g_event_count);
    TEST_ASSERT_EQUAL_INT(ALARM_EVENT_CLEAR, g_last_event);
}

/* ═══════════════════════════════════════════════════════════════════════
 * CHANGE tests
 * ═══════════════════════════════════════════════════════════════════════ */

void test_change_no_event_when_value_unchanged_from_zero(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_CHANGE, 0x0000u, 0x0000u);

    g_mock_register_value = 0x0000u;
    alarm_engine_run(&ctx);  /* prev=0, value=0 → no change */
    alarm_engine_run(&ctx);  /* still 0 → no change */

    TEST_ASSERT_EQUAL_INT(0, g_event_count);
}

void test_change_no_event_when_value_unchanged(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_CHANGE, 0x0000u, 0x0000u);

    g_mock_register_value = 0x0005u;
    alarm_engine_run(&ctx);  /* first call: prev=0, value=5, non-zero and different → TRIGGER */
    g_event_count = 0;       /* reset counter */

    alarm_engine_run(&ctx);  /* second call: prev=5, value=5, same → no event */

    TEST_ASSERT_EQUAL_INT(0, g_event_count);
}

void test_change_triggers_on_each_distinct_change(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_CHANGE, 0x0000u, 0x0000u);

    g_mock_register_value = 0x0001u;
    alarm_engine_run(&ctx);  /* TRIGGER (0 → 1) */

    g_mock_register_value = 0x0002u;
    alarm_engine_run(&ctx);  /* TRIGGER (1 → 2) */

    g_mock_register_value = 0x0003u;
    alarm_engine_run(&ctx);  /* TRIGGER (2 → 3) */

    TEST_ASSERT_EQUAL_INT(3, g_event_count);
    TEST_ASSERT_EQUAL_INT(ALARM_EVENT_TRIGGER, g_last_event);
}

void test_change_triggers_on_return_to_zero(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_CHANGE, 0x0000u, 0x0000u);

    g_mock_register_value = 0x0001u;
    alarm_engine_run(&ctx);  /* TRIGGER (0 → 1) */

    g_mock_register_value = 0x0000u;
    alarm_engine_run(&ctx);  /* TRIGGER (1 → 0), never CLEAR */

    TEST_ASSERT_EQUAL_INT(2, g_event_count);
    TEST_ASSERT_EQUAL_INT(ALARM_EVENT_TRIGGER, g_last_event);
}

/* ═══════════════════════════════════════════════════════════════════════
 * read_fn failure
 * ═══════════════════════════════════════════════════════════════════════ */

void test_read_fn_failure_skips_entry(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_BITMASK, 0x0000u, 0x0100u);

    g_mock_read_available = false;  /* simulate unavailable register */
    g_mock_register_value = 0x0100u;
    alarm_engine_run(&ctx);

    TEST_ASSERT_EQUAL_INT(0, g_event_count);
}

void test_read_fn_failure_does_not_clear_active_alarm(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_BITMASK, 0x0000u, 0x0100u);

    g_mock_register_value = 0x0100u;
    alarm_engine_run(&ctx);  /* TRIGGER, is_active = true */

    g_mock_read_available = false;
    alarm_engine_run(&ctx);  /* should be skipped, NOT cleared */

    TEST_ASSERT_EQUAL_INT(1, g_event_count);
    TEST_ASSERT_TRUE(state.is_active);  /* sticky flag still set */
}

/* ═══════════════════════════════════════════════════════════════════════
 * NULL / invalid context
 * ═══════════════════════════════════════════════════════════════════════ */

void test_null_ctx_does_not_crash(void)
{
    alarm_engine_run(NULL);
    TEST_PASS();
}

void test_zero_count_does_not_crash(void)
{
    alarm_engine_ctx_t ctx;
    alarm_entry_t      entry;
    alarm_state_t      state;

    build_ctx(&ctx, &entry, &state, ALARM_COND_BITMASK, 0x0000u, 0x0100u);
    ctx.count = 0u;

    alarm_engine_run(&ctx);
    TEST_ASSERT_EQUAL_INT(0, g_event_count);
}

/* ── Main ─────────────────────────────────────────────────────────────── */

int main(void)
{
    UNITY_BEGIN();

    /* BITMASK */
    RUN_TEST(test_bitmask_triggers_when_bits_match);
    RUN_TEST(test_bitmask_no_event_when_bits_clear);
    RUN_TEST(test_bitmask_sticky_fires_only_once_while_active);
    RUN_TEST(test_bitmask_fires_clear_when_bits_cleared);
    RUN_TEST(test_bitmask_retriggerable_after_clear);

    /* RANGE */
    RUN_TEST(test_range_no_event_when_value_in_range);
    RUN_TEST(test_range_triggers_when_value_below_lo);
    RUN_TEST(test_range_triggers_when_value_above_hi);
    RUN_TEST(test_range_sticky_suppresses_repeated_triggers);
    RUN_TEST(test_range_clears_when_value_returns_to_range);

    /* CHANGE */
    RUN_TEST(test_change_no_event_when_value_unchanged_from_zero);
    RUN_TEST(test_change_no_event_when_value_unchanged);
    RUN_TEST(test_change_triggers_on_each_distinct_change);
    RUN_TEST(test_change_triggers_on_return_to_zero);

    /* read_fn failure */
    RUN_TEST(test_read_fn_failure_skips_entry);
    RUN_TEST(test_read_fn_failure_does_not_clear_active_alarm);

    /* NULL / edge cases */
    RUN_TEST(test_null_ctx_does_not_crash);
    RUN_TEST(test_zero_count_does_not_crash);

    return UNITY_END();
}
