/**
 * @file test_inverter_map.c
 * @brief Unit tests for devices/inverter/inverter_map.c
 *
 * Covered scenarios
 * ─────────────────
 *  profile content – inverter1_profile name, table_count, read_chunk,
 *                    pool address layout, and key register entries.
 *                    (inverter1_profile is referenced directly by
 *                    inverter_module.c – there is no lookup function to
 *                    test, since it is the only Inverter hardware model
 *                    currently implemented.)
 *  table ordering  – rows are sorted by device_address ascending
 *                    (required by device_find_slot binary search).
 *  pool uniqueness – all pool_address values within a profile are unique.
 */

#include "unity/unity.h"
#include "inverter/inverter_map.h"
#include "device_register_map.h"

/* ── Fixtures ─────────────────────────────────────────────────────────── */

void setUp(void)  {}
void tearDown(void) {}

/* ── Profile content ──────────────────────────────────────────────────── */

void test_inverter1_profile_has_correct_name(void)
{
    TEST_ASSERT_EQUAL_STRING("Inverter1", inverter1_profile.name);
}

void test_inverter1_profile_table_is_not_null(void)
{
    TEST_ASSERT_NOT_NULL(inverter1_profile.table);
}

void test_inverter1_profile_table_count_is_32(void)
{
    /* inverter_map.c defines 32 registers for Inverter#1 */
    TEST_ASSERT_EQUAL_UINT(32u, (unsigned)inverter1_profile.table_count);
}

void test_inverter1_profile_read_chunk_is_20(void)
{
    TEST_ASSERT_EQUAL_UINT16(20u, inverter1_profile.read_chunk);
}

/* ── Pool address layout ──────────────────────────────────────────────── */

void test_inverter1_pool_base_starts_at_0x0000(void)
{
    /* First pool_address in the table must be 0x0000 */
    TEST_ASSERT_EQUAL_UINT16(0x0000u, inverter1_profile.table[0].pool_address);
}

void test_inverter1_pool_last_address_is_0x001F(void)
{
    size_t last = inverter1_profile.table_count - 1u;
    TEST_ASSERT_EQUAL_UINT16(0x001Fu,
                             inverter1_profile.table[last].pool_address);
}

/* ── Key register entries ─────────────────────────────────────────────── */

void test_inverter1_has_device_status_at_0x0000(void)
{
    const device_register_mapping_t *e =
        device_find_slot(&inverter1_profile, 0x0000u);

    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_UINT16(0x0000u, e->device_address);
    TEST_ASSERT_EQUAL_INT(ACCESS_RO, e->access);
}

void test_inverter1_has_ac_output_voltage_at_0x0010(void)
{
    const device_register_mapping_t *e =
        device_find_slot(&inverter1_profile, 0x0010u);

    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_INT(ACCESS_RO, e->access);
}

void test_inverter1_has_battery_soc_at_0x0042(void)
{
    const device_register_mapping_t *e =
        device_find_slot(&inverter1_profile, 0x0042u);

    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_INT(ACCESS_RO, e->access);
}

void test_inverter1_has_output_voltage_set_at_0x0100_rw(void)
{
    const device_register_mapping_t *e =
        device_find_slot(&inverter1_profile, 0x0100u);

    TEST_ASSERT_NOT_NULL(e);
    TEST_ASSERT_EQUAL_INT(ACCESS_RW, e->access);
}

/* ── Table ordering (ascending device_address) ────────────────────────── */

void test_inverter1_table_is_sorted_ascending(void)
{
    for (size_t i = 1u; i < inverter1_profile.table_count; i++) {
        TEST_ASSERT_TRUE(
            inverter1_profile.table[i].device_address >
            inverter1_profile.table[i - 1u].device_address
        );
    }
}

/* ── Pool address uniqueness ──────────────────────────────────────────── */

void test_inverter1_pool_addresses_are_unique(void)
{
    size_t count = inverter1_profile.table_count;

    for (size_t i = 0u; i < count; i++) {
        for (size_t j = i + 1u; j < count; j++) {
            if (inverter1_profile.table[i].pool_address ==
                inverter1_profile.table[j].pool_address) {
                TEST_FAIL_MESSAGE("Duplicate pool_address found in inverter1 table");
            }
        }
    }
    TEST_PASS();
}

/* ── Main ─────────────────────────────────────────────────────────────── */

int main(void)
{
    UNITY_BEGIN();

    /* profile content */
    RUN_TEST(test_inverter1_profile_has_correct_name);
    RUN_TEST(test_inverter1_profile_table_is_not_null);
    RUN_TEST(test_inverter1_profile_table_count_is_32);
    RUN_TEST(test_inverter1_profile_read_chunk_is_20);

    /* pool address layout */
    RUN_TEST(test_inverter1_pool_base_starts_at_0x0000);
    RUN_TEST(test_inverter1_pool_last_address_is_0x001F);

    /* key register entries */
    RUN_TEST(test_inverter1_has_device_status_at_0x0000);
    RUN_TEST(test_inverter1_has_ac_output_voltage_at_0x0010);
    RUN_TEST(test_inverter1_has_battery_soc_at_0x0042);
    RUN_TEST(test_inverter1_has_output_voltage_set_at_0x0100_rw);

    /* table ordering */
    RUN_TEST(test_inverter1_table_is_sorted_ascending);

    /* pool uniqueness */
    RUN_TEST(test_inverter1_pool_addresses_are_unique);

    return UNITY_END();
}
