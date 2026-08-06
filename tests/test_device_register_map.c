/**
 * @file test_device_register_map.c
 * @brief Unit tests for device_register_map.c
 *
 * Covered scenarios
 * ─────────────────
 *  pool init        – device_register_map_init() zeros the pool.
 *  pool read/write  – pool_read_register / pool_write_register basic access.
 *  out-of-range     – both functions return false for address >= INTERNAL_POOL_SIZE.
 *  device_find_slot – binary search finds exact match; returns NULL for missing.
 *  read_to_pool     – device_map_read_to_pool() stores raw values (no masking).
 *  by device_addr   – pool_read_by_device_addr / pool_write_by_device_addr.
 *  masked read      – pool_read_masked_by_device_addr() applies caller mask.
 */

#include "unity/unity.h"
#include "device_register_map.h"

/* ── Minimal test profile ─────────────────────────────────────────────── */

static const device_register_mapping_t test_table[] = {
    { 0x0000u, 0x0010u, ACCESS_RO, "reg_0x0000"    },
    { 0x0001u, 0x0011u, ACCESS_RW, "reg_0x0001"    },
    { 0x0002u, 0x0012u, ACCESS_RW, "reg_0x0002"    },
    { 0x0010u, 0x0020u, ACCESS_WO, "reg_0x0010_wo" },
    { 0x0020u, 0x0030u, ACCESS_RW, "reg_0x0020"    },
};

static const device_map_profile_t test_profile = {
    .name        = "TestDevice",
    .table       = test_table,
    .table_count = sizeof(test_table) / sizeof(test_table[0]),
    .read_chunk  = 10u,
};

/* ── Fixtures ─────────────────────────────────────────────────────────── */

void setUp(void)
{
    device_register_map_init();
}

void tearDown(void) {}

/* ── Pool init ────────────────────────────────────────────────────────── */

void test_init_zeros_pool(void)
{
    uint16_t value = 0xDEADu;

    pool_write_register(0x0010u, 0x1234u);
    device_register_map_init();

    bool ok = pool_read_register(0x0010u, &value);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT16(0x0000u, value);
}

/* ── Direct pool read/write ───────────────────────────────────────────── */

void test_pool_write_then_read_same_value(void)
{
    bool ok_write = pool_write_register(0x0050u, 0xABCDu);
    TEST_ASSERT_TRUE(ok_write);

    uint16_t value = 0u;
    bool ok_read = pool_read_register(0x0050u, &value);
    TEST_ASSERT_TRUE(ok_read);
    TEST_ASSERT_EQUAL_UINT16(0xABCDu, value);
}

void test_pool_write_at_last_valid_address(void)
{
    uint16_t addr  = INTERNAL_POOL_SIZE - 1u;
    bool     ok    = pool_write_register(addr, 0x5A5Au);
    TEST_ASSERT_TRUE(ok);

    uint16_t value = 0u;
    pool_read_register(addr, &value);
    TEST_ASSERT_EQUAL_UINT16(0x5A5Au, value);
}

void test_pool_read_out_of_range_returns_false(void)
{
    uint16_t value = 0u;
    bool ok = pool_read_register(INTERNAL_POOL_SIZE, &value);
    TEST_ASSERT_FALSE(ok);
}

void test_pool_write_out_of_range_returns_false(void)
{
    bool ok = pool_write_register(INTERNAL_POOL_SIZE, 0x1234u);
    TEST_ASSERT_FALSE(ok);
}

void test_pool_write_overwrites_previous_value(void)
{
    pool_write_register(0x0010u, 0x0001u);
    pool_write_register(0x0010u, 0x0002u);

    uint16_t value = 0u;
    pool_read_register(0x0010u, &value);
    TEST_ASSERT_EQUAL_UINT16(0x0002u, value);
}

/* ── device_find_slot (binary search) ────────────────────────────────── */

void test_find_slot_returns_entry_for_first_address(void)
{
    const device_register_mapping_t *entry =
        device_find_slot(&test_profile, 0x0000u);

    TEST_ASSERT_NOT_NULL(entry);
    TEST_ASSERT_EQUAL_UINT16(0x0000u, entry->device_address);
    TEST_ASSERT_EQUAL_UINT16(0x0010u, entry->pool_address);
}

void test_find_slot_returns_entry_for_middle_address(void)
{
    const device_register_mapping_t *entry =
        device_find_slot(&test_profile, 0x0002u);

    TEST_ASSERT_NOT_NULL(entry);
    TEST_ASSERT_EQUAL_UINT16(0x0002u, entry->device_address);
    TEST_ASSERT_EQUAL_UINT16(0x0012u, entry->pool_address);
}

void test_find_slot_returns_entry_for_last_address(void)
{
    const device_register_mapping_t *entry =
        device_find_slot(&test_profile, 0x0020u);

    TEST_ASSERT_NOT_NULL(entry);
    TEST_ASSERT_EQUAL_UINT16(0x0020u, entry->device_address);
}

void test_find_slot_returns_null_for_missing_address(void)
{
    const device_register_mapping_t *entry =
        device_find_slot(&test_profile, 0x0099u);

    TEST_ASSERT_NULL(entry);
}

void test_find_slot_returns_null_for_address_between_entries(void)
{
    /* Table has 0x0001 and 0x0010, but nothing at 0x0005 */
    const device_register_mapping_t *entry =
        device_find_slot(&test_profile, 0x0005u);

    TEST_ASSERT_NULL(entry);
}

/* ── device_map_read_to_pool ──────────────────────────────────────────── */

void test_read_to_pool_stores_values_at_correct_pool_addresses(void)
{
    uint16_t buf[] = { 0x1111u, 0x2222u, 0x3333u };

    /*
     * Read 3 consecutive registers starting at device_address 0x0000.
     * Expected pool mapping (raw, no masking):
     *   0x0000 → pool 0x0010 = 0x1111
     *   0x0001 → pool 0x0011 = 0x2222
     *   0x0002 → pool 0x0012 = 0x3333
     */
    device_map_read_to_pool(&test_profile, buf, 0x0000u, 3);

    uint16_t v0 = 0u, v1 = 0u, v2 = 0u;
    pool_read_register(0x0010u, &v0);
    pool_read_register(0x0011u, &v1);
    pool_read_register(0x0012u, &v2);

    TEST_ASSERT_EQUAL_UINT16(0x1111u, v0);
    TEST_ASSERT_EQUAL_UINT16(0x2222u, v1);
    TEST_ASSERT_EQUAL_UINT16(0x3333u, v2);  /* raw – no mask applied at pool */
}

void test_read_to_pool_ignores_unmapped_device_addresses(void)
{
    /*
     * Buffer covers device_address 0x0004 and 0x0005, neither of which
     * is in test_table.  Pool should remain all-zero.
     */
    uint16_t buf[] = { 0xAAAAu, 0xBBBBu };
    device_map_read_to_pool(&test_profile, buf, 0x0004u, 2);

    uint16_t v = 0xFFFFu;
    pool_read_register(0x0010u, &v);
    TEST_ASSERT_EQUAL_UINT16(0x0000u, v);
}

/* ── pool_read_by_device_addr ─────────────────────────────────────────── */

void test_read_by_device_addr_returns_pool_value(void)
{
    pool_write_register(0x0011u, 0xCAFEu);

    uint16_t value = 0u;
    bool ok = pool_read_by_device_addr(&test_profile, 0x0001u, &value);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT16(0xCAFEu, value);
}

void test_read_by_device_addr_returns_false_for_unknown_address(void)
{
    uint16_t value = 0u;
    bool ok = pool_read_by_device_addr(&test_profile, 0x00FFu, &value);
    TEST_ASSERT_FALSE(ok);
}

/* ── pool_write_by_device_addr ────────────────────────────────────────── */

void test_write_by_device_addr_updates_pool(void)
{
    bool ok = pool_write_by_device_addr(&test_profile, 0x0001u, 0x1234u);
    TEST_ASSERT_TRUE(ok);

    uint16_t value = 0u;
    pool_read_register(0x0011u, &value);
    TEST_ASSERT_EQUAL_UINT16(0x1234u, value);
}

void test_write_by_device_addr_returns_false_for_unknown_address(void)
{
    bool ok = pool_write_by_device_addr(&test_profile, 0x00FFu, 0xDEADu);
    TEST_ASSERT_FALSE(ok);
}

/* ── pool_read_masked_by_device_addr ──────────────────────────────────── */

void test_read_masked_applies_caller_mask(void)
{
    /* Pool slot for device 0x0002 is 0x0012 */
    pool_write_register(0x0012u, 0x3333u);

    uint16_t value = 0u;
    bool ok = pool_read_masked_by_device_addr(&test_profile, 0x0002u,
                                              0x00FFu, &value);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT16(0x0033u, value);  /* 0x3333 & 0x00FF */
}

void test_read_masked_full_mask_returns_raw(void)
{
    pool_write_register(0x0011u, 0xABCDu);

    uint16_t value = 0u;
    bool ok = pool_read_masked_by_device_addr(&test_profile, 0x0001u,
                                              0xFFFFu, &value);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT16(0xABCDu, value);
}

void test_read_masked_returns_false_for_unknown_address(void)
{
    uint16_t value = 0u;
    bool ok = pool_read_masked_by_device_addr(&test_profile, 0x00FFu,
                                              0xFFFFu, &value);
    TEST_ASSERT_FALSE(ok);
}

/* ── Main ─────────────────────────────────────────────────────────────── */

int main(void)
{
    UNITY_BEGIN();

    /* pool init */
    RUN_TEST(test_init_zeros_pool);

    /* direct pool read/write */
    RUN_TEST(test_pool_write_then_read_same_value);
    RUN_TEST(test_pool_write_at_last_valid_address);
    RUN_TEST(test_pool_read_out_of_range_returns_false);
    RUN_TEST(test_pool_write_out_of_range_returns_false);
    RUN_TEST(test_pool_write_overwrites_previous_value);

    /* device_find_slot */
    RUN_TEST(test_find_slot_returns_entry_for_first_address);
    RUN_TEST(test_find_slot_returns_entry_for_middle_address);
    RUN_TEST(test_find_slot_returns_entry_for_last_address);
    RUN_TEST(test_find_slot_returns_null_for_missing_address);
    RUN_TEST(test_find_slot_returns_null_for_address_between_entries);

    /* device_map_read_to_pool */
    RUN_TEST(test_read_to_pool_stores_values_at_correct_pool_addresses);
    RUN_TEST(test_read_to_pool_ignores_unmapped_device_addresses);

    /* pool_read_by_device_addr */
    RUN_TEST(test_read_by_device_addr_returns_pool_value);
    RUN_TEST(test_read_by_device_addr_returns_false_for_unknown_address);

    /* pool_write_by_device_addr */
    RUN_TEST(test_write_by_device_addr_updates_pool);
    RUN_TEST(test_write_by_device_addr_returns_false_for_unknown_address);

    /* pool_read_masked_by_device_addr */
    RUN_TEST(test_read_masked_applies_caller_mask);
    RUN_TEST(test_read_masked_full_mask_returns_raw);
    RUN_TEST(test_read_masked_returns_false_for_unknown_address);

    return UNITY_END();
}
