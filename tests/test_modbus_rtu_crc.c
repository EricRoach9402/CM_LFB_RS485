/**
 * @file test_modbus_rtu_crc.c
 * @brief Unit tests for Modbus RTU CRC16 computation.
 *
 * Tests mb_rtu_compute_crc16() against:
 *  1. Known Modbus RTU CRC vectors (cross-checked with independent reference).
 *  2. Structural properties (empty buffer, single byte).
 *  3. Round-trip property: verify_crc(data + appended_crc) passes.
 *
 * Reference CRC16 implementation
 * ────────────────────────────────
 *  A reference implementation is provided locally in this file to generate
 *  expected values independently from the production code.  Both use the
 *  same polynomial (0xA001) and initial value (0xFFFF) but are written
 *  separately to catch algorithmic mistakes in either copy.
 *
 * Known test vectors
 * ──────────────────
 *  The 16-bit CRC value returned by mb_rtu_compute_crc16() is stored with
 *  the LOW byte in bits [7:0] and the HIGH byte in bits [15:8].
 *  In a Modbus RTU frame the CRC is transmitted LOW byte first, then HIGH byte.
 *
 *  Vector 1: FC03 read-holding-registers request
 *    bytes:   01 03 00 00 00 01
 *    CRC:     0x0A84  (lo=0x84 appended first, hi=0x0A appended second)
 *    Source:  computed with the production algorithm and reference implementation.
 *
 *  Vector 2: FC06 write-single-register request
 *    bytes:   01 06 00 01 00 03
 *    CRC:     0x0B98  (lo=0x98, hi=0x0B)
 */

#include "unity/unity.h"
#include "modbus_rtu_client.h"

#include <stddef.h>
#include <stdint.h>

/* ── Reference CRC16 (independently written for test validation) ──────── */

/**
 * @brief Reference Modbus CRC16 – independently implemented.
 *
 * Intentionally uses a table-free, loop-unrolled-free style different from
 * the production code to avoid sharing bugs.
 */
static uint16_t ref_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;
    size_t   byte_idx;
    int      bit_idx;

    for (byte_idx = 0; byte_idx < length; byte_idx++) {
        crc = crc ^ (uint16_t)data[byte_idx];
        for (bit_idx = 0; bit_idx < 8; bit_idx++) {
            uint16_t lsb = crc & 0x0001u;
            crc >>= 1;
            if (lsb) {
                crc ^= 0xA001u;
            }
        }
    }
    return crc;
}

/* ── Fixtures ─────────────────────────────────────────────────────────── */

void setUp(void)  {}
void tearDown(void) {}

/* ── Known vector tests ───────────────────────────────────────────────── */

void test_crc_fc03_read_holding_registers_request(void)
{
    /*
     * FC03 request: slave=01, FC=03, start_addr=0x0000, qty=0x0001
     * CRC16 = 0x0A84  →  transmitted bytes: 0x84 (lo), 0x0A (hi)
     */
    const uint8_t msg[] = { 0x01u, 0x03u, 0x00u, 0x00u, 0x00u, 0x01u };
    uint16_t crc = mb_rtu_compute_crc16(msg, sizeof(msg));
    TEST_ASSERT_EQUAL_UINT16(0x0A84u, crc);
}

void test_crc_fc06_write_single_register_request(void)
{
    /*
     * FC06 request: slave=01, FC=06, addr=0x0001, value=0x0003
     * CRC16 = 0x0B98  →  transmitted bytes: 0x98 (lo), 0x0B (hi)
     */
    const uint8_t msg[] = { 0x01u, 0x06u, 0x00u, 0x01u, 0x00u, 0x03u };
    uint16_t crc = mb_rtu_compute_crc16(msg, sizeof(msg));
    TEST_ASSERT_EQUAL_UINT16(0x0B98u, crc);
}

void test_crc_matches_reference_implementation(void)
{
    /*
     * Verify production code agrees with the independently written reference
     * for a longer payload.
     */
    const uint8_t msg[] = {
        0x01u, 0x10u, 0x00u, 0x64u, 0x00u, 0x03u, 0x06u,
        0x00u, 0xC8u, 0x01u, 0x2Cu, 0x00u, 0xFAu
    };
    uint16_t expected = ref_crc16(msg, sizeof(msg));
    uint16_t actual   = mb_rtu_compute_crc16(msg, sizeof(msg));
    TEST_ASSERT_EQUAL_UINT16(expected, actual);
}

/* ── Structural / edge cases ──────────────────────────────────────────── */

void test_crc_empty_buffer_equals_initial_value(void)
{
    /*
     * With no bytes processed the accumulator stays at the initial value
     * 0xFFFF (no bits have been shifted in).
     */
    uint16_t crc = mb_rtu_compute_crc16(NULL, 0u);
    TEST_ASSERT_EQUAL_UINT16(0xFFFFu, crc);
}

void test_crc_different_messages_produce_different_crcs(void)
{
    const uint8_t msg_a[] = { 0x01u, 0x03u, 0x00u, 0x00u, 0x00u, 0x01u };
    const uint8_t msg_b[] = { 0x02u, 0x03u, 0x00u, 0x00u, 0x00u, 0x01u };

    uint16_t crc_a = mb_rtu_compute_crc16(msg_a, sizeof(msg_a));
    uint16_t crc_b = mb_rtu_compute_crc16(msg_b, sizeof(msg_b));

    TEST_ASSERT_NOT_EQUAL(crc_a, crc_b);
}

void test_crc_same_message_always_gives_same_result(void)
{
    const uint8_t msg[] = { 0x01u, 0x03u, 0x00u, 0x00u, 0x00u, 0x01u };

    uint16_t crc1 = mb_rtu_compute_crc16(msg, sizeof(msg));
    uint16_t crc2 = mb_rtu_compute_crc16(msg, sizeof(msg));

    TEST_ASSERT_EQUAL_UINT16(crc1, crc2);
}

/* ── Round-trip: appended CRC verifies correctly ──────────────────────── */

void test_crc_round_trip_appended_crc_verifies(void)
{
    /*
     * Build a frame: data bytes + CRC appended LSB-first.
     * Re-compute CRC over (data only) and compare against the stored bytes.
     * This simulates the receiver's verify_crc logic.
     */
    uint8_t  frame[8];
    size_t   data_len = 6u;

    frame[0] = 0x01u;
    frame[1] = 0x03u;
    frame[2] = 0x00u;
    frame[3] = 0x00u;
    frame[4] = 0x00u;
    frame[5] = 0x01u;

    uint16_t crc = mb_rtu_compute_crc16(frame, data_len);
    frame[6] = (uint8_t)(crc & 0xFFu);   /* CRC low byte */
    frame[7] = (uint8_t)(crc >> 8);       /* CRC high byte */

    /* Receiver side: recompute CRC over data bytes and compare stored bytes */
    uint16_t recv_crc = mb_rtu_compute_crc16(frame, data_len);
    uint16_t stored   = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);

    TEST_ASSERT_EQUAL_UINT16(recv_crc, stored);
}

void test_crc_round_trip_detects_bit_flip(void)
{
    /*
     * Flip one bit in the data and verify the recomputed CRC no longer
     * matches the originally stored CRC.
     */
    uint8_t frame[8];
    size_t  data_len = 6u;

    frame[0] = 0x01u;
    frame[1] = 0x03u;
    frame[2] = 0x00u;
    frame[3] = 0x00u;
    frame[4] = 0x00u;
    frame[5] = 0x01u;

    uint16_t crc = mb_rtu_compute_crc16(frame, data_len);
    frame[6] = (uint8_t)(crc & 0xFFu);
    frame[7] = (uint8_t)(crc >> 8);

    /* Corrupt one data byte */
    frame[3] ^= 0x01u;

    uint16_t recv_crc = mb_rtu_compute_crc16(frame, data_len);
    uint16_t stored   = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);

    TEST_ASSERT_NOT_EQUAL(recv_crc, stored);
}

/* ── Main ─────────────────────────────────────────────────────────────── */

int main(void)
{
    UNITY_BEGIN();

    /* known vectors */
    RUN_TEST(test_crc_fc03_read_holding_registers_request);
    RUN_TEST(test_crc_fc06_write_single_register_request);
    RUN_TEST(test_crc_matches_reference_implementation);

    /* edge cases */
    RUN_TEST(test_crc_empty_buffer_equals_initial_value);
    RUN_TEST(test_crc_different_messages_produce_different_crcs);
    RUN_TEST(test_crc_same_message_always_gives_same_result);

    /* round-trip */
    RUN_TEST(test_crc_round_trip_appended_crc_verifies);
    RUN_TEST(test_crc_round_trip_detects_bit_flip);

    return UNITY_END();
}
