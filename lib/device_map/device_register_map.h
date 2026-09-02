/**
 * @file device_register_map.h
 * @brief Generic Modbus device register → internal pool mapping engine.
 *
 * Architecture
 * ────────────
 *  device_register_mapping_t  – one row in a per-unit table:
 *    device_address → pool_address (absolute position in internal_pool[])
 *
 *  device_map_profile_t       – describes one physical device unit.
 *    Each unit has its own table; pool_address values are absolute and
 *    hardcoded in the table.  No runtime pool-base arithmetic required.
 *
 * Pool contract
 * ─────────────
 *  internal_pool[] always stores the raw register value read from the device.
 *  No masking is applied at the ingest stage.  Consumers that need a masked
 *  view call pool_read_masked_by_device_addr() and supply their own mask.
 *  Bit-level alarm monitoring is handled by alarm_engine (ALARM_COND_BITMASK).
 *
 * Pool address block allocation (project-wide convention)
 * ─────────────────────────────────────────────────────────
 *  internal_pool[] is one flat array shared by every device family in the
 *  project.  Existing families (Inverter, UPS) each derive their
 *  pool_address values from "device_address + 0xB000", so their literal
 *  addresses happen to interleave inside 0xB000-0xD1FF without colliding
 *  at the individual-address level – this is verified automatically at
 *  startup (see device_register_map_register_profile() below), not by
 *  address-range bookkeeping.
 *
 *  For any NEW device family, do not reuse that offset pattern: claim an
 *  entirely fresh, visually distinct block instead, e.g.:
 *
 *    0x0000 – 0xAFFF   reserved / unused
 *    0xB000 – 0xDFFF   in use today (Inverter + UPS, interleaved as above)
 *    0xE000 – 0xFFFF   reserved for the next device family
 *
 *  This is documentation, not compiler-enforced.  The real safety net is
 *  device_register_map_register_profile(): call it once per profile right
 *  after a unit's profile is looked up (see start_inverter_modules() /
 *  start_ups_modules()).  It cross-checks every pool_address in the new
 *  profile against every previously registered profile and aborts
 *  registration on overlap.
 *
 * Adding a new device unit
 * ────────────────────────
 *  1. Create a new mapping table in devices/<name>/<name>_map.c with
 *     absolute pool_address values inside your family's reserved block.
 *  2. Expose a device_map_profile_t in devices/<name>/<name>_map.h.
 *  3. Register it in the module's uid→profile lookup table.
 *  4. Call device_register_map_register_profile(profile) once the unit's
 *     profile is resolved (see the existing start_xxx_modules() for the
 *     pattern).
 *  No changes needed to this file.
 */

#ifndef DEVICE_REGISTER_MAP_H
#define DEVICE_REGISTER_MAP_H

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "modbus_defines.h"
#include "config_loader.h"

/**
 * @brief Total addressable registers in the shared internal pool.
 * Must be large enough to cover all per-unit pool_address values.
 */
#define INTERNAL_POOL_SIZE 65535

/**
 * @brief Max number of device_map_profile_t that can be tracked by
 * device_register_map_register_profile()'s collision checker.
 */
#define DEVICE_REGISTER_MAP_MAX_PROFILES 32

/* ── Access permission ────────────────────────────────────────────────── */

/**
 * @brief Register access permission for external read/write requests.
 *
 * Used in check_register_access_range() to reject operations that violate
 * the declared permission for a register.
 */
typedef enum {
    ACCESS_RO = 0,  /**< Read-only:  external reads allowed, writes rejected  */
    ACCESS_RW = 1,  /**< Read-write: both reads and writes allowed            */
    ACCESS_WO = 2,  /**< Write-only: external writes allowed, reads rejected  */
} register_access_t;

/* ── Table entry ──────────────────────────────────────────────────────── */

/**
 * @brief Maps one Modbus holding-register on the device to an absolute pool slot.
 *
 * Columns
 * ───────
 *  device_address  – FC03 register address on the physical hardware.
 *  pool_address    – Absolute index into internal_pool[].  Hardcoded per unit;
 *                    no runtime base arithmetic.
 *  access          – Permission for external read/write requests.
 *  description     – Human-readable register name.
 *
 * Sorting
 * ───────
 *  Rows MUST be sorted by device_address ascending so that device_find_slot()
 *  (binary search) and segment reads both work correctly.
 *
 * Masking
 * ───────
 *  No bit_mask column.  The pool stores raw hardware values; masking is the
 *  responsibility of the consumer (alarm_engine, CMOS, etc.).  Use
 *  pool_read_masked_by_device_addr() when a masked view is needed.
 */
typedef struct {
    uint16_t          device_address;  /**< FC03 register address on the device        */
    uint16_t          pool_address;    /**< Absolute position in internal_pool[]        */
    register_access_t access;          /**< Read/write permission for external requests */
    const char       *description;
} device_register_mapping_t;

/* ── Device unit profile ──────────────────────────────────────────────── */

/**
 * @brief Describes one physical device unit (one UPS, one sensor, etc.).
 *
 * Each physical unit has its own table with unique absolute pool_address
 * values.  Units of the same hardware model have the same device_address
 * layout but different pool_address values.
 */
typedef struct {
    const char                      *name;
    const device_register_mapping_t *table;
    size_t                           table_count;
    uint16_t                         read_chunk;  /**< Max registers per FC03 request */
} device_map_profile_t;

/* ── Shared internal pool ─────────────────────────────────────────────── */

/*
 * Process-shared register pool (mmap MAP_SHARED, PTHREAD_PROCESS_SHARED lock).
 *
 * internal_pool is a pointer; use INTERNAL_POOL_SIZE for the slot count,
 * not sizeof(internal_pool).
 */
extern uint16_t         *internal_pool;
extern pthread_rwlock_t *internal_pool_lock;

/* ── Lifecycle ────────────────────────────────────────────────────────── */

/**
 * @brief Allocate the register pool and initialise the rwlock.
 *
 * Call once during startup, before any process that reads the pool is
 * created.
 */
void device_register_map_init(void);

/**
 * @brief Write a unit's connection_state into the shared region.
 *
 * Call from the parent process when a unit connects or disconnects.
 */
void shared_connection_state_set(const module_config_t *cfg,
                                   connection_state_t     state);

/**
 * @brief Read a unit's connection_state from the shared region.
 * @return CONNECTION_DISCONNECTED if cfg has no registered slot.
 */
connection_state_t shared_connection_state_get(const module_config_t *cfg);

/* ── Startup pool-address collision check ─────────────────────────────── */

/**
 * @brief Register one profile with the pool-address collision checker.
 *
 * Compares every pool_address in profile->table against every profile
 * registered earlier in the process lifetime.  Call this once per unit,
 * right after its profile is resolved (see start_inverter_modules() /
 * start_ups_modules() for the pattern).
 *
 * On collision, both profiles' name, description, and pool_address are
 * logged.  Registration is rejected when an overlap is found.
 *
 * @param profile  Profile about to be put into service.
 * @return 0 if no collision was found (profile is now tracked), -1 if a
 *         collision was found or the tracking table is full.
 */
int device_register_map_register_profile(const device_map_profile_t *profile);

/* ── Core mapping API ─────────────────────────────────────────────────── */

/**
 * @brief Binary-search a profile's table for a given device_address.
 * @return Pointer to the matching entry, or NULL if not found.
 */
const device_register_mapping_t *device_find_slot(
    const device_map_profile_t *profile,
    uint16_t                    device_address);

/**
 * @brief Write a Modbus read buffer into the pool for one device unit.
 *
 * For each register in read_buffer[0..read_count-1], looks up
 * (start_device_address + i) in the profile table and stores the raw value
 * at internal_pool[pool_address] (absolute).  No masking is applied.
 *
 * @param profile              Unit profile with the mapping table.
 * @param read_buffer          Values returned by FC03.
 * @param start_device_address First device_address in read_buffer.
 * @param read_count           Number of consecutive registers.
 */
void device_map_read_to_pool(const device_map_profile_t *profile,
                             const uint16_t             *read_buffer,
                             uint16_t                    start_device_address,
                             int                         read_count);

/* ── Direct pool access (by absolute pool_address) ────────────────────── */

/**
 * @brief Thread-safe read of one slot by its absolute pool_address.
 * @param pool_address  Absolute index into internal_pool[].
 * @param out_value     Destination for the cached value.
 * @return false if pool_address is out of range.
 */
bool pool_read_register(uint16_t  pool_address,
                        uint16_t *out_value);

/**
 * @brief Thread-safe write of one slot by its absolute pool_address.
 * @return false if pool_address is out of range.
 */
bool pool_write_register(uint16_t pool_address,
                         uint16_t value);

/* ── Profile-assisted pool access (by device_address) ─────────────────── */

/**
 * @brief Read a raw register value by its device_address (uses binary search).
 *
 * Looks up device_address in the profile table to find the absolute
 * pool_address, then performs a thread-safe pool read.  The returned value
 * is the raw hardware value with no masking applied.
 *
 * @param profile         Unit profile containing the mapping table.
 * @param device_address  FC03 register address on the physical device.
 * @param out_value       Destination for the cached pool value.
 * @return false if device_address is not in the table or index is out of range.
 */
bool pool_read_by_device_addr(const device_map_profile_t *profile,
                              uint16_t                    device_address,
                              uint16_t                   *out_value);

/**
 * @brief Read a register value with a caller-supplied bit mask applied.
 *
 * Equivalent to pool_read_by_device_addr() followed by (raw & mask).
 * Use this when the consumer needs only specific bits from a register
 * (e.g. reading a status field within a combined status word).
 *
 * @param profile         Unit profile containing the mapping table.
 * @param device_address  FC03 register address on the physical device.
 * @param mask            Bit mask to apply: *out_value = raw & mask.
 * @param out_value       Destination for the masked value.
 * @return false if device_address is not in the table or index is out of range.
 */
bool pool_read_masked_by_device_addr(const device_map_profile_t *profile,
                                     uint16_t                    device_address,
                                     uint16_t                    mask,
                                     uint16_t                   *out_value);

/**
 * @brief Write a register value by its device_address (uses binary search).
 * @return false if device_address is not found or index is out of range.
 */
bool pool_write_by_device_addr(const device_map_profile_t *profile,
                               uint16_t                    device_address,
                               uint16_t                    value);

/* ── Access permission check ──────────────────────────────────────────── */

/**
 * @brief Validate access permission for a pool address range.
 *
 * Iterates the profile table to find all entries whose pool_address falls
 * within [pool_start, pool_start + count - 1] and verifies each has the
 * required access permission.
 *
 * Use ACCESS_RO as required_access for incoming read requests,
 * and ACCESS_WO for incoming write requests.
 *
 * @param profile          Unit profile containing the mapping table.
 * @param pool_start       First pool address of the requested range.
 * @param count            Number of consecutive pool registers.
 * @param required_access  ACCESS_RO (read) or ACCESS_WO (write).
 * @return 0 on success, or a Modbus exception code on failure:
 *         MODBUS_EX_ILLEGAL_DATA_ADDRESS – pool_start/count invalid.
 *         MODBUS_EX_ILLEGAL_FUNCTION     – register does not permit operation.
 */
uint8_t check_register_access_range(const device_map_profile_t *profile,
                                    uint16_t                    pool_start,
                                    uint16_t                    count,
                                    register_access_t           required_access);

#endif /* DEVICE_REGISTER_MAP_H */
