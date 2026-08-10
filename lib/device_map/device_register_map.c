/**
 * @file device_register_map.c
 * @brief Generic Modbus device register → internal pool mapping engine.
 *
 * Pool addresses are absolute: each per-unit mapping table hardcodes its
 * pool_address values.  No runtime pool-base arithmetic is performed here.
 *
 * The pool stores raw register values.  No masking is applied at this layer.
 * Consumers needing a masked view use pool_read_masked_by_device_addr().
 */

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>

#include "device_register_map.h"
#include "log.h"

/* ── Internal pool (process-shared) ───────────────────────────────────── */

typedef struct {
    pthread_rwlock_t   lock;
    uint16_t           pool[INTERNAL_POOL_SIZE];
    connection_state_t inverter_conn[MAX_INVERTER_COUNT];
    connection_state_t ups_conn[MAX_UPS_COUNT];
} shared_pool_region_t;

static shared_pool_region_t *g_shared_pool_region = NULL;

uint16_t         *internal_pool      = NULL;
pthread_rwlock_t *internal_pool_lock = NULL;

/* ── Startup pool-address collision check state ───────────────────────── */

static const device_map_profile_t *registered_profiles[DEVICE_REGISTER_MAP_MAX_PROFILES];
static size_t                      registered_profile_count = 0;

/* ── Lifecycle ────────────────────────────────────────────────────────── */

void device_register_map_init(void)
{
    if (g_shared_pool_region == NULL) {
        g_shared_pool_region = mmap(NULL, sizeof(*g_shared_pool_region),
                                    PROT_READ | PROT_WRITE,
                                    MAP_SHARED | MAP_ANONYMOUS, -1, 0);
        if (g_shared_pool_region == MAP_FAILED) {
            LOG_ERROR("[device_map] mmap shared pool failed: %s",
                      strerror(errno));
            exit(EXIT_FAILURE);
        }

        memset(g_shared_pool_region, 0, sizeof(*g_shared_pool_region));

        pthread_rwlockattr_t attr;
        pthread_rwlockattr_init(&attr);
        pthread_rwlockattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        if (pthread_rwlock_init(&g_shared_pool_region->lock, &attr) != 0) {
            LOG_ERROR("[device_map] pthread_rwlock_init(PROCESS_SHARED) failed");
            exit(EXIT_FAILURE);
        }
        pthread_rwlockattr_destroy(&attr);

        internal_pool      = g_shared_pool_region->pool;
        internal_pool_lock = &g_shared_pool_region->lock;
    } else {
        memset(internal_pool, 0, INTERNAL_POOL_SIZE * sizeof(uint16_t));
    }

    registered_profile_count = 0;
}

/* ── Connection state (process-shared) ────────────────────────────────── */

static connection_state_t *shared_conn_slot_for(const module_config_t *cfg)
{
    if (!cfg || !g_shared_pool_region) {
        return NULL;
    }

    for (int i = 0; i < global_config.inverter_count; i++) {
        if (&global_config.inverter[i] == cfg) {
            return &g_shared_pool_region->inverter_conn[i];
        }
    }

    for (int i = 0; i < global_config.ups_count; i++) {
        if (&global_config.ups[i] == cfg) {
            return &g_shared_pool_region->ups_conn[i];
        }
    }

    return NULL;
}

void shared_connection_state_set(const module_config_t *cfg,
                                   connection_state_t     state)
{
    connection_state_t *slot = shared_conn_slot_for(cfg);
    if (slot) {
        *slot = state;
    }
}

connection_state_t shared_connection_state_get(const module_config_t *cfg)
{
    connection_state_t *slot = shared_conn_slot_for(cfg);
    if (!slot) {
        return CONNECTION_DISCONNECTED;
    }
    return *slot;
}

/* ── Startup pool-address collision check ─────────────────────────────── */

int device_register_map_register_profile(const device_map_profile_t *profile)
{
    if (!profile || !profile->table) {
        return -1;
    }

    bool collision_found = false;

    for (size_t i = 0; i < profile->table_count; i++) {
        uint16_t pool_addr = profile->table[i].pool_address;

        for (size_t p = 0; p < registered_profile_count; p++) {
            const device_map_profile_t *other = registered_profiles[p];

            for (size_t j = 0; j < other->table_count; j++) {
                if (other->table[j].pool_address != pool_addr) {
                    continue;
                }

                LOG_ERROR("[device_map] pool_address 0x%04X collides: "
                          "'%s' (%s) vs already-registered '%s' (%s)",
                          pool_addr,
                          profile->name, profile->table[i].description,
                          other->name, other->table[j].description);
                collision_found = true;
            }
        }
    }

    if (collision_found) {
        return -1;
    }

    if (registered_profile_count >= DEVICE_REGISTER_MAP_MAX_PROFILES) {
        LOG_ERROR("[device_map] cannot register profile '%s': "
                  "DEVICE_REGISTER_MAP_MAX_PROFILES (%d) reached.",
                  profile->name, DEVICE_REGISTER_MAP_MAX_PROFILES);
        return -1;
    }

    registered_profiles[registered_profile_count++] = profile;
    return 0;
}

/* ── Core mapping API ─────────────────────────────────────────────────── */

const device_register_mapping_t *device_find_slot(
    const device_map_profile_t *profile,
    uint16_t                    device_address)
{
    if (!profile || !profile->table || profile->table_count == 0) {
        return NULL;
    }

    size_t low  = 0;
    size_t high = profile->table_count;

    while (low < high) {
        size_t   mid  = low + (high - low) / 2;
        uint16_t addr = profile->table[mid].device_address;

        if (addr == device_address) {
            return &profile->table[mid];
        }
        if (addr < device_address) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }

    return NULL;
}

void device_map_read_to_pool(const device_map_profile_t *profile,
                             const uint16_t             *read_buffer,
                             uint16_t                    start_device_address,
                             int                         read_count)
{
    if (!profile || !read_buffer || read_count <= 0 ||
        !profile->table || profile->table_count == 0) {
        return;
    }

    for (int i = 0; i < read_count; i++) {
        uint16_t dev_addr = (uint16_t)(start_device_address + (uint16_t)i);

        const device_register_mapping_t *m =
            device_find_slot(profile, dev_addr);

        if (!m) {
            LOG_VERBOSE("[device_map] %s: no mapping for dev 0x%04X",
                        profile->name, dev_addr);
            continue;
        }

        if (m->pool_address >= INTERNAL_POOL_SIZE) {
            LOG_WARNING("[device_map] %s: pool_address 0x%04X out of bounds "
                        "(dev 0x%04X)",
                        profile->name, m->pool_address, dev_addr);
            continue;
        }

        /* Store the raw value; no masking at the ingest stage. */
        uint16_t raw = read_buffer[i];

        pthread_rwlock_wrlock(internal_pool_lock);
        internal_pool[m->pool_address] = raw;
        pthread_rwlock_unlock(internal_pool_lock);

        LOG_VERBOSE("[device_map] %s: dev 0x%04X -> pool[0x%04X] = 0x%04X",
                    profile->name, dev_addr, m->pool_address, raw);
    }
}

/* ── Direct pool access ───────────────────────────────────────────────── */

bool pool_read_register(uint16_t pool_address, uint16_t *out_value)
{
    if (!out_value || pool_address >= INTERNAL_POOL_SIZE) {
        return false;
    }

    pthread_rwlock_rdlock(internal_pool_lock);
    *out_value = internal_pool[pool_address];
    pthread_rwlock_unlock(internal_pool_lock);

    return true;
}

bool pool_write_register(uint16_t pool_address, uint16_t value)
{
    if (pool_address >= INTERNAL_POOL_SIZE) {
        return false;
    }

    pthread_rwlock_wrlock(internal_pool_lock);
    internal_pool[pool_address] = value;
    pthread_rwlock_unlock(internal_pool_lock);

    return true;
}

/* ── Profile-assisted pool access ─────────────────────────────────────── */

bool pool_read_by_device_addr(const device_map_profile_t *profile,
                              uint16_t                    device_address,
                              uint16_t                   *out_value)
{
    if (!profile || !out_value) {
        return false;
    }

    const device_register_mapping_t *m = device_find_slot(profile, device_address);
    if (!m) {
        return false;
    }

    return pool_read_register(m->pool_address, out_value);
}

bool pool_read_masked_by_device_addr(const device_map_profile_t *profile,
                                     uint16_t                    device_address,
                                     uint16_t                    mask,
                                     uint16_t                   *out_value)
{
    if (!profile || !out_value) {
        return false;
    }

    uint16_t raw = 0u;
    if (!pool_read_by_device_addr(profile, device_address, &raw)) {
        return false;
    }

    *out_value = (uint16_t)(raw & mask);
    return true;
}

bool pool_write_by_device_addr(const device_map_profile_t *profile,
                               uint16_t                    device_address,
                               uint16_t                    value)
{
    if (!profile) {
        return false;
    }

    const device_register_mapping_t *m = device_find_slot(profile, device_address);
    if (!m) {
        return false;
    }

    return pool_write_register(m->pool_address, value);
}

/* ── Access permission check ──────────────────────────────────────────── */

uint8_t check_register_access_range(const device_map_profile_t *profile,
                                    uint16_t                    pool_start,
                                    uint16_t                    count,
                                    register_access_t           required_access)
{
    if (!profile || !profile->table || count == 0) {
        return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
    }

    uint16_t pool_end = (uint16_t)(pool_start + count - 1u);

    for (uint16_t pa = pool_start; pa <= pool_end; pa++) {

        bool found = false;

        for (size_t i = 0; i < profile->table_count; i++) {
            if (profile->table[i].pool_address != pa) {
                continue;
            }

            found = true;
            register_access_t reg_access = profile->table[i].access;

            if (required_access == ACCESS_RO && reg_access == ACCESS_WO) {
                LOG_WARNING("[device_map] read rejected: pool 0x%04X is write-only (%s)",
                            pa, profile->table[i].description);
                return MODBUS_EX_ILLEGAL_FUNCTION;
            }

            if (required_access == ACCESS_WO && reg_access == ACCESS_RO) {
                LOG_WARNING("[device_map] write rejected: pool 0x%04X is read-only (%s)",
                            pa, profile->table[i].description);
                return MODBUS_EX_ILLEGAL_FUNCTION;
            }

            break;
        }

        if (!found) {
            LOG_WARNING("[device_map] access rejected: pool 0x%04X not mapped in profile '%s'",
                        pa, profile->name);
            return MODBUS_EX_ILLEGAL_DATA_ADDRESS;
        }
    }

    return 0;
}
