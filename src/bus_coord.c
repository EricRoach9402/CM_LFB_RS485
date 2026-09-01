/**
 * @file bus_coord.c
 * @brief RS-485 bus mutex by serial path.
 */

#include <pthread.h>
#include <stdbool.h>
#include <string.h>

#include "bus_coord.h"
#include "log.h"

#define BUS_COORD_MAX_BUSES 8
#define BUS_COORD_PATH_MAX 256

typedef struct {
    char path[BUS_COORD_PATH_MAX];
    bool allocated;
    unsigned int depth;
    pthread_t owner;
    pthread_cond_t idle_cond;
} bus_slot_t;

static bool path_is_empty(const char *serial_path);
static bus_slot_t *find_slot_locked(const char *serial_path);
static bus_slot_t *alloc_slot_locked(const char *serial_path);

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static bus_slot_t g_buses[BUS_COORD_MAX_BUSES];

/**
 * @brief Reset all bus slots; call once at startup.
 */
void bus_coord_init(void)
{
    pthread_mutex_lock(&g_lock);

    for (int i = 0; i < BUS_COORD_MAX_BUSES; i++) {
        if (g_buses[i].allocated) {
            pthread_cond_destroy(&g_buses[i].idle_cond);
        }
        memset(&g_buses[i], 0, sizeof(g_buses[i]));
    }

    pthread_mutex_unlock(&g_lock);
}

/**
 * @brief Acquire exclusive ownership of serial_path.
 * @param serial_path Shared RS-485 device node.
 */
void bus_coord_acquire(const char *serial_path)
{
    if (path_is_empty(serial_path)) {
        return;
    }

    pthread_mutex_lock(&g_lock);

    bus_slot_t *slot = find_slot_locked(serial_path);
    if (!slot) {
        slot = alloc_slot_locked(serial_path);
        if (!slot) {
            pthread_mutex_unlock(&g_lock);
            LOG_ERROR("[bus_coord] no free slot for path '%s'; "
                      "proceeding without coordination.",
                      serial_path);
            return;
        }
    }

    while (slot->depth > 0 &&
           !pthread_equal(slot->owner, pthread_self())) {
        pthread_cond_wait(&slot->idle_cond, &g_lock);
    }

    if (slot->depth == 0) {
        slot->owner = pthread_self();
    }
    slot->depth++;

    pthread_mutex_unlock(&g_lock);
}

/**
 * @brief Release one nesting level for serial_path.
 * @param serial_path Same path passed to bus_coord_acquire().
 */
void bus_coord_release(const char *serial_path)
{
    if (path_is_empty(serial_path)) {
        return;
    }

    pthread_mutex_lock(&g_lock);

    bus_slot_t *slot = find_slot_locked(serial_path);
    if (!slot || slot->depth == 0) {
        pthread_mutex_unlock(&g_lock);
        LOG_WARNING("[bus_coord] release without matching acquire "
                    "(path='%s').",
                    serial_path);
        return;
    }

    if (!pthread_equal(slot->owner, pthread_self())) {
        pthread_mutex_unlock(&g_lock);
        LOG_WARNING("[bus_coord] release by non-owner thread "
                    "(path='%s').",
                    serial_path);
        return;
    }

    slot->depth--;
    if (slot->depth == 0) {
        pthread_cond_broadcast(&slot->idle_cond);
    }

    pthread_mutex_unlock(&g_lock);
}

/**
 * @brief Return true when path is NULL or empty.
 * @param serial_path Serial path to test.
 * @return true if path is empty.
 */
static bool path_is_empty(const char *serial_path)
{
    return (serial_path == NULL || serial_path[0] == '\0');
}

/**
 * @brief Find an allocated bus slot by path.
 * @param serial_path Serial path key.
 * @return Slot pointer or NULL.
 */
static bus_slot_t *find_slot_locked(const char *serial_path)
{
    for (int i = 0; i < BUS_COORD_MAX_BUSES; i++) {
        if (g_buses[i].allocated &&
            strcmp(g_buses[i].path, serial_path) == 0) {
            return &g_buses[i];
        }
    }
    return NULL;
}

/**
 * @brief Allocate a new bus slot for serial_path.
 * @param serial_path Serial path key.
 * @return Slot pointer or NULL if table is full.
 */
static bus_slot_t *alloc_slot_locked(const char *serial_path)
{
    for (int i = 0; i < BUS_COORD_MAX_BUSES; i++) {
        if (!g_buses[i].allocated) {
            bus_slot_t *slot = &g_buses[i];

            memset(slot->path, 0, sizeof(slot->path));
            strncpy(slot->path, serial_path, sizeof(slot->path) - 1u);
            slot->allocated = true;
            slot->depth = 0;
            pthread_cond_init(&slot->idle_cond, NULL);
            return slot;
        }
    }
    return NULL;
}
