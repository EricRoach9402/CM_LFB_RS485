/**
 * @file bus_coord.h
 * @brief RS-485 bus mutex by serial path.
 */

#ifndef BUS_COORD_H
#define BUS_COORD_H

/**
 * @brief Reset all bus slots; call once at startup.
 */
void bus_coord_init(void);

/**
 * @brief Acquire exclusive ownership of serial_path.
 * @param serial_path Shared RS-485 device node; NULL/empty is a no-op.
 */
void bus_coord_acquire(const char *serial_path);

/**
 * @brief Release one nesting level for serial_path.
 * @param serial_path Same path passed to bus_coord_acquire().
 */
void bus_coord_release(const char *serial_path);

#endif /* BUS_COORD_H */
