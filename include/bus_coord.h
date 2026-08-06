/**
 * @file bus_coord.h
 * @brief Exclusive Modbus wire ownership keyed by serial_path.
 *
 * Modules that share the same RS-485 path (e.g. Inverter + UPS on
 * /dev/ttyUSB0) must wrap every on-wire transaction with
 * bus_coord_acquire() / bus_coord_release().  Distinct paths do not
 * contend.  NULL or empty path is a no-op (TCP / no shared bus).
 *
 * Same-thread nesting is supported so a caller may hold the bus across
 * a multi-frame sequence (e.g. init probe + register programming) while
 * helpers also acquire around each frame.
 */

#ifndef BUS_COORD_H
#define BUS_COORD_H

/**
 * @brief Reset all bus slots. Call once at process startup.
 */
void bus_coord_init(void);

/**
 * @brief Acquire exclusive ownership of serial_path.
 *
 * Blocks until no other thread holds the path.  Nested acquires by the
 * same thread increment a depth counter.  No-op if path is NULL/empty.
 *
 * @param serial_path  Device node shared on the RS-485 bus.
 */
void bus_coord_acquire(const char *serial_path);

/**
 * @brief Release one nesting level of ownership for serial_path.
 *
 * When depth reaches zero, other waiters are woken.  No-op if path is
 * NULL/empty.  Must be paired with a prior acquire by the same thread.
 *
 * @param serial_path  Same path passed to bus_coord_acquire().
 */
void bus_coord_release(const char *serial_path);

#endif /* BUS_COORD_H */
