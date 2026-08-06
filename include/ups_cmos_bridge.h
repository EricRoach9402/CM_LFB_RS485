/**
 * @file ups_cmos_bridge.h
 * @brief CMOS subscriber / publisher bridge for UPS.
 *
 * Parent process
 * ──────────────
 *  ups_cmos_bridge_start() / stop() – CMOS subscriber thread for HMI write
 *  commands.  Called from ups_module.c.
 *
 * Child process
 * ─────────────
 *  ups_cmos_pub_run() – periodic status publisher.  Started from main.
 *
 * Periodic status push (topic "ups"):
 *  Driven by ups1_profile's mapping table (devices/ups/ups_map.c).
 */

#ifndef UPS_CMOS_BRIDGE_H
#define UPS_CMOS_BRIDGE_H

/**
 * @brief Start the CMOS subscriber thread in the parent process.
 * @return 0 on success, -1 on failure.
 */
int ups_cmos_bridge_start(void);

/**
 * @brief Stop the CMOS subscriber thread.
 */
void ups_cmos_bridge_stop(void);

/**
 * @brief Publisher main loop for the UPS child process.
 *
 * Does not return until SIGTERM or SIGINT.
 */
void ups_cmos_pub_run(void);

#endif /* UPS_CMOS_BRIDGE_H */
