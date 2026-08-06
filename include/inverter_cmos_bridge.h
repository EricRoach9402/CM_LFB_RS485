/**
 * @file inverter_cmos_bridge.h
 * @brief CMOS subscriber / publisher bridge for Inverter.
 *
 * Parent process
 * ──────────────
 *  inverter_cmos_bridge_start() / stop() – CMOS subscriber thread for HMI
 *  write commands.  Called from inverter_module.c.
 *
 * Child process
 * ─────────────
 *  inverter_cmos_pub_run() – periodic status publisher.  Started from main.
 */

#ifndef INVERTER_CMOS_BRIDGE_H
#define INVERTER_CMOS_BRIDGE_H

/**
 * @brief Start the CMOS subscriber thread in the parent process.
 * @return 0 on success, -1 on failure.
 */
int inverter_cmos_bridge_start(void);

/**
 * @brief Stop the CMOS subscriber thread.
 */
void inverter_cmos_bridge_stop(void);

/**
 * @brief Publisher main loop for the Inverter child process.
 *
 * Does not return until SIGTERM or SIGINT.
 */
void inverter_cmos_pub_run(void);

#endif /* INVERTER_CMOS_BRIDGE_H */
