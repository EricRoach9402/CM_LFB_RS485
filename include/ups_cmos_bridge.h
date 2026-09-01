/**
 * @file ups_cmos_bridge.h
 * @brief UPS CMOS bridge API.
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
 */
void ups_cmos_pub_run(void);

#endif /* UPS_CMOS_BRIDGE_H */
