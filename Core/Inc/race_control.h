#ifndef RACE_CTRL_H
#define RACE_CTRL_H


/**
 * @brief     Perform everything that is needed to read the driver's input and
 *            actuate the motors accordingly.
 */
void DAS_do_drive_routine();

float DAS_get_pwr_map();
float DAS_get_sc_map();
float DAS_get_tv_map();

#endif