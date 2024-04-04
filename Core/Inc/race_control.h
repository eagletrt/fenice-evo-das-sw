#ifndef RACE_CTRL_H
#define RACE_CTRL_H

#include <math.h>
#include <stdbool.h>

#define ENABLE_REGEN (false)
#define ENABLE_CONTROLS (true)
#define CONTROL_FAIL_COUNT 10

bool equal_d(float a, float b);
bool equal_d_threshold(float a, float b, double threshold);

/**
 * @brief     Perform everything that is needed to read the driver's input and
 *            actuate the motors accordingly.
 */
void DAS_do_drive_routine();
void DAS_get_torques(double *torque_l, double *torque_r);

double _DAS_get_speed_ref();
double _DAS_get_torque_ref();

float DAS_get_pwr_map();
float DAS_get_sc_map();
float DAS_get_tv_map();

#endif