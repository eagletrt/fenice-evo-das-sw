#ifndef BRAKE_ACTUATOR_H
#define BRAKE_ACTUATOR_H

#include "ecu_config.h"
#include "encoders.h"

#define PID_ERRORS_VECTOR
#define N_PID_PREV_ERRORS 5
#include "pid.h"

//TODO: need testing in order to define proper values
#define BRAKE_ACTUATOR_SPEED_LIMIT 5.0
#define BRAKE_ACTUATOR_RANGE_LIMIT 100.0

#define BRAKING_ACTUATOR_FREQ_HZ   200
#define BRAKING_ACTUATOR_PERIOD_MS 1000 / BRAKING_ACTUATOR_FREQ_HZ

void brake_actuator_update_set_point(float setPoint);

bool brake_actuator_is_enabled();

void brake_actuator_enable();

void brake_actuator_disable();

void brake_actuator_set_speed(float speed);

void brake_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windup);

void brake_actuator_update_pid();

void brake_actuator_update_speed();

void brake_actuator_update_can();

#endif
