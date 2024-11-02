#ifndef STEER_ACTUATOR_H
#define STEER_ACTUATOR_H

#include "ecu_config.h"
#include "encoders.h"

#define PID_ERRORS_VECTOR
#define N_PID_PREV_ERRORS 5
#include "pid.h"

#define STEER_ACTUATOR_SPEED_LIMIT 5.0
#define STEER_ACTUATOR_ANGLE_LIMIT 100.0

void steer_actuator_update_set_point(float setPoint);

bool steer_actuator_is_enabled();

void steer_actuator_enable();

void steer_actuator_disable();

void steer_actuator_set_speed(float speed);

void steer_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windup);  // TODO? spostare questi valori in ecu config e liberare il main

void steer_actuator_update_pid();

void steer_actuator_update_speed();

void steer_actuator_update_can();

#endif
