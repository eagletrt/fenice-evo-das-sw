#include "ecu_config.h"

#ifndef AS_STEERING_ACTUATOR_ENABLED == 1

#include "../Lib/micro-libs/pid/pid.h"
#include "encoders.h"
#include "main.h"

void steering_actuator_pid_init(float kp, float ki, float kd, float sampleTime);

void steering_actuator_update_pid(float status);

void steering_actuator_update_set_point(float setPoint);

float steering_actuator_computePID();

bool steering_actuator_set_speed(float speed);

#endif
