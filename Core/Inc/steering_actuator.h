#ifndef STEER_ACTUATOR_H
#define STEER_ACTUATOR_H

#include "ecu_config.h"

#include "../Lib/micro-libs/pid/pid.h"
#include "encoders.h"
#include "main.h"

#define N_PREV_ERRORS 5

extern bool steer_actuator_enabled;

typedef struct pidController_t{
    float kp;
    float ki;
    float kd;
    float integrator;
    // float prev_error;
    float error;
    float sample_time;
    float set_point;
    float anti_windUp;
    size_t n_prev_errors;
    float prev_errors[N_PREV_ERRORS];
    int prev_error_index;
} PidController_t;

void steer_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windUp, size_t n_prev_errors);

void steer_actuator_update_set_point(float setPoint);

void steer_actuator_pid_reset();

void steer_actuator_enable();

void steer_actuator_disable();

void steer_actuator_update_pid(float status);

void steer_actuator_set_speed(float speed);

float steer_actuator_computePID();

#endif
