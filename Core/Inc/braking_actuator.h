#ifndef BRAKE_ACTUATOR_H
#define BRAKE_ACTUATOR_H

#include "ecu_config.h"
#include "encoders.h"

#define PID_ERRORS_VECTOR
#define N_PID_PREV_ERRORS 5
#include "pid.h"

#define EXTEND GPIO_PIN_SET // the actutor "extends" (cw rotation w.r.t. back of actuator)
#define RETRACT GPIO_PIN_RESET // the actuator "retracts" (ccw rotation w.r.t. back of actuator)

#define MOTOR_GO GPIO_PIN_RESET // the driver is enabled, a step command will be accepted
#define MOTOR_STOP GPIO_PIN_SET // the driver is disabled, any step command will be discarded

#define BRAKE_ACTUATOR_SPEED_LIMIT 30.0 //mm/s ~ 3kHz
#define BRAKE_ACTUATOR_RANGE_LIMIT 20.0 //mm - range movement starting from 0
#define BRAKE_PRESSURE_RANGE_LIMIT 100.0

#define BRAKING_ACTUATOR_FREQ_HZ   200
#define BRAKING_ACTUATOR_PERIOD_MS 1000 / BRAKING_ACTUATOR_FREQ_HZ
#define MM_STEP 0.01 // mm/step

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
