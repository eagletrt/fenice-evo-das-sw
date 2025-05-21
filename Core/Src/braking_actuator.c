#include "braking_actuator.h"

#if AS_BRAKE_ACTUATOR_ENABLED == 1

#include "can_messages.h"
#include "can_user_functions.h"
#include "pedals.h"
#include "ecu_config.h"
#include "encoders.h"
#include <math.h>
#include <stdbool.h>

float pid_prev_errors[N_PID_PREV_ERRORS];
PidController_t pid_controller;

bool brake_actuator_enabled = false;

void brake_actuator_update_set_point(float set_point) {
  pid_controller.set_point = set_point;
}

bool brake_actuator_is_enabled() {
  return brake_actuator_enabled;
}

void brake_actuator_enable() {
  pid_reset(&pid_controller);
  brake_actuator_enabled = true;
  ecumsg_as_commands_status_state.data.brakestatus = primary_as_commands_status_brakestatus_on;
}

void brake_actuator_disable() {
  brake_actuator_enabled = false;
  ecumsg_as_commands_status_state.data.brakestatus = primary_as_commands_status_brakestatus_off;
  brake_actuator_set_speed(0.0);
}

void brake_actuator_set_speed(float speed)
{
  float brake_bar = PED_get_brake_bar();

  #if ENC_BRAKE_ACTUATOR_ENABLED
    //TODO: use correct timer
    // check encoder if actuator is in range
    float distance = get_distance(TIM4);
    if (fabs(brake_bar) > BRAKE_PRESSURE_RANGE_LIMIT || distance < 0 || distance > BRAKE_ACTUATOR_RANGE_LIMIT) speed = 0.0;
  #else
    if (fabs(brake_bar) > BRAKE_PRESSURE_RANGE_LIMIT) speed = 0.0;
  #endif

  if (fabs(speed) > BRAKE_ACTUATOR_SPEED_LIMIT){
    if (speed > 0.0) speed = BRAKE_ACTUATOR_SPEED_LIMIT;
    else speed = -BRAKE_ACTUATOR_SPEED_LIMIT;
  }
  
  //TODO: change GPIO pin for braking actuator

  // HAL_GPIO_WritePin(ActuatorDir_GPIO_Port, ActuatorDir_Pin, speed < 0.0 ? RETRACT : EXTEND);

  // // set new pwm frequency leaving constant the duty cycle ~ 50% of the new arr
  // float steps_per_sec = fabs(speed) / MM_STEP;

  // if (steps_per_sec > 0.0f) {
  //     //84e6 is the timer clock, need to be changed based on the ioc file
  //     uint32_t timer_clk = 84e6 / (TIMX->PSC + 1);
  //     uint16_t arr = (uint16_t)((timer_clk / steps_per_sec) - 1);
  //     // limit ARR to working range found by testing
  //     if (arr > 4999) arr = 4999; // ~ 200 Hz
  //     if (arr < 1249) arr = 1249; // ~ 600 Hz

  //     TIMX->ARR = arr;
  //     TIMX->CCRY = arr / 2; // 50% duty cycle
  // } else {
  //     TIMX->CCRY = 0; // stop pulses
  // }
}

void brake_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windup) {
  pid_init(&pid_controller, kp, kd, ki, sample_time, anti_windup, pid_prev_errors, N_PID_PREV_ERRORS);
}

void brake_actuator_update_pid() {
  if (brake_actuator_enabled) {
    pid_update(&pid_controller, PED_get_brake_bar() / 100.0f);
  }
}

void brake_actuator_update_speed() {
  if (brake_actuator_enabled) {
    float speed = pid_compute(&pid_controller);
    brake_actuator_set_speed(speed);
  }
}

void brake_actuator_update_can() {
  if (ecumsg_as_commands_set_status_state.info.is_new){
    if (ecumsg_as_commands_set_status_state.data.brakestatus != brake_actuator_enabled) {
      if (ecumsg_as_commands_set_status_state.data.brakestatus) {
        brake_actuator_enable();
      } else {
        brake_actuator_disable();
      }
    }
    ecumsg_as_commands_set_status_state.info.is_new = false;
  }

  if (ecumsg_as_commands_set_value_state.info.is_new) {
    if (brake_actuator_enabled) {
      brake_actuator_update_set_point(ecumsg_as_commands_set_value_state.data.brake);
    }
    ecumsg_as_commands_set_value_state.info.is_new = false;
  }
}

#endif
