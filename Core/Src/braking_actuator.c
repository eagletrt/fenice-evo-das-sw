#include "braking_actuator.h"

#if AS_BRAKE_ACTUATOR_ENABLED == 1

#include "can_messages.h"
#include "can_user_functions.h"
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
  //TODO: define new encoder method
  float brake_angle = ENC_C_get_angle_deg();
  
  if (fabs(brake_angle) > BRAKE_ACTUATOR_ANGLE_LIMIT) speed = 0.0;
  
  if (fabs(speed) > BRAKE_ACTUATOR_SPEED_LIMIT){
    if (speed > 0.0) speed = BRAKE_ACTUATOR_SPEED_LIMIT;
    else speed = -BRAKE_ACTUATOR_SPEED_LIMIT;
  }
  
  //TODO: change GPIO Port for braking
  HAL_GPIO_WritePin(STEERING_REVERSE_GPIO_Port, STEERING_REVERSE_Pin, speed < 0.0 ? 0 : 1);
  TIM4->CCR2 = (uint32_t)(65535 * (fabs(speed) / BRAKE_ACTUATOR_SPEED_LIMIT));
}

void brake_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windup) {
  pid_init(&pid_controller, kp, kd, ki, sample_time, anti_windup, pid_prev_errors, N_PID_PREV_ERRORS);
}

void brake_actuator_update_pid() {
  if (brake_actuator_enabled) {
    //TODO: define new encoder method
    pid_update(&pid_controller, ENC_C_get_angle_deg());
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
