#include "steering_actuator.h"


#if AS_STEER_ACTUATOR_ENABLED == 1

#include "can_messages.h"
#include "can_user_functions.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

float pid_prev_errors[N_PID_PREV_ERRORS];
PidController_t pid_controller;

bool steer_actuator_enabled = false;

void steer_actuator_update_set_point(float set_point) {
    pid_controller.set_point = set_point;
}

bool steer_actuator_is_enabled() {
    return steer_actuator_enabled;
}

void steer_actuator_enable() {
    pid_reset(&pid_controller);
    steer_actuator_enabled = true;
}

void steer_actuator_disable() {
    steer_actuator_enabled = false;
    steer_actuator_set_speed(0.0);
}

void steer_actuator_set_speed(float speed)
{
    float steering_angle = ENC_C_get_angle_deg();

    if (fabs(steering_angle) > STEER_ACTUATOR_ANGLE_LIMIT) speed = 0.0;

    if (fabs(speed) > STEER_ACTUATOR_SPEED_LIMIT){
        if (speed > 0.0) speed = STEER_ACTUATOR_SPEED_LIMIT;
        else speed = -STEER_ACTUATOR_SPEED_LIMIT;
    }
    
    HAL_GPIO_WritePin(STEERING_REVERSE_GPIO_Port, STEERING_REVERSE_Pin, speed < 0.0 ? 1 : 0);
    TIM4->CCR2 = (uint32_t)(65535 * (fabs(speed) / STEER_ACTUATOR_ANGLE_LIMIT));
}

void steer_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windup) {
    pid_init(&pid_controller, kp, kd, ki, sample_time, anti_windup, pid_prev_errors, N_PID_PREV_ERRORS);
}

void steer_actuator_update_pid() {
    if (steer_actuator_enabled) {
        pid_update(&pid_controller, ENC_C_get_angle_deg());
    }
}

void steer_actuator_update_speed() {
    if (steer_actuator_enabled) {
        steer_actuator_set_speed(pid_compute(&pid_controller));
    }
}

void steer_actuator_update_can() {
    if (ecumsg_ecu_set_steer_actuator_status_steering_wheel_state.info.is_new){
    if (ecumsg_ecu_set_steer_actuator_status_steering_wheel_state.data.status != steer_actuator_enabled) {
      if (ecumsg_ecu_set_steer_actuator_status_steering_wheel_state.data.status) {
        steer_actuator_enable();
      } else {
        steer_actuator_disable();
      }
    }
    ecumsg_ecu_set_steer_actuator_status_steering_wheel_state.info.is_new = false;
  }

  if (ecumsg_ecu_set_steer_actuator_status_tlm_state.info.is_new){
    if (ecumsg_ecu_set_steer_actuator_status_tlm_state.data.status != steer_actuator_enabled) {
      if (ecumsg_ecu_set_steer_actuator_status_tlm_state.data.status) {
        steer_actuator_enable();
      } else {
        steer_actuator_disable();
      }
    }
    ecumsg_ecu_set_steer_actuator_status_tlm_state.info.is_new = false;
  }

  if (ecumsg_ecu_set_steer_actuator_angle_state.info.is_new) {
    steer_actuator_update_set_point(ecumsg_ecu_set_steer_actuator_angle_state.data.angle);
    ecumsg_ecu_set_steer_actuator_angle_state.info.is_new = false;
  }
}

// float steer_actuator_compute_pid() {
//     float derivative = (pid.error - pid.prev_errors[(pid.prev_error_index + 1) % pid.n_prev_errors]) / (pid.sample_time * pid.n_prev_errors);
//     float integral = pid.ki * pid.integrator;
//     if (integral > pid.anti_windUp) {
//         integral = pid.anti_windUp;
//     } else if (integral < -pid.anti_windUp) {
//         integral = -pid.anti_windUp;
//     }
//     float value = pid.kp * pid.error + integral + pid.kd * derivative;


    secondary_debug_signal_1_converted_t msg;
    secondary_debug_signal_1_t raw;
//     msg.field_1 = value / 5.0;
//     msg.field_2 = pid.kp * pid.error / 5.0;
//     msg.field_3 = integral / 5.0;
//     msg.field_4 = pid.kd * derivative / 5.0;

    CAN_MessageTypeDef can_msg;
    can_msg.id = SECONDARY_DEBUG_SIGNAL_1_FRAME_ID;
    can_msg.size = SECONDARY_DEBUG_SIGNAL_1_BYTE_SIZE;

    secondary_debug_signal_1_conversion_to_raw_struct(&raw, &msg);
//     secondary_debug_signal_1_pack(can_msg.data, &raw, SECONDARY_DEBUG_SIGNAL_1_BYTE_SIZE);

//     CAN_send(&can_msg, &hcan2);

//     return value;
// }

#endif
