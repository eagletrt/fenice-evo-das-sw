#include "steering_actuator.h"
#include "stdbool.h"
#include "stdio.h"
#include "can_messages.h"
#include "can_user_functions.h"
#include "math.h"

#if AS_STEER_ACTUATOR_ENABLED == 1

PidController_t pid;

bool steer_actuator_enabled = false;

void steer_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windUp, size_t n_prev_errors) {
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.integrator = 0.0;
    // pid.prev_error = 0.0;
    pid.n_prev_errors = n_prev_errors;
    pid.prev_error_index = pid.n_prev_errors - 1;
    for (int i = 0; i < pid.n_prev_errors; ++i) {
        pid.prev_errors[i] = 0.0;
    }
    pid.error = 0.0;
    pid.set_point = 0.0;
    pid.sample_time = sample_time;
    pid.anti_windUp = anti_windUp;
}

void steer_actuator_update_set_point(float set_point) {
    pid.set_point = set_point;
}

void steer_actuator_pid_reset() {
    pid.integrator = 0.0;
    for (int i = 0; i < pid.n_prev_errors; ++i) {
        pid.prev_errors[i] = 0.0;
    }
}

void steer_actuator_enable() {
    steer_actuator_pid_reset();
    steer_actuator_enabled = true;
}

void steer_actuator_disable() {
    steer_actuator_enabled = false;
    steer_actuator_set_speed(0.0);
}

void steer_actuator_update_pid(float status) {
    pid.prev_error_index = (pid.prev_error_index + 1) % pid.n_prev_errors;
    pid.prev_errors[pid.prev_error_index] = pid.error;
    pid.error = pid.set_point - status;
    pid.integrator += pid.error * pid.sample_time;
}

void steer_actuator_set_speed(float speed)
{
    float speed_limit = 5.0;
    float angle_limit = 90.0;
    float steering_angle = ENC_C_get_angle_deg();

    if (fabs(steering_angle) > angle_limit) speed = 0.0;

    if (fabs(speed) > speed_limit){
        if (speed > 0.0) speed = speed_limit;
        else speed = -speed_limit;
    }
    
    HAL_GPIO_WritePin(STEERING_REVERSE_GPIO_Port, STEERING_REVERSE_Pin, speed < 0.0 ? 1 : 0);
    TIM4->CCR2 = (uint32_t)(65535 * (fabs(speed) / speed_limit));
}

float steer_actuator_computePID() {
    float derivative = (pid.error - pid.prev_errors[(pid.prev_error_index + 1) % pid.n_prev_errors]) / (pid.sample_time * pid.n_prev_errors);
    float integral = pid.ki * pid.integrator;
    if (integral > pid.anti_windUp) {
        integral = pid.anti_windUp;
    } else if (integral < -pid.anti_windUp) {
        integral = -pid.anti_windUp;
    }
    float value = pid.kp * pid.error + integral + pid.kd * derivative;


    secondary_debug_signal_1_converted_t msg;
    secondary_debug_signal_1_t raw;
    msg.field_1 = value / 5.0;
    msg.field_2 = pid.kp * pid.error / 5.0;
    msg.field_3 = integral / 5.0;
    msg.field_4 = pid.kd * derivative / 5.0;

    CAN_MessageTypeDef can_msg;
    can_msg.id = SECONDARY_DEBUG_SIGNAL_1_FRAME_ID;
    can_msg.size = SECONDARY_DEBUG_SIGNAL_1_BYTE_SIZE;

    secondary_debug_signal_1_conversion_to_raw_struct(&raw, &msg);
    secondary_debug_signal_1_pack(can_msg.data, &raw, SECONDARY_DEBUG_SIGNAL_1_BYTE_SIZE);

    CAN_send(&can_msg, &hcan2);

    return value;
}

#endif
