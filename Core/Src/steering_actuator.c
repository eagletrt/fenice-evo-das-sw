#include "steering_actuator.h"

#if AS_STEERING_ACTUATOR_ENABLED == 1

struct PIDController {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prevError;
    float error;
    float sampleTime;
    float setPoint;
} pid;

void steering_actuator_pid_init(float kp, float ki, float kd, float sampleTime) {
    // PIDInit(&steering_actuator_pid, kp, ki, kd, 1.000 * ENC_STEER_PERIOD_MS);
    pid.kp         = kp;
    pid.ki         = ki;
    pid.kd         = kd;
    pid.integrator = 0.0;
    pid.prevError  = 0.0;
    pid.error      = 0.0;
    pid.setPoint   = 0.0;
    pid.sampleTime = sampleTime;
}

void steering_actuator_update_set_point(float setPoint) {
    pid.setPoint = setPoint;
}

void steering_actuator_update_pid(float status) {
    pid.prevError = pid.error;
    pid.error     = pid.setPoint - status;
    pid.integrator += pid.error * pid.sampleTime;
    if (pid.integrator > 0.5) {
        pid.integrator = 0.5;
    } else if (pid.setPoint < -0.5) {
        pid.integrator = -0.5;
    }
}

bool steering_actuator_set_speed(float speed) {
    if (speed < 0.0) {
        if (speed >= -1.0) {
            HAL_GPIO_WritePin(STEERING_REVERSE_GPIO_Port, STEERING_REVERSE_Pin, 1);
            TIM4->CCR2 = (uint32_t)(65535 * (-speed));
            return true;
        }
    } else {
        if (speed <= 1.0) {
            HAL_GPIO_WritePin(STEERING_REVERSE_GPIO_Port, STEERING_REVERSE_Pin, 0);
            TIM4->CCR2 = (uint32_t)(65535 * speed);
            return true;
        }
    }
    return false;
}

float steering_actuator_computePID() {
    float derivator = (pid.error - pid.prevError) / pid.sampleTime;
    float value     = pid.kp * pid.error + pid.ki * pid.integrator + pid.kd * derivator;
    if (value > 0.5) {
        return 0.5;
    }
    if (value < -0.5) {
        return -0.5;
    }
    return value;
}

#endif
