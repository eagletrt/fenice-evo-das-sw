#include "encoders.h"

#include "can_messages.h"
#include "inverters.h"
#include "limits.h"
#include "logger.h"
#include "spi.h"
#include "tim.h"
#include "time_base.h"
#include "usart.h"

#include <float.h>
#include <math.h>

float _ENC_L_rollavg[ENC_ROLLAVG_SIZE] = {}, _ENC_R_rollavg[ENC_ROLLAVG_SIZE] = {};
float _ENC_C_inplace_average = 0.0f;
uint8_t _ENC_L_rollavg_idx = 0, _ENC_R_rollavg_idx = 0, _ENC_C_median_window_idx = 0;

uint32_t _ENC_L_last_ticks = 0; /*< Last read ticks (milliseconds) for the left encoder */
uint32_t _ENC_R_last_ticks = 0; /*< Last read ticks (milliseconds) for the right encoder */

uint32_t _ENC_L_last_cnt = 0; /*< Last read timer counter for the left encoder */
uint32_t _ENC_R_last_cnt = 0; /*< last read timer counter for the right encoder */

float _ENC_encoder_resolution_m = 0.000005f;    /*< Resolution of the magneting ring in meters */
float _ENC_speed_multiplier     = 5.238726792f; /*< Multiplier to convert encoder speed to wheel speed */

/*
    Note on how _ENC_speed_multiplier is calculated:

    speed_mult = wheel circumference / encoder circumference
               = (pi * wheel_diameter) / (pi * enc_diameter)
               = (3.1415926535 * 0.395) / (3.1415926535 * 0.0754)
               = 5.238726792
*/

/*
    120 numero di poli nella circonferenza
*/

float _ENC_ms_to_radsec(float);

/**
 * @brief     Calculate the ground speed in meters/second
 *
 * @param     tim_counter Timer ticks since the last read
 * @param     ellapsed_ms Milliseconds elapsed since the last read
 * @return    float Ground speed in meters/second
 */
//  (tim_counter / (120.0 * 400)) * 2*M_PI / (ellapsed_ms/1000.0)
float _ENC_calculate_wheel_speed(int64_t tim_counter, uint32_t elapsed_us) {
    return (tim_counter / (120.0 * 400)) * 2 * M_PI * 1000000.0 / elapsed_us;
}

/**
 * @brief     Calulate the difference between the current value of the timer
 * counter and its value from the last read
 * @param     htim The timer from which to read
 * @return    uint32_t Timer ticks since the last read
 */
int64_t _ENC_get_tim_cnt(TIM_HandleTypeDef *htim) {
    uint32_t *last_cnt_ptr = (htim == &ENC_L_TIM) ? &_ENC_L_last_cnt : &_ENC_R_last_cnt;
    int64_t last_cnt       = *last_cnt_ptr;
    int64_t cnt            = __HAL_TIM_GET_COUNTER(htim);
    int64_t diff           = last_cnt - cnt;
    *last_cnt_ptr          = cnt;
    return diff;
}

/**
 * @brief     Calculate the ground speed from the right wheel encoder
 */
void ENC_L_push_speed_rads() {
    uint32_t ellapsed_us = get_time() - _ENC_L_last_ticks;
    int64_t tim_counter  = _ENC_get_tim_cnt(&ENC_L_TIM);
    float speed          = -_ENC_calculate_wheel_speed(tim_counter, ellapsed_us);  // impulses * 2*PI / encoder_poles / seconds
    _ENC_L_last_ticks    = get_time();
    _ENC_L_rollavg[_ENC_L_rollavg_idx] = speed;
    _ENC_L_rollavg_idx                 = (_ENC_L_rollavg_idx + 1) % ENC_ROLLAVG_SIZE;
}

float ENC_get_radsec(float *array) {
    float acc = 0;
    float max = array[0], min = array[0];

    for (uint8_t i = 0; i < ENC_ROLLAVG_SIZE; i++) {
        acc += array[i];
        if (array[i] > max)
            max = array[i];
        if (array[i] < min)
            min = array[i];
    }

    acc -= max + min;

    return acc / (ENC_ROLLAVG_SIZE - 2);
}
float ENC_L_get_radsec() {
    return ENC_get_radsec(_ENC_L_rollavg);
}

float ENC_R_get_radsec() {
    return ENC_get_radsec(_ENC_R_rollavg);
}

/**
 * @brief     Calculate the ground speed from the right wheel encoder
 */
void ENC_R_push_speed_rads() {
    uint32_t elapsed_us                = get_time() - _ENC_R_last_ticks;
    int64_t tim_counter                = _ENC_get_tim_cnt(&ENC_R_TIM);
    float speed                        = -_ENC_calculate_wheel_speed(tim_counter, elapsed_us);  // impulses * 2*PI / encoder_poles / seconds
    _ENC_R_last_ticks                  = get_time();
    _ENC_R_rollavg[_ENC_R_rollavg_idx] = speed;
    _ENC_R_rollavg_idx                 = (_ENC_R_rollavg_idx + 1) % ENC_ROLLAVG_SIZE;
}

float raw_to_angle(uint8_t byte0, uint8_t byte1) {
    // byte1 contains the 7 most significant bits of the angle, while byte0 contains the 5 least significant bits
    uint16_t parsed = ((uint16_t)(byte1 & 0b01111111) << 5) | (byte0 & 0b11111000) >> 3;
    return parsed / 4095.f * 360.f;
}

#define STEERING_WHEEL_ENCODER_CENTER_DEG 98.6f
float calibrate_angle(float angle) {
    angle = angle - STEERING_WHEEL_ENCODER_CENTER_DEG;
    angle = angle * -1.0;
    if (angle < -200.0) {
        angle += 360.0;
    }
    return angle;
}

float update_rolling_average(float average, float new_value) {
    static const float a = 1.0f / ENC_ROLLAVG_SIZE;
    average              = average * (1.0f - a) + new_value * a;
    return average;
}

/**
 * @brief     Calculate the ground speed from steering wheel encoder
 */
void ENC_C_push_angle_deg() {
    uint8_t buf[2] = {0};
    /*
        Clock rate must be <= 4 MHz (from datasheet)
        Also, the interval between two consecutive conversions must be > 20 μs
    */
    volatile HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi2, buf, 2, 100);
    if (status != HAL_OK) {
        (void)status;
    }

    float uncalibrated_angle = raw_to_angle(buf[0], buf[1]);

    float angle            = calibrate_angle(uncalibrated_angle);
    _ENC_C_inplace_average = update_rolling_average(_ENC_C_inplace_average, angle);
}

/**
 * @brief     Read the absolute steering wheel angle
 */
float ENC_C_get_angle_deg() {
    return _ENC_C_inplace_average;
}

void ENC_send_vals_in_CAN() {
    ecumsg_steer_angle_state.data.angle  = ENC_C_get_angle_deg();
    ecumsg_steer_angle_state.info.is_new = true;

    ecumsg_front_angular_velocity_state.data.fl = ENC_L_get_radsec();
    ecumsg_front_angular_velocity_state.data.fr = ENC_R_get_radsec();
    // ecumsg_angular_velocity_state.data.inverter_l = (M_PI / 60.0) *
    // INV_get_RPM(INV_LEFT); ecumsg_angular_velocity_state.data.inverter_r =
    // (M_PI / 60.0) * INV_get_RPM(INV_RIGHT);
    ecumsg_front_angular_velocity_state.info.is_new = true;
}

float _ENC_ms_to_radsec(float vel) {
    float enc_circ_m = 0.2368760861f;
    float rpm        = vel / enc_circ_m;
    return rpm * 0.10472f;
}
