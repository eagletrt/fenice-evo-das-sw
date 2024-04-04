#include "can_messages.h"
#include "encoders.h"
#include "logger.h"
#include "tim.h"
#include "limits.h"
#include "time_base.h"
#include "spi.h"
#include "inverters.h"
#include <float.h>
#include "usart.h"

#include <math.h>

float _ENC_L_rollavg[ENC_ROLLAVG_SIZE] = {},
      _ENC_R_rollavg[ENC_ROLLAVG_SIZE] = {};
float _ENC_C_inplace_average = 0.0f;
uint8_t _ENC_L_rollavg_idx = 0, _ENC_R_rollavg_idx = 0, _ENC_C_median_window_idx = 0; 

uint32_t _ENC_L_last_ticks = 0; /*< Last read ticks (milliseconds) for the left encoder */
uint32_t _ENC_R_last_ticks = 0; /*< Last read ticks (milliseconds) for the right encoder */

uint32_t _ENC_L_last_cnt = 0;   /*< Last read timer counter for the left encoder */
uint32_t _ENC_R_last_cnt = 0;   /*< last read timer counter for the right encoder */

float _ENC_encoder_resolution_m = 0.000005f; /*< Resolution of the magneting ring in meters */
float _ENC_speed_multiplier = 5.238726792f;  /*< Multiplier to convert encoder speed to wheel speed */

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
    return (tim_counter / (120.0 * 400)) * 2*M_PI * 1000000.0 / elapsed_us;
}

/**
 * @brief     Calulate the difference between the current value of the timer counter
 *            and its value from the last read
 * @param     htim The timer from which to read
 * @return    uint32_t Timer ticks since the last read
 */
int64_t _ENC_get_tim_cnt(TIM_HandleTypeDef *htim) {
    uint32_t *last_cnt_ptr = (htim == &ENC_L_TIM) ? &_ENC_L_last_cnt : &_ENC_R_last_cnt;
    int64_t last_cnt = *last_cnt_ptr;
    int64_t cnt = __HAL_TIM_GET_COUNTER(htim);
    int64_t diff = last_cnt - cnt;
    *last_cnt_ptr = cnt;
    return diff;
}


/**
 * @brief     Calculate the ground speed from the right wheel encoder
 */
void ENC_L_push_speed_rads() {
    uint32_t ellapsed_us = get_time() - _ENC_L_last_ticks;
    int64_t tim_counter = _ENC_get_tim_cnt(&ENC_L_TIM);
    float speed = - _ENC_calculate_wheel_speed(tim_counter, ellapsed_us); //impulses * 2*PI / encoder_poles / seconds
    _ENC_L_last_ticks = get_time();
    _ENC_L_rollavg[_ENC_L_rollavg_idx] = speed;
    _ENC_L_rollavg_idx = (_ENC_L_rollavg_idx + 1) % ENC_ROLLAVG_SIZE;
}

float ENC_get_radsec(float *array) {
    float acc = 0;
    float max = array[0], min = array[0];

    for(uint8_t i = 0; i < ENC_ROLLAVG_SIZE; i++) {
        acc += array[i];
        if(array[i] > max) max = array[i];
        if(array[i] < min) min = array[i];
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
    uint32_t elapsed_us = get_time() - _ENC_R_last_ticks;
    int64_t tim_counter = _ENC_get_tim_cnt(&ENC_R_TIM);
    float speed = - _ENC_calculate_wheel_speed(tim_counter, elapsed_us); //impulses * 2*PI / encoder_poles / seconds
    _ENC_R_last_ticks = get_time();
    _ENC_R_rollavg[_ENC_R_rollavg_idx] = speed;
    _ENC_R_rollavg_idx = (_ENC_R_rollavg_idx + 1) % ENC_ROLLAVG_SIZE;
}

/**
 * @brief     Calculate the ground speed from steering wheel encoder
 */
void ENC_C_push_angle_deg(){
    float calib_center_ang = 280.9f;
    uint16_t buf = 0;

    /* Clock rate must be <= 4 MHz (from datasheet) */
    /* Also, the interval between two consecutive conversions must be > 20 μs */
    HAL_SPI_Receive(&hspi2, (uint8_t*)&buf, 2, 100);
    buf = buf >> 3; 
    buf = buf & 0x0FFF;

    /* Now `buf` contains [LSB|MSB] (MSB arrived first and is written first - little endian) */

    float raw = 360.0f / 4095.f * buf;
    float angle = raw - calib_center_ang;
    angle *= -1.0;
    if(angle > 180.0f) angle -= 360.0f;

    /* Update array of values*/
    static const float a = 1.0f/ENC_ROLLAVG_SIZE;
    _ENC_C_inplace_average = _ENC_C_inplace_average * (1.0f - a) + angle * a;
}

/**
 * @brief     Read the absolute steering wheel angle
 */
float ENC_C_get_angle_deg() {
    return _ENC_C_inplace_average;
}

void ENC_send_vals_in_CAN() {
    ecumsg_steer_angle_state.data.angle = ENC_C_get_angle_deg();
    ecumsg_steer_angle_state.info.is_new = true;

    ecumsg_speed_state.data.fl = ENC_L_get_radsec();
    ecumsg_speed_state.data.fr = ENC_R_get_radsec();
    // ecumsg_speed_state.data.inverter_l = (M_PI / 60.0) * INV_get_RPM(INV_LEFT);
    // ecumsg_speed_state.data.inverter_r = (M_PI / 60.0) * INV_get_RPM(INV_RIGHT);
    ecumsg_speed_state.info.is_new = true;
}

float _ENC_ms_to_radsec(float vel) {
    float enc_circ_m = 0.2368760861f;
    float rpm = vel / enc_circ_m;
    return rpm * 0.10472f;
}
