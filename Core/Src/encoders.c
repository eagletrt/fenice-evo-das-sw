#include "can_messages.h"
#include "encoders.h"
#include "logger.h"
#include "tim.h"
#include "limits.h"
#include "time_base.h"
#include "spi.h"
#include <float.h>

#include <math.h>

float _ENC_L_rollavg[ENC_ROLLAVG_SIZE],
      _ENC_R_rollavg[ENC_ROLLAVG_SIZE],
      _ENC_C_median_window[ENC_ROLLAVG_SIZE] = {};
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
float _ENC_calculate_wheel_speed(uint32_t tim_counter, uint32_t elapsed_us) {
    return (tim_counter / (120.0 * 400)) * 2*M_PI * 1000000.0 / elapsed_us;
}

/**
 * @brief     Calulate the difference between the current value of the timer counter
 *            and its value from the last read
 * @param     htim The timer from which to read
 * @return    uint32_t Timer ticks since the last read
 */
uint32_t _ENC_get_tim_cnt(TIM_HandleTypeDef *htim) {
    uint32_t *last_cnt = (htim == &ENC_L_TIM) ? &_ENC_L_last_cnt : &_ENC_R_last_cnt;
    uint32_t cnt = __HAL_TIM_GET_COUNTER(htim);
    uint32_t diff = fmin(
        *last_cnt - cnt,
        UINT32_MAX - (*last_cnt - cnt)
    );
    *last_cnt = cnt;
    return diff;
}


/**
 * @brief     Calculate the ground speed from the right wheel encoder
 */
void ENC_L_push_speed_rads() {
    uint32_t ellapsed_us = get_time() - _ENC_L_last_ticks;
    uint32_t tim_counter = _ENC_get_tim_cnt(&ENC_L_TIM);
    float speed = _ENC_calculate_wheel_speed(tim_counter, ellapsed_us); //impulses * 2*PI / encoder_poles / seconds
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
    uint32_t ellapsed_us = get_time() - _ENC_R_last_ticks;
    uint32_t tim_counter = _ENC_get_tim_cnt(&ENC_R_TIM);
    float speed = _ENC_calculate_wheel_speed(tim_counter, ellapsed_us); //impulses * 2*PI / encoder_poles / seconds
    _ENC_R_last_ticks = get_time();
    _ENC_R_rollavg[_ENC_R_rollavg_idx] = speed;
    _ENC_R_rollavg_idx = (_ENC_R_rollavg_idx + 1) % ENC_ROLLAVG_SIZE;
}

/**
 * @brief     Calculate the ground speed from steering wheel encoder
 */
void ENC_C_push_angle_deg(){
    float calib_center_ang = 193.8f;
    uint16_t buf = 0;
    
    /* Clock rate must be <= 4 MHz (from datasheet) */
    /* Also, the interval between two consecutive conversions must be > 20 μs */
    HAL_SPI_Receive(&hspi2, (uint8_t*)&buf, 2, 100);

    /* Now `buf` contains [LSB|MSB] (MSB arrived first and is written first - little endian) */
    uint8_t msb = (uint8_t)(buf >> 8);
    uint8_t lsb = (uint8_t)(buf & 0x00FF);
    // LOG_write(LOGLEVEL_DEBUG, "[ENC] msb: 0x%x, lsb: 0x%x", msb, lsb);

    /* We need `buf` to be [MSB|LSB], but the transmission is 12 bits long so we discard the last 4 */
    buf = ((uint16_t)(lsb) << 8) | (msb);
    buf = (buf >> 3) & 0x0FFF;

    float angle = 360.0f / 4096.f * buf;

    /* Update array of values*/
    for (int i = ENC_ROLLAVG_SIZE-1; i > 0; i--)
        _ENC_C_median_window[i] = _ENC_C_median_window[i-1];
    
    _ENC_C_median_window[0] = angle; // -(angle - calib_center_ang);

}

/**
 * @brief     Read the absolute steering wheel angle
 */
float ENC_C_get_angle_deg() {
     
    float min = FLT_MAX, max = FLT_MIN, median = 0;
    for (int i = 0; i < ENC_ROLLAVG_SIZE; i++){
        if (_ENC_C_median_window[i] < min){
            min = _ENC_C_median_window[i];
        }
        if (_ENC_C_median_window[i] > max){
            max = _ENC_C_median_window[i];
        }
        median += _ENC_C_median_window[i];
    }
    median = median - max - min;
    median /= (ENC_ROLLAVG_SIZE-2);

    return median;
}

void ENC_send_vals_in_CAN() {
    CANMSG_SteerVal.data.angle = ENC_C_get_angle_deg();
    CANMSG_SteerVal.data.angle = 0.0;
    CANMSG_SteerVal.info.is_new = true;

    CANMSG_Speed.data.encoder_l = ENC_L_get_radsec();
    CANMSG_Speed.data.encoder_r = ENC_R_get_radsec();
    CANMSG_Speed.info.is_new = true;
}

float _ENC_ms_to_radsec(float vel) {
    float enc_circ_m = 0.2368760861f;
    float rpm = vel / enc_circ_m;
    return rpm * 0.10472f;
}
