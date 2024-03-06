/**
 * @file      encoders.h
 * @author    Alex Sartori [alex.sartori1997@gmail.com]
 * @date      2021-10-11
 * 
 * @brief     Handle reading of Fenice's wheel and steering encoders
 * 
 */

#ifndef ENCODERS_H
#define ENCODERS_H

#include "stdint.h"
#include "stdbool.h"


#define ENC_L_TIM htim2 /*< Left encoder timer */
#define ENC_R_TIM htim5 /*< Right encoder timer */
#define ENC_C_SPI hspi2 /*< Center encoder SPI */

#define ENC_ROLLAVG_SIZE 5
#define ENC_WHEEL_RADIUS 0.203

#define ENC_SPEED_FREQ_HZ  1000 /* 1000Hz */
#define ENC_SPEED_PERIOD_MS  1000/ENC_SPEED_FREQ_HZ /* 1ms */

#define ENC_STEER_FREQ_HZ  200 /* 200Hz */
#define ENC_STEER_PERIOD_MS  1000/ENC_STEER_FREQ_HZ /* 5ms */


/**
 * @brief     Compute the speed of the front-left wheel in radians/second
 */
float ENC_L_get_radsec();

/**
 * @brief     Compute the speed of the front-right wheel in radians/second
 */
float ENC_R_get_radsec();

/**
 * @brief     Calculate the ground speed from steering wheel encoder
 */
void ENC_C_push_angle_deg();

/**
 * @brief     Get the steering encoder's absolute position in degrees   
 */
float ENC_C_get_angle_deg();


/**
 * @brief     Send updates over CAN on the current angle of the steering wheel
 */
void ENC_send_vals_in_CAN();



void ENC_R_push_speed_rads();
void ENC_L_push_speed_rads();

#endif