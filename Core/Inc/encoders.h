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

#include "stdbool.h"
#include "stdint.h"
#include "ecu_config.h"

#define ENC_L_TIM htim2 /*< Left encoder timer */
#define ENC_R_TIM htim5 /*< Right encoder timer */
// #define ENC_C_SPI hspi2 /*< Center encoder SPI */

#define ENC_ROLLAVG_SIZE 5

#define ENC_SPEED_FREQ_HZ   1000                     /* 1000Hz */
#define ENC_SPEED_PERIOD_MS 1000 / ENC_SPEED_FREQ_HZ /* 1ms */

#define ENC_STEER_FREQ_HZ   200                      /* 200Hz */
#define ENC_STEER_PERIOD_MS 1000 / ENC_STEER_FREQ_HZ /* 5ms */

#define STEERING_ACTUATOR_FREQ_HZ   200
#define STEERING_ACTUATOR_PERIOD_MS 1000 / STEERING_ACTUATOR_FREQ_HZ

#if ENC_BRAKE_ACTUATOR_ENABLED
    #define MM_STEP_BRAKE_ACTUATOR (0.01f) // mm/step actuator
    #define ENC_MM_TICK_BRAKE_ACTUATOR (1.220703125e-4f) // mm/tick encoder
    #define CPR_ENC_BRAKE_ACTUATOR 16384

    #define ENC_BRAKE_FREQ_HZ   200                      /* 200Hz */
    #define ENC_BRAKE_PERIOD_MS 1000 / ENC_BRAKE_FREQ_HZ /* 5ms */
#endif

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


#if ENC_BRAKE_ACTUATOR_ENABLED

/**
 * @brief   Computes the relative rotation based on encoder ticks, scaled by the CPR value.
 */
uint32_t get_relative_counter(TIM_HandleTypeDef *htim);

/**
 * @brief   Returns the number of ticks read by the encoder
 */
uint32_t get_absolute_counter(TIM_HandleTypeDef *htim);

/**
 * @brief   Read from the encoder the ticks and convert them into distance mm
 */
float get_distance(TIM_HandleTypeDef *htim);

#endif

#endif