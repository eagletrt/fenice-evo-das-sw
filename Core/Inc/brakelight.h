#ifndef BKLIGHT_H
#define BKLIGHT_H

#include "tim.h"


#define BKL_TIM htim3

/**
 * @brief     The available states of the brakelight
 */
typedef enum {
    BKL_OFF,    /*< Off */
    BKL_LVL_1,  /*< Minimum intensity */
    BKL_LVL_2,  /*< Medium intensity */
    BKL_LVL_3   /*< Maximum intensity */
} BKL_StateTypeDef;


/**
 * @brief     Init the brakelight and start the pwm channels
 */
void BKL_Init();

/**
 * @brief     Set the state of the brakelight to 'state'
 */
void BKL_set_state(BKL_StateTypeDef state);

/**
 * @brief     Set the curve of the brakelight
 */
void BKL_set_curve(float brk_percent);

/**
 * @brief     Update, at each call, the status of a light sequence that alternates channels and PWM duty-cycles to test the brakelight
 */
void BKL_light_show_step();

#endif