#include "stdint.h"
#include "stm32f4xx_hal.h"

#ifndef TIME_BASE_TIMER_BITS
#define TIME_BASE_TIMER_BITS 16
#endif
#define TIME_BASE_MASK ((1 << TIME_BASE_TIMER_BITS) - 1)

/**
 * @brief     Inizializate the struct, the timer should be at microsecond precision.
 */
void time_base_init(TIM_HandleTypeDef *htim);

/**
 * @brief     Return timer in microsecond
 */
uint64_t get_time();

/**
 * @brief     Update the time_data.counter every HAL_TIM_PeriodElapsedCallback  
 */
void time_base_elapsed();
