#include "time_base.h"

#include "timer_utils.h"

struct time_base_t {
    TIM_HandleTypeDef *htim;
    uint64_t counter;
};

static struct time_base_t time_data;

/**
 * @brief     Inizializate the struct
 */
void time_base_init(TIM_HandleTypeDef *htim) {
    time_data.htim    = htim;
    time_data.counter = 0ull;
}

/**
 * @brief     Return timer in millisecond  
 */
uint64_t get_time() {
    return (time_data.counter & (~TIME_BASE_MASK)) | (__HAL_TIM_GET_COUNTER(time_data.htim) & TIME_BASE_MASK);
}

void time_base_elapsed() {
    time_data.counter += (1 << TIME_BASE_TIMER_BITS);
}
