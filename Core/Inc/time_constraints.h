/**
 * @file      time_constraints.h
 * @author    Alex Sartori [alex.sartori1997@gmail.com]
 * @date      2022-06-30
 * @prefix    TCS
 * 
 * @brief     Handle timing constraints of the DAS such as timeouts
 */

#ifndef TIME_CONSTRAINTS_H
#define TIME_CONSTRAINTS_H

#include "stdbool.h"
#include "stdint.h"

void WDG_init();
void WDG_update_and_check_timestamps();
bool WDG_is_car_in_safe_state();

#endif