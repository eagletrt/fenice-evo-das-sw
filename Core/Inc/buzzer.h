/**
 * @file      buzzer.h
 * @author    Alex Sartori [alex.sartori1997@gmail.com]
 * @date      2021-10-04
 * 
 * @brief     Handle buzzer beeps abstracting the hardware, i.e. providing a
 *            transparent interface regardless of active or passive buzzers
 */

#ifndef BUZZER_H
#define BUZZER_H

#include "stdint.h"


/**
 * @brief     Play a frequency on the buzzer for a given number of milliseconds and return
 * @param     ms Duration of the sound in milliseconds
 */
void BUZ_beep_ms_sync(uint16_t);

/**
 * @brief     Asynchronously play a frequency on the buzzer for a given number of milliseconds
 * @param     ms Duration of the sound in milliseconds
 */
void BUZ_beep_ms_async(uint16_t);

/**
 * @brief     Function to periodically invoke to check anad stop async calls
 */
void BUZ_timer_callback();

#endif