/**
 * @file      pedals.h
 * @author    Alex Sartori [alex.sartori1997@gmail.com]
 * @date      2022-03-12
 * @prefix    PED
 * 
 * @brief     Module to read pedals' potentiometers
 */

#ifndef PEDALS_H
#define PEDALS_H

#include "stdint.h"
#include "stdbool.h"
#include "spi.h"

// #define PED_ADC hadc1              /*< ADC connected to the pedals */
#define PED_DEADZONE_PERCENT 20.0f /*< Initial portion of pedal travel to ignore */
#define PED_IMPL_THRESHOLD   60.0f /*< Percent of potentiometer offset that triggers the error */
#define BRK_IMPL_THRESHOLD    4.5f
#define BRK_THRESHOLD_HIGH    0.9f
#define BRK_THRESHOLD_LOW     0.4f
#define BRK_DRIVE_THRESHOLD   2.0f

#define EEPROM_SPI hspi1

#define PED_DEBUG 1

typedef struct {
    bool ADC_internal, ADC_overrun, ADC_DMA_error, implausibility_err;
} PED_PedalErrors;

typedef enum {
    PED_CALIB_APPS_MIN,
    PED_CALIB_APPS_MAX,
    PED_CALIB_BSE_MIN,
    PED_CALIB_BSE_MAX
} PED_CalibTypeDef;

extern PED_PedalErrors PED_errors; /*< Pedal errors active */


/**
 * @brief     Initialize ADCs and DMA streams
 */
void PED_init();

/**
 * @brief     Return the value of the accelerator pedal in the range 0.0f - 100.0f
 */
float PED_get_accelerator_percent();

/**
 *  @brief     Return the torque requested by the accelerator pedal in Nm in the range 0.0f - 55.0f Nm
 */
float PED_get_accelerator_torque(float acc_percent);

/**
 * @brief     Return the value of the brake pedal in the range 0.0f - 100.0f
 */
float PED_get_brake_bar();

/**
 * @brief     Send updates over CAN on the current values of the pedals
 */
void PED_send_vals_in_CAN();

/**
 * @brief     Set the specified pedal bound (e.g., APPS_MAX) at the current level being read
 */
void PED_calibrate(PED_CalibTypeDef);

void PED_update_plausibility_check();

bool PED_is_brake_ok();

#if PED_DEBUG
void PED_log_dbg_info();
#endif

#endif