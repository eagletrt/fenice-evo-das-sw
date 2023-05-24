#include "pedals.h"
// #include "adc.h"
#include "logger.h"
#include "can_messages.h"
#include "math.h"

#include "limits.h"

#include "../Lib/micro-libs/eeprom-config/eeprom-config.h"

/* Private variables and constants ------------------------------------------ */
PED_PedalErrors PED_errors = {};
uint32_t _PED_impl_start_timestamp = 0;
uint32_t _PED_implausibility_max_time_ms = 100;

uint32_t _PED_ADC_raw_values[4] = {0U};             /*< Buffer for DMA transfers */
#define _PED_ADC_RAW_BRKF (_PED_ADC_raw_values[0])  /*< Raw ADC value of front brake transducer */
#define _PED_ADC_RAW_BRKR (_PED_ADC_raw_values[1])  /*< Raw ADC value of rear brake transducer */
#define _PED_ADC_RAW_ACC2 (_PED_ADC_raw_values[2])  /*< Raw ADC value of APPS 2 */
#define _PED_ADC_RAW_ACC1 (_PED_ADC_raw_values[3])  /*< Raw ADC value of APPS 1 */

typedef struct {
    uint16_t BRKF_MIN;
    uint16_t BRKF_MAX;
    uint16_t BRKR_MIN;
    uint16_t BRKR_MAX;
    uint16_t ACC1_MIN;
    uint16_t ACC1_MAX;
    uint16_t ACC2_MIN;
    uint16_t ACC2_MAX;
} _PED_CALIB;

#define _PED_CALIB_BRKF_MIN (((_PED_CALIB*)EEPROM_config_get(&pedals_config))->BRKF_MIN)
#define _PED_CALIB_BRKF_MAX (((_PED_CALIB*)EEPROM_config_get(&pedals_config))->BRKF_MAX)
#define _PED_CALIB_BRKR_MIN (((_PED_CALIB*)EEPROM_config_get(&pedals_config))->BRKR_MIN)
#define _PED_CALIB_BRKR_MAX (((_PED_CALIB*)EEPROM_config_get(&pedals_config))->BRKR_MAX)
#define _PED_CALIB_ACC1_MIN (((_PED_CALIB*)EEPROM_config_get(&pedals_config))->ACC1_MIN)
#define _PED_CALIB_ACC1_MAX (((_PED_CALIB*)EEPROM_config_get(&pedals_config))->ACC1_MAX)
#define _PED_CALIB_ACC2_MIN (((_PED_CALIB*)EEPROM_config_get(&pedals_config))->ACC2_MIN)
#define _PED_CALIB_ACC2_MAX (((_PED_CALIB*)EEPROM_config_get(&pedals_config))->ACC2_MAX)

EEPROM_ConfigTypeDef pedals_config = {};

/* Prototypes --------------------------------------------------------------- */
bool _PED_are_values_plausible(uint16_t, uint16_t);
void PED_update_plausibility_check();
float _PED_from_raw_to_percent(uint16_t, uint16_t, uint16_t);
float _PED_remove_dead_zone(float);


void PED_init() {
    _PED_CALIB calib;

    /* Set calibration values */
    calib.BRKF_MIN = 399;
    calib.BRKF_MAX = 3590;
    calib.BRKR_MIN = 399;
    calib.BRKR_MAX = 3590;
    calib.ACC1_MIN = 550;
    calib.ACC1_MAX = 2500;
    calib.ACC2_MIN = 1900;
    calib.ACC2_MAX = 3920;

    EEPROM_config_init(&pedals_config, &EEPROM_SPI, CS_EEPROM_GPIO_Port, CS_EEPROM_Pin, 0x0, 0x3, &calib, sizeof(calib));

    /* Begin DMA stream with the ADC */
    // HAL_ADC_Start_DMA(&PED_ADC, _PED_ADC_raw_values, 4);
}

float PED_get_accelerator_percent() {
    float acc1_percent = _PED_from_raw_to_percent(_PED_ADC_RAW_ACC1, _PED_CALIB_ACC1_MIN, _PED_CALIB_ACC1_MAX);
    float acc2_percent = _PED_from_raw_to_percent(_PED_ADC_RAW_ACC2, _PED_CALIB_ACC2_MIN, _PED_CALIB_ACC2_MAX);
    float acc_avg = (acc1_percent + acc2_percent) / 2.0f;
    float acc_no_deadzone = _PED_remove_dead_zone(acc_avg);

    return acc_no_deadzone;
}

float PED_get_brake_percent() {
    float bf_percent = _PED_from_raw_to_percent(_PED_ADC_RAW_BRKF, _PED_CALIB_BRKF_MIN, _PED_CALIB_BRKF_MAX);
    float br_percent = _PED_from_raw_to_percent(_PED_ADC_RAW_BRKR, _PED_CALIB_BRKR_MIN, _PED_CALIB_BRKR_MAX);
    float brk_max = (bf_percent > br_percent) ? bf_percent : br_percent;

    return brk_max;
}

float _PED_from_raw_to_percent(uint16_t val, uint16_t min, uint16_t max) {
    if (val < min) val = min;
    if (val > max) val = max;
    return (100.0 * (val - min) / (max - min));
}

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
// }

// void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
//     uint32_t e = hadc->ErrorCode;
//     LOG_write(LOGLEVEL_ERR, "[ADC] Error: 0x%x", e);

//     if (e & HAL_ADC_ERROR_INTERNAL) {
//         LOG_write(LOGLEVEL_ERR, "[ADC]\t+ HAL_ADC_ERROR_INTERNAL");
//         PED_errors.ADC_internal = true;
//     }
//     if (e & HAL_ADC_ERROR_OVR) {
//         LOG_write(LOGLEVEL_ERR, "[ADC]\t+ HAL_ADC_ERROR_OVR");
//         PED_errors.ADC_overrun = true;
//     }
//     if (e & HAL_ADC_ERROR_DMA) {
//         LOG_write(LOGLEVEL_ERR, "[ADC]\t+ HAL_ADC_ERROR_DMA");
//         PED_errors.ADC_DMA_error = true;
//     }
// }

/**
 * @brief     Check that ADC values comply with rule T/11.8.8 and set or clear
 *            the corresponding flag in PED_errors.
 */
void PED_update_plausibility_check() {
    if (_PED_are_values_plausible(_PED_ADC_RAW_ACC1, _PED_ADC_RAW_ACC2)) {
        _PED_impl_start_timestamp = 0;
    } else {
        if (_PED_impl_start_timestamp == 0)
            _PED_impl_start_timestamp = HAL_GetTick();
        if ((HAL_GetTick() - _PED_impl_start_timestamp) > _PED_implausibility_max_time_ms)
            PED_errors.implausibility_err = true;
    }
}

/**
 * @brief     Check that potentiometer values differ at most by 10% of their normal offset
 */
bool _PED_are_values_plausible(uint16_t val_1, uint16_t val_2) {
    if (_PED_ADC_RAW_ACC1 < 100 || _PED_ADC_RAW_ACC2 < 100)
        return 0;
    int16_t pot_offset = abs(_PED_CALIB_ACC1_MIN - _PED_CALIB_ACC2_MIN);
    int16_t threshold = pot_offset * (PED_IMPL_THRESHOLD / 100.0f);
    int16_t diff = abs(val_2 - val_1);
    return diff <= (pot_offset + threshold) && diff >= (pot_offset - threshold);
}

/**
 * @brief     Provide a dead zone for the first X% of pedal travel. At (X+1)% the value
 *            should not abruptibly begin from (x+1)% but begin from 0%, so scale the remaining
 *            (100-X)% from 0% to 100%.
 */
float _PED_remove_dead_zone(float val) {
    val = (val - PED_DEADZONE_PERCENT) / (100.0f - PED_DEADZONE_PERCENT) * 100.0f;
    return (val >= 0) ? val : 0;
}

void PED_send_vals_in_CAN() {
    CANMSG_PedVals.data.apps = PED_get_accelerator_percent();
    CANMSG_PedVals.data.bse_front = _PED_from_raw_to_percent(_PED_ADC_RAW_BRKF, _PED_CALIB_BRKF_MIN, _PED_CALIB_BRKF_MAX);
    CANMSG_PedVals.data.bse_rear = _PED_from_raw_to_percent(_PED_ADC_RAW_BRKR, _PED_CALIB_BRKR_MIN, _PED_CALIB_BRKR_MAX);
    CANMSG_PedVals.info.is_new = true;
}

void PED_calibrate(PED_CalibTypeDef calib) {
    switch (calib) {
        case PED_CALIB_APPS_MIN:
            _PED_CALIB_ACC1_MIN = _PED_ADC_RAW_ACC1;
            _PED_CALIB_ACC2_MIN = _PED_ADC_RAW_ACC2;
            break;
        case PED_CALIB_APPS_MAX:
            _PED_CALIB_ACC1_MAX = _PED_ADC_RAW_ACC1;
            _PED_CALIB_ACC2_MAX = _PED_ADC_RAW_ACC2;
            break;
        case PED_CALIB_BSE_MIN:
            LOG_write(LOGLEVEL_WARN, "[PED] Brake calibration not implemented");
            break;
        case PED_CALIB_BSE_MAX:
            LOG_write(LOGLEVEL_WARN, "[PED] Brake calibration not implemented");
            break;
    }
    pedals_config.dirty = true;
    EEPROM_config_write(&pedals_config);
    PED_errors.implausibility_err = false;
}

bool PED_is_brake_ok() {
    if (_PED_ADC_RAW_BRKF < (_PED_CALIB_BRKF_MIN - 100) || _PED_ADC_RAW_BRKF > (_PED_CALIB_BRKF_MAX + 100))
        return 0;
    if (_PED_ADC_RAW_BRKR < (_PED_CALIB_BRKR_MIN - 100) || _PED_ADC_RAW_BRKR > (_PED_CALIB_BRKR_MAX + 100))
        return 0;
    return 1;
}

#if PED_DEBUG
    void PED_log_dbg_info() {
        LOG_write(LOGLEVEL_DEBUG, "PED/Cal  | %8s: %-6d %8s: %-6d %8s: %-6d %8s: %-6d",
            "ACC1 Max", _PED_CALIB_ACC1_MAX, "ACC1 Min", _PED_CALIB_ACC1_MIN,
            "ACC2 Max", _PED_CALIB_ACC2_MAX, "ACC2 Min", _PED_CALIB_ACC2_MIN
        );
        LOG_write(LOGLEVEL_DEBUG, "PED/ADCs | %8s: %-6d %8s: %-6.1f %8s: %-6d %8s: %-6.1f",
            "ACC1 Raw", _PED_ADC_RAW_ACC1, "ACC1 %%", _PED_from_raw_to_percent(_PED_ADC_RAW_ACC1,_PED_CALIB_ACC1_MIN, _PED_CALIB_ACC1_MAX),
            "ACC2 Raw", _PED_ADC_RAW_ACC2, "ACC2 %%", _PED_from_raw_to_percent(_PED_ADC_RAW_ACC2, _PED_CALIB_ACC2_MIN, _PED_CALIB_ACC2_MAX)
        );
        LOG_write(LOGLEVEL_DEBUG, "PED/ADCs | %8s: %-6d %8s: %-6.1f %8s: %-6d %8s: %-6.1f",
            "BRKF Raw", _PED_ADC_RAW_BRKF, "BRKF %%", _PED_from_raw_to_percent(_PED_ADC_RAW_BRKF, _PED_CALIB_BRKF_MIN, _PED_CALIB_BRKF_MAX),
            "BRKR Raw", _PED_ADC_RAW_BRKR, "BRKR %%", _PED_from_raw_to_percent(_PED_ADC_RAW_BRKR, _PED_CALIB_BRKR_MIN, _PED_CALIB_BRKR_MAX)
        );
    }
#endif