#include "pedals.h"

#include "../Lib/micro-libs/eeprom-config/eeprom-config.h"
#include "adc_fsm.h"
#include "can_messages.h"
#include "limits.h"
#include "logger.h"
#include "math.h"

/* Private variables and constants ------------------------------------------ */
PED_PedalErrors PED_errors               = {};
uint32_t _PED_impl_start_timestamp       = 0;
uint32_t _PED_implausibility_max_time_ms = 100;

typedef struct {
    uint16_t BRKF_MIN;
    uint16_t BRKF_MAX;
    uint16_t BRKR_MIN;
    uint16_t BRKR_MAX;
    uint16_t APPS1_MIN;
    uint16_t APPS1_MAX;
    uint16_t APPS2_MIN;
    uint16_t APPS2_MAX;
    uint16_t BPPS1_MIN;
    uint16_t BPPS1_MAX;
    uint16_t BPPS2_MIN;
    uint16_t BPPS2_MAX;
} _PED_CALIB;

#define _PED_CALIB_BRKF_MIN  (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->BRKF_MIN)
#define _PED_CALIB_BRKF_MAX  (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->BRKF_MAX)
#define _PED_CALIB_BRKR_MIN  (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->BRKR_MIN)
#define _PED_CALIB_BRKR_MAX  (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->BRKR_MAX)
#define _PED_CALIB_APPS1_MIN (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->APPS1_MIN)
#define _PED_CALIB_APPS1_MAX (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->APPS1_MAX)
#define _PED_CALIB_APPS2_MIN (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->APPS2_MIN)
#define _PED_CALIB_APPS2_MAX (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->APPS2_MAX)
#define _PED_CALIB_BPPS1_MIN (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->BPPS1_MIN)
#define _PED_CALIB_BPPS1_MAX (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->BPPS1_MAX)
#define _PED_CALIB_BPPS2_MIN (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->BPPS2_MIN)
#define _PED_CALIB_BPPS2_MAX (((_PED_CALIB *)EEPROM_config_get(&pedals_config))->BPPS2_MAX)

#define _PEDALS_CONFIG_BRKF_MIN

EEPROM_ConfigTypeDef pedals_config = {};

/* Prototypes --------------------------------------------------------------- */
bool _PED_are_values_plausible(uint32_t, uint32_t);
void PED_update_plausibility_check();
float _PED_from_raw_to_percent(uint32_t, uint32_t, uint32_t);
float _PED_remove_dead_zone(float);

void PED_init() {
    _PED_CALIB calib;

    /* Set calibration values */
    calib.BRKF_MIN  = 400;
    calib.BRKF_MAX  = 1030;
    calib.BRKR_MIN  = 520;
    calib.BRKR_MAX  = 700;
    calib.APPS1_MIN = 1740;
    calib.APPS1_MAX = 3500;
    calib.APPS2_MIN = 460;
    calib.APPS2_MAX = 2300;
    calib.BPPS1_MIN = 0;
    calib.BPPS1_MAX = 100;
    calib.BPPS2_MIN = 0;
    calib.BPPS2_MAX = 100;

    EEPROM_config_init(&pedals_config, &EEPROM_SPI, CS_EEPROM_GPIO_Port, CS_EEPROM_Pin, 0x0, 0x3, &calib, sizeof(calib));
}

float PED_get_accelerator_percent() {
    float acc1_percent = _PED_from_raw_to_percent(
        ADC_get_APPS1(),
        1800,  // _PED_CALIB_APPS1_MIN,
        3600   // _PED_CALIB_APPS1_MAX
    );
    float acc2_percent = _PED_from_raw_to_percent(
        ADC_get_APPS2(),
        600,  // 286, // _PED_CALIB_APPS2_MIN,
        2300  // 800 // _PED_CALIB_APPS2_MAX
    );
    float acc_avg         = (acc1_percent + acc2_percent) / 2.0f;
    static float mean     = 0.0f;
    const int samples     = 50;
    mean                  = mean * (1.0f - 1.0f / samples) + acc_avg * (1.0f / samples);
    float acc_no_deadzone = _PED_remove_dead_zone(mean);
    return acc_no_deadzone / 100.0f;

    // return acc_no_deadzone;
    // static float avg = 0.0f;
    // avg = avg * 0.99 + acc_no_deadzone * 0.01;
    // avg = avg * 0.8 + acc_no_deadzone * 0.2;
    // return avg / 100.0;
}

float PED_get_accelerator_torque(float acc_percent) {
    float mp = DAS_get_power_map();
    mp       = mp > 0.0 ? mp : 0.0;
    mp       = mp > 1.0 ? 1.0 : mp;
    return PED_MAX_TORQUE * acc_percent * mp;
}

float PED_get_brake_bar() {
    uint32_t brk_f, brk_r;
    get_brk_average(&brk_f, &brk_r);

    float bf_mV = ((float)brk_f * 3.3f / 4096.0f) / VOLTAGE_DIVIDER;
    float br_mV = ((float)brk_r * 3.3f / 4096.0f) / VOLTAGE_DIVIDER;

    float bf_bar = ((bf_mV - 0.5f) / 4.0f) * 100.0f;
    float br_bar = ((br_mV - 0.5f) / 4.0f) * 100.0f;

    // old conversion with the voltage divider
    // float bf_bar = ((brk_f/4096.0 * 3.3 - 0.3)/ 4.0) * 100.0;
    // float br_bar = ((brk_r/4096.0 * 3.3 - 0.35)/4.0) * 100.0;
    float brk_max = (bf_bar > br_bar) ? bf_bar : br_bar;
    return brk_max;
}

float _PED_from_raw_to_percent(uint32_t val, uint32_t min, uint32_t max) {
    if (val < min)
        val = min;
    if (val > max)
        val = max;
    volatile float x = (100.0f * ((float)val - (float)min) / ((float)max - (float)min));
    return x;
}

/**
 * @brief     Check that ADC values comply with rule T/11.8.8 and set or clear
 *            the corresponding flag in PED_errors.
 */
void PED_update_plausibility_check() {
    if (_PED_are_values_plausible(ADC_get_APPS1(), ADC_get_APPS2())) {
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
bool _PED_are_values_plausible(uint32_t val_1, uint32_t val_2) {
    /* Detect if shorted to GND */
    if (val_1 < 100 || val_2 < 100)
        return 0;

    /* Detect if shorted to Vcc */
    if (val_1 > 4050 || val_2 > 4050)
        return 0;

    // TODO: check if the above values make sense

    /* Find the normal offset between the two pots */
    int16_t pot_offset = abs(_PED_CALIB_APPS1_MIN - _PED_CALIB_APPS2_MIN);

    /* Calculate the maximum difference they can have */
    int16_t threshold = pot_offset * (PED_IMPL_THRESHOLD / 100.0f);

    /* Calcluate the current difference they have */
    int16_t diff = abs(val_2 - val_1);

    /* Check it's below the threshold */
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
    uint32_t brk_f, brk_r;
    get_brk_average(&brk_f, &brk_r);
    float bf_bar = ((brk_f / 4096.0 * 3.3 - 0.3) / 4.0) * 100.0;
    float br_bar = ((brk_r / 4096.0 * 3.3 - 0.35) / 4.0) * 100.0;

    ecumsg_pedal_throttle_state.data.throttle      = PED_get_accelerator_percent();
    ecumsg_pedal_throttle_state.info.is_new        = true;
    ecumsg_pedal_brakes_pressure_state.data.front  = bf_bar;
    ecumsg_pedal_brakes_pressure_state.data.rear   = br_bar;
    ecumsg_pedal_brakes_pressure_state.info.is_new = true;
}

void PED_calibrate(PED_CalibTypeDef calib) {
    switch (calib) {
        case PED_CALIB_APPS_MIN:
            _PED_CALIB_APPS1_MIN = ADC_get_APPS1();
            _PED_CALIB_APPS2_MIN = ADC_get_APPS2();
            break;
        case PED_CALIB_APPS_MAX:
            _PED_CALIB_APPS1_MAX = ADC_get_APPS1();
            _PED_CALIB_APPS2_MAX = ADC_get_APPS2();
            break;
        case PED_CALIB_BSE_MIN:
            LOG_write(LOGLEVEL_WARN, "[PED] Brake calibration not implemented");
            break;
        case PED_CALIB_BSE_MAX:
            LOG_write(LOGLEVEL_WARN, "[PED] Brake calibration not implemented");
            break;
    }
    pedals_config.dirty = true;
    if (EEPROM_config_write(&pedals_config)) {
        LOG_write(LOGLEVEL_INFO, "[PED] Calibration saved");
    } else {
        LOG_write(LOGLEVEL_WARN, "[PED] Calibration not saved");
    }
    PED_errors.implausibility_err = false;
    LOG_write(LOGLEVEL_INFO, "[PED] Calibration done");
}

bool PED_is_brake_ok() {
    // taking directly raw numbers are highly effected by noise
    // uint32_t brake_front = ADC_get_BRK_F();
    // uint32_t brake_rear = ADC_get_BRK_R();
    uint32_t brake_front, brake_rear;
    get_brk_average(&brake_front, &brake_rear);
    if (brake_front < 200 || brake_front > 3300)
        return 0;
    if (brake_rear < 200 || brake_rear > 3300)
        return 0;
    return 1;
}

#if PED_DEBUG
void PED_log_dbg_info() {
    LOG_write(
        LOGLEVEL_DEBUG,
        "PED/Cal  | %8s: %-6d %8s: %-6d %8s: %-6d %8s: %-6d",
        "ACC1 Max",
        _PED_CALIB_APPS1_MAX,
        "ACC1 Min",
        _PED_CALIB_APPS1_MIN,
        "ACC2 Max",
        _PED_CALIB_APPS2_MAX,
        "ACC2 Min",
        _PED_CALIB_APPS2_MIN);
    LOG_write(
        LOGLEVEL_DEBUG,
        "PED/ADCs | %8s: %-6d %8s: %-6.1f %8s: %-6d %8s: %-6.1f",
        "ACC1 Raw",
        ADC_get_APPS1(),
        "ACC1 %%",
        _PED_from_raw_to_percent(ADC_get_APPS1(), _PED_CALIB_APPS1_MIN, _PED_CALIB_APPS1_MAX),
        "ACC2 Raw",
        ADC_get_APPS2(),
        "ACC2 %%",
        _PED_from_raw_to_percent(ADC_get_APPS2(), _PED_CALIB_APPS2_MIN, _PED_CALIB_APPS2_MAX));
    LOG_write(LOGLEVEL_DEBUG, "PED/ADCs | %8s: %-6.4f", "BRK perc", PED_get_brake_bar());
    uint32_t brk_f, brk_r;
    get_brk_average(&brk_f, &brk_r);
    LOG_write(LOGLEVEL_DEBUG, "PED/ADCs | %8s: %-6d %8s: %-6d", "BRKF Raw", brk_f, "BRKR Raw", brk_r);
}
#endif