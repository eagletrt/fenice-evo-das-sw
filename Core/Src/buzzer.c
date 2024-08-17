#include "buzzer.h"

#include "logger.h"
#include "pwm.h"
#include "tim.h"

uint32_t _BUZ_beep_start_ms    = 0;
uint32_t _BUZ_beep_duration_ms = 0;

void BUZ_beep_ms_sync(uint16_t ms) {
    BUZ_beep_ms_async(ms);
    HAL_Delay(ms);
    HAL_GPIO_WritePin(RTD_BUZZER_GPIO_Port, RTD_BUZZER_Pin, RESET);
}

void BUZ_beep_ms_async(uint16_t ms) {
    HAL_GPIO_WritePin(RTD_BUZZER_GPIO_Port, RTD_BUZZER_Pin, SET);
    _BUZ_beep_start_ms    = HAL_GetTick();
    _BUZ_beep_duration_ms = ms;
}

void BUZ_timer_callback() {
    if ((HAL_GetTick() - _BUZ_beep_start_ms) >= _BUZ_beep_duration_ms) {
        HAL_GPIO_WritePin(RTD_BUZZER_GPIO_Port, RTD_BUZZER_Pin, RESET);
        _BUZ_beep_start_ms    = 0;
        _BUZ_beep_duration_ms = 0;
    }
}
