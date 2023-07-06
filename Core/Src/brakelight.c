#include "brakelight.h"
#include "pwm.h"


void _BKL_set_ch_pwm(float, uint32_t);

void BKL_Init(){
    // autoreload register is set to 255 for 14ms period in cubemx
    // pwm_set_period(&BKL_TIM, 1.0f);
    pwm_start_channel(&BKL_TIM,TIM_CHANNEL_1);
    pwm_start_channel(&BKL_TIM,TIM_CHANNEL_2);
    pwm_start_channel(&BKL_TIM,TIM_CHANNEL_3);
}

void BKL_set_state(BKL_StateTypeDef state) {
    float dc = 0.0f;

    switch (state) {
        case BKL_OFF:
            dc=0;
            break;
        case BKL_LVL_1:
            dc = 0.3f;
            break;
        case BKL_LVL_2:
            dc = 0.5f;
            break;
        case BKL_LVL_3:
            dc = 1.0f;
            break;
        default:
            dc = 0.5f;
            break;
    }

    _BKL_set_ch_pwm(dc, TIM_CHANNEL_1);
    _BKL_set_ch_pwm(dc, TIM_CHANNEL_2);
    _BKL_set_ch_pwm(dc, TIM_CHANNEL_3);
}

void BKL_set_curve(float brk_percent) {
    static float dc=0;

    if (brk_percent > 4.0f){
      dc=1.0;
    } else if (brk_percent < 1.5f){
      dc=0;
    }
    _BKL_set_ch_pwm(dc, TIM_CHANNEL_1);
    _BKL_set_ch_pwm(dc, TIM_CHANNEL_2);
    _BKL_set_ch_pwm(dc, TIM_CHANNEL_3);
}

void _BKL_set_ch_pwm(float duty_cycle, uint32_t ch) {
    pwm_set_duty_cicle(&BKL_TIM, ch, duty_cycle);
}

void BKL_light_show_step() {
    float dts[] = { 0.03f, 0.06f, 0.12f, 0.25f, 0.5f, 1.0f, 0.5f, 0.25f, 0.12f, 0.06f , 0.03f };
    uint8_t chs[] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3 };
    static uint32_t timestamp = 0;
    static uint8_t dc_idx = 0;
    static uint8_t ch_offset = 0;

    if ((HAL_GetTick() - timestamp) < 100)
        return;

    timestamp = HAL_GetTick();
    dc_idx = (dc_idx + 1) % 11;
    ch_offset = (ch_offset + 1) % 3;

    _BKL_set_ch_pwm(dts[dc_idx], chs[(ch_offset+0) % 3]);
    _BKL_set_ch_pwm(0.0f, chs[(ch_offset+1) % 3]);
    _BKL_set_ch_pwm(0.0f, chs[(ch_offset+2) % 3]);
}