#include "adc.h"
#include "adc_fsm.h"
#include "logger.h"
#include <float.h>

_ADC_ChannelTypeDef _ADC_curr_ch = ADC_CH_APPS1;
uint32_t _ADC_raw_reads[ADC_NUM_CHANNELS] = {0};
uint32_t brk_f_median_window[BRK_MED_SIZE] = {};
uint32_t brk_r_median_window[BRK_MED_SIZE] = {};


void _ADC_set_mux_channel(uint8_t ch) {
    HAL_GPIO_WritePin(MUX_0_GPIO_Port, MUX_0_Pin, (ch & 0x01) << 0);
    HAL_GPIO_WritePin(MUX_1_GPIO_Port, MUX_1_Pin, (ch & 0x02) << 1);
    HAL_GPIO_WritePin(MUX_2_GPIO_Port, MUX_2_Pin, (ch & 0x04) << 2);
    HAL_GPIO_WritePin(MUX_3_GPIO_Port, MUX_3_Pin, (ch & 0x08) << 3);
    // LOG_write(LOGLEVEL_DEBUG, "MUX: %d", ch);
    // LOG_write(LOGLEVEL_DEBUG, "MUX0=%d, MUX1=%d, MUX2=%d, MUX3=%d", 
    //     HAL_GPIO_ReadPin(MUX_0_GPIO_Port, MUX_0_Pin),
    //     HAL_GPIO_ReadPin(MUX_1_GPIO_Port, MUX_1_Pin),
    //     HAL_GPIO_ReadPin(MUX_2_GPIO_Port, MUX_2_Pin),
    //     HAL_GPIO_ReadPin(MUX_3_GPIO_Port, MUX_3_Pin)
    // );
}

bool ADC_StartMuxCapure() {
    /* Begin DMA stream with the ADC */
    if (HAL_ADC_Start_DMA(&hadc1, _ADC_raw_reads +_ADC_curr_ch, 1) != HAL_OK) {
        LOG_write(LOGLEVEL_ERR, "ADC DMA start failed");
        return true;
    }
    return false;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    /* Increment ADC channel */
    _ADC_curr_ch = (_ADC_curr_ch + 1) % ADC_NUM_CHANNELS;
    _ADC_set_mux_channel(_ADC_curr_ch);
    brk_push_average();
}

uint32_t ADC_get_APPS1() {
    return _ADC_raw_reads[ADC_CH_APPS1];
}

uint32_t ADC_get_APPS2() {
    return _ADC_raw_reads[ADC_CH_APPS2];
}

uint32_t ADC_get_BPPS1() {
    return _ADC_raw_reads[ADC_CH_BPPS1];
}

uint32_t ADC_get_BPPS2() {
    return _ADC_raw_reads[ADC_CH_BPPS2];
}

uint32_t ADC_get_BRK_F() {
    return _ADC_raw_reads[ADC_CH_BRK_F];
}

uint32_t ADC_get_BRK_R() {
    return _ADC_raw_reads[ADC_CH_BRK_R];
}

uint32_t ADC_get_SD_FB4() {
    return _ADC_raw_reads[ADC_CH_SD_FB4];
}

uint32_t ADC_get_SD_FB0() {
    return _ADC_raw_reads[ADC_CH_SD_FB0];
}

uint32_t ADC_get_SD_FB1() {
    return _ADC_raw_reads[ADC_CH_SD_FB1];
}

uint32_t ADC_get_SD_FB3() {
    return _ADC_raw_reads[ADC_CH_SD_FB3];
}

uint32_t ADC_get_SD_FB2() {
    return _ADC_raw_reads[ADC_CH_SD_FB2];
}

uint32_t ADC_get_SD_OUT() {
    return _ADC_raw_reads[ADC_CH_SD_OUT];
}

uint32_t ADC_get_SD_IN() {
    return _ADC_raw_reads[ADC_CH_SD_IN];
}

uint32_t ADC_get_PITOT() {
    return _ADC_raw_reads[ADC_CH_PITOT];
}

float ADC_to_voltage(uint32_t raw) {
    return ((raw * 3.3) / (float)4095)*(9.0);
}

unsigned int ADC_is_closed(float voltage) {
    return voltage > 9.0;
}

void brk_push_average() {
    for (int i = 0; i < BRK_MED_SIZE - 1; i++) {
        brk_f_median_window[i] = brk_f_median_window[i + 1];
    }
    brk_f_median_window[BRK_MED_SIZE - 1] = ADC_get_BRK_F();

    for (int i = 0; i < BRK_MED_SIZE - 1; i++) {
        brk_r_median_window[i] = brk_r_median_window[i + 1];
    }
    brk_r_median_window[BRK_MED_SIZE - 1] = ADC_get_BRK_R();
}

void get_brk_average(uint32_t *brk_f, uint32_t *brk_r) {
    float min_f = FLT_MAX, max_f = FLT_MIN, median_f = 0;
    float min_r = FLT_MAX, max_r = FLT_MIN, median_r = 0;

    for (int i = 0; i < BRK_MED_SIZE; i++){
        if (brk_f_median_window[i] < min_f){
            min_f = brk_f_median_window[i];
        }
        if (brk_f_median_window[i] > max_f){
            max_f = brk_f_median_window[i];
        }
        median_f += brk_f_median_window[i];
    }
    
    for (int i = 0; i < BRK_MED_SIZE; i++){
        if (brk_r_median_window[i] < min_r){
            min_r = brk_r_median_window[i];
        }
        if (brk_r_median_window[i] > max_r){
            max_r = brk_r_median_window[i];
        }
        median_r += brk_r_median_window[i];
    }
    median_f = median_f - min_f - max_f;
    median_r = median_r - min_r - max_r;
    
    *brk_f = median_f / (BRK_MED_SIZE - 2);
    *brk_r = median_r / (BRK_MED_SIZE - 2);
}