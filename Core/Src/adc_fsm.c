#include "adc.h"
#include "adc_fsm.h"

typedef enum {
    ADC_CH_PITOT = 0,
    ADC_CH_APPS1,
    ADC_CH_APPS2,
    ADC_CH_BPPS1,
    ADC_CH_BPPS2,
    ADC_CH_BRK_F,
    ADC_CH_BRK_R,
    ADC_NUM_CHANNELS
} _ADC_ChannelTypeDef;

_ADC_ChannelTypeDef _ADC_curr_ch = ADC_CH_APPS1;
uint8_t _ADC_raw_reads[ADC_NUM_CHANNELS] = {0};


bool ADC_StartMux() {
    /* Begin DMA stream with the ADC */
    HAL_ADC_Start_DMA(NULL, _ADC_raw_reads[_ADC_curr_ch], 1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    /* Increment ADC channel */
    _ADC_curr_ch = (_ADC_curr_ch + 1) % ADC_NUM_CHANNELS;
    HAL_ADC_Start_DMA(NULL, _ADC_raw_reads[_ADC_curr_ch], 1);
}

uint8_t ADC_get_APPS1() {
    return _ADC_raw_reads[ADC_CH_APPS1];
}

uint8_t ADC_get_APPS2() {
    return _ADC_raw_reads[ADC_CH_APPS2];
}

uint8_t ADC_get_BPPS1() {
    return _ADC_raw_reads[ADC_CH_BPPS1];
}

uint8_t ADC_get_BPPS2() {
    return _ADC_raw_reads[ADC_CH_BPPS2];
}

uint8_t ADC_get_BRK_F() {
    return _ADC_raw_reads[ADC_CH_BRK_F];
}

uint8_t ADC_get_BRK_R() {
    return _ADC_raw_reads[ADC_CH_BRK_R];
}