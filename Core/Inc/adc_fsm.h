#ifndef ADC_FSM_H
#define ADC_FSM_H

#include "stdint.h"
#include "stdbool.h"


bool ADC_StartMuxCapure();

uint32_t ADC_get_APPS1();
uint32_t ADC_get_APPS2();
uint32_t ADC_get_BPPS1();
uint32_t ADC_get_BPPS2();
uint32_t ADC_get_BRK_F();
uint32_t ADC_get_BRK_R();
uint32_t ADC_get_SD_FB4();
uint32_t ADC_get_SD_FB0();
uint32_t ADC_get_SD_FB1();
uint32_t ADC_get_SD_FB3();
uint32_t ADC_get_SD_FB2();
uint32_t ADC_get_SD_OUT();
uint32_t ADC_get_SD_IN();
uint32_t ADC_get_PITOT();

float ADC_to_voltage(uint32_t raw);
unsigned int ADC_is_closed(float voltage);
#endif