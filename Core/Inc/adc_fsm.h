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

#endif