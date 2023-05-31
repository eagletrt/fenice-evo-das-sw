#ifndef ADC_FSM_H
#define ADC_FSM_H

#include "stdint.h"
#include "stdbool.h"


bool ADC_StartMux();

uint8_t ADC_get_APPS1();
uint8_t ADC_get_APPS2();
uint8_t ADC_get_BPPS1();
uint8_t ADC_get_BPPS2();
uint8_t ADC_get_BRK_F();
uint8_t ADC_get_BRK_R();


#endif