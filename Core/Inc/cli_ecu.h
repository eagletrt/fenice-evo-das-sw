#ifndef CLI_ECU_H
#define CLI_ECU_H

#include "cli.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#include "pedals.h"
#include "tim.h"
#include "timer_utils.h"
#include "adc_fsm.h"
#include "fsm.h"

// #define NORMAL_COLOR           "\033[0m"
// #define RED_BG(S)              "\033[0;41m" S NORMAL_COLOR
// #define YELLOW_BG(S)           "\033[0;43m" S NORMAL_COLOR
// #define CYAN_BG(S)             "\033[0;46m" S NORMAL_COLOR
// #define RED_BG_ON_YELLOW_FG(S) "\033[0;31;43m" S NORMAL_COLOR

extern cli_t cli_ecu;

void cli_ecu_init();

/**
 * @brief Print messages in the cli if dmesg is enabled
 */
void cli_ecu_debug(char *text);
void _cli_timer_handler(TIM_HandleTypeDef *htim);
void cli_watch_flush_handler();

#endif