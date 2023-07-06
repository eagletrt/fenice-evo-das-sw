#ifndef ADC_FSM_H
#define ADC_FSM_H

#include "stdint.h"
#include "stdbool.h"
#define BRK_MED_SIZE 20

typedef enum {
    ADC_CH_PITOT = 0, /*< 0000 */
    ADC_CH_APPS2,     /*< 1000 */
    ADC_CH_APPS1,     /*< 0100 */
    ADC_CH_BPPS2,     /*< 1100 */
    ADC_CH_BPPS1,     /*< 0010 */
    ADC_CH_BRK_F,     /*< 0001 */
    ADC_CH_BRK_R,     /*< 1001 */
    ADC_CH_SEVEN,
    ADC_CH_EIGHT,
    ADC_CH_SD_FB4,
    ADC_CH_SD_FB0,    /* Cockpit Mushroom */
    ADC_CH_SD_FB1,
    ADC_CH_SD_FB3,    /* Inertial switch */
    ADC_CH_SD_FB2,    /* BOTS */
    ADC_CH_SD_OUT,
    ADC_CH_SD_IN,
    ADC_NUM_CHANNELS
} _ADC_ChannelTypeDef;

/** Mux channels. Encoded as LSB! */
/** SD is close when raw value is grater then 1700
 *  voltage on mux = raw * alimentation_voltage (3.3V) / 4095
 *  due to voltage divider, voltage on mux = raw * alimentation_voltage (3.3V) / 4095 * 9
 *  AD is on 12 bits, so 4095 is the max value
*/

extern _ADC_ChannelTypeDef _ADC_curr_ch;
extern uint32_t _ADC_raw_reads[];


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

void brk_push_average();

void get_brk_average(uint32_t *brk_f, uint32_t *brk_r);

float ADC_to_voltage(uint32_t raw);
unsigned int ADC_is_closed(float voltage);
#endif