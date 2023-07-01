#include "adc.h"
#include "adc_fsm.h"
#include "logger.h"


/** Mux channels. Encoded as LSB! */
/** SD is close when raw value is grater then 1700
 *  voltage on mux = raw * alimentation_voltage (3.3V) / 4095
 *  due to voltage divider, voltage on mux = raw * alimentation_voltage (3.3V) / 4095 * 9
 *  AD is on 12 bits, so 4095 is the max value
*/
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
    ADC_CH_SD_FB0,
    ADC_CH_SD_FB1,
    ADC_CH_SD_FB3,
    ADC_CH_SD_FB2,
    ADC_CH_SD_OUT,
    ADC_CH_SD_IN,
    ADC_NUM_CHANNELS
} _ADC_ChannelTypeDef;

_ADC_ChannelTypeDef _ADC_curr_ch = ADC_CH_APPS1;
uint32_t _ADC_raw_reads[ADC_NUM_CHANNELS] = {0};


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
    HAL_ADC_Start_DMA(&hadc1, _ADC_raw_reads + _ADC_curr_ch, 1);
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