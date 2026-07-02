/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(RTD_BUZZER_GPIO_Port, RTD_BUZZER_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : SD_CLOSE_Pin */
    GPIO_InitStruct.Pin = SD_CLOSE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SD_CLOSE_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BRAKE_LIGHT_Pin */
    GPIO_InitStruct.Pin = BRAKE_LIGHT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BRAKE_LIGHT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : RTD_BUZZER_Pin */
    GPIO_InitStruct.Pin = RTD_BUZZER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RTD_BUZZER_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : TS_Button_Pin */
    GPIO_InitStruct.Pin = TS_Button_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(TS_Button_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 2 */

void gpio_set_brake_light(bool on) {
    HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool gpio_shutdown_closed(void) {
    return HAL_GPIO_ReadPin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin) == GPIO_PIN_SET;
}

void gpio_set_shutdown(bool closed) {
    HAL_GPIO_WritePin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin, closed ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool gpio_is_ts_button_pressed(void) {
    return HAL_GPIO_ReadPin(TS_Button_GPIO_Port, TS_Button_Pin) == GPIO_PIN_SET;
}

/* USER CODE END 2 */
