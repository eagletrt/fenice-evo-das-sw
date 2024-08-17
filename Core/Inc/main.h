/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MUX_0_Pin                  GPIO_PIN_13
#define MUX_0_GPIO_Port            GPIOC
#define MUX_1_Pin                  GPIO_PIN_14
#define MUX_1_GPIO_Port            GPIOC
#define MUX_2_Pin                  GPIO_PIN_15
#define MUX_2_GPIO_Port            GPIOC
#define MUX_3_Pin                  GPIO_PIN_0
#define MUX_3_GPIO_Port            GPIOC
#define STEER_ACT_CURR_Pin         GPIO_PIN_1
#define STEER_ACT_CURR_GPIO_Port   GPIOC
#define CS_EEPROM_Pin              GPIO_PIN_4
#define CS_EEPROM_GPIO_Port        GPIOC
#define SD_CLOSE_Pin               GPIO_PIN_1
#define SD_CLOSE_GPIO_Port         GPIOB
#define PTT_Pin                    GPIO_PIN_2
#define PTT_GPIO_Port              GPIOB
#define BRAKE_LIGHT_CH1_Pin        GPIO_PIN_6
#define BRAKE_LIGHT_CH1_GPIO_Port  GPIOC
#define BRAKE_LIGHT_CH2_Pin        GPIO_PIN_7
#define BRAKE_LIGHT_CH2_GPIO_Port  GPIOC
#define BRAKE_LIGHT_CH3_Pin        GPIO_PIN_8
#define BRAKE_LIGHT_CH3_GPIO_Port  GPIOC
#define AUX_BUZZER_Pin             GPIO_PIN_9
#define AUX_BUZZER_GPIO_Port       GPIOC
#define RTD_BUZZER_Pin             GPIO_PIN_8
#define RTD_BUZZER_GPIO_Port       GPIOA
#define STEERING_REVERSE_Pin       GPIO_PIN_4
#define STEERING_REVERSE_GPIO_Port GPIOB
#define STEERING_PWM_Pin           GPIO_PIN_7
#define STEERING_PWM_GPIO_Port     GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
