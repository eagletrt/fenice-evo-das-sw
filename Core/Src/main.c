/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "brakelight.h"
#include "fsm.h"
#include "logger.h"
#include "pwm.h"
#include "buzzer.h"
#include "timer_utils.h"
#include "time_base.h"
#include "encoders.h"
#include "can_messages.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_DBG_BUF_LEN 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t _MAIN_last_loop_start_ms = 0;
VFSM_state_t vfsm_current_state = VFSM_STATE_INIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void _MAIN_print_dbg_line(char *title, char *txt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Redefine the weak function in logger.c to use the UART as textual output */
void _LOG_write_raw(char *txt) {
    HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt), 1000);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\033[K\r\n", 5, 100);
}

/* Call pwm-lib's callback upon TIM pulse-finished */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (&htim8 == htim)
      _pwm_tim_pulse_finished_handler(htim);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM13_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize logger */
  LOG_init(LOGLEVEL_DEBUG, true, true, false);

  /* Initialize PWM channels */
  pwm_set_period(&htim8, 1);
  pwm_set_duty_cicle(&htim8, TIM_CHANNEL_4, 0.5);

  /* Check for correct clock source */
  if (__HAL_RCC_GET_PLL_OSCSOURCE() != RCC_PLLSOURCE_HSE)
      LOG_write(LOGLEVEL_ERR, "The system clock is not using the external crystal, PORCODIO");

  
  LOG_write(LOGLEVEL_INFO, "\e[1;1H\e[2J");
  LOG_print_fenice_logo("            -    D A S   f i r m w a r e   v 2 . 0   -            ");

  /* Signal Startup */
  pwm_start_channel(&htim8, TIM_CHANNEL_4);
  HAL_Delay(500);
  pwm_stop_channel(&htim8, TIM_CHANNEL_4);

  /* Start encoders' timers */
  if (HAL_TIM_Encoder_Start(&ENC_L_TIM, TIM_CHANNEL_ALL) != HAL_OK) LOG_write(LOGLEVEL_ERR, "Timer start failed - TIM2");
  if (HAL_TIM_Encoder_Start(&ENC_R_TIM, TIM_CHANNEL_ALL) != HAL_OK) LOG_write(LOGLEVEL_ERR, "Timer start failed - TIM5");

  uint32_t last_enc_calc = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Initialize Brakelight */
  BKL_Init();

  /* Initilize time_base */
  HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim10, TIM_CHANNEL_1);
  time_base_init(&htim13);

  /* Shutdown circuit should already be open by default, but ensure it is */
  HAL_GPIO_WritePin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin, GPIO_PIN_RESET);
  
  /* Close the shutdown circuit */
  HAL_GPIO_WritePin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin, GPIO_PIN_SET);

  while (1)
  {
    /* Step forward the FSM */
    vfsm_current_state = VFSM_run_state(vfsm_current_state, NULL);

    /* Flush CAN TX queue */
    CANMSG_flush_TX();

    if(HAL_GetTick() - primary_INTERVAL_SPEED >= last_enc_calc) {
      last_enc_calc = HAL_GetTick();
      ENC_send_vals_in_CAN();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Record loop duration */
    HAL_Delay(1000);
    uint32_t loop_duration = HAL_GetTick() - _MAIN_last_loop_start_ms;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
  static uint64_t last_speed_sample = 0, last_angle_sample = 0;
  // if (htim == &htim1) {
  //   _MAIN_update_watchdog = true;
  //   BUZ_timer_callback();
  // } else 
  if(htim == &htim10){
      if ((true) && (get_time() - last_speed_sample > ENC_SPEED_PERIOD_MS)){
        ENC_R_push_speed_rads();
        ENC_L_push_speed_rads();
        last_speed_sample = get_time();
      }
      
      if ((true) && (get_time() - last_angle_sample > ENC_STEER_PERIOD_MS)){
        ENC_C_push_angle_deg();
        last_angle_sample = get_time();
      }
  } else if (htim == &htim13) {
      time_base_elapsed();
  } 
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
