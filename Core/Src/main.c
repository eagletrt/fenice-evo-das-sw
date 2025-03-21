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

#include "adc.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_fsm.h"
#include "brakelight.h"
#include "buzzer.h"
#include "can_messages.h"
#include "can_user_functions.h"
#include "cli_ecu.h"
#include "das_version.h"
#include "encoders.h"
#include "fsm.h"
#include "inverters.h"
#include "logger.h"
#include "pedals.h"
#include "pwm.h"
#include "stdio.h"
#include "steering_actuator.h"
#include "string.h"
#include "time_base.h"
#include "time_constraints.h"
#include "timer_utils.h"
#include "tractive_system.h"
#include "usart.h"

#include <inttypes.h>
#include <time.h>

#define _XOPEN_SOURCE
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG            0
#define MAIN_DBG_BUF_LEN 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t _MAIN_last_loop_start_ms   = 0;
uint32_t _MAIN_avg_loop_duration_ms = 0;
uint32_t _MAIN_last_ms_showed       = 0;

uint8_t _MAIN_dbg_uart_buf[MAIN_DBG_BUF_LEN];
bool _MAIN_is_dbg_uart_free               = true;
uint16_t _MAIN_dbg_uart_line_idx          = 0;
bool _MAIN_update_watchdog                = false; /* Every 10ms TIM1 sets this variable to true */
uint16_t _MAIN_timer_feedbacks            = 0;
bool _MAIN_last_tlm_status                = false;
volatile bool _MAIN_update_steer_actuator_pid   = false;
volatile bool _MAIN_update_steer_actuator_speed = false;

state_t current_state = STATE_INIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void _MAIN_print_dbg_line(char *title, char *txt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MAIN_print_dbg_info();
void _MAIN_process_ped_calib_msg();
void _update_ecu_feedbacks();

/* Redefine the weak function in logger.c to use the UART as textual output */
void _LOG_write_raw(char *txt) {
    HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\033[K\r\n", 5, 100);
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
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
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
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM4_Init();
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
    // LOG_print_fenice_logo("            -    D A S   f i r m w a r e   v 2 . 0
    // -            ");

    /* Signal Startup */
    pwm_start_channel(&htim8, TIM_CHANNEL_4);
    HAL_Delay(500);
    pwm_stop_channel(&htim8, TIM_CHANNEL_4);

    /* Start encoders' timers */
    if (HAL_TIM_Encoder_Start(&ENC_L_TIM, TIM_CHANNEL_ALL) != HAL_OK)
        LOG_write(LOGLEVEL_ERR, "Timer start failed - TIM2");
    if (HAL_TIM_Encoder_Start(&ENC_R_TIM, TIM_CHANNEL_ALL) != HAL_OK)
        LOG_write(LOGLEVEL_ERR, "Timer start failed - TIM5");

    /* Initialize the general purpose timer TIM1 to trigger every 10ms on CH2 */
    __HAL_TIM_SetAutoreload(&htim1, TIM_MS_TO_TICKS(&htim1, 10));
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);

    /* Initialize Brakelight */
    BKL_Init();

    /* Initilize time_base */
    HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(&htim10, TIM_CHANNEL_1);
    time_base_init(&htim13);
    HAL_TIM_Base_Start_IT(&htim7);

    /* Initialize the ADC capture loop */
    ADC_StartMuxCapure();

    /* Initialize pedals module */
    PED_init();

    /* Initialize CAN queues */
    CANMSG_init();

    /* Initialize watchdogs */
    WDG_init();

    /* Initialize CLI */
    // cli_ecu_init();

    uint32_t last_enc_calc = 0;

#if AS_STEER_ACTUATOR_ENABLED == 1
  //pid_parametersss
  #define OSC_PULSE 13.1 //rad/s
  #define KP_MAX 0.25
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  steer_actuator_pid_init(0.97 * KP_MAX, 0.1 * KP_MAX, 0.0, ENC_STEER_PERIOD_MS / 1000.0, 5.0);
#endif

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    /* Shutdown circuit should already be open by default, but ensure it is */
    HAL_GPIO_WritePin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin, GPIO_PIN_RESET);

    /* Close the shutdown circuit */
    HAL_GPIO_WritePin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin, GPIO_PIN_SET);

    struct tm timeinfo;
    strptime(__DATE__ " " __TIME__, "%b %d %Y %H:%M:%S", &timeinfo);
    ecumsg_ecu_version_state.data.canlib_build_time    = CANLIB_BUILD_TIME;
    ecumsg_ecu_version_state.data.component_build_time = mktime(&timeinfo);

    while (1) {
        _MAIN_last_loop_start_ms = HAL_GetTick();

        ecumsg_ecu_version_state.info.is_new = true;

        /* Step forward the FSM */
        current_state = run_state(current_state, NULL);

        /* Update debug information over UART */
#if DEBUG == 1
        MAIN_print_dbg_info();
#endif

        /* Iterate over inverter registers */
        INV_read_next_register();

        // float trq   = PED_get_accelerator_torque(PED_get_accelerator_percent());
        // float trq_2 = trq;
        // _INV_minimum_cell_voltage_limit(INV_get_RPM(INV_LEFT), INV_get_RPM(INV_RIGHT), &trq, &trq_2);

        if (ecumsg_ecu_set_power_maps_state.info.is_new) {
            ecumsg_ecu_set_power_maps_state.info.is_new = false;
            ecumsg_ecu_power_maps_state.data.map_power  = DAS_get_power_map();
            ecumsg_ecu_power_maps_state.data.sc_state   = DAS_get_sc_state();
            ecumsg_ecu_power_maps_state.data.tv_state   = DAS_get_tv_state();
            ecumsg_ecu_power_maps_state.data.reg_state  = DAS_get_reg_state();
        }
        ecumsg_ecu_power_maps_state.info.is_new = 1;

        /* Flush CAN TX queue */
        CANMSG_flush_TX();

        /* Parse all RX'd messages */
        CANMSG_process_RX_queue();

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        /* Update watchdog timers every 10ms and after 1s of settling time */
        if (_MAIN_update_watchdog && HAL_GetTick() > 1000)
            WDG_update_and_check_timestamps();

        /* Check for fatal errors and open the shutdown circuit */
        PED_update_plausibility_check();
        if (PED_errors.implausibility_err) {
            // HAL_GPIO_WritePin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin, GPIO_PIN_RESET);
            ecumsg_ecu_errors_state.data.error_pedal_implausibility = 1;
        } else {
            ecumsg_ecu_errors_state.data.error_pedal_implausibility = 0;
        }
        if (PED_errors.ADC_DMA_error || PED_errors.ADC_internal || PED_errors.ADC_overrun) {
            // HAL_GPIO_WritePin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin, GPIO_PIN_RESET);
            // ecumsg_ecu_errors_state.data.das_error_pedal_adc = 1;
        }
        // wait one second before doing the check (read some samples)
        if (HAL_GetTick() > 1000) {
            if (!PED_is_brake_ok()) {
                HAL_GPIO_WritePin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin, GPIO_PIN_RESET);
                ecumsg_ecu_errors_state.data.error_pedal_implausibility = 1;
            } else {
                // HAL_GPIO_WritePin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin, GPIO_PIN_SET);
                ecumsg_ecu_errors_state.data.error_pedal_implausibility = 0;
            }
        }

        /* Send ECU feedbacks */
        _update_ecu_feedbacks();
        ecumsg_ecu_errors_state.info.is_new = true;

        /* Send pedal and steer values to the steering wheel for visualization */
        PED_send_vals_in_CAN();

        /* Light up the brakelight if the pedal is pressed */
        float brk_percent = PED_get_brake_bar();
        BKL_set_curve(brk_percent);

        if (HAL_GetTick() - SECONDARY_FRONT_ANGULAR_VELOCITY_CYCLE_TIME_MS >= last_enc_calc) {
            last_enc_calc = HAL_GetTick();
            ENC_send_vals_in_CAN();
        }

        /* Check if we have PUSH TO TALK messages to process */
        if (ecumsg_ecu_set_ptt_status_state.info.is_new) {
            if (ecumsg_ecu_set_ptt_status_state.data.status != ecumsg_ecu_ptt_status_state.data.status) {
                if (ecumsg_ecu_set_ptt_status_state.data.status == primary_ecu_set_ptt_status_status_on) {
                    HAL_GPIO_WritePin(PTT_GPIO_Port, PTT_Pin, GPIO_PIN_SET);
                    ecumsg_ecu_ptt_status_state.data.status = primary_ecu_ptt_status_status_on;
                    ecumsg_ecu_ptt_status_state.info.is_new = true;
                } else if (ecumsg_ecu_set_ptt_status_state.data.status == primary_ecu_set_ptt_status_status_off) {
                    HAL_GPIO_WritePin(PTT_GPIO_Port, PTT_Pin, GPIO_PIN_RESET);
                    ecumsg_ecu_ptt_status_state.data.status = primary_ecu_ptt_status_status_off;
                    ecumsg_ecu_ptt_status_state.info.is_new = true;
                }
            }
            ecumsg_ecu_set_ptt_status_state.info.is_new = false;
        }


    if (ecumsg_tlm_status_state.info.is_new) {
      static int count = 0;
      static uint32_t last_ms = 0;
      if (ecumsg_tlm_status_state.data.status == primary_tlm_status_status_on) {
        if (!_MAIN_last_tlm_status) {
          if (count % 50 == 0) {
            if (count % 100 == 0) {
              pwm_stop_channel(&htim8, TIM_CHANNEL_4);
            } else {
              pwm_start_channel(&htim8, TIM_CHANNEL_4);
            }
          }
          if (count > 400) {
            count = 0;
            _MAIN_last_tlm_status = true;
          }
          if (HAL_GetTick() - last_ms > 1) {
            last_ms = HAL_GetTick();
            count++;
          }
        }
      } else {
        if (_MAIN_last_tlm_status) {
          if (count == 0) {
            pwm_start_channel(&htim8, TIM_CHANNEL_4);
          }
          if (count > 200) {
            pwm_stop_channel(&htim8, TIM_CHANNEL_4);
            count = 0;
            _MAIN_last_tlm_status = false;
          }
          if (HAL_GetTick() - last_ms > 1) {
            last_ms = HAL_GetTick();
            count++;
          }
        }
      } 
    }


        /* Record loop duration */
        uint32_t loop_duration     = HAL_GetTick() - _MAIN_last_loop_start_ms;
        _MAIN_avg_loop_duration_ms = loop_duration;

#if AS_STEER_ACTUATOR_ENABLED == 1
        steer_actuator_update_can();

        if (_MAIN_update_steer_actuator_pid) {
		    _MAIN_update_steer_actuator_pid = false;
		    steer_actuator_update_pid();
	    }

        if (_MAIN_update_steer_actuator_speed) {
            _MAIN_update_steer_actuator_speed = false;

            // const unsigned int period = 750; //ms
            // const float amplitude = 60.0;
            // float target = sin((float)(HAL_GetTick() % period) / period * 2 * M_PI) * amplitude;
            // steer_actuator_update_set_point(target);
            steer_actuator_update_speed();
        }

        // if (steer_actuator_is_enabled() && HAL_GetTick() - ecumsg_ecu_set_steer_actuator_angle_state.info.timestamp > 1000) {
        //         ecumsg_ecu_steek_actuator_status_state.data.status = primary_ecu_steer_actuator_status_status_off;
        //         steer_actuator_disable();
        // }

        ecumsg_as_commands_status_state.data.brakestatus = ecumsg_as_commands_set_status_state.data.brakestatus;
        ecumsg_as_commands_status_state.data.throttlestatus = ecumsg_as_commands_set_status_state.data.throttlestatus;
        static uint32_t last_steering_actuator_update = 0;
        if(HAL_GetTick() - last_steering_actuator_update > 100) {
            last_steering_actuator_update = HAL_GetTick();
            ecumsg_as_commands_status_state.info.is_new = true;
        }
#endif


#ifdef DEBUG_CLI
        cli_watch_flush_handler();
        cli_loop(&cli_ecu);
#endif
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 180;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 2;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Activate the Over-Drive mode
   */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
   */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/**
 * @brief     Read the CAN calibration message and call the PED module
 */
void _MAIN_process_ped_calib_msg() {
    /*
  PED_CalibTypeDef calib = 0U;
  primary_set_pedal_calibration_t *msg = &(CANMSG_SetPedalsCalibration.data);

  if (msg->pedal == primary_set_pedal_calibration_pedal_ACCELERATOR){
    if (msg->bound == primary_set_pedal_calibration_bound_SET_MAX)
      calib = PED_CALIB_APPS_MAX;
    else
      calib = PED_CALIB_APPS_MIN;
  }
  else {
    if (msg->pedal == primary_set_pedal_calibration_pedal_BRAKE)
      calib = PED_CALIB_BSE_MAX;
    else
      calib = PED_CALIB_BSE_MIN;
  }
  PED_calibrate(calib);
 */
}

/**
 * @brief     Update ECU Feedback
 */
void _update_ecu_feedbacks() {
    if (HAL_GetTick() - ecumsg_ecu_feedbacks_state.info.timestamp >= 100) {
        ecumsg_ecu_feedbacks_state.data.feedbacks_sd_cock_fb     = ADC_is_closed(ADC_to_voltage(ADC_get_SD_FB0()));
        ecumsg_ecu_feedbacks_state.data.feedbacks_sd_fb1         = ADC_is_closed(ADC_to_voltage(ADC_get_SD_FB1()));
        ecumsg_ecu_feedbacks_state.data.feedbacks_sd_bots_fb     = ADC_is_closed(ADC_to_voltage(ADC_get_SD_FB2()));
        ecumsg_ecu_feedbacks_state.data.feedbacks_sd_interial_fb = ADC_is_closed(ADC_to_voltage(ADC_get_SD_FB3()));
        ecumsg_ecu_feedbacks_state.data.feedbacks_sd_fb4         = ADC_is_closed(ADC_to_voltage(ADC_get_SD_FB4()));
        ecumsg_ecu_feedbacks_state.data.feedbacks_sd_in          = ADC_is_closed(ADC_to_voltage(ADC_get_SD_IN()));
        ecumsg_ecu_feedbacks_state.data.feedbacks_sd_out         = ADC_is_closed(ADC_to_voltage(ADC_get_SD_OUT()));
        ecumsg_ecu_feedbacks_state.data.feedbacks_sd_ctrl_pin    = HAL_GPIO_ReadPin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin);

        ecumsg_ecu_feedbacks_state.info.is_new = true;
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint64_t last_speed_sample = 0, last_angle_sample = 0, last_steering_actuator_update = 0;
    if (htim->Instance == htim1.Instance) {
        _MAIN_update_watchdog = true;
        BUZ_timer_callback();
    } else if (htim->Instance == htim10.Instance) {
        if ((true) && (get_time() - last_speed_sample > ENC_SPEED_PERIOD_MS)) {
            ENC_R_push_speed_rads();
            ENC_L_push_speed_rads();
            last_speed_sample = get_time();
        }

        if ((true) && (get_time() - last_angle_sample > ENC_STEER_PERIOD_MS)) {
            ENC_C_push_angle_deg();
#if AS_STEER_ACTUATOR_ENABLED == 1
            _MAIN_update_steer_actuator_pid = true;
#endif
            last_angle_sample = get_time();
        }

        if (get_time() - last_steering_actuator_update > STEERING_ACTUATOR_PERIOD_MS) {
#if AS_STEER_ACTUATOR_ENABLED == 1
            _MAIN_update_steer_actuator_speed = true;
#endif
            last_steering_actuator_update = get_time();
        }

    } else if (htim->Instance == htim13.Instance) {
        time_base_elapsed();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim6.Instance) {
        _cli_timer_handler(htim);
    } else if (htim->Instance == htim7.Instance) {
        HAL_ADC_Start_DMA(&hadc1, _ADC_raw_reads + _ADC_curr_ch, 1);
    }
}

void _MAIN_print_dbg_line(char *title, char *txt) {
    uint16_t n_bytes = snprintf(_MAIN_dbg_uart_buf, MAIN_DBG_BUF_LEN, "%10s | %s\033[K\r\n", title, txt);

    _MAIN_is_dbg_uart_free = false;
    HAL_UART_Transmit_IT(&huart2, _MAIN_dbg_uart_buf, n_bytes);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2)
        _MAIN_is_dbg_uart_free = true;
}

void MAIN_print_dbg_info() {
    if (!_MAIN_is_dbg_uart_free)
        return;

    if (_MAIN_dbg_uart_line_idx == 0 && (HAL_GetTick() - _MAIN_last_ms_showed < 200))
        return;
    _MAIN_last_ms_showed = HAL_GetTick();

    uint8_t buf_len = 150;
    char buf[buf_len];

    switch (_MAIN_dbg_uart_line_idx) {
        case 0:
            LOG_print_fenice_logo("            -    D A S   f i r m w a r e   v 2 . 0   -            ");
            break;
        case 1:
            snprintf(buf, buf_len, "%8s: %-6d %8s: %8s %8s: %12s", "Code", INT_COMPONENT_VERSION, "Time", __TIME__, "Date", __DATE__);
            _MAIN_print_dbg_line("BUILD", buf);
            break;
        case 2:
            snprintf(buf, buf_len, "%8s: %-3ld ms", "Loop len", _MAIN_avg_loop_duration_ms);
            _MAIN_print_dbg_line("MAIN", buf);
            break;
        case 3:
            snprintf(
                buf,
                buf_len,
                " %8s: %d %8s: %d %8s: %d %8s: %d %8s: %d ",
                "fsm",
                ecumsg_ecu_errors_state.data.error_fsm,
                "imu",
                ecumsg_ecu_errors_state.data.error_imu_tout,
                "invl",
                ecumsg_ecu_errors_state.data.error_invl_tout,
                "invr",
                ecumsg_ecu_errors_state.data.error_invr_tout,
                "irts",
                ecumsg_ecu_errors_state.data.error_irts_tout);
            _MAIN_print_dbg_line("", buf);
            break;
        case 4:
            snprintf(
                buf,
                buf_len,
                " %8s: %d %8s: %d %8s: %d %8s: %d",
                "ped adc",
                ecumsg_ecu_errors_state.data.error_pedal_adc,
                "ped imp",
                ecumsg_ecu_errors_state.data.error_pedal_implausibility,
                "steer",
                ecumsg_ecu_errors_state.data.error_steer_tout,
                "ts err",
                ecumsg_ecu_errors_state.data.error_ts_tout);
            _MAIN_print_dbg_line("", buf);
            break;

        case 5:
            snprintf(
                buf,
                buf_len,
                "%26s: %-3.1fV %10s: %-3.1fV %14s: %-3.1fV %25s: %-3.1fV",
                "SD_FB0 (Cockpit Mushroom)",
                ADC_to_voltage(ADC_get_SD_FB0()),
                "SD_FB1",
                ADC_to_voltage(ADC_get_SD_FB1()),
                "SD_FB2 (BOTS)",
                ADC_to_voltage(ADC_get_SD_FB2()),
                "SD_FB3 (Inertial switch)",
                ADC_to_voltage(ADC_get_SD_FB3()));
            _MAIN_print_dbg_line("SD", buf);
            break;
        case 6:
            snprintf(
                buf,
                buf_len,
                "%8s: %-3.1fV %10s: %-3.1fV %10s: %-3.1fV %12s: %d %4s: %d",
                "SD_FB4",
                ADC_to_voltage(ADC_get_SD_FB4()),
                "SD_IN",
                ADC_to_voltage(ADC_get_SD_IN()),
                "SD_OUT",
                ADC_to_voltage(ADC_get_SD_OUT()),
                "SD CTRL PIN",
                HAL_GPIO_ReadPin(SD_CLOSE_GPIO_Port, SD_CLOSE_Pin),
                "TOT",
                is_SD_closed());
            _MAIN_print_dbg_line("", buf);
            break;
        case 7:
            snprintf(buf, buf_len, "%8s: %-4.1f%%", "Err rate", CAN_error_rate * 100);
            _MAIN_print_dbg_line("CAN", buf);
            break;
        case 8:
            snprintf(buf, buf_len, "%8s: %-6s", "State", state_names[current_state]);
            _MAIN_print_dbg_line("FSM", buf);
            break;
        case 9:
            //   snprintf(buf, buf_len, "%8s: %-6s %8s: 0x%06X %8s: 0x%06X",
            //     "Status", TS_state_names[TS_get_status()], "Errors",
            //     ecumsg_hv_errors_state.data.errors_can, "Warns",
            //     ecumsg_hv_errors_state.data.errors_can);
            //   _MAIN_print_dbg_line("BMS-HV", buf);
            break;
        case 10:
            snprintf(
                buf,
                buf_len,
                "%8s: %-6d %8s: %-6d %8s: %-6d %8s: %-6s",
                "-",
                0,
                "FRG",
                INV_get_FRG_state(INV_LEFT),
                "RFE",
                INV_get_RFE_state(INV_LEFT),
                "RUN",
                "N/A");
            _MAIN_print_dbg_line("INV/L", buf);
            break;
        case 11:
            snprintf(
                buf,
                buf_len,
                "%8s: %-6d %8s: %-6.1f %8s: %-6.1f %8s: %-6.1f",
                "DriveEna",
                INV_is_drive_enabled(INV_LEFT),
                "RPM",
                INV_get_RPM(INV_LEFT),
                "Mot T",
                INV_get_motor_temp(INV_LEFT),
                "IGBT T",
                INV_get_IGBT_temp(INV_LEFT));
            _MAIN_print_dbg_line("", buf);
            break;
        case 12:
            snprintf(
                buf,
                buf_len,
                "%8s: %-6d %8s: %-6d %8s: %-6d %8s: %-6s",
                "-",
                0,
                "FRG",
                INV_get_FRG_state(INV_RIGHT),
                "RFE",
                INV_get_RFE_state(INV_RIGHT),
                "RUN",
                "N/A");
            _MAIN_print_dbg_line("INV/R", buf);
            break;
        case 13:
            snprintf(
                buf,
                buf_len,
                "%8s: %-6d %8s: %-6.1f %8s: %-6.1f %8s: %-6.1f",
                "DriveEna",
                INV_is_drive_enabled(INV_RIGHT),
                "RPM",
                INV_get_RPM(INV_RIGHT),
                "Mot T",
                INV_get_motor_temp(INV_RIGHT),
                "IGBT T",
                INV_get_IGBT_temp(INV_RIGHT));
            _MAIN_print_dbg_line("", buf);
            break;
        case 14:
        //   snprintf(buf, buf_len, "%8s: %-6d %8s: %-6d %8s: %-6d %8s: %-6d",
        //     "Encoder", INV_L_get_errors()->encoder_error, "No pwr v",
        //     INV_L_get_errors()->no_pwr_voltage, "Mot temp",
        //     INV_L_get_errors()->hi_motor_temp, "Dev temp",
        //     INV_L_get_errors()->hi_device_temp);
        //   _MAIN_print_dbg_line("INV/L Errs", buf);
        //   break;
        case 15:
            //   snprintf(buf, buf_len, "%8s: %-6d %8s: %-6d %8s: %-6d %8s: %-6d",
            //     "Encoder", INV_R_get_errors()->encoder_error, "No pwr v",
            //     INV_R_get_errors()->no_pwr_voltage, "Mot temp",
            //     INV_R_get_errors()->hi_motor_temp, "Dev temp",
            //     INV_R_get_errors()->hi_device_temp);
            //   _MAIN_print_dbg_line("INV/R Errs", buf);
            //   break;
            //   _MAIN_print_dbg_line("---",
            //   "----------------------------------------");
            break;
        case 16:
#if PED_DEBUG
            PED_log_dbg_info();
#endif
            snprintf(
                buf,
                buf_len,
                "%8s: %-5.1f%% %8s: %-5.1f%% %8s: %-5.1f%%",
                "APPS",
                PED_get_accelerator_percent(),
                "Brake/F",
                PED_get_brake_bar(),
                "Brake/R",
                0.0f);
            _MAIN_print_dbg_line("PED", buf);
            break;
        case 17:
            snprintf(
                buf,
                buf_len,
                "%8s: %-6d %8s: %-6d %8s: %-6d %8s: %-6d",
                "ADC/HW",
                PED_errors.ADC_internal,
                "ADC/OVR",
                PED_errors.ADC_overrun,
                "ADC/DMA",
                PED_errors.ADC_DMA_error,
                "Impl",
                PED_errors.implausibility_err);
            _MAIN_print_dbg_line("PED Errs", buf);
            break;
        case 18:
            snprintf(
                buf,
                buf_len,
                "%8s: %-6.3f %8s: %-6.1f %8s: %-6.1f",
                "Steer",
                ENC_C_get_angle_deg(),
                "W/L",
                ENC_L_get_radsec(),
                "W/R",
                ENC_R_get_radsec());
            _MAIN_print_dbg_line("ENC", buf);
            break;
        case 19:
            //   snprintf(buf, buf_len, "%8s: %-6.1f %8s: %-6.1f %8s: %-6.1f %8s:
            //   %-6.1f",
            //     "TTLL", CTRL_get_torque_L(), "TTRR", CTRL_get_torque_R(), "Est.V.",
            //     CTRL_get_vest(), "AvgDelay", CTRL_avg_wait_ms);
            //   _MAIN_print_dbg_line("CTRL", buf);
            break;
        case 20:
            // snprintf(buf, buf_len, "%8s: %-6.1f %8s: %-6.1f %8s: %-6.1f",
            //   "X", CANMSG_IMUAcc.data.accel_x, "Y", CANMSG_IMUAcc.data.accel_y,
            //   "Z", CANMSG_IMUAcc.data.accel_z);
            // _MAIN_print_dbg_line("IMU/Acc", buf);
            break;
        case 21:
            // snprintf(buf, buf_len, "%8s: %-6.1f %8s: %-6.1f %8s: %-6.1f",
            //   "X", CANMSG_IMUAng.data.ang_rate_x, "Y",
            //   CANMSG_IMUAng.data.ang_rate_y, "Z", CANMSG_IMUAng.data.ang_rate_z);
            // _MAIN_print_dbg_line("IMU/Ang", buf);
            break;
        default:
            LOG_write(LOGLEVEL_ERR, "Overrun in line index when printing debug info");
            break;
    }

    _MAIN_dbg_uart_line_idx = (_MAIN_dbg_uart_line_idx + 1) % 21;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
