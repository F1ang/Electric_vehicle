/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

#include "stdint.h"
#include "arm_math.h"
#include "math.h"
#include "stdio.h"
#include "stdarg.h"
#include "SEGGER_RTT.h"

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
#define PWM_PERIOD_CYCLES ADV_TIM_CLK_MHz*1000000/PWM_FREQUENCY
#define M1_ENCODER_PPR 1000
#define PWM_FREQUENCY 16000
#define M1_HALL_IC_FILTER 15
#define DEADTIME_NS SW_DEADTIME_NS
#define TIM_CLOCK_DIVIDER 1
#define M1_ENC_IC_FILTER 6
#define SW_DEADTIME_NS 30
#define M1_PULSE_NBR (4*(M1_ENCODER_PPR)-1)
#define DEAD_TIME_COUNTS DEAD_TIME_COUNTS_1
#define DEAD_TIME_COUNTS_1 DEAD_TIME_ADV_TIM_CLK_MHz*DEADTIME_NS/1000
#define DEAD_TIME_ADV_TIM_CLK_MHz 168
#define M1_HALL_TIM_PERIOD 65535
#define ADV_TIM_CLK_MHz 168/TIM_CLOCK_DIVIDER
#define LIN_A_Pin GPIO_PIN_13
#define LIN_A_GPIO_Port GPIOB
#define LIN_B_Pin GPIO_PIN_14
#define LIN_B_GPIO_Port GPIOB
#define LIN_C_Pin GPIO_PIN_15
#define LIN_C_GPIO_Port GPIOB
#define HALL_C_Pin GPIO_PIN_6
#define HALL_C_GPIO_Port GPIOC
#define HALL_B_Pin GPIO_PIN_7
#define HALL_B_GPIO_Port GPIOC
#define HALL_A_Pin GPIO_PIN_8
#define HALL_A_GPIO_Port GPIOC
#define HIN_A_Pin GPIO_PIN_8
#define HIN_A_GPIO_Port GPIOA
#define HIN_B_Pin GPIO_PIN_9
#define HIN_B_GPIO_Port GPIOA
#define HIN_C_Pin GPIO_PIN_10
#define HIN_C_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define LED1(i) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, i)
#define LED2(i) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, i)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
