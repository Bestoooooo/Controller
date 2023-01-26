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
#include "stm32f3xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CPread_Pin GPIO_PIN_0
#define CPread_GPIO_Port GPIOA
#define STATUS_Pin GPIO_PIN_1
#define STATUS_GPIO_Port GPIOA
#define AC_Voltage_Pin GPIO_PIN_4
#define AC_Voltage_GPIO_Port GPIOA
#define CHG_Pin GPIO_PIN_5
#define CHG_GPIO_Port GPIOA
#define ERR_Pin GPIO_PIN_6
#define ERR_GPIO_Port GPIOA
#define PP_Pin GPIO_PIN_7
#define PP_GPIO_Port GPIOA
#define Current1_Pin GPIO_PIN_0
#define Current1_GPIO_Port GPIOB
#define Current2_Pin GPIO_PIN_1
#define Current2_GPIO_Port GPIOB
#define Current4_Pin GPIO_PIN_2
#define Current4_GPIO_Port GPIOB
#define DIP_SW3_Pin GPIO_PIN_10
#define DIP_SW3_GPIO_Port GPIOB
#define DIP_SW4_Pin GPIO_PIN_11
#define DIP_SW4_GPIO_Port GPIOB
#define DIP_SW5_Pin GPIO_PIN_12
#define DIP_SW5_GPIO_Port GPIOB
#define DIP_SW6_Pin GPIO_PIN_13
#define DIP_SW6_GPIO_Port GPIOB
#define DIP_SW7_Pin GPIO_PIN_14
#define DIP_SW7_GPIO_Port GPIOB
#define DIP_SW8_Pin GPIO_PIN_15
#define DIP_SW8_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_8
#define EN_GPIO_Port GPIOA
#define RS485_TERM_Pin GPIO_PIN_9
#define RS485_TERM_GPIO_Port GPIOA
#define RCD_Failure_Pin GPIO_PIN_11
#define RCD_Failure_GPIO_Port GPIOA
#define RCD_Test_Pin GPIO_PIN_12
#define RCD_Test_GPIO_Port GPIOA
#define CON_Pin GPIO_PIN_15
#define CON_GPIO_Port GPIOA
#define Current8_Pin GPIO_PIN_3
#define Current8_GPIO_Port GPIOB
#define LO_Pin GPIO_PIN_4
#define LO_GPIO_Port GPIOB
#define CC_Pin GPIO_PIN_5
#define CC_GPIO_Port GPIOB
#define CCR_Pin GPIO_PIN_6
#define CCR_GPIO_Port GPIOB
#define LD_Pin GPIO_PIN_7
#define LD_GPIO_Port GPIOB
#define DIP_SW1_Pin GPIO_PIN_8
#define DIP_SW1_GPIO_Port GPIOB
#define DIP_SW2_Pin GPIO_PIN_9
#define DIP_SW2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
