/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define M1_A_Pin GPIO_PIN_5
#define M1_A_GPIO_Port GPIOA
#define M2_A_Pin GPIO_PIN_6
#define M2_A_GPIO_Port GPIOA
#define M2_B_Pin GPIO_PIN_7
#define M2_B_GPIO_Port GPIOA
#define M3_A_Pin GPIO_PIN_12
#define M3_A_GPIO_Port GPIOD
#define M3_B_Pin GPIO_PIN_13
#define M3_B_GPIO_Port GPIOD
#define M4_A_Pin GPIO_PIN_6
#define M4_A_GPIO_Port GPIOC
#define M4_B_Pin GPIO_PIN_7
#define M4_B_GPIO_Port GPIOC
#define M4_IN2_Pin GPIO_PIN_12
#define M4_IN2_GPIO_Port GPIOC
#define M3_IN2_Pin GPIO_PIN_0
#define M3_IN2_GPIO_Port GPIOD
#define M4_IN1_Pin GPIO_PIN_1
#define M4_IN1_GPIO_Port GPIOD
#define M3_IN1_Pin GPIO_PIN_2
#define M3_IN1_GPIO_Port GPIOD
#define M1_IN1_Pin GPIO_PIN_3
#define M1_IN1_GPIO_Port GPIOD
#define M2_IN1_Pin GPIO_PIN_4
#define M2_IN1_GPIO_Port GPIOD
#define M1_IN2_Pin GPIO_PIN_5
#define M1_IN2_GPIO_Port GPIOD
#define M2_IN2_Pin GPIO_PIN_6
#define M2_IN2_GPIO_Port GPIOD
#define M1_B_Pin GPIO_PIN_3
#define M1_B_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
