/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define ENC2_A2_Pin GPIO_PIN_0
#define ENC2_A2_GPIO_Port GPIOA
#define ENC2_B2_Pin GPIO_PIN_1
#define ENC2_B2_GPIO_Port GPIOA
#define SERVO_PWM_Pin GPIO_PIN_2
#define SERVO_PWM_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_10
#define AIN2_GPIO_Port GPIOE
#define PWMA_Pin GPIO_PIN_11
#define PWMA_GPIO_Port GPIOE
#define AIN1_Pin GPIO_PIN_12
#define AIN1_GPIO_Port GPIOE
#define BIN2_Pin GPIO_PIN_13
#define BIN2_GPIO_Port GPIOE
#define PWMB_Pin GPIO_PIN_14
#define PWMB_GPIO_Port GPIOE
#define BIN1_Pin GPIO_PIN_15
#define BIN1_GPIO_Port GPIOE
#define ENC1_B1_Pin GPIO_PIN_6
#define ENC1_B1_GPIO_Port GPIOB
#define ENC1_A1_Pin GPIO_PIN_7
#define ENC1_A1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
