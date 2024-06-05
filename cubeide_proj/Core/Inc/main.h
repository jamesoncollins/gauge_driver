/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32wbxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern void DWT_Delay_us(volatile uint32_t microseconds);
extern int my_transfer (I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                 int16_t Size, uint32_t Timeout);
extern void My_MX_USART1_UART_DeInit();
extern void My_MX_USART1_UART_Init();
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
#define INT_ACC_Pin GPIO_PIN_2
#define INT_ACC_GPIO_Port GPIOC
#define INT_ACC_EXTI_IRQn EXTI2_IRQn
#define INT_GYR_Pin GPIO_PIN_3
#define INT_GYR_GPIO_Port GPIOC
#define INT_GYR_EXTI_IRQn EXTI3_IRQn
#define IGN_Pin GPIO_PIN_5
#define IGN_GPIO_Port GPIOC
#define RESET_MOTOR_Pin GPIO_PIN_2
#define RESET_MOTOR_GPIO_Port GPIOB
#define SPEEDO_Pin GPIO_PIN_10
#define SPEEDO_GPIO_Port GPIOB
#define TACH_Pin GPIO_PIN_11
#define TACH_GPIO_Port GPIOB
#define DIR_TACH_Pin GPIO_PIN_0
#define DIR_TACH_GPIO_Port GPIOB
#define DIR_SPEED_Pin GPIO_PIN_1
#define DIR_SPEED_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA
#define PWREN_Pin GPIO_PIN_10
#define PWREN_GPIO_Port GPIOC
#define STEP_ODO_Pin GPIO_PIN_0
#define STEP_ODO_GPIO_Port GPIOD
#define DIR_ODO_Pin GPIO_PIN_1
#define DIR_ODO_GPIO_Port GPIOD
#define STEP_TACH_Pin GPIO_PIN_4
#define STEP_TACH_GPIO_Port GPIOB
#define STEP_SPEED_Pin GPIO_PIN_5
#define STEP_SPEED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
