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
#include "stm32h7xx_hal.h"

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
void MX_PWR_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MODW_ADR2_Pin GPIO_PIN_8
#define MODW_ADR2_GPIO_Port GPIOI
#define ADR_SER2_Pin GPIO_PIN_13
#define ADR_SER2_GPIO_Port GPIOC
#define ADR_SER0_Pin GPIO_PIN_9
#define ADR_SER0_GPIO_Port GPIOI
#define ADR_SER1_Pin GPIO_PIN_11
#define ADR_SER1_GPIO_Port GPIOI
#define MODW_ADR1_Pin GPIO_PIN_1
#define MODW_ADR1_GPIO_Port GPIOK
#define MODW_ADR0_Pin GPIO_PIN_2
#define MODW_ADR0_GPIO_Port GPIOK
#define IO_INT_Pin GPIO_PIN_7
#define IO_INT_GPIO_Port GPIOG
#define MOD_SPI_NCS_Pin GPIO_PIN_13
#define MOD_SPI_NCS_GPIO_Port GPIOH
#define SUB_MOD_ADR0_Pin GPIO_PIN_3
#define SUB_MOD_ADR0_GPIO_Port GPIOD
#define SUB_MOD_ADR1_Pin GPIO_PIN_6
#define SUB_MOD_ADR1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
