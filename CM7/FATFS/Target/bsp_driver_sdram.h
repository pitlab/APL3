/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    bsp_driver_sdram.h (based on stm32h743i_eval_sdram.h)
  * @brief   This file contains the common defines and functions prototypes for
  *          the bsp_driver_sdram.c driver.
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
#ifndef __STM32H7_SDRAM_H
#define __STM32H7_SDRAM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_sdram.h"

/* USER CODE BEGIN 0 */

/**
  * @brief  SDRAM status structure definition
  */
#define   SDRAM_OK         ((uint8_t)0x00)
#define   SDRAM_ERROR      ((uint8_t)0x01)

/* Exported constants --------------------------------------------------------*/

#define SDRAM_DEVICE_ADDR  ((uint32_t)0xC1000000)	//zostaw wolne pierwsze 16MB, pozsotałem 48 przeznacz na partycję
#define SDRAM_DEVICE_SIZE  ((uint32_t)0x3000000)  	//SDRAM device size in MBytes 64-16 MB

/* #define SDRAM_MEMORY_WIDTH            FMC_SDRAM_MEM_BUS_WIDTH_8  */
#define SDRAM_MEMORY_WIDTH            FMC_SDRAM_MEM_BUS_WIDTH_16
/* #define SDRAM_MEMORY_WIDTH            FMC_SDRAM_MEM_BUS_WIDTH_32 */

//#define SDCLOCK_PERIOD                FMC_SDRAM_CLOCK_PERIOD_2
/* #define SDCLOCK_PERIOD                FMC_SDRAM_CLOCK_PERIOD_3 */

//#define REFRESH_COUNT                ((uint32_t)0x0603)      // SDRAM refresh counter (100Mhz SD clock)

#define SDRAM_TIMEOUT     ((uint32_t)0xFFFF)

/* DMA definitions for SDRAM DMA transfer */
/*
#define __MDMAx_CLK_ENABLE                 __HAL_RCC_MDMA_CLK_ENABLE
#define __MDMAx_CLK_DISABLE                __HAL_RCC_MDMA_CLK_DISABLE
#define SDRAM_MDMAx_CHANNEL               MDMA_Channel0
#define SDRAM_MDMAx_IRQn                  MDMA_IRQn
*/

/**
  * @brief  FMC SDRAM Mode definition register defines
  */
/*
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)
*/

/* Exported functions --------------------------------------------------------*/
uint8_t BSP_SDRAM_Init(void);
uint8_t BSP_SDRAM_ReadData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize);
uint8_t BSP_SDRAM_ReadData_DMA(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize);
uint8_t BSP_SDRAM_WriteData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize);
uint8_t BSP_SDRAM_WriteData_DMA(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize);
uint8_t BSP_SDRAM_Sendcmd(FMC_SDRAM_CommandTypeDef *SdramCmd);

/* USER CODE END 0 */

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7_SDRAM_H */
