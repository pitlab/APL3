/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys_def_CM4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint8_t chZbocze[8];	//flaga określająca na które zbocze mamy reagować dla wszystkich kanałow wyjściowych serw
volatile uint16_t sPoprzedniStanTimera2, sPoprzedniStanTimera4;
uint16_t sSerwo[KANALY_SERW];	//sterowane kanałów serw
uint16_t sOdbRC1[KANALY_ODB];	//odbierane kanały na odbiorniku 1 RC
uint16_t sOdbRC2[KANALY_ODB];	//odbierane kanały na odbiorniku 2 RC
uint8_t chNumerKanWejRC1, chNumerKanWejRC2;	//liczniki kanałów w odbieranym sygnale PPM
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern MDMA_HandleTypeDef hmdma_mdma_channel0_dma1_stream1_tc_0;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	uint16_t sTemp;
	uint16_t sBiezacyStanTimera = htim2.Instance->CNT;

	//obsługa wyjścia TIM2_CH1
	if (htim2.Instance->SR & TIM_FLAG_CC1)
	{
		if (chZbocze[2])	//zbocze narastajace, odmierz długość impulsu
		{
			htim2.Instance->CCR1 = sBiezacyStanTimera + sSerwo[2];
			htim2.Instance->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
			htim2.Instance->CCMR1 |= TIM_CCMR1_OC1M_1;		//Set channel to inactive level on match
			chZbocze[2] = 0;
		}
		else	//zbocze opadające
		{
			htim2.Instance->CCR1 = sBiezacyStanTimera + OKRES_PWM - sSerwo[2];
			htim2.Instance->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
			htim2.Instance->CCMR1 |= TIM_CCMR1_OC1M_0;		//Set channel to active level on match
			chZbocze[2] = 1;
		}
	}

	//obsługa wejścia TIM2_CH4 szeregowego sygnału PPM2. Sygnał aktywny niski. Kolejne impulsy zą pomiędzy zboczami narastającymi
	if (htim2.Instance->SR & TIM_FLAG_CC4)
	{
		if (sBiezacyStanTimera > sPoprzedniStanTimera2)
			sTemp = sBiezacyStanTimera - sPoprzedniStanTimera2;  //długość impulsu
		else
			sTemp = 0xFFFF - sPoprzedniStanTimera2 + sBiezacyStanTimera;  //długość impulsu

		//impuls o długości większej niż 3ms traktowany jest jako przerwa między paczkami impulsów
		if (sTemp > PRZERWA_PPM)
		{
			chNumerKanWejRC2 = 0;
			//nPPMStartTime[0] = nCurrServoTim;   //potrzebne do detekcji braku sygnału
		}
		else
		{
			sOdbRC2[chNumerKanWejRC2] = sTemp;
			chNumerKanWejRC2++;
		}
		sPoprzedniStanTimera2 = sBiezacyStanTimera;
	}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	uint16_t sTemp;
	uint16_t sBiezacyStanTimera = htim4.Instance->CNT;

	//obsługa wyjścia TIM4_CH4
	if (htim4.Instance->SR & TIM_FLAG_CC4)
	{
		if (chZbocze[0])	//zbocze narastajace, odmierz długość impulsu
		{
			htim4.Instance->CCR4 = sBiezacyStanTimera + sSerwo[0];
			htim4.Instance->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
			htim4.Instance->CCMR2 |= TIM_CCMR2_OC4M_1;		//Set channel to inactive level on match
			chZbocze[0] = 0;
		}
		else	//zbocze opadające
		{
			htim4.Instance->CCR4 = sBiezacyStanTimera + OKRES_PWM - sSerwo[0];
			htim4.Instance->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
			htim4.Instance->CCMR2 |= TIM_CCMR2_OC4M_0;		//Set channel to active level on match
			chZbocze[0] = 1;
		}
	}

	//obsługa wejścia TIM4_CH3 szeregowego sygnału PPM1. Sygnał aktywny niski. Kolejne impulsy zą pomiędzy zboczami narastającymi
	if (htim4.Instance->SR & TIM_FLAG_CC3)
	{
		if (sBiezacyStanTimera > sPoprzedniStanTimera4)
			sTemp = sBiezacyStanTimera - sPoprzedniStanTimera4;  //długość impulsu
		else
			sTemp = 0xFFFF - sPoprzedniStanTimera4 + sBiezacyStanTimera;  //długość impulsu

		//impuls o długości większej niż 3ms traktowany jest jako przerwa między paczkami impulsów
		if (sTemp > PRZERWA_PPM)
		{
			chNumerKanWejRC1 = 0;
			//nPPMStartTime[0] = nCurrServoTim;   //potrzebne do detekcji braku sygnału
		}
		else
		{
			sOdbRC1[chNumerKanWejRC1] = sTemp;
			chNumerKanWejRC1++;
		}
		sPoprzedniStanTimera4 = sBiezacyStanTimera;
	}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
