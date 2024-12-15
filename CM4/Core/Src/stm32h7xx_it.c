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
volatile uint32_t nPoprzedniStanTimera2;	//timer 32 bitowy
volatile uint16_t sPoprzedniStanTimera3, sPoprzedniStanTimera4;
uint16_t sSerwo[KANALY_SERW];	//sterowane kanałów serw
volatile uint16_t sOdbRC1[KANALY_ODB];	//odbierane kanały na odbiorniku 1 RC
volatile uint16_t sOdbRC2[KANALY_ODB];	//odbierane kanały na odbiorniku 2 RC
uint8_t chNumerKanWejRC1, chNumerKanWejRC2;	//liczniki kanałów w odbieranym sygnale PPM
volatile uint8_t chNumerKanSerw;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern MDMA_HandleTypeDef hmdma_mdma_channel0_dma1_stream1_tc_0;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;
extern UART_HandleTypeDef huart8;
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
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
	//obsługa wyjścia TIM1_CH1 do generowania kanałów 50Hz [8..15]
	if (htim1.Instance->SR & TIM_FLAG_CC1)
	{
		if (chZbocze[chNumerKanSerw])	//zbocze narastajace, odmierz długość impulsu
		{
			htim1.Instance->CCR1 += sSerwo[chNumerKanSerw];
			htim1.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
			htim1.Instance->CCMR1 |= TIM_CCMR1_OC1M_1;		//Set channel to inactive level on match
			chZbocze[chNumerKanSerw] = 0;
		}
		else	//zbocze opadające
		{
			htim1.Instance->CCR1 += OKRES_PWM - sSerwo[chNumerKanSerw];
			htim1.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
			htim1.Instance->CCMR1 |= TIM_CCMR1_OC1M_0;		//Set channel to active level on match
			chZbocze[chNumerKanSerw] = 1;

			chNumerKanSerw++;						//ustaw następny kanał
			if (chNumerKanSerw == KANALY_SERW)
				chNumerKanSerw = 8;
			switch (chNumerKanSerw)					//ustaw dekoder
			{
			case 8:
				HAL_GPIO_WritePin(ADR_SER0_GPIO_Port, ADR_SER0_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ADR_SER1_GPIO_Port, ADR_SER1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ADR_SER2_GPIO_Port, ADR_SER2_Pin, GPIO_PIN_RESET);
				break;
			case 9:
				HAL_GPIO_WritePin(ADR_SER0_GPIO_Port, ADR_SER0_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(ADR_SER1_GPIO_Port, ADR_SER1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ADR_SER2_GPIO_Port, ADR_SER2_Pin, GPIO_PIN_RESET);
				break;
			case 10:
				HAL_GPIO_WritePin(ADR_SER0_GPIO_Port, ADR_SER0_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ADR_SER1_GPIO_Port, ADR_SER1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(ADR_SER2_GPIO_Port, ADR_SER2_Pin, GPIO_PIN_RESET);
				break;
			case 11:
				HAL_GPIO_WritePin(ADR_SER0_GPIO_Port, ADR_SER0_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(ADR_SER1_GPIO_Port, ADR_SER1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(ADR_SER2_GPIO_Port, ADR_SER2_Pin, GPIO_PIN_RESET);
				break;
			case 12:
				HAL_GPIO_WritePin(ADR_SER0_GPIO_Port, ADR_SER0_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ADR_SER1_GPIO_Port, ADR_SER1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ADR_SER2_GPIO_Port, ADR_SER2_Pin, GPIO_PIN_SET);
				break;
			case 13:
				HAL_GPIO_WritePin(ADR_SER0_GPIO_Port, ADR_SER0_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(ADR_SER1_GPIO_Port, ADR_SER1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ADR_SER2_GPIO_Port, ADR_SER2_Pin, GPIO_PIN_SET);
				break;
			case 14:
				HAL_GPIO_WritePin(ADR_SER0_GPIO_Port, ADR_SER0_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ADR_SER1_GPIO_Port, ADR_SER1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(ADR_SER2_GPIO_Port, ADR_SER2_Pin, GPIO_PIN_SET);
				break;
			case 15:
				HAL_GPIO_WritePin(ADR_SER0_GPIO_Port, ADR_SER0_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(ADR_SER1_GPIO_Port, ADR_SER1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(ADR_SER2_GPIO_Port, ADR_SER2_Pin, GPIO_PIN_SET);
				break;
			}
		}
		htim1.Instance->SR &= ~TIM_FLAG_CC1;	//kasuj przerwanie przez zapis zera
	}

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	uint32_t nTemp;		//Licznik 32-bitowy

	//obsługa wyjścia TIM2_CH1
	if (htim2.Instance->SR & TIM_FLAG_CC1)
	{
		if (chZbocze[2])	//zbocze narastajace, odmierz długość impulsu
		{
			htim2.Instance->CCR1 += sSerwo[2];
			htim2.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
			htim2.Instance->CCMR1 |= TIM_CCMR1_OC1M_1;		//Set channel to inactive level on match
			chZbocze[2] = 0;
		}
		else	//zbocze opadające
		{
			htim2.Instance->CCR1 += OKRES_PWM - sSerwo[2];
			htim2.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
			htim2.Instance->CCMR1 |= TIM_CCMR1_OC1M_0;		//Set channel to active level on match
			chZbocze[2] = 1;
		}
		htim2.Instance->SR &= ~TIM_FLAG_CC1;	//kasuj przerwanie przez zapis zera
	}

	//obsługa wejścia TIM2_CH4 szeregowego sygnału PPM2. Sygnał aktywny niski. Kolejne impulsy zą pomiędzy zboczami narastającymi
	if (htim2.Instance->SR & TIM_FLAG_CC4)
	{
		if (htim2.Instance->CCR4 > nPoprzedniStanTimera2)
			nTemp = htim2.Instance->CCR4 - nPoprzedniStanTimera2;  //długość impulsu
		else
			nTemp = 0xFFFFFFFF - nPoprzedniStanTimera2 + htim2.Instance->CCR4;  //długość impulsu

		//impuls o długości większej niż 3ms traktowany jest jako przerwa między paczkami impulsów
		if (nTemp > PRZERWA_PPM)
		{
			chNumerKanWejRC2 = 0;
			//nPPMStartTime[0] = nCurrServoTim;   //potrzebne do detekcji braku sygnału
		}
		else
		{
			sOdbRC2[chNumerKanWejRC2] = nTemp;
			chNumerKanWejRC2++;
		}
		nPoprzedniStanTimera2 = htim2.Instance->CCR4;
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
	//Licznik 16-bitowy

	//obsługa wyjścia TIM3_CH3: Serwo kanał 3
	if (htim3.Instance->SR & TIM_FLAG_CC3)
	{
		if (chZbocze[3])	//zbocze narastajace, odmierz długość impulsu
		{
			htim3.Instance->CCR3 += sSerwo[3];
			htim3.Instance->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
			htim3.Instance->CCMR2 |= TIM_CCMR2_OC3M_1;		//Set channel to inactive level on match
			chZbocze[3] = 0;
		}
		else	//zbocze opadające
		{
			htim3.Instance->CCR3 += OKRES_PWM - sSerwo[3];
			htim3.Instance->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
			htim3.Instance->CCMR2 |= TIM_CCMR2_OC3M_0;		//Set channel to active level on match
			chZbocze[3] = 1;
		}
		htim3.Instance->SR &= ~TIM_FLAG_CC3;	//kasuj przerwanie przez zapis zera
	}

	//obsługa wyjścia TIM3_CH4: Serwo kanał 4
	if (htim3.Instance->SR & TIM_FLAG_CC4)
	{
		if (chZbocze[4])	//zbocze narastajace, odmierz długość impulsu
		{
			htim3.Instance->CCR4 += sSerwo[4];
			htim3.Instance->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
			htim3.Instance->CCMR2 |= TIM_CCMR2_OC4M_1;		//Set channel to inactive level on match
			chZbocze[4] = 0;
		}
		else	//zbocze opadające
		{
			htim3.Instance->CCR4 += OKRES_PWM - sSerwo[4];
			htim3.Instance->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
			htim3.Instance->CCMR2 |= TIM_CCMR2_OC4M_0;		//Set channel to active level on match
			chZbocze[4] = 1;
		}
		htim3.Instance->SR &= ~TIM_FLAG_CC4;	//kasuj przerwanie przez zapis zera
	}

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
	uint16_t sTemp;			//Licznik 16-bitowy

	//obsługa wyjścia TIM4_CH4: Serwo kanał 0, zanegowane na inwerterze
	if (htim4.Instance->SR & TIM_FLAG_CC4)
	{
		if (chZbocze[0])	//zbocze narastajace, odmierz długość impulsu
		{
			htim4.Instance->CCR4 += sSerwo[0];
			htim4.Instance->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
			htim4.Instance->CCMR2 |= TIM_CCMR2_OC4M_1;		//Set channel to inactive level on match
			chZbocze[0] = 0;
		}
		else	//zbocze opadające
		{
			htim4.Instance->CCR4 +=  OKRES_PWM - sSerwo[0];
			htim4.Instance->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
			htim4.Instance->CCMR2 |= TIM_CCMR2_OC4M_0;		//Set channel to active level on match
			chZbocze[0] = 1;
		}
		htim4.Instance->SR &= ~TIM_FLAG_CC4;	//kasuj przerwanie przez zapis zera
	}

	//obsługa wejścia TIM4_CH3 szeregowego sygnału PPM1. Sygnał aktywny niski. Kolejne impulsy zą pomiędzy zboczami narastającymi
	if (htim4.Instance->SR & TIM_FLAG_CC3)
	{
		if (htim4.Instance->CCR3 > sPoprzedniStanTimera4)
			sTemp = htim4.Instance->CCR3 - sPoprzedniStanTimera4;  //długość impulsu
		else
			sTemp = 0xFFFF - sPoprzedniStanTimera4 + htim4.Instance->CCR3;  //długość impulsu

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
		sPoprzedniStanTimera4 = htim4.Instance->CCR3;	//odczyt CCRx kasuje przerwanie
	}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM8 capture compare interrupt.
  */
void TIM8_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */
	//obsługa wyjścia TIM8_CH1: Serwo kanał 5
	if (htim8.Instance->SR & TIM_FLAG_CC1)
	{
		if (chZbocze[5])	//zbocze narastajace, odmierz długość impulsu
		{
			htim8.Instance->CCR1 += sSerwo[5];
			htim8.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
			htim8.Instance->CCMR1 |= TIM_CCMR1_OC1M_1;		//Set channel to inactive level on match
			chZbocze[5] = 0;
		}
		else	//zbocze opadające
		{
			htim8.Instance->CCR1 += OKRES_PWM - sSerwo[5];
			htim8.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
			htim8.Instance->CCMR1 |= TIM_CCMR1_OC1M_0;		//Set channel to active level on match
			chZbocze[5] = 1;
		}
		htim8.Instance->SR &= ~TIM_FLAG_CC1;	//kasuj przerwanie przez zapis zera
	}

	//obsługa wyjścia TIM8_CH3N: Serwo kanał 7
	if (htim8.Instance->SR & TIM_FLAG_CC3)
	{
		if (chZbocze[7])	//zbocze narastajace, odmierz długość impulsu
		{
			htim8.Instance->CCR3 += sSerwo[7];
			htim8.Instance->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
			htim8.Instance->CCMR2 |= TIM_CCMR2_OC3M_1;		//Set channel to inactive level on match
			chZbocze[7] = 0;
		}
		else	//zbocze opadające
		{
			htim8.Instance->CCR3 += OKRES_PWM - sSerwo[7];
			htim8.Instance->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
			htim8.Instance->CCMR2 |= TIM_CCMR2_OC3M_0;		//Set channel to active level on match
			chZbocze[7] = 1;
		}
		htim8.Instance->SR &= ~TIM_FLAG_CC3;	//kasuj przerwanie przez zapis zera
	}
  /* USER CODE END TIM8_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN UART8_IRQn 0 */

  /* USER CODE END UART8_IRQn 0 */
  HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN UART8_IRQn 1 */

  /* USER CODE END UART8_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
