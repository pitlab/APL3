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
//#include "sys_def_CM4.h"
#include "sys_def_wspolny.h"
#include "odbiornikRC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern uint32_t PobierzCzas(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint8_t chZbocze[8];	//flaga określająca na które zbocze mamy reagować dla wszystkich kanałow wyjściowych serw
volatile uint32_t nPoprzedniStanTimera2;	//timer 32 bitowy
//volatile uint16_t sPoprzedniStanTimera3, sPoprzedniStanTimera4;
uint16_t sSerwo[KANALY_SERW];	//sterowane kanałów serw
//volatile uint16_t sOdbRC1[KANALY_ODB_RC];	//odbierane kanały na odbiorniku 1 RC
//volatile uint16_t sOdbRC2[KANALY_ODB_RC];	//odbierane kanały na odbiorniku 2 RC
//uint8_t chNumerKanWejRC1, chNumerKanWejRC2;	//liczniki kanałów w odbieranym sygnale PPM
volatile uint8_t chNumerKanSerw;
volatile uint16_t sCzasH;
extern stRC_t stRC;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern MDMA_HandleTypeDef hmdma_mdma_channel0_dma1_stream1_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel1_dma1_stream1_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel2_dma1_stream1_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel3_dma1_stream1_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel4_dma1_stream1_tc_0;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_i2c3_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;
extern DMA_HandleTypeDef hdma_i2c4_rx;
extern DMA_HandleTypeDef hdma_i2c4_tx;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
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
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c3_rx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c3_tx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
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
			default:	break;
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
		if (htim2.Instance->CCR4 > stRC.sPoprzedniaWartoscTimera2)
			nTemp = htim2.Instance->CCR4 - stRC.sPoprzedniaWartoscTimera2;  //długość impulsu
		else
			nTemp = 0xFFFFFFFF - stRC.sPoprzedniaWartoscTimera2 + htim2.Instance->CCR4;  //długość impulsu

		//impuls o długości większej niż 3ms traktowany jest jako przerwa między paczkami impulsów
		if (nTemp > PRZERWA_PPM)
		{
			stRC.nCzasWe2 = PobierzCzas();
			stRC.chNrKan2 = 0;
			stRC.chStatus2 = RAMKA_OK;
		}
		else
		if ((nTemp > PPM_MIN) && (nTemp < PPM_MAX) && (stRC.chStatus2 == RAMKA_OK))
		{
			stRC.sOdb2[stRC.chNrKan2] = nTemp;
			stRC.sZdekodowaneKanaly2 |= (1 << stRC.chNrKan2);	//ustaw bit zdekodowanego kanału
			stRC.chNrKan2++;
		}
		else
			stRC.chStatus2 = RAMKA_WADLIWA;

		stRC.sPoprzedniaWartoscTimera2 = htim2.Instance->CCR4;
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
	uint16_t sTemp;

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
		if (htim4.Instance->CCR3 > stRC.sPoprzedniaWartoscTimera1)
			sTemp = htim4.Instance->CCR3 - stRC.sPoprzedniaWartoscTimera1;  //długość impulsu
		else
			sTemp = 0xFFFF - stRC.sPoprzedniaWartoscTimera1 + htim4.Instance->CCR3;  //długość impulsu

		//impuls o długości większej niż 3ms traktowany jest jako przerwa między paczkami impulsów
		if (sTemp > PRZERWA_PPM)
		{
			stRC.nCzasWe1 = PobierzCzas();
			stRC.chNrKan1 = 0;
			stRC.chStatus1 = RAMKA_OK;
		}
		else
		if ((sTemp > PPM_MIN) && (sTemp < PPM_MAX) && (stRC.chStatus1 == RAMKA_OK))
		{
			stRC.sOdb1[stRC.chNrKan1] = sTemp;
			stRC.sZdekodowaneKanaly1 |= (1 << stRC.chNrKan1);	//ustaw bit zdekodowanego kanału
			stRC.chNrKan1++;
		}
		else
			stRC.chStatus1 = RAMKA_WADLIWA;
		stRC.sPoprzedniaWartoscTimera1 = htim4.Instance->CCR3;	//odczyt CCRx kasuje przerwanie
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
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
  //Aby liczenie czasu było dokładne w dłuższych okresach, przy przekręceniu się 16-bitowego sprzętowego licznika inkrementuj drugi 16-bitowy licznik programowy
  sCzasH++;
  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles I2C3 event interrupt.
  */
void I2C3_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_EV_IRQn 0 */

  /* USER CODE END I2C3_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_EV_IRQn 1 */

  /* USER CODE END I2C3_EV_IRQn 1 */
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

/**
  * @brief This function handles I2C4 event interrupt.
  */
void I2C4_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C4_EV_IRQn 0 */

  /* USER CODE END I2C4_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c4);
  /* USER CODE BEGIN I2C4_EV_IRQn 1 */

  /* USER CODE END I2C4_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C4 error interrupt.
  */
void I2C4_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C4_ER_IRQn 0 */

  /* USER CODE END I2C4_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c4);
  /* USER CODE BEGIN I2C4_ER_IRQn 1 */

  /* USER CODE END I2C4_ER_IRQn 1 */
}

/**
  * @brief This function handles BDMA channel2 global interrupt.
  */
void BDMA_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN BDMA_Channel2_IRQn 0 */

  /* USER CODE END BDMA_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c4_rx);
  /* USER CODE BEGIN BDMA_Channel2_IRQn 1 */

  /* USER CODE END BDMA_Channel2_IRQn 1 */
}

/**
  * @brief This function handles BDMA channel3 global interrupt.
  */
void BDMA_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN BDMA_Channel3_IRQn 0 */

  /* USER CODE END BDMA_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c4_tx);
  /* USER CODE BEGIN BDMA_Channel3_IRQn 1 */

  /* USER CODE END BDMA_Channel3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
