/* USER CODE BEGIN Header */
//////////////////////////////////////////////////////////////////////////////
//
// Obsługa przerwań
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
// Aby wygenerować sygnał PPM timer musi być taktowany zegarem 1MHz (200MHZ/199+1) 1/f = 1us
// Aby wygenerować sygnał DShot150 timer musi być taktowany 6MHz -> 0,1667us. T0H to 15 cyknięć zegara, T1H to 30 cyknięć, cały bit to 40 cyknięć
// Aby wygenerować sygnał DShot300 timer musi być taktowany 12MHz...
// Aby wygenerować sygnał DShot600 timer musi być taktowany 24MHz...
// Aby wygenerować sygnał DShot1200 timer musi być taktowany 48MHz...
// Ponieważ 200MHz nie dzieli się przez 6 i 12 zegar kontrolera musi wynosić 204Mz. Żeby uzyskać DShot600 podzielnik musi wynosić 24 i zegar musi mieć wartość 216MHz
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "sys_def_CM4.h"
#include "sys_def_wspolny.h"
#include "WeWyRC.h"
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
//volatile uint8_t chZbocze[8];	//flaga określająca na które zbocze mamy reagować dla wszystkich kanałow wyjściowych serw
volatile uint32_t nPoprzedniStanTimera2;	//timer 32 bitowy
volatile uint8_t chNumerKanSerw;
volatile uint16_t sCzasH;
extern stRC_t stRC;
extern unia_wymianyCM4_t uDaneCM4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_i2c3_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;
extern DMA_HandleTypeDef hdma_i2c4_rx;
extern DMA_HandleTypeDef hdma_i2c4_tx;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c4;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch4;
extern DMA_HandleTypeDef hdma_tim8_ch3;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;
extern UART_HandleTypeDef huart8;
extern uint8_t chKonfigWeRC[LICZBA_WEJSC_RC];
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
	//obsługa wyjścia TIM1_CH1 jako PWM do generowania kanałów 50Hz [8..15]
	if (htim1.Instance->SR & TIM_FLAG_CC1)
	{
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
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, uDaneCM4.dane.sSerwo[chNumerKanSerw]);
		HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);		//serwo kanał 7
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

	//obsługa wejścia TIM2_CH4 szeregowego sygnału PPM2. Sygnał aktywny niski. Kolejne impulsy zą pomiędzy zboczami narastającymi
	if (htim2.Instance->SR & TIM_FLAG_CC4)
	{
		//przerwania timera interpretuj jako impulsy tylko gdy wejście RC jest skonfigurowane jako CPPM. Gdy jest ustawiony S-Bus to generuje zakłócenia
		if (chKonfigWeRC[KANAL_RC2] == ODB_RC_CPPM)
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
				stRC.chStatus |= STATRC_RAMKA2_OK;
			}
			else
			if ((nTemp > PPM_MIN) && (nTemp < PPM_MAX) && (stRC.chStatus & STATRC_RAMKA2_OK))
			{
				stRC.sOdb2[stRC.chNrKan2] = nTemp;
				stRC.sZdekodowaneKanaly2 |= (1 << stRC.chNrKan2);	//ustaw bit zdekodowanego kanału
				stRC.chNrKan2++;
			}
			else
				stRC.chStatus &= ~STATRC_RAMKA2_OK;
		}
		stRC.sPoprzedniaWartoscTimera2 = htim2.Instance->CCR4;	//odczyt CCRx kasuje przerwanie
	}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	uint16_t sTemp;

	//obsługa wejścia TIM4_CH3 szeregowego sygnału PPM1. Sygnał aktywny niski. Kolejne impulsy zą pomiędzy zboczami narastającymi
	if (htim4.Instance->SR & TIM_FLAG_CC3)
	{
		//przerwania timera interpretuj jako impulsy tylko gdy wejście RC jest skonfigurowane jako CPPM. Gdy jest ustawiony S-Bus to generuje zakłócenia
		if (chKonfigWeRC[KANAL_RC1] == ODB_RC_CPPM)
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
				stRC.chStatus |= STATRC_RAMKA1_OK;
			}
			else
			if ((sTemp > PPM_MIN) && (sTemp < PPM_MAX) && (stRC.chStatus & STATRC_RAMKA1_OK))
			{
				stRC.sOdb1[stRC.chNrKan1] = sTemp;
				stRC.sZdekodowaneKanaly1 |= (1 << stRC.chNrKan1);	//ustaw bit zdekodowanego kanału
				stRC.chNrKan1++;
			}
			else
				stRC.chStatus &= ~STATRC_RAMKA1_OK;
		}
		stRC.sPoprzedniaWartoscTimera1 = htim4.Instance->CCR3;	//odczyt CCRx kasuje przerwanie
	}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
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
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim8_ch3);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim8_ch1);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream4 global interrupt.
  */
void DMA2_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */

  /* USER CODE END DMA2_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim3_ch3);
  /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

  /* USER CODE END DMA2_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim3_ch4);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_ch1);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_ch3);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
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
