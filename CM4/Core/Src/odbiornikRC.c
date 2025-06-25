//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł ubsługi odbiorników RC
//
// (c) PitLab 2025
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "odbiornikRC.h"
#include "konfig_fram.h"
#include "petla_glowna.h"
#include "fram.h"

//definicje SBus: https://github.com/uzh-rpg/rpg_quadrotor_control/wiki/SBUS-Protocol

stRC_t stRC;	//struktura danych odbiorników RC
extern unia_wymianyCM4_t uDaneCM4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart2_rx;

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
uint8_t chBuforOdbioruSBus1[ROZMIAR_BUF_SBUS];
uint8_t chBuforOdbioruSBus2[ROZMIAR_BUF_SBUS];
uint8_t chBuforNadawczySBus1[ROZMIAR_BUF_SBUS] =  {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};

uint32_t nCzasWysylkiSbus;


////////////////////////////////////////////////////////////////////////////////
// Funkcja wczytuje z FRAM konfigurację odbiorników RC
// Parametry Sbus 100kBps, 8E2
// Parametry:
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujOdbiornikiRC(void)
{
	uint8_t chTyp, chErr = ERR_OK;
	TIM_IC_InitTypeDef sConfigIC = {0};
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//czytaj konfigurację obu odbiorników RC
	chTyp = CzytajFRAM(FAU_KONF_ODB_RC);

	//ustaw Port PB8 jako UART4_RX lub TIM4_CH3
	__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	if ((chTyp & MASKA_TYPU_RC1) == ODB_RC_CPPM)
	{
	    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		__HAL_RCC_TIM4_CLK_ENABLE();

		sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		sConfigIC.ICFilter = 7;
		chErr = HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);
	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == ODB_RC_SBUS)
	{
	    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		__HAL_RCC_DMA1_CLK_ENABLE();
	    __HAL_RCC_UART4_CLK_ENABLE();

		huart4.Instance = UART4;
		huart4.Init.BaudRate = 100000;
		huart4.Init.WordLength = UART_WORDLENGTH_9B;	//bit parzystości liczy sie jako dane
		huart4.Init.StopBits = UART_STOPBITS_2;
		huart4.Init.Parity = UART_PARITY_EVEN;
		if (huart4.Init.Mode == UART_MODE_TX)//jeżeli właczony jest nadajnik to włącz jeszcze odbiornik a jeżeli nie to tylko odbiornik
			huart4.Init.Mode = UART_MODE_TX_RX;
		else
			huart4.Init.Mode = UART_MODE_RX;
		huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart4.Init.OverSampling = UART_OVERSAMPLING_16;
		huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
		huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
		huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
		huart4.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
		chErr = HAL_UART_Init(&huart4);
		chErr = HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_2);
		chErr = HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_2);
		//chErr = HAL_UARTEx_EnableFifoMode(&huart4);
		chErr = HAL_UARTEx_DisableFifoMode(&huart4);

		// UART4 DMA RX Init
		hdma_uart4_rx.Instance = DMA1_Stream7;
		hdma_uart4_rx.Init.Request = DMA_REQUEST_UART4_RX;
		hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_uart4_rx.Init.Mode = DMA_NORMAL;
		hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		hdma_uart4_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_uart4_rx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_uart4_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
		chErr = HAL_DMA_Init(&hdma_uart4_rx);
		__HAL_LINKDMA(&huart4, hdmarx, hdma_uart4_rx);

		// UART4 interrupt Init
		HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(UART4_IRQn);
		HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    	HAL_UART_Receive_DMA(&huart4, chBuforOdbioruSBus1, ROZMIAR_BUF_SBUS);	//włącz odbiór pierwszej ramki
	}


	//ustaw Port PA3 jako UART2_RX lub TIM2_CH4
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == ODB_RC_CPPM)
	{
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		__HAL_RCC_TIM2_CLK_ENABLE();

		sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		sConfigIC.ICFilter = 7;
		chErr = HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == ODB_RC_SBUS)
	{
	    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		__HAL_RCC_DMA2_CLK_ENABLE();
		__HAL_RCC_USART2_CLK_ENABLE();

		huart2.Instance = USART2;
		huart2.Init.BaudRate = 100000;
		huart2.Init.WordLength = UART_WORDLENGTH_9B;
		huart2.Init.StopBits = UART_STOPBITS_2;
		huart2.Init.Parity = UART_PARITY_EVEN;
		huart2.Init.Mode = UART_MODE_RX;
		huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart2.Init.OverSampling = UART_OVERSAMPLING_16;
		huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
		huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
		huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
		huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
		huart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
		chErr = HAL_UART_Init(&huart2);
		chErr = HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_2);
		chErr = HAL_UARTEx_EnableFifoMode(&huart2);

		//ponieważ nie ma jak zarezerwować kanału DMA w HAL bo nie cały UART jest dostępny, więc USART2 będzie pracował na przerwaniach
		/*/ UART2 DMA RX Init
		hdma_uart2_rx.Instance = DMA2_Stream0;
		hdma_uart2_rx.Init.Request = DMA_REQUEST_USART2_RX;
		hdma_uart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_uart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_uart2_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_uart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_uart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_uart2_rx.Init.Mode = DMA_NORMAL;
		hdma_uart2_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_uart2_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		hdma_uart2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_uart2_rx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_uart2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
		chErr = HAL_DMA_Init(&hdma_uart2_rx);
		__HAL_LINKDMA(&huart2, hdmarx, hdma_uart2_rx);
		HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
		//HAL_UART_Receive_DMA(&huart2, chBuforOdbioruSBus2, ROZMIAR_BUF_SBUS);		*/

		HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
		HAL_UART_Receive_IT(&huart2, chBuforOdbioruSBus2, ROZMIAR_BUF_SBUS);	//włącz odbiór pierwszej ramki
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja wczytuje z FRAM konfigurację 2 pierwszych kanałów serw i konfiguruje je jako PWM lub S-Bus
// Parametry Sbus 100kBps, 8E2
// Parametry:
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujWyjsciaSBus(void)
{
	uint8_t chTyp, chErr = ERR_OK;
	//TIM_IC_InitTypeDef sConfigIC = {0};
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//czytaj konfigurację dwu pierwszych kanałów serw
	chTyp = CzytajFRAM(FAU_KONF_SERWA12);

	//kanał 1 - konfiguracja pinu PB9
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if ((chTyp & MASKA_TYPU_RC1) == SERWO_PWM400)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_PWM50)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_IO)	//wyjście IO do debugowania
	{
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_ALTER1)		//wyjście jako S-Bus
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		__HAL_RCC_DMA1_CLK_ENABLE();
		__HAL_RCC_UART4_CLK_ENABLE();

		huart4.Instance = UART4;
		huart4.Init.BaudRate = 100000;
		huart4.Init.WordLength = UART_WORDLENGTH_9B;
		huart4.Init.StopBits = UART_STOPBITS_2;
		huart4.Init.Parity = UART_PARITY_EVEN;
		if (huart4.Init.Mode == UART_MODE_RX)		//jeżeli właczony jest odbiornik to włącz jeszcze nadajnik a jeżeli nie to tylko nadajnik
			huart4.Init.Mode = UART_MODE_TX_RX;
		else
			huart4.Init.Mode = UART_MODE_TX_RX;
		huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart4.Init.OverSampling = UART_OVERSAMPLING_16;
		huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
		huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
		huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
		huart4.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
		chErr = HAL_UART_Init(&huart4);
		chErr = HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_2);
		chErr = HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_2);
		//chErr = HAL_UARTEx_EnableFifoMode(&huart4);
		chErr = HAL_UARTEx_DisableFifoMode(&huart4);

		HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(UART4_IRQn);

		hdma_uart4_tx.Instance = DMA2_Stream0;
		hdma_uart4_tx.Init.Request = DMA_REQUEST_UART4_TX;
		hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_uart4_tx.Init.Mode = DMA_NORMAL;
		hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_uart4_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	    hdma_uart4_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	    hdma_uart4_tx.Init.MemBurst = DMA_MBURST_SINGLE;
	    hdma_uart4_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
		chErr = HAL_DMA_Init(&hdma_uart4_tx);
	    __HAL_LINKDMA(&huart4, hdmatx, hdma_uart4_tx);
		HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	}

	//kanał 2 - konfiguracja pinu PB10 -TIM2_CH3, USART3_TX, MOD_QSPI_CS
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_PWM400)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_PWM50)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	/*else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_ALTER1)	//wyjście jako S-Bus
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		__HAL_RCC_USART3_CLK_ENABLE();

		huart3.Instance = USART3;
		huart3.Init.BaudRate = 100000;
		huart3.Init.WordLength = UART_WORDLENGTH_9B;
		huart3.Init.StopBits = UART_STOPBITS_2;
		huart3.Init.Parity = UART_PARITY_EVEN;
		if (huart3.Init.Mode == UART_MODE_RX)		//jeżeli właczony jest odbiornik to włącz jeszcze nadajnik a jeżeli nie to tylko nadajnik
			huart3.Init.Mode = UART_MODE_TX_RX;
		else
			huart3.Init.Mode = UART_MODE_TX_RX;
		huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart3.Init.OverSampling = UART_OVERSAMPLING_16;
		huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
		huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
		huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
		huart3.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
		chErr = HAL_UART_Init(&huart3);
		chErr = HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_2);
		chErr = HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_2);
		//chErr = HAL_UARTEx_EnableFifoMode(&huart3);
		chErr = HAL_UARTEx_DisableFifoMode(&huart3);

		HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
	}*/
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_ALTER1)	//MOD_QSPI_CS
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}


	chTyp = CzytajFRAM(FAU_KONF_SERWA34);

	//kanał 3 - konfiguracja pinu PA15 TIM2_CH1
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if ((chTyp & MASKA_TYPU_RC1) == SERWO_PWM400)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_PWM50)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_IO)	//wyjście IO do debugowania
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_ALTER1)		//
	{

	}


	//kanał 4 - konfiguracja pinu  PB0 TIM3_CH3
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_PWM400)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_PWM50)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_ALTER1)	//ADC12_INP9
	{
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}


	chTyp = CzytajFRAM(FAU_KONF_SERWA56);

	//kanał 5 - konfiguracja pinu PB1 TIM3_CH4
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if ((chTyp & MASKA_TYPU_RC1) == SERWO_PWM400)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_PWM50)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_IO)	//wyjście IO do debugowania
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_ALTER1)		//ADC12_INP5
	{
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}


	//kanał 6 - konfiguracja pinu  PI5 TIM8_CH1
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_PWM400)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_PWM50)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_ALTER1)	//ADC1_INP2
	{
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	}


	chTyp = CzytajFRAM(FAU_KONF_SERWA78);

	//kanał 7 - konfiguracja pinu PI10  - brak timera na porcie
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if ((chTyp & MASKA_TYPU_RC1) == SERWO_PWM400)
	{

	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_PWM50)
	{

	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_IO)	//wyjście IO do debugowania
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	}
	else
	if ((chTyp & MASKA_TYPU_RC1) == SERWO_ALTER1)		//
	{

	}


	//kanał 8 - konfiguracja pinu  PH15 TIM8_CH3N
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_PWM400)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_PWM50)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	}
	else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_ALTER1)
	{

	}


	//kanały 9-16 - konfiguracja pinu  PA8 TIM1_CH1
	chTyp = CzytajFRAM(FAU_KONF_SERWA916);
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if (chTyp == SERWO_PWM400)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else
	if (chTyp == SERWO_PWM50)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else
	if (chTyp == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else
	if (chTyp == SERWO_ALTER1)	//ESC9-10 200Hz
	{

	}
	else
	if (chTyp == SERWO_ALTER2)	//ESC9-12 100Hz
	{

	}

	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Porównuje dane z obu odbiorników RC i wybiera ten lepszy przepisując jego dane do struktury danych CM4
// Parametry:
// [we] *stRC - wskaźnik na strukturę danych odbiorników RC
// [wy] *psDaneCM4 - wskaźnik na strukturę danych CM4
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t DywersyfikacjaOdbiornikowRC(stRC_t* stRC, stWymianyCM4_t* psDaneCM4)
{
	uint8_t chErr = ERR_OK;
	uint32_t nCzasBiezacy = PobierzCzas();
	uint32_t nCzasRC1, nCzasRC2;
	uint8_t n;

	//Sprawdź kiedy przyszły ostatnie dane RC
	nCzasRC1 = MinalCzas2(stRC->nCzasWe1, nCzasBiezacy);
	nCzasRC2 = MinalCzas2(stRC->nCzasWe2, nCzasBiezacy);

	//dekoduj dane jeżeli jest nowa ramka
	if (nCzasRC1 < OKRES_RAMKI_PPM_RC)
	{
		chErr = DekodowanieRamkiBSBus(chBuforOdbioruSBus1, stRC->sOdb1);
		if (chErr == ERR_OK)
			stRC->sZdekodowaneKanaly1 = 0xFFFF;
	}

	if (nCzasRC2 < OKRES_RAMKI_PPM_RC)
	{
		chErr = DekodowanieRamkiBSBus(chBuforOdbioruSBus2, stRC->sOdb2);
		if (chErr == ERR_OK)
			stRC->sZdekodowaneKanaly2 = 0xFFFF;
	}

	if ((nCzasRC1 < 2*OKRES_RAMKI_PPM_RC) && (nCzasRC2 > 2*OKRES_RAMKI_PPM_RC))	//działa odbiornik 1, nie działa 2
	{
		for (n=0; n<KANALY_ODB_RC; n++)
		{
			if (stRC->sZdekodowaneKanaly1 & (1<<n))
			{
				psDaneCM4->sKanalRC[n] = stRC->sOdb1[n]; 	//przepisz zdekodowane kanały
				stRC->sZdekodowaneKanaly1 &= ~(1<<n);		//kasuj bit obrobionego kanału
			}
		}
	}
	else
	if ((nCzasRC1 > 2*OKRES_RAMKI_PPM_RC) && (nCzasRC2 < 2*OKRES_RAMKI_PPM_RC))	//nie działa odbiornik 1, działa 2
	{
		for (n=0; n<KANALY_ODB_RC; n++)
		{
			if (stRC->sZdekodowaneKanaly2 & (1<<n))
			{
				psDaneCM4->sKanalRC[n] = stRC->sOdb2[n]; 	//przepisz zdekodowane kanały
				stRC->sZdekodowaneKanaly2 &= ~(1<<n);		//kasuj bit obrobionego kanału
			}
		}
	}
	else
	if ((nCzasRC1 > 2*OKRES_RAMKI_PPM_RC) && (nCzasRC2 > 2*OKRES_RAMKI_PPM_RC))
	{
		//HAL_UART_Receive_DMA(&huart4, chBuforOdbioruSBus1, ROZMIAR_BUF_SBUS);
		//HAL_UART_Receive_IT(&huart4, chBuforOdbioruSBus2, ROZMIAR_BUF_SBUS);
	}
	else		//działają oba odbiorniki, określ który jest lepszy
	{

	}

	//sprawdź czy trzeba już wysłać nową ramkę Sbus
	nCzasRC1 = MinalCzas2(nCzasWysylkiSbus, nCzasBiezacy);
	if (nCzasRC1 >= OKRES_RAMKI_SBUS)
	{
		nCzasWysylkiSbus = nCzasBiezacy;
		HAL_UART_Transmit_DMA(&huart4, chBuforNadawczySBus1, ROZMIAR_BUF_SBUS);	//wyślij kolejną ramkę
	}

	return chErr;
}



//
void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart4);
}



void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)		//wysłano ramkę SBUS1
	{

	}
}



////////////////////////////////////////////////////////////////////////////////
// Dekoduje dane z ramki wejściowej odbiorników RC i skaluje je do wygodnego w obsłudze zakresu PPM_MIN..PPM_MAX
// Parametry:
// [we] *chRamkaWe - wskaźnik na dane ramki wejsciowej
// [wy] *sKanaly - wskaźnik na tablicę kanałów RC
// Zwraca: kod błędu
// Czas wykonania: ok. 5us
////////////////////////////////////////////////////////////////////////////////
uint8_t DekodowanieRamkiBSBus(uint8_t* chRamkaWe, int16_t *sKanaly)
{
	uint8_t* chNaglowek;
	uint8_t n;

	//dane mogą być przesunięte wzgledem początku więc znajdź nagłówek i synchronizuj się do niego
	for (n=0; n<KANALY_ODB_RC; n++)
	{
		if (*(chRamkaWe + n) == SBUS_NAGLOWEK)
		{
			chNaglowek = chRamkaWe + n;
			break;
		}
	}
	if (n > MAX_PRZESUN_NAGL)
		return ERR_ZLA_ILOSC_DANYCH;

	*(sKanaly +  0) =  ((((int16_t)*(chNaglowek +  1)       | (((int16_t)*(chNaglowek +  2) << 8) & 0x7E0)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly +  1) = (((((int16_t)*(chNaglowek +  2) >> 3) | (((int16_t)*(chNaglowek +  3) << 5) & 0x7E0)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly +  2) = (((((int16_t)*(chNaglowek +  3) >> 6) | (((int16_t)*(chNaglowek +  4) << 2) & 0x3FC) | (((uint16_t)*(chNaglowek + 5) << 10) & 0x400)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly +  3) = (((((int16_t)*(chNaglowek +  5) >> 1) | (((int16_t)*(chNaglowek +  6) << 7) & 0x780)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly +  4) = (((((int16_t)*(chNaglowek +  6) >> 4) | (((int16_t)*(chNaglowek +  7) << 4) & 0x7F0)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN ;
	*(sKanaly +  5) = (((((int16_t)*(chNaglowek +  7) >> 7) | (((int16_t)*(chNaglowek +  8) << 1) & 0x1FE) | (((uint16_t)*(chNaglowek + 9) << 9) & 0x600)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly +  6) = (((((int16_t)*(chNaglowek +  9) >> 2) | (((int16_t)*(chNaglowek + 10) << 6) & 0x7C0)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly +  7) = (((((int16_t)*(chNaglowek + 10) >> 5) | (((int16_t)*(chNaglowek + 11) << 3) & 0x7F8)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly +  8) = (((((int16_t)*(chNaglowek + 12) >> 0) | (((int16_t)*(chNaglowek + 13) << 8) & 0x700)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly +  9) = (((((int16_t)*(chNaglowek + 13) >> 3) | (((int16_t)*(chNaglowek + 14) << 5) & 0x7E0)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly + 10) = (((((int16_t)*(chNaglowek + 14) >> 6) | (((int16_t)*(chNaglowek + 15) << 2) & 0x3FC) | (((uint16_t)*(chNaglowek + 16) << 10) & 0x400)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly + 11) = (((((int16_t)*(chNaglowek + 16) >> 1) | (((int16_t)*(chNaglowek + 17) << 7) & 0x780)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly + 12) = (((((int16_t)*(chNaglowek + 17) >> 4) | (((int16_t)*(chNaglowek + 18) << 4) & 0x7F0)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly + 13) = (((((int16_t)*(chNaglowek + 18) >> 7) | (((int16_t)*(chNaglowek + 19) << 1) & 0x1FE) | (((uint16_t)*(chNaglowek + 20) << 9) & 0x600)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly + 14) = (((((int16_t)*(chNaglowek + 20) >> 2) | (((int16_t)*(chNaglowek + 21) << 6) & 0x7C0)) - SBUS_MIN) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	*(sKanaly + 15) = (((((int16_t)*(chNaglowek + 21) >> 5) | (((int16_t)*(chNaglowek + 22) << 3) & 0x7F8)) - SBUS_MIN ) * (PPM_MAX - PPM_MIN) / (SBUS_MAX - SBUS_MIN)) + PPM_MIN;
	return ERR_OK;
}
