//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł ubsługi wejść odbiorników RC i wyjść serw/ESC
//
// (c) PitLab 2025
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "WeWyRC.h"
#include "konfig_fram.h"
#include "petla_glowna.h"
#include "fram.h"
#include "dshot.h"

//definicje SBus: https://github.com/uzh-rpg/rpg_quadrotor_control/wiki/SBUS-Protocol
//S-Bus oraz DShot mają rozdzielczość 11-bitów, wiec przyjmuję taką rozdzielczość sterowania


stRC_t stRC;	//struktura danych odbiorników RC
extern unia_wymianyCM4_t uDaneCM4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch4;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern DMA_HandleTypeDef hdma_tim8_ch3;

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
uint8_t chBuforOdbioruSBus1[ROZMIAR_BUF_SBUS];
uint8_t chBuforOdbioruSBus2[ROZMIAR_BUF_SBUS];
uint8_t chBuforNadawczySBus[ROZMIAR_BUF_SBUS] =  {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
uint8_t chKonfigWeRC[LICZBA_WEJSC_RC];
uint8_t chKonfigWyRC[LICZBA_WYJSC_RC];

uint32_t nCzasWysylkiSbus;
extern uint32_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM1"))) nBuforDShot[KANALY_MIKSERA][DS_BITOW_DANYCH + DS_BITOW_PRZERWY];

////////////////////////////////////////////////////////////////////////////////
// Funkcja wczytuje z FRAM konfigurację odbiorników RC
// Parametry Sbus 100kBps, 8E2
// Parametry:
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujWejsciaRC(void)
{
	uint8_t chErr = BLAD_OK;
	uint8_t chDaneKonfig;
	TIM_IC_InitTypeDef sConfigIC = {0};
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//Część wspólna dla wszystkich ustawień
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

	//czytaj konfigurację obu kanałów wejściowych RC
	CzytajBuforFRAM(FAU_KONF_ODB_RC, &chDaneKonfig, 1);
	chKonfigWeRC[KANAL_RC1] = chDaneKonfig & MASKA_TYPU_RC1;
	chKonfigWeRC[KANAL_RC2] = chDaneKonfig >> 4;

	//ustaw Port PB8 jako UART4_RX lub TIM4_CH3
	__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8;
	if (chKonfigWeRC[KANAL_RC1] == ODB_RC_CPPM)
	{
	    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		__HAL_RCC_TIM4_CLK_ENABLE();

		sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		sConfigIC.ICFilter = 7;
		chErr |= HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);
	}
	else
	if (chKonfigWeRC[KANAL_RC1] == ODB_RC_SBUS)
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
		chErr |= HAL_UART_Init(&huart4);
		chErr |= HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_2);
		chErr |= HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_2);
		//chErr |= HAL_UARTEx_EnableFifoMode(&huart4);
		chErr |= HAL_UARTEx_DisableFifoMode(&huart4);

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
		chErr |= HAL_DMA_Init(&hdma_uart4_rx);
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

	if (chKonfigWeRC[KANAL_RC2] == ODB_RC_CPPM)
	{
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		__HAL_RCC_TIM2_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);	//odbiór PPM wymaga przerwań

		sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		sConfigIC.ICFilter = 7;
		chErr |= HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
	}
	else
	if (chKonfigWeRC[KANAL_RC2] == ODB_RC_SBUS)
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
		chErr |= HAL_UART_Init(&huart2);
		chErr |= HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_2);
		chErr |= HAL_UARTEx_EnableFifoMode(&huart2);

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
		chErr |= HAL_DMA_Init(&hdma_uart2_rx);
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
// Wczytuje z FRAM konfigurację kanałów serw i konfiguruje je jako PWM, DShot, S-Bus, IO, lub wejście analogowe
// Parametry:
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujWyjsciaRC(void)
{
	uint8_t chErr = BLAD_OK;
	uint8_t chDaneKonfig;
	//TIM_IC_InitTypeDef sConfigIC = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint32_t nHCLK = HAL_RCC_GetHCLKFreq();

	//Część wspólna dla wszystkich ustawień
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;	//częstotliwość do 12 MHz
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;			//domyslnie jest timer lbo UART
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = IMPULS_PWM;

	//czytaj konfigurację kanałów wyjściowych RC: Bity 0..3 = Wyjście nieparzyste, bity 4..7 = Wyjście parzyste
	for (uint8_t n=0; n<4; n++)
	{
		CzytajBuforFRAM(FAU_KONF_SERWA12 + n, &chDaneKonfig, 1);
		chKonfigWyRC[KANAL_RC1 + 2*n] = chDaneKonfig & MASKA_TYPU_RC1;
		chKonfigWyRC[KANAL_RC2 + 2*n] = chDaneKonfig >> 4;
	}

	//**** kanał 1 - konfiguracja portu PB9 TIM4_CH4 **********************************************************
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	if ((chKonfigWyRC[KANAL_RC1] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
		htim4.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim4.Init.Period = OKRES_PWM;
		htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim4);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_4, &nBuforDShot[KANAL_RC1][0], KANALY_MIKSERA);
	}
	else
	if (chKonfigWyRC[KANAL_RC1] == SERWO_IO)	//wyjście IO do debugowania
	{
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if (chKonfigWyRC[KANAL_RC1] == SERWO_SBUS)		//wyjście jako S-Bus 100 kBps, 8E2
	{
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
		chErr |= HAL_UART_Init(&huart4);
		chErr |= HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_2);
		chErr |= HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_2);
		//chErr = HAL_UARTEx_EnableFifoMode(&huart4);
		chErr |= HAL_UARTEx_DisableFifoMode(&huart4);

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
		chErr |= HAL_DMA_Init(&hdma_uart4_tx);
	    __HAL_LINKDMA(&huart4, hdmatx, hdma_uart4_tx);
		HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	}
	else
		chErr |= ERR_BRAK_KONFIG;

	//**** kanał 2 - konfiguracja portu PB10 - TIM2_CH3, USART3_TX, MOD_QSPI_CS **********************************************************
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	if ((chKonfigWyRC[KANAL_RC2] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
		htim2.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = OKRES_PWM;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim2);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, &nBuforDShot[KANAL_RC2][0], KANALY_MIKSERA);
	}
	else
	if (chKonfigWyRC[KANAL_RC2] == SERWO_DSHOT150)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC2);
	else
	if (chKonfigWyRC[KANAL_RC2] == SERWO_DSHOT300)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC2);
	else
	if (chKonfigWyRC[KANAL_RC2] == SERWO_DSHOT600)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC2);
	else
	if (chKonfigWyRC[KANAL_RC2] == SERWO_DSHOT1200)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC2);
	else
	if (chKonfigWyRC[KANAL_RC2] == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	/*else
	if (((chTyp & MASKA_TYPU_RC2) >> 4) == SERWO_SBUS)	//wyjście jako S-Bus 100 kBps, 8E2
	{
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
		chErr |= HAL_UART_Init(&huart3);
		chErr |= HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_2);
		chErr |= HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_2);
		//chErr |= HAL_UARTEx_EnableFifoMode(&huart3);
		chErr |= HAL_UARTEx_DisableFifoMode(&huart3);

		HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
	}*/
	else
	if (chKonfigWyRC[KANAL_RC2] == SERWO_ALTER)	//MOD_QSPI_CS
	{
		GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
		chErr |= ERR_BRAK_KONFIG;


	//**** kanał 3 - konfiguracja portu PA15 TIM2_CH1 **********************************************************
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	if ((chKonfigWyRC[KANAL_RC3] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

		htim2.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = OKRES_PWM;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim2);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &nBuforDShot[KANAL_RC3][0], KANALY_MIKSERA);
	}
	else
	if (chKonfigWyRC[KANAL_RC3] == SERWO_DSHOT150)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC3);
	else
	if (chKonfigWyRC[KANAL_RC3] == SERWO_DSHOT300)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC3);
	else
	if (chKonfigWyRC[KANAL_RC3] == SERWO_DSHOT600)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC3);
	else
	if (chKonfigWyRC[KANAL_RC3] == SERWO_DSHOT1200)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC3);
	else
	if (chKonfigWyRC[KANAL_RC3] == SERWO_IO)	//wyjście IO do debugowania
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else
		chErr |= ERR_BRAK_KONFIG;


	//**** kanał 4 - konfiguracja portu PB0 TIM3_CH3 **********************************************************
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	if ((chKonfigWyRC[KANAL_RC4] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
		htim3.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = OKRES_PWM;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim3);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, &nBuforDShot[KANAL_RC4][0], KANALY_MIKSERA);
	}
	else
	if (chKonfigWyRC[KANAL_RC4] == SERWO_DSHOT150)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC4);
	else
	if (chKonfigWyRC[KANAL_RC4] == SERWO_DSHOT300)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC4);
	else
	if (chKonfigWyRC[KANAL_RC4] == SERWO_DSHOT600)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC4);
	else
	if (chKonfigWyRC[KANAL_RC4] == SERWO_DSHOT1200)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC4);
	else
	if (chKonfigWyRC[KANAL_RC4] == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if (chKonfigWyRC[KANAL_RC4] == SERWO_ANALOG)	//ADC12_INP9
	{
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
		chErr |= ERR_BRAK_KONFIG;


	//**** kanał 5 - konfiguracja portu PB1 TIM3_CH4 **********************************************************
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	if ((chKonfigWyRC[KANAL_RC5] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
		htim3.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = OKRES_PWM;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim3);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, &nBuforDShot[KANAL_RC5][0], KANALY_MIKSERA);
	}
	else
	if (chKonfigWyRC[KANAL_RC5] == SERWO_DSHOT150)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC5);
	else
	if (chKonfigWyRC[KANAL_RC5] == SERWO_DSHOT300)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC5);
	else
	if (chKonfigWyRC[KANAL_RC5] == SERWO_DSHOT600)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC5);
	else
	if (chKonfigWyRC[KANAL_RC5] == SERWO_DSHOT1200)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC5);
	else
	if (chKonfigWyRC[KANAL_RC5] == SERWO_IO)	//wyjście IO do debugowania
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
	if (chKonfigWyRC[KANAL_RC5] == SERWO_ANALOG)		//ADC12_INP5
	{
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else
		chErr |= ERR_BRAK_KONFIG;


	//**** kanał 6 - konfiguracja portu PI5 TIM8_CH1 **********************************************************
	__HAL_RCC_GPIOI_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	if ((chKonfigWyRC[KANAL_RC6] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
		htim8.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim8.Init.Period = OKRES_PWM;
		htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim8);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, &nBuforDShot[KANAL_RC6][0], KANALY_MIKSERA);
	}
	else
	if (chKonfigWyRC[KANAL_RC6] == SERWO_DSHOT150)
	{
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC6);
	}
	else
	if (chKonfigWyRC[KANAL_RC6] == SERWO_DSHOT300)
	{
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC6);
	}
	else
	if (chKonfigWyRC[KANAL_RC6] == SERWO_DSHOT600)
	{
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC6);
	}
	else
	if (chKonfigWyRC[KANAL_RC6] == SERWO_DSHOT1200)
	{
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC6);
	}
	else
	if (chKonfigWyRC[KANAL_RC6] == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	}
	else
	if (chKonfigWyRC[KANAL_RC6] == SERWO_ANALOG)	//ADC1_INP2
	{
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	}
	else
		chErr |= ERR_BRAK_KONFIG;


	//**** kanał 7 - konfiguracja portu PI10  - brak timera na porcie **********************************************************
	__HAL_RCC_GPIOI_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_10;

	if (chKonfigWyRC[KANAL_RC7] == SERWO_IO)	//wyjście IO do debugowania
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	}
	else
		chErr |= ERR_BRAK_KONFIG;


	//**** kanał 8 - konfiguracja portu PH15 TIM8_CH3N **********************************************************
	__HAL_RCC_GPIOH_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);		//domyslnie ma być timer. w przypadku IO lub ADC, konfiguracja będzie nadpisana

	if ((chKonfigWyRC[KANAL_RC8] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
		htim8.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim8.Init.Period = OKRES_PWM;
		htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim8);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_3, &nBuforDShot[KANAL_RC8][0], KANALY_MIKSERA);
	}
	else
	if (chKonfigWyRC[KANAL_RC8] == SERWO_DSHOT150)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC8);
	else
	if (chKonfigWyRC[KANAL_RC8] == SERWO_DSHOT300)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC8);
	else
	if (chKonfigWyRC[KANAL_RC8] == SERWO_DSHOT600)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC8);
	else
	if (chKonfigWyRC[KANAL_RC8] == SERWO_DSHOT1200)
		chErr |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC8);
	else
	if (chKonfigWyRC[KANAL_RC8] == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	}
	else
		chErr |= ERR_BRAK_KONFIG;


	//**** kanały 9-16 - konfiguracja portu PA8 TIM1_CH1 **********************************************************
	//pracuje jako PWM bez DMA, ponieważ brakuje zasobów
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	if ((chKonfigWyRC[KANAL_RC916] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);		//na tym kanala generowanie PWM wymaga przerwań aby przełączyć dekoden kanałów. DMA nie jest używane

		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

		htim1.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim1.Init.Period = OKRES_PWM;
		htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim1);
		chErr |= HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	}
	else
	if (chKonfigWyRC[KANAL_RC916] == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else
		chErr |= ERR_BRAK_KONFIG;

	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje dane wysyłane na wyjścia RC
// Parametry: *dane - wskaźnik na strukturę zawierajacą wartości wyjścia regulatorów PID
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktualizujWyjsciaRC(stWymianyCM4_t *dane)
{
	uint8_t chErr = BLAD_OK;

	//aktualizuj młodsze 8 kanałów serw/ESC. Starsza mogą być tylko PWM
	for (uint8_t n=0; n<KANALY_MIKSERA; n++)
	{
		//sprawdź rodzaj ustawionego protokołu wyjscia
		switch (chKonfigWyRC[n])
		{
		case SERWO_PWM400:
			for (uint8_t m=0; m<KANALY_MIKSERA; m++)
				nBuforDShot[n][m] = dane->sSerwo[n] / 2 + 1000;		//przesunięcie z 0..2000 do 1000..2000us
			break;

		case SERWO_PWM200:	//co drugi w buforze jest kanał, pozostałe są zerami
			for (uint8_t m=0; m<KANALY_MIKSERA/2; m++)
			{
				nBuforDShot[n][2*m+0] = dane->sSerwo[n] / 2 + 1000;		//przesunięcie z 0..2000 do 1000..2000us
				nBuforDShot[n][2*m+1] = 0;
			}
			break;

		case SERWO_PWM100:	//co czwarty w buforze jest kanał, pozostałe są zerami
			for (uint8_t m=0; m<KANALY_MIKSERA/4; m++)
			{
				nBuforDShot[n][4*m+0] = dane->sSerwo[n] / 2 + 1000;		//przesunięcie z 0..2000 do 1000..2000us
				nBuforDShot[n][4*m+1] = 0;
				nBuforDShot[n][4*m+2] = 0;
				nBuforDShot[n][4*m+3] = 0;
			}
			break;

		case SERWO_PWM50:	//pierwszy w buforze jest kanał, pozostałe są zerami
			nBuforDShot[n][0] = dane->sSerwo[n] / 2 + 1000;		//przesunięcie z 0..2000 do 1000..2000us
			for (uint8_t m=1; m<KANALY_MIKSERA; m++)
				nBuforDShot[n][m] = 0;
			break;

		//if DSHot
		case SERWO_DSHOT150:
		case SERWO_DSHOT300:
		case SERWO_DSHOT600:
		case SERWO_DSHOT1200:	chErr |= AktualizujDShotDMA(dane->sSerwo[n], n);	break;

		default: chErr = ERR_BRAK_KONFIG;
		}	//switch
	}	//for

	//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);		//serwo kanał 7
	//HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_15);	//serwo kanał 8
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
	uint8_t chErr = BLAD_OK;
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
		if (chErr == BLAD_OK)
			stRC->sZdekodowaneKanaly1 = 0xFFFF;
	}

	if (nCzasRC2 < OKRES_RAMKI_PPM_RC)
	{
		chErr = DekodowanieRamkiBSBus(chBuforOdbioruSBus2, stRC->sOdb2);
		if (chErr == BLAD_OK)
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
		HAL_UART_Transmit_DMA(&huart4, chBuforNadawczySBus, ROZMIAR_BUF_SBUS);	//wyślij kolejną ramkę
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
	return BLAD_OK;
}
