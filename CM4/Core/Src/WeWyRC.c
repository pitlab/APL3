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
#include "GNSS.h"
#include "kontroler_lotu.h"


//definicje SBus: https://github.com/uzh-rpg/rpg_quadrotor_control/wiki/SBUS-Protocol
//S-Bus oraz DShot mają rozdzielczość 11-bitów, wiec przyjmuję taką rozdzielczość sterowania


stRC_t stRC;	//struktura danych odbiorników RC
extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
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


uint8_t chBuforAnalizySBus1[ROZMIAR_BUF_ANA_SBUS];
uint8_t chBuforAnalizySBus2[ROZMIAR_BUF_ANA_SBUS];
volatile uint8_t chWskNapBufAnaSBus1, chWskOprBufAnaSBus1; 	//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych S-Bus1
volatile uint8_t chWskNapBufAnaSBus2, chWskOprBufAnaSBus2; 	//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych S-Bus2
uint8_t chWskNapRamkiSBus1, chWskNapRamkiSBus2;				//wskaźniki napełniania ramek SBus
uint8_t chBuforOdbioruSBus1[ROZM_BUF_ODB_SBUS];
uint8_t chBuforOdbioruSBus2[ROZM_BUF_ODB_SBUS];
uint8_t chRamkaSBus1[ROZMIAR_RAMKI_SBUS];
uint8_t chRamkaSBus2[ROZMIAR_RAMKI_SBUS];
uint8_t chBuforNadawczySBus[ROZMIAR_RAMKI_SBUS] =  {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00};
uint8_t chKonfigWeRC[LICZBA_WEJSC_RC];	//określa jakiego typu sygnał wchodzi z odbiornika
uint8_t chKonfigWyRC[LICZBA_WYJSC_RC];
uint8_t chKorektaPoczatkuRamki;
uint32_t nCzasWysylkiSbus;
extern uint32_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM1"))) nBuforTimDMA[KANALY_MIKSERA][DS_BITOW_DANYCH + DS_BITOW_PRZERWY];
uint8_t chKanalDrazkaRC[LICZBA_DRAZKOW];	//przypisanie kanałów odbiornika RC do funkcji drążków aparatury
uint8_t chFunkcjaKanaluRC[KANALY_FUNKCYJNE];	//przypisanie funkcji do kanału wejściowego odbiornika RC



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
    	HAL_UART_Receive_DMA(&huart4, chBuforOdbioruSBus1, ROZM_BUF_ODB_SBUS);	//włącz odbiór pierwszej ramki
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
		//huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
		//huart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;

		chErr |= HAL_UART_Init(&huart2);
		chErr |= HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_2);
		//chErr |= HAL_UARTEx_EnableFifoMode(&huart2);
		chErr |= HAL_UARTEx_DisableFifoMode(&huart2);

		//ponieważ nie ma jak zarezerwować kanału DMA w HAL bo dostępna jest tylko część odbiorcza, więc USART2 będzie pracował na przerwaniach
		HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
		HAL_UART_Receive_IT(&huart2, chBuforOdbioruSBus2, ROZM_BUF_ODB_SBUS);	//włącz odbiór pierwszej ramki
	}

	//odczytaj z FRAM minima i maksima kanałów RC aby móc je znormalizować
    for (uint16_t n=0; n<KANALY_ODB_RC; n++)
    {
    	stRC.sMin1[n] = CzytajFramU16(FAU_WE_RC1_MIN + n*2);	//16*2U minimalna wartość sygnału RC dla każego kanału
    	stRC.sMax1[n] = CzytajFramU16(FAU_WE_RC1_MAX + n*2);	//16*2U maksymalna wartość sygnału RC dla każego kanału
    	stRC.sMin2[n] = CzytajFramU16(FAU_WE_RC2_MIN + n*2);	//16*2U minimalna wartość sygnału RC dla każego kanału
    	stRC.sMax2[n] = CzytajFramU16(FAU_WE_RC2_MAX + n*2);	//16*2U maksymalna wartość sygnału RC dla każego kanału
    }

    //odczytaj z FRAM numery kanałów dla 4 drążków RC
    for (uint16_t n=0; n<LICZBA_DRAZKOW; n++)
    {
    	chKanalDrazkaRC[n] = CzytajFramU16(FAU_KAN_DRAZKA_RC + n);	//4*1U Numer kanału przypisany do funkcji drążka aparatury: przechylenia, pochylenia, odchylenia i wysokości
#ifdef TESTY
    	assert(chKanalDrazkaRC[n] < LICZBA_DRAZKOW);
#endif
    }

    //odczytaj z FRAM numery funkcji dla wyższych niż 4 kanałów RC
    for (uint16_t n=0; n<KANALY_FUNKCYJNE; n++)
    {
    	chFunkcjaKanaluRC[n] = CzytajFramU16(FAU_FUNKCJA_KAN_RC + n);	//12*1U Numer funkcji przypisanej do kanału RC (5..16)
#ifdef TESTY
    	assert(chFunkcjaKanaluRC[n] < LICZBA_FUNKCJI_RC);
#endif
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
	TIM_OC_InitTypeDef sConfigOC = {0};
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint32_t nHCLK = HAL_RCC_GetHCLKFreq();

	//Część wspólna dla wszystkich ustawień
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;	//częstotliwość do 12 MHz
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;			//domyślnie jest timer albo UART
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = IMPULS_PWM;

	//czytaj konfigurację kanałów wyjściowych RC: Bity 0..3 = Wyjście nieparzyste, bity 4..7 = Wyjście parzyste
	for (uint8_t n=0; n<LICZBA_KONFIG_WYJSC_RC-1; n++)
	{
		CzytajBuforFRAM(FAU_KONF_SERWA12 + n, &chDaneKonfig, 1);
		chKonfigWyRC[KANAL_RC1 + 2*n] = chDaneKonfig & MASKA_TYPU_RC1;
		chKonfigWyRC[KANAL_RC2 + 2*n] = chDaneKonfig >> 4;
	}
	CzytajBuforFRAM(FAU_KONF_SERWA916, &chKonfigWyRC[8], 1);		//konfiguracją ostatniej grupy wyjść jest nietypowa, wiec odczytaj osobno


	//**** kanał 1 - konfiguracja portu PB9 TIM4_CH4 **********************************************************
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	if ((chKonfigWyRC[KANAL_RC1] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;		//wyjście przechodzi przez inwerter, więc wymaga dodatkowego odwrócenia sygnału aby finalnie było niezmienione
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
		htim4.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim4.Init.Period = OKRES_PWM;
		htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim4);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_4, &nBuforTimDMA[KANAL_RC1][0], KANALY_MIKSERA);
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
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;		//wyjście przechodzi przez inwerter, więc wymaga dodatkowego odwrócenia sygnału aby finalnie było niezmienione
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
		htim2.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = OKRES_PWM;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim2);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, &nBuforTimDMA[KANAL_RC2][0], KANALY_MIKSERA);
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
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

		htim2.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = OKRES_PWM;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim2);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &nBuforTimDMA[KANAL_RC3][0], KANALY_MIKSERA);
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
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
		htim3.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = OKRES_PWM;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim3);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, &nBuforTimDMA[KANAL_RC4][0], KANALY_MIKSERA);
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
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
		htim3.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = OKRES_PWM;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim3);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, &nBuforTimDMA[KANAL_RC5][0], KANALY_MIKSERA);
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
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
		htim8.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim8.Init.Period = OKRES_PWM;
		htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim8);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, &nBuforTimDMA[KANAL_RC6][0], KANALY_MIKSERA);
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
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);
		htim8.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim8.Init.Period = OKRES_PWM;
		htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim8);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_3, &nBuforTimDMA[KANAL_RC8][0], KANALY_MIKSERA);
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
	//pracuje jako PWM bez DMA, ponieważ w przerwaniu musi zmieniać stan dekodera a swoją drogą nie ma już wolnych kanałów DMA
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	if ((chKonfigWyRC[KANAL_RC916] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);		//na tym kanale generowanie PWM wymaga przerwań aby przełączyć dekoden kanałów. DMA nie jest używane

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

	//wstępnie ustaw jakieś wartosci początkowe
	for (uint16_t n=8; n<KANALY_SERW; n++)
		uDaneCM4.dane.sSerwo[n] = 1000 + 10 * n;

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
				nBuforTimDMA[n][m] = dane->sSerwo[n] / 2 + 1000;		//przesunięcie z 0..2000 do 1000..2000us
			break;

		case SERWO_PWM200:	//co drugi w buforze jest kanał, pozostałe są zerami
			for (uint8_t m=0; m<KANALY_MIKSERA/2; m++)
			{
				nBuforTimDMA[n][2*m+0] = dane->sSerwo[n] / 2 + 1000;		//przesunięcie z 0..2000 do 1000..2000us
				nBuforTimDMA[n][2*m+1] = 0;
			}
			break;

		case SERWO_PWM100:	//co czwarty w buforze jest kanał, pozostałe są zerami
			for (uint8_t m=0; m<KANALY_MIKSERA/4; m++)
			{
				nBuforTimDMA[n][4*m+0] = dane->sSerwo[n] / 2 + 1000;		//przesunięcie z 0..2000 do 1000..2000us
				nBuforTimDMA[n][4*m+1] = 0;
				nBuforTimDMA[n][4*m+2] = 0;
				nBuforTimDMA[n][4*m+3] = 0;
			}
			break;

		case SERWO_PWM50:	//pierwszy w buforze jest kanał, pozostałe są zerami
			nBuforTimDMA[n][0] = dane->sSerwo[n] / 2 + 1000;		//przesunięcie z 0..2000 do 1000..2000us
			for (uint8_t m=1; m<KANALY_MIKSERA; m++)
				nBuforTimDMA[n][m] = 0;
			break;

		//if DSHot
		case SERWO_DSHOT150:
		case SERWO_DSHOT300:
		case SERWO_DSHOT600:
		case SERWO_DSHOT1200:	chErr |= AktualizujDShotDMA(dane->sSerwo[n], n);	break;

		default: chErr = ERR_BRAK_KONFIG;
		}	//switch
	}	//for

	//starsze 8 wyjść jest aktualizowanych w obsłudze przerwania TIM1_CC_IRQHandler() w pliku stm32h7xx_it.c

	//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);	//serwo kanał 7
	//HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_15);	//serwo kanał 8
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


//fizyczny odbiór HAL_UART_RxCpltCallback znajduje się w pliku GNSS.c



////////////////////////////////////////////////////////////////////////////////
// Odbiór danych SBus z bufora kołowego i formowanie ich w ramkę
// Parametry:
// [we] *chRamkaSBus - wskaźnik na dane formowanej ramki
// [we/wy] *chWskNapRamki - wskaźnik na wskaźnik napełniania ramki SBus
// [we] *chBuforAnalizy - wskaźnik na bufor z danymi wejsciowymi
// [we] chWskNapBuf - wskaźnik napełniania bufora wejsciowego
// [wy] *chWskOprBuf - wskaźnik na wskaźnik opróżniania bufora wejściowego
// Zwraca: kod wykonania operacji: BLAD_GOTOWE gdy jest kompletna ramka, BLAD_OK gdy skończyły się dane ale nie skompetowano jeszcze ramki
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t FormowanieRamkiSBus(uint8_t *chRamkaSBus, uint8_t *chWskNapRamki, uint8_t *chBuforAnalizy, uint8_t chWskNapBuf, uint8_t *chWskOprBuf)
{
	uint8_t chDane;

	while (chWskNapBuf != *chWskOprBuf)
	{
		chDane = chBuforAnalizy[*chWskOprBuf];

		// detekcja początku ramki po wykryciu nagłówka i poprzedzającej go stopki
		if ((chDane == SBUS_NAGLOWEK) && (chBuforAnalizy[(*chWskOprBuf - 1) & MASKA_ROZM_BUF_ANA_SBUS] == SBUS_STOPKA))
			*chWskNapRamki = 0;

		chRamkaSBus[*chWskNapRamki] = chDane;
		if (*chWskNapRamki < ROZMIAR_RAMKI_SBUS)
			(*chWskNapRamki)++;

		(*chWskOprBuf)++;
		(*chWskOprBuf) &= MASKA_ROZM_BUF_ANA_SBUS;	//zapętlenie wskaźnika bufora kołowego

		if ((chDane == SBUS_STOPKA ) && (*chWskNapRamki == ROZMIAR_RAMKI_SBUS))
			return BLAD_GOTOWE;	//odebrano całą ramkę. Reszta danych będzie obrobiona w następnym przebiegu
	}
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna zbiór nowych ekstremalnych wartości wszystkich kanalów z obu odbiorników RC
// Parametry: brak - funkcja do wywyołania z zewnątrz
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RozpocznijZbieranieEkstremowWejscRC(void)
{
	//rozpoacznij zbieranie gdy jeszcze nie jest rozpoczęte
	if ((stRC.chStatus & STATRC_ZBIERAJ_EKSTREMA) != STATRC_ZBIERAJ_EKSTREMA)
	{
		stRC.chStatus |= STATRC_ZBIERAJ_EKSTREMA;
		for (uint16_t n=0; n<KANALY_ODB_RC; n++)
		{
			stRC.sMin1[n] = PPM_MAX;
			stRC.sMax1[n] = PPM_MIN;
			stRC.sMin2[n] = PPM_MAX;
			stRC.sMax2[n] = PPM_MIN;
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Zbiera ekstremalne wartości wszystkich kanalów z obu odbiorników RC
// Parametry: [io] *stRC - wskaźnik na strukturę przechowujaca dane odbiorników
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ZbierajEkstremaWejscRC(stRC_t *stRC)
{
	if (stRC->chStatus & STATRC_ZBIERAJ_EKSTREMA)
	{
		for (uint16_t n=0; n<KANALY_ODB_RC; n++)
		{
			if (stRC->sOdb1[n] < stRC->sMin1[n])
				stRC->sMin1[n] = stRC->sOdb1[n];
			if (stRC->sOdb1[n] > stRC->sMax1[n])
				stRC->sMax1[n] = stRC->sOdb1[n];
			if ((stRC->sMin1[n] < PPM_M75) && (stRC->sMax1[n] > PPM_P75))
				stRC->chStatus |= STATRC_ZEBRANO_EKSTR1;

			if (stRC->sOdb2[n] < stRC->sMin2[n])
				stRC->sMin2[n] = stRC->sOdb2[n];
			if (stRC->sOdb2[n] > stRC->sMax2[n])
				stRC->sMax2[n] = stRC->sOdb2[n];
			if ((stRC->sMin2[n] < PPM_M75) && (stRC->sMax2[n] > PPM_P75))
				stRC->chStatus |= STATRC_ZEBRANO_EKSTR2;
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Kończy zbiór ekstremalnych wartości wszystkich kanalów z obu odbiorników RC
// Wartości o ile zostały zebrane są zapisywane we FRAM a jezęli nie to przywracane są poprzednie wartości
// Parametry: brak - funkcja do wywołania z zewnątrz
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ZapiszEkstremaWejscRC(void)
{
	if ((stRC.chStatus & STATRC_ZEBRANO_EKSTR1) || (stRC.chStatus & STATRC_ZEBRANO_EKSTR2))
	{
		for (uint16_t n=0; n<KANALY_ODB_RC; n++)
		{
			//odbiornik 1
			if (stRC.chStatus & STATRC_ZEBRANO_EKSTR1)
			{
				ZapiszFramU16(FAU_WE_RC1_MIN + n*2, stRC.sMin1[n]);		//16*2U minimalna wartość sygnału RC dla każego kanału
				ZapiszFramU16(FAU_WE_RC1_MAX + n*2, stRC.sMax1[n]);		//16*2U maksymalna wartość sygnału RC dla każego kanału
			}
			else	//jeżeli nie zebrano danych, bo np. nie ma podłączonego odbiornika to przywróć ostatnio zapisane wartosci
			{
				stRC.sMin1[n] = CzytajFramU16(FAU_WE_RC1_MIN + n*2);
				stRC.sMax1[n] = CzytajFramU16(FAU_WE_RC1_MAX + n*2);
			}

			//odbiornik 2
			if (stRC.chStatus & STATRC_ZEBRANO_EKSTR2)
			{
				ZapiszFramU16(FAU_WE_RC2_MIN + n*2, stRC.sMin2[n]);
				ZapiszFramU16(FAU_WE_RC2_MAX + n*2, stRC.sMin2[n]);
			}
			else
			{
				stRC.sMin2[n] = CzytajFramU16(FAU_WE_RC2_MIN + n*2);
				stRC.sMax2[n] = CzytajFramU16(FAU_WE_RC2_MAX + n*2);
			}
		}
		stRC.chStatus &= ~(STATRC_ZBIERAJ_EKSTREMA | STATRC_ZEBRANO_EKSTR1 | STATRC_ZEBRANO_EKSTR2);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Normalizuje sygnały RC sprowadzajac je do standardowego zakresu 1000..2000
// Parametry:
// [i] sWejscie - wejście sygnału odebranego z aparatury
// [i] sMin, sMax - minimalna i maksymalna wartość kanału
// [o] *sWyjscie - wskaźnik na wartość kanałów po normalizacji
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void NormalizujWejsciaRC(uint16_t sWejscie, uint16_t sMin, uint16_t sMax, uint16_t *sWyjscie)
{
	for (uint16_t n=0; n<KANALY_ODB_RC; n++)
	{

	}

}



////////////////////////////////////////////////////////////////////////////////
// Porównuje dane z obu odbiorników RC i wybiera ten lepszy przepisując jego dane do struktury danych CM4
// Zrobić: Sygnały z róznych typów odbiorników i róznych protokołów powinny zostać znormalizowane przed porównaniem
// Parametry:
// [we] *stRC - wskaźnik na strukturę danych odbiorników RC
// [wy] *psDaneCM4 - wskaźnik na strukturę danych CM4
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t DywersyfikacjaOdbiornikowRC(stRC_t *stRC, stWymianyCM4_t *psDaneCM4, stWymianyCM7_t *psDaneCM7)
{
	uint8_t chErr = BLAD_OK;
	uint32_t nCzasBiezacy = PobierzCzas();
	uint32_t nCzasRC1, nCzasRC2;
	uint8_t n;

	//Sprawdź kiedy przyszły ostatnie dane RC
	nCzasRC1 = MinalCzas2(stRC->nCzasWe1, nCzasBiezacy);
	nCzasRC2 = MinalCzas2(stRC->nCzasWe2, nCzasBiezacy);

	//if ((nCzasRC1 < 2*OKRES_RAMKI_PPM_RC) && (nCzasRC2 > 2*OKRES_RAMKI_PPM_RC))	//działa odbiornik 1, nie działa 2
	if (nCzasRC1 < 2*OKRES_RAMKI_PPM_RC)	//działa odbiornik 1
	{
		for (n=0; n<KANALY_ODB_RC; n++)
		{
			if (stRC->sZdekodowaneKanaly1 & (1<<n))
			{
				if ((psDaneCM7->chOdbiornikRC == ODB_RC1) || (psDaneCM7->chOdbiornikRC == ODB_OBA))
					psDaneCM4->sKanalRC[n] = stRC->sOdb1[n]; 	//przepisz zdekodowane kanały
				stRC->sZdekodowaneKanaly1 &= ~(1<<n);		//kasuj bit obrobionego kanału
			}
		}
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);				//kanał serw 5 skonfigurowany jako IO
	}
	else
	//if ((nCzasRC1 > 2*OKRES_RAMKI_PPM_RC) && (nCzasRC2 < 2*OKRES_RAMKI_PPM_RC))	//nie działa odbiornik 1, działa 2
	if (nCzasRC2 < 2*OKRES_RAMKI_PPM_RC)	//działa odbiornik 2
	{
		for (n=0; n<KANALY_ODB_RC; n++)
		{
			if (stRC->sZdekodowaneKanaly2 & (1<<n))
			{
				if ((psDaneCM7->chOdbiornikRC == ODB_RC2) || (psDaneCM7->chOdbiornikRC == ODB_OBA))
					psDaneCM4->sKanalRC[n] = stRC->sOdb2[n]; 	//przepisz zdekodowane kanały
				stRC->sZdekodowaneKanaly2 &= ~(1<<n);		//kasuj bit obrobionego kanału
			}
		}
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	}
	else	//Odzyskiwanie synchronizacji: Jeżeli nie było nowych danych przez czas 2x trwania ramki to wymuś odbiór
	if (nCzasRC1 > 2*OKRES_RAMKI_PPM_RC)
	{
		HAL_UART_Receive_DMA(&huart4, chBuforOdbioruSBus1, ROZM_BUF_ODB_SBUS);
	}
	else
	if (nCzasRC2 > 2*OKRES_RAMKI_PPM_RC)
	{
		HAL_UART_Receive_IT(&huart4, chBuforOdbioruSBus2, ROZM_BUF_ODB_SBUS);
	}


	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Analizuje gotowy znormalizowany sygnał z odbiornika RC uruchamiając specjalne funkcje sterowane drążkami aparatury
// Parametry:
// [we] *psDaneCM4 - wskaźnik na strukturę danych CM4
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t AnalizujSygnalRC(stWymianyCM4_t* psDaneCM4)
{
	uint8_t chBlad = BLAD_OK;

	//Dodać  sprawdzenie czy przyszły nowe dane z odbiornika

    //sprawdź warunek uzbrojenia silników, czyli gaz na mininum i kierunek w prawo
	if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[WYSO]] < PPM_M90) && (psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] > PPM_P90))
		chBlad = UzbrojSilniki(psDaneCM4);

    //sprawdź warunek rozbrojenia silników, czyli gaz na mininum i kierunek w lewo
	if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[WYSO]] < PPM_M90) && (psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] < PPM_M90))
		RozbrojSilniki(psDaneCM4);

    //sprawdź warunek ..., czyli gaz na maksimum i kierunek w lewo
	if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[WYSO]] > PPM_P90) && (psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] < PPM_M90) && ((psDaneCM4->chFlagiLotu & FL_SILN_UZBROJONE) != FL_SILN_UZBROJONE))
    {
        //obsługa ...
    }

    //sprawdź warunek zerowania zużycia energii, czyli gaz na maksimum i kierunek w prawo
    if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[WYSO]] > PPM_P90) && (psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] > PPM_P90) && ((psDaneCM4->chFlagiLotu & FL_SILN_UZBROJONE) != FL_SILN_UZBROJONE))
    {
        //obsługa resetownia licznika energii
    }
    return chBlad;
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
	//ramka S-Bus zaczyna się od nagłówka 0x0F, który powinien być pierwszym odebranym bajtem.
	//Jeżeli nie trafia na początek, to zmniejsz liczbę odebieranych danych, tak aby przesunął się na początek
	chKorektaPoczatkuRamki = n;
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



////////////////////////////////////////////////////////////////////////////////
// Funkcja nadrzędna wywoływana z pętli głównej, skupiająca w sobie funkcje potrzebne do obsługi ramki S-Bus
// Parametry: brak
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaRamkiSBus(void)
{
	uint8_t chBlad;

	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);				//kanał serw 5 skonfigurowany jako IO

	//obsługa kanału 1
	chBlad = FormowanieRamkiSBus(chRamkaSBus1, &chWskNapRamkiSBus1, chBuforAnalizySBus1, (uint8_t)chWskNapBufAnaSBus1, (uint8_t*)&chWskOprBufAnaSBus1);
	if (chBlad == BLAD_GOTOWE)
	{
		chBlad = DekodowanieRamkiBSBus(chRamkaSBus1, stRC.sOdb1);
		if (chBlad == BLAD_OK)
		{
			stRC.sZdekodowaneKanaly1 = 0xFFFF;
			stRC.nCzasWe1 = PobierzCzas();
		}
	}

	//obsługa kanału 2
	chBlad = FormowanieRamkiSBus(chRamkaSBus2, &chWskNapRamkiSBus2, chBuforAnalizySBus2, (uint8_t)chWskNapBufAnaSBus2, (uint8_t*)&chWskOprBufAnaSBus2);
	if (chBlad == BLAD_GOTOWE)
	{
		chBlad = DekodowanieRamkiBSBus(chRamkaSBus2, stRC.sOdb2);
		if (chBlad == BLAD_OK)
		{
			stRC.sZdekodowaneKanaly2 = 0xFFFF;
			stRC.nCzasWe2 = PobierzCzas();
		}
	}

	//scalenie obu kanałów w jedne dane dane odbiornika RC
	chBlad = DywersyfikacjaOdbiornikowRC(&stRC, &uDaneCM4.dane, &uDaneCM7.dane);


	//sprawdź czy trzeba już wysłać nową ramkę Sbus
	uint32_t nCzasRC = MinalCzas(nCzasWysylkiSbus);
	if (nCzasRC >= OKRES_RAMKI_SBUS)
	{
		nCzasWysylkiSbus = PobierzCzas();
		HAL_UART_Transmit_DMA(&huart4, chBuforNadawczySBus, ROZM_BUF_ODB_SBUS);	//wyślij kolejną ramkę
	}
	return chBlad;
}




