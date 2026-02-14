//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł ubsługi wejść odbiorników RC i wyjść serw/ESC
//
// (c) PitLab 2025
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "WeWyRC.h"
#include "sbus.h"
#include "konfig_fram.h"
#include "petla_glowna.h"
#include "GNSS.h"
#include "fram.h"
#include "dshot.h"
#include "kontroler_lotu.h"
#include "sample_audio.h"

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
UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch4;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern DMA_HandleTypeDef hdma_tim8_ch3;



extern uint8_t chBuforOdbioruSBus1[ROZM_BUF_ODB_SBUS];
extern uint8_t chBuforOdbioruSBus2[ROZM_BUF_ODB_SBUS];

uint8_t chKonfigWeRC[LICZBA_WEJSC_RC];	//określa jakiego typu sygnał wchodzi z odbiornika
uint8_t chKonfigWyRC[LICZBA_WYJSC_RC];
extern uint32_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM1"))) nBuforTimDMA[KANALY_MIKSERA][DS_BITOW_DANYCH + DS_BITOW_PRZERWY];
uint8_t chKanalDrazkaRC[LICZBA_DRAZKOW];	//przypisanie kanałów odbiornika RC do funkcji drążków aparatury
uint8_t chFunkcjaKanaluRC[KANALY_FUNKCYJNE];	//funkcje przypisane do kanałów wejściowych odbiornika RC
uint8_t chFunkcjaWyjscRC[KANALY_WYJSC_RC];		//funkcje przypisane do kanałów wyjściowych


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
		chErr |= InicjujUart4RxJakoSbus(&GPIO_InitStruct);
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
		chErr |= InicjujUart2RxJakoSbus(&GPIO_InitStruct);
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

	//odczytaj konfigurację funkcji pełnionych przez kanały wyjściowe RC
	CzytajBuforFRAM(FAU_FUNKCJA_WY_RC , chFunkcjaWyjscRC, KANALY_WYJSC_RC);


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
		InicjujUart4TxJakoSbus(&GPIO_InitStruct);
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
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje dane wysyłane na wyjścia RC 1..8 w zależności od przypisanej funkcji
// Parametry: *dane - wskaźnik na strukturę zawierajacą wartości wyjścia regulatorów PID
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktualizujWyjsciaRC(stWymianyCM4_t *daneCM4)
{
	uint8_t chErr = BLAD_OK;
	uint32_t nWyjście;

	//aktualizuj młodsze 8 kanałów serw/ESC. Starsze mogą być tylko PWM
	for (uint8_t n=0; n<KANALY_MIKSERA; n++)
	{
		nWyjście = PobierzFunkcjeWyjsciaRC(n, chFunkcjaWyjscRC, daneCM4);
		daneCM4->sWyjscieRC[n] = nWyjście;

		//sprawdź rodzaj ustawionego protokołu wyjscia
		switch (chKonfigWyRC[n])
		{
		case SERWO_PWM400:
			for (uint8_t m=0; m<KANALY_MIKSERA; m++)
				nBuforTimDMA[n][m] = nWyjście;
			break;

		case SERWO_PWM200:	//co drugi w buforze jest kanał, pozostałe są zerami
			for (uint8_t m=0; m<KANALY_MIKSERA/2; m++)
			{
				nBuforTimDMA[n][2*m+0] = nWyjście;
				nBuforTimDMA[n][2*m+1] = 0;
			}
			break;

		case SERWO_PWM100:	//co czwarty w buforze jest kanał, pozostałe są zerami
			for (uint8_t m=0; m<KANALY_MIKSERA/4; m++)
			{
				nBuforTimDMA[n][4*m+0] = nWyjście;
				nBuforTimDMA[n][4*m+1] = 0;
				nBuforTimDMA[n][4*m+2] = 0;
				nBuforTimDMA[n][4*m+3] = 0;
			}
			break;

		case SERWO_PWM50:	//pierwszy w buforze jest kanał, pozostałe są zerami
			nBuforTimDMA[n][0] = nWyjście;
			for (uint8_t m=1; m<KANALY_MIKSERA; m++)
				nBuforTimDMA[n][m] = 0;
			break;

		//if DSHot
		case SERWO_DSHOT150:
		case SERWO_DSHOT300:
		case SERWO_DSHOT600:
		case SERWO_DSHOT1200:	chErr |= AktualizujDShotDMA(PobierzFunkcjeWyjsciaRC(n, chFunkcjaWyjscRC, daneCM4), n);	break;

		default: chErr = ERR_BRAK_KONFIG;
		}	//switch
	}	//for

	//starsze 8 wyjść jest aktualizowanych w obsłudze przerwania TIM1_CC_IRQHandler() w pliku stm32h7xx_it.c
	//tutaj tylko ustaw wyjscia na podstawie konfiguracji
	for (uint8_t n=KANALY_MIKSERA; n<KANALY_WYJSC_RC; n++)
		daneCM4->sWyjscieRC[n] = PobierzFunkcjeWyjsciaRC(n, chFunkcjaWyjscRC, daneCM4);

	//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);	//serwo kanał 7
	//HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_15);	//serwo kanał 8
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera wartość do wysterowania kanału wyjściowego zdefiniowaną funkcjonalnością
// Parametry:
//  chNrWyjscia - indeks wyjścia RC
//  *chFunkcja - wskaźnik na tablicę zawierajacą funkcję dla każdego wyjścia
//  *daneCM4 - wskaźnik na strukturę danych rdzenia CM4
// Zwraca: wysterowanie kanału
////////////////////////////////////////////////////////////////////////////////
uint32_t PobierzFunkcjeWyjsciaRC(uint8_t chNrWyjscia, uint8_t *chFunkcja, stWymianyCM4_t *daneCM4)
{
	uint32_t nWyjście;
	uint8_t chIndeksFunkcji = chFunkcja[chNrWyjscia];

	switch(chIndeksFunkcji)
	{
	case FSER_SILNIK1:
	case FSER_SILNIK2:
	case FSER_SILNIK3:
	case FSER_SILNIK4:
	case FSER_SILNIK5:
	case FSER_SILNIK6:
	case FSER_SILNIK7:
	case FSER_SILNIK8:	nWyjście = daneCM4->sSilnik[chIndeksFunkcji - FSER_SILNIK1];	break;	//steruj silnikiem 1
	case FSER_WE_RC1:
	case FSER_WE_RC2:
	case FSER_WE_RC3:
	case FSER_WE_RC4:
	case FSER_WE_RC5:
	case FSER_WE_RC6:
	case FSER_WE_RC7:
	case FSER_WE_RC8:
	case FSER_WE_RC9:
	case FSER_WE_RC10:
	case FSER_WE_RC11:
	case FSER_WE_RC12:
	case FSER_WE_RC13:
	case FSER_WE_RC14:
	case FSER_WE_RC15:
	case FSER_WE_RC16:	nWyjście = daneCM4->sKanalRC[chIndeksFunkcji - FSER_WE_RC1];	break;	//przepisanie wejścia na wyjście

	default:	nWyjście = 0;
	}
	return nWyjście;
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
// [wy] *psDaneCM7 - wskaźnik na strukturę danych CM7
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t AnalizujSygnalRC(stWymianyCM4_t* psDaneCM4)
{
	uint8_t chBlad = BLAD_OK;

	//Dodać  sprawdzenie czy przyszły nowe dane z odbiornika

    //sprawdź warunek uzbrojenia silników, czyli gaz na mininum i kierunek w prawo
	if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[WYSO]] < PPM_M90) && (psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] > PPM_P90))
	{
		chBlad = UzbrojSilniki(psDaneCM4);
		if (chBlad == BLAD_OK)
			psDaneCM4->chWymowSampla = PGA_UZBROJONY;
	}

    //sprawdź warunek rozbrojenia silników, czyli gaz na mininum i kierunek w lewo
	if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[WYSO]] < PPM_M90) && (psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] < PPM_M90))
	{
		RozbrojSilniki(psDaneCM4);
		psDaneCM4->chWymowSampla = PGA_ROZBROJONY;
	}

    //sprawdź warunek ..., czyli gaz na maksimum i kierunek w lewo
	if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[WYSO]] > PPM_P90) && (psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] < PPM_M90) && ((psDaneCM4->chTrybLotu & BTR_UZBROJONY) != BTR_UZBROJONY))
    {
        //obsługa ...
    }

    //sprawdź warunek zerowania zużycia energii, czyli gaz na maksimum i kierunek w prawo
    if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[WYSO]] > PPM_P90) && (psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] > PPM_P90) && ((psDaneCM4->chTrybLotu & BTR_UZBROJONY) != BTR_UZBROJONY))
    {
        //obsługa resetownia licznika energii
    }
    return chBlad;
}




