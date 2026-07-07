//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł ubsługi wejść odbiorników RC i wyjść serw/ESC
//
// (c) PitLab 2025-26
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <DShot.h>
#include <Fram.h>
#include <KanalyPID.h>
#include <SBus.h>
#include <WS281x.h>
#include "WeWyRC.h"
#include "KonfigFram.h"
#include "Czas.h"
#include "GNSS.h"
#include "KontrolerLotu.h"
#include "SampleAudio.h"
#include "RegulatorPID.h"
#include "Uarty.h"

//Wartości z odbiorników są normalizowane do zakresu 0-1000-2000 (WE_RC_MIN - WE_RC_NEUTR - WE_RC_MAX). Tak są liczone dane w mikserze.
//S-Bus oraz DShot mają rozdzielczość 11-bitów, wiec przyjmuję taką rozdzielczość sterowania definiując stałą PPM11BIT
//Surowy sygnał SBus przed normalizacją ma zakres 192-992-1792  patrz: https://github.com/uzh-rpg/rpg_quadrotor_control/wiki/SBUS-Protocol
//Surowy sygnał Crossfire przed normalizacją ma zakres ?-992-? patrz: https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#0x320x0a-general

//Definicje parametrów wyjściowych
//Sygnał PWM dla serw i regulatorów w tradycyjnym zakresie 1000-1500-2000us jest sterowany z rozdzielczoscią 0,5us, więc finalne wartości wynoszą 2000-3000-4000
//Sygnał DShot jest z zakresu 48-1048-2048, więc jest przesunięty o wartość DS_OFFSET_DSHOT względem standardu



//stRC2_t stRC;	//struktura danych odbiorników RC	//stara
stRC_t stRC1, stRC2;	//struktura danych odbiorników RC1 i RC2
extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch4;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern DMA_HandleTypeDef hdma_tim8_ch3;
extern uint32_t nKolorWS281x[LICZBA_LED_WS281X];
extern uint8_t cWskaznikLed;
extern uint16_t sWysterowanieMin;		//wartość wysterowania regulatorów dla uzyskania obrotów minimalnych w trakcie lotu
extern uint16_t sWysterowanieMax;		//wartość wysterowania regulatorów dla uzyskania obrotów maksymalnych. Dalsze zwiększanie wysterowania nic nie daje, więc w ten sposób wykluczamy go z zakresu regulacji
extern stKonfPID_t stKonfigPID[LICZBA_PID];
extern stStrojPID_t stStrojPID[LICZBA_KAN_RC_DO_STROJENIA_PID];

uint8_t chKonfigWeRC[LICZBA_WEJSC_RC];	//określa jakiego typu sygnał wchodzi z odbiornika
uint8_t chKonfigWyRC[LICZBA_WYJSC_RC];
extern uint32_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM1"))) nBuforTimDMA[KANALY_MIKSERA][DS_BITOW_DANYCH + DS_BITOW_PRZERWY];
uint8_t chKanalDrazkaRC[LICZBA_DRAZKOW];	//przypisanie kanałów odbiornika RC do funkcji drążków aparatury
uint8_t chFunkcjaKanaluRC[KANALY_FUNKCYJNE];		//funkcje przypisane do rozszerzonych kanałów wejściowych odbiornika RC

uint8_t chFunkcjaWyjscRC[KANALY_WYJSC_RC];		//funkcje przypisane do kanałów wyjściowych
uint8_t chFunkcjaSilnika[KANALY_MIKSERA];		//funkcje przypisane do silników: normalna praca lub analiza FFT rezonansu drgań ramy
uint8_t chRozmiarSekwencjiDMA[KANALY_MIKSERA+1];	//rozmiar paczki danych przesyłanych do DMA w zależności od częstotliwości odświezania. Dla 400Hz paczka ma 1 ważną daną, dla 200Hz jedną ważną i jedną nieważną, dla 50Hz jest 1 ważna i 7 pustych
uint8_t chBityKonfiguracji = 0;
//volatile uint16_t sFlagiNapelnieniaBuforow;		//flagi inforujące pętlę główną o potrzebie napełnienia podwójnego bufora DMA: DShot lub programowalnych LEDów
uint16_t sPoprzedniStanKanaluRozszerzonego[KANALY_FUNKCYJNE];	//poprzedni stan do detekcji uruchomiania funkcji wywoływanych kanałami wejsciowymi RC
uint8_t cDzielnikAktualizacjiLED;
uint16_t sWysterowanieIdentSiln;	//wysterowanie silników podczas procesu identfikacji


////////////////////////////////////////////////////////////////////////////////
// Funkcja wczytuje z FRAM konfigurację odbiorników RC
// Parametry Sbus 100kBps, 8E2
// Parametry:
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujWejsciaRC(void)
{
	uint8_t cBłąd = BLAD_OK;
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
		cBłąd |= HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);
	}
	else
	if (chKonfigWeRC[KANAL_RC1] == ODB_RC_SBUS)
	{
		cBłąd |= InicjujUart4RxJakoSbus(&GPIO_InitStruct);
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
		cBłąd |= HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
	}
	else
	if (chKonfigWeRC[KANAL_RC2] == ODB_RC_SBUS)
	{
		cBłąd |= InicjujUart2RxJakoSbus(&GPIO_InitStruct);
	}

	//odczytaj z FRAM minima i maksima kanałów RC aby móc je normalizować
    for (uint16_t n=0; n<KANALY_ODB_RC; n++)
    {
    	stRC1.sKanMin[n] = CzytajFramU16(FAU_WE_RC1_MIN + n*2);	//16*2U minimalna wartość sygnału RC dla każdego kanału
    	stRC1.sKanMax[n] = CzytajFramU16(FAU_WE_RC1_MAX + n*2);	//16*2U maksymalna wartość sygnału RC dla każdego kanału
    	stRC2.sKanMin[n] = CzytajFramU16(FAU_WE_RC2_MIN + n*2);	//16*2U minimalna wartość sygnału RC dla każdego kanału
    	stRC2.sKanMax[n] = CzytajFramU16(FAU_WE_RC2_MAX + n*2);	//16*2U maksymalna wartość sygnału RC dla każdego kanału
    }

    //odczytaj z FRAM numery kanałów dla 4 drążków RC
    for (uint16_t n=0; n<LICZBA_DRAZKOW; n++)
    {
    	cBłąd |= CzytajFramU8zWalidacja(FAU_KAN_DRAZKA_RC + n, &chKanalDrazkaRC[n],  0,  LICZBA_DRAZKOW,  0);	//4*1U Numer kanału przypisany do funkcji drążka aparatury: przechylenia, pochylenia, odchylenia i wysokości
    }

    //odczytaj z FRAM numery funkcji dla wyższych niż 4 kanałów RC
    for (uint16_t n=0; n<KANALY_FUNKCYJNE; n++)
    {
    	//cBłąd |= CzytajFramU8zWalidacja(FAU_FUNKCJA_MIN_KAN_RC + n, &chFunkcjaMinKanaluRC[n],  0,  LICZBA_FUNKCJI_RC,  0);	//12*1U Numer funkcji przypisanej do kanału RC (5..16) przełączonego na minimum
    	//cBłąd |= CzytajFramU8zWalidacja(FAU_FUNKCJA_MAX_KAN_RC + n, &chFunkcjaMaxKanaluRC[n],  0,  LICZBA_FUNKCJI_RC,  0);	//12*1U Numer funkcji przypisanej do kanału RC (5..16) przełączonego na maksimum
    	cBłąd |= CzytajFramU8zWalidacja(FAU_FUNKCJA_KAN_RC + n, &chFunkcjaKanaluRC[n],  0,  LICZBA_FUNKCJI_RC,  0);	//12*1U Numer funkcji przypisanej do kanału RC (5..16)
    }

    //domyślnie silniki pełnią funkcję napedu
    for (uint16_t n=0; n<KANALY_MIKSERA; n++)
    	chFunkcjaSilnika[n] = FSIL_NAPED;

    //ustaw kolor LEDów
    cBłąd |= InicjujKoloryWS281x();
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wczytuje z FRAM konfigurację kanałów serw i konfiguruje je jako PWM, DShot, S-Bus, IO, lub wejście analogowe
// Parametry:
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujWyjsciaRC(void)
{
	uint8_t cBłąd = BLAD_OK;
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
	CzytajBuforFRAM(FAU_FUNKCJA_WY_RC, chFunkcjaWyjscRC, KANALY_WYJSC_RC);
	AktualizujWyjsciaRC(&uDaneCM4.dane);


	//**** Wyjście 1 - konfiguracja portu PB9 TIM4_CH4 **********************************************************
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	if (chKonfigWyRC[KANAL_RC1] == SERWO_WS281X)
		UstawTrybWS281x(KANAL_RC1);
	else
	if ((chKonfigWyRC[KANAL_RC1] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		chBityKonfiguracji |= 0x01;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;		//wyjście przechodzi przez inwerter, więc wymaga dodatkowego odwrócenia sygnału aby finalnie było niezmienione
		cBłąd |= HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
		htim4.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim4.Init.Period = OKRES_PWM;
		htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		cBłąd |= HAL_TIM_PWM_Init(&htim4);
		cBłąd |= HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_4, &nBuforTimDMA[KANAL_RC1][0], chRozmiarSekwencjiDMA[KANAL_RC1]);
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
		cBłąd |= BLAD_BRAK_KONFIG;

	//**** Wyjście 2 - konfiguracja portu PB10 - TIM2_CH3, USART3_TX, MOD_QSPI_CS **********************************************************
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	if (chKonfigWyRC[KANAL_RC2] == SERWO_WS281X)
		UstawTrybWS281x(KANAL_RC2);
	else
	if ((chKonfigWyRC[KANAL_RC2] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		__HAL_RCC_TIM2_CLK_ENABLE();
		__HAL_RCC_DMA2_CLK_ENABLE();
		chBityKonfiguracji |= 0x02;
		htim2.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = OKRES_PWM;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		cBłąd |= HAL_TIM_PWM_Init(&htim2);

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.Pulse = OKRES_PWM / 2;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;		//wyjście przechodzi przez inwerter, więc wymaga dodatkowego odwrócenia sygnału aby finalnie było niezmienione
		cBłąd |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

		hdma_tim2_ch3.Instance = DMA2_Stream7;
		hdma_tim2_ch3.Init.Request = DMA_REQUEST_TIM2_CH3;
		hdma_tim2_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tim2_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tim2_ch3.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tim2_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim2_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_tim2_ch3.Init.Mode = DMA_CIRCULAR;
		hdma_tim2_ch3.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_tim2_ch3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_tim2_ch3.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_tim2_ch3.Init.PeriphBurst = DMA_PBURST_SINGLE;
		HAL_DMA_Init(&hdma_tim2_ch3);

		__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC3);
		__HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC3], hdma_tim2_ch3);
		cBłąd |= HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, &nBuforTimDMA[KANAL_RC2][0], chRozmiarSekwencjiDMA[KANAL_RC2]);
	}
	else
	if (chKonfigWyRC[KANAL_RC2] == SERWO_DSHOT150)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC2);
	else
	if (chKonfigWyRC[KANAL_RC2] == SERWO_DSHOT300)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC2);
	else
	if (chKonfigWyRC[KANAL_RC2] == SERWO_DSHOT600)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC2);
	else
	if (chKonfigWyRC[KANAL_RC2] == SERWO_DSHOT1200)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC2);
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
		cBłąd |= HAL_UART_Init(&huart3);
		cBłąd |= HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_2);
		cBłąd |= HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_2);
		//cBłąd |= HAL_UARTEx_EnableFifoMode(&huart3);
		cBłąd |= HAL_UARTEx_DisableFifoMode(&huart3);

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
		cBłąd |= BLAD_BRAK_KONFIG;


	//**** Wyjście RC 3 - konfiguracja portu PA15, TIM2_CH1 **********************************************************W
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	if ((chKonfigWyRC[KANAL_RC3] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		chBityKonfiguracji |= 0x04;
		__HAL_RCC_TIM2_CLK_ENABLE();
		__HAL_RCC_DMA2_CLK_ENABLE();

		htim2.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = OKRES_PWM;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		cBłąd |= HAL_TIM_PWM_Init(&htim2);

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.Pulse = OKRES_PWM / 2;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		cBłąd |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

		hdma_tim2_ch1.Instance = DMA2_Stream6;
		hdma_tim2_ch1.Init.Request = DMA_REQUEST_TIM2_CH1;
		hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;
		hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_tim2_ch1);

		__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
		__HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
		cBłąd |= HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &nBuforTimDMA[KANAL_RC3][0], chRozmiarSekwencjiDMA[KANAL_RC3]);
	}
	else
	if (chKonfigWyRC[KANAL_RC3] == SERWO_DSHOT150)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC3);
	else
	if (chKonfigWyRC[KANAL_RC3] == SERWO_DSHOT300)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC3);
	else
	if (chKonfigWyRC[KANAL_RC3] == SERWO_DSHOT600)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC3);
	else
	if (chKonfigWyRC[KANAL_RC3] == SERWO_DSHOT1200)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC3);
	else
	if (chKonfigWyRC[KANAL_RC3] == SERWO_IO)	//wyjście IO do debugowania
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else
		cBłąd |= BLAD_BRAK_KONFIG;


	//**** Wyjście 4 - konfiguracja portu PB0, TIM3_CH3 **********************************************************
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	TIM3->CCR3 = 1000;

	if (chKonfigWyRC[KANAL_RC4] == SERWO_WS281X)
		UstawTrybWS281x(KANAL_RC4);
	else
	if ((chKonfigWyRC[KANAL_RC4] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		chBityKonfiguracji |= 0x08;
		__HAL_RCC_TIM3_CLK_ENABLE();
		__HAL_RCC_DMA2_CLK_ENABLE();

		htim3.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = OKRES_PWM;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		cBłąd |= HAL_TIM_PWM_Init(&htim3);

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.Pulse = OKRES_PWM / 2;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		cBłąd |= HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

		hdma_tim3_ch3.Instance = DMA2_Stream4;
		hdma_tim3_ch3.Init.Request = DMA_REQUEST_TIM3_CH3;
		hdma_tim3_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tim3_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tim3_ch3.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tim3_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim3_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_tim3_ch3.Init.Mode = DMA_CIRCULAR;
		hdma_tim3_ch3.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_tim3_ch3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_tim3_ch3);

		//__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC3);
		//__HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC3], hdma_tim3_ch3);
		cBłąd |= HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, &nBuforTimDMA[KANAL_RC4][0], chRozmiarSekwencjiDMA[KANAL_RC4]);
	}
	else
	if (chKonfigWyRC[KANAL_RC4] == SERWO_DSHOT150)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC4);
	else
	if (chKonfigWyRC[KANAL_RC4] == SERWO_DSHOT300)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC4);
	else
	if (chKonfigWyRC[KANAL_RC4] == SERWO_DSHOT600)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC4);
	else
	if (chKonfigWyRC[KANAL_RC4] == SERWO_DSHOT1200)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC4);
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
		cBłąd |= BLAD_BRAK_KONFIG;


	//**** Wyjście 5 - konfiguracja portu PB1 TIM3_CH4 **********************************************************
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	if (chKonfigWyRC[KANAL_RC5] == SERWO_WS281X)
		UstawTrybWS281x(KANAL_RC5);
	else
	if ((chKonfigWyRC[KANAL_RC5] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		__HAL_RCC_TIM3_CLK_ENABLE();
		__HAL_RCC_DMA2_CLK_ENABLE();

		htim3.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = OKRES_PWM;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		cBłąd |= HAL_TIM_PWM_Init(&htim3);

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.Pulse = OKRES_PWM / 2;;
		cBłąd |= HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

		hdma_tim3_ch4.Instance = DMA2_Stream5;
		hdma_tim3_ch4.Init.Request = DMA_REQUEST_TIM3_CH4;
		hdma_tim3_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tim3_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tim3_ch4.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tim3_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim3_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_tim3_ch4.Init.Mode = DMA_CIRCULAR;
		hdma_tim3_ch4.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_tim3_ch4.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_tim3_ch4);

		__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC4);
		__HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC4], hdma_tim3_ch4);
		cBłąd |= HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, &nBuforTimDMA[KANAL_RC5][0], chRozmiarSekwencjiDMA[KANAL_RC5]);
	}
	else
	if (chKonfigWyRC[KANAL_RC5] == SERWO_DSHOT150)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC5);
	else
	if (chKonfigWyRC[KANAL_RC5] == SERWO_DSHOT300)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC5);
	else
	if (chKonfigWyRC[KANAL_RC5] == SERWO_DSHOT600)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC5);
	else
	if (chKonfigWyRC[KANAL_RC5] == SERWO_DSHOT1200)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC5);
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
		cBłąd |= BLAD_BRAK_KONFIG;


	//**** Wyjście 6 - konfiguracja portu PI5, TIM8_CH1 **********************************************************
	__HAL_RCC_GPIOI_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	if (chKonfigWyRC[KANAL_RC6] == SERWO_WS281X)
		UstawTrybWS281x(KANAL_RC6);
	else
	if ((chKonfigWyRC[KANAL_RC6] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		__HAL_RCC_TIM8_CLK_ENABLE();
		__HAL_RCC_DMA2_CLK_ENABLE();

		htim8.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim8.Init.Period = OKRES_PWM;
		htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		cBłąd |= HAL_TIM_PWM_Init(&htim8);

		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		cBłąd |= HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);

		hdma_tim8_ch1.Instance = DMA2_Stream3;
		hdma_tim8_ch1.Init.Request = DMA_REQUEST_TIM8_CH1;
		hdma_tim8_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tim8_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tim8_ch1.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tim8_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim8_ch1.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim8_ch1.Init.Mode = DMA_CIRCULAR;
		hdma_tim8_ch1.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_tim8_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		cBłąd |= HAL_DMA_Init(&hdma_tim8_ch1);
		cBłąd |= HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, &nBuforTimDMA[KANAL_RC6][0], chRozmiarSekwencjiDMA[KANAL_RC6]);
	}
	else
	if (chKonfigWyRC[KANAL_RC6] == SERWO_DSHOT150)
	{
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC6);
	}
	else
	if (chKonfigWyRC[KANAL_RC6] == SERWO_DSHOT300)
	{
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC6);
	}
	else
	if (chKonfigWyRC[KANAL_RC6] == SERWO_DSHOT600)
	{
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC6);
	}
	else
	if (chKonfigWyRC[KANAL_RC6] == SERWO_DSHOT1200)
	{
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC6);
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
		cBłąd |= BLAD_BRAK_KONFIG;


	//**** Wyjście 7 - konfiguracja portu PI10  - brak timera na porcie **********************************************************
	__HAL_RCC_GPIOI_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_10;

	if (chKonfigWyRC[KANAL_RC7] == SERWO_IO)	//wyjście IO do debugowania
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	}
	else
		cBłąd |= BLAD_BRAK_KONFIG;


	//**** Wyjście 8 - konfiguracja portu PH15 TIM8_CH3N **********************************************************
	__HAL_RCC_GPIOH_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);		//domyslnie ma być timer. w przypadku IO lub ADC, konfiguracja będzie nadpisana

	if (chKonfigWyRC[KANAL_RC8] == SERWO_WS281X)
	{
		UstawTrybWS281x(KANAL_RC8);
	}
	else
	if ((chKonfigWyRC[KANAL_RC8] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		__HAL_RCC_TIM8_CLK_ENABLE();
		__HAL_RCC_DMA2_CLK_ENABLE();

		htim8.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim8.Init.Period = OKRES_PWM;
		htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		cBłąd |= HAL_TIM_PWM_Init(&htim8);

		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		cBłąd |= HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);

		hdma_tim8_ch3.Instance = DMA2_Stream2;
		hdma_tim8_ch3.Init.Request = DMA_REQUEST_TIM8_CH1;
		hdma_tim8_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tim8_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tim8_ch3.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tim8_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim8_ch3.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim8_ch3.Init.Mode = DMA_CIRCULAR;
		hdma_tim8_ch3.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_tim8_ch3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		cBłąd |= HAL_DMA_Init(&hdma_tim8_ch3);
		cBłąd |= HAL_TIMEx_PWMN_Start_DMA(&htim8, TIM_CHANNEL_3, &nBuforTimDMA[KANAL_RC8][0], chRozmiarSekwencjiDMA[KANAL_RC8]);	//specjalna funkcja dla kanału komplementarnego N
	}
	else
	if (chKonfigWyRC[KANAL_RC8] == SERWO_DSHOT150)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT150, KANAL_RC8);
	else
	if (chKonfigWyRC[KANAL_RC8] == SERWO_DSHOT300)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT300, KANAL_RC8);
	else
	if (chKonfigWyRC[KANAL_RC8] == SERWO_DSHOT600)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT600, KANAL_RC8);
	else
	if (chKonfigWyRC[KANAL_RC8] == SERWO_DSHOT1200)
		cBłąd |= UstawTrybDShot(PROTOKOL_DSHOT1200, KANAL_RC8);
	else
	if (chKonfigWyRC[KANAL_RC8] == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	}
	else
		cBłąd |= BLAD_BRAK_KONFIG;


	//**** Wyjście 9-16 - konfiguracja portu PA8 TIM1_CH1 **********************************************************
	//pracuje jako PWM bez DMA, ponieważ w przerwaniu musi zmieniać stan dekodera a swoją drogą nie ma już wolnych kanałów DMA
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	if ((chKonfigWyRC[KANAL_RC916] & SERWO_PWMXXX) == SERWO_PWMXXX)	//dotyczy całej rodziny prędkości PWM
	{
		__HAL_RCC_TIM1_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);		//na tym kanale generowanie PWM wymaga przerwań aby przełączyć dekoden kanałów. DMA nie jest używane

		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		cBłąd |= HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

		htim1.Init.Prescaler = (nHCLK / ZEGAR_PWM) - 1;	//finalnie trzeba uzyskać zegar 2 MHz aby PWM miał taką samą rozdzielczość 2000 kroków co DShot
		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim1.Init.Period = OKRES_PWM;
		htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		cBłąd |= HAL_TIM_PWM_Init(&htim1);
		cBłąd |= HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	}
	else
	if (chKonfigWyRC[KANAL_RC916] == SERWO_IO)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else
		cBłąd |= BLAD_BRAK_KONFIG;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Callback opróżnienia połowy bufora timerów służy do ustawienie informacji
// o potrzebie przeładowania pierwszej połowy bufora kołowego. Używane do sterowania LED-ami
// Parametry: *hdma - wskaźnik na DMA zgłaszające koniec transmisji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
    	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) && (chKonfigWyRC[KANAL_RC2] == SERWO_WS281X))	//kanał 2 serw
    	{
    		if (AktualizujWS281xDMA(NAPELNIJ_BUF1_CH2, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
    			HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
    		//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);			//kanał serw 7 skonfigurowany jako IO
    	}

    	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) && (chKonfigWyRC[KANAL_RC3] == SERWO_WS281X))	//kanał 3 serw
    	{
    	    if (AktualizujWS281xDMA(NAPELNIJ_BUF1_CH3, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
    	    	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
    	}
    }

    if(htim->Instance == TIM3)
    {
    	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) && (chKonfigWyRC[KANAL_RC4] == SERWO_WS281X))	//kanał 4 serw
    	{
    	    if (AktualizujWS281xDMA(NAPELNIJ_BUF1_CH4, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
    	    	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
    	}

    	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) && (chKonfigWyRC[KANAL_RC5] == SERWO_WS281X))	//kanał 5 serw
    	{
    	    if (AktualizujWS281xDMA(NAPELNIJ_BUF1_CH5, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
    	    	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
    	}
    }

	if(htim->Instance == TIM8)
	{
		if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) && (chKonfigWyRC[KANAL_RC6] == SERWO_WS281X))	//kanał 6 serw
		{
			if (AktualizujWS281xDMA(NAPELNIJ_BUF1_CH6, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
		        HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_1);
		    //HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);			//kanał serw 7 skonfigurowany jako IO
		}

		if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) && (chKonfigWyRC[KANAL_RC8] == SERWO_WS281X))	//kanał 8 serw
		{
			if (AktualizujWS281xDMA(NAPELNIJ_BUF1_CH8, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
				HAL_TIMEx_PWMN_Stop_DMA(&htim8, TIM_CHANNEL_3);		//specjalna funkcja dla kanału zanegownego
			//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);	//serwo kanał 1
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Callback opróżnienia bufora timerów służy do ustawienie informacji
// o potrzebie przeładowania drugiej połowy bufora
// Parametry: *hdma - wskaźnik na DMA zgłaszające koniec transmisji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
    	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) && (chKonfigWyRC[KANAL_RC2] == SERWO_WS281X))
    	{
    		if (AktualizujWS281xDMA(NAPELNIJ_BUF2_CH2, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
    		    HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
    		//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);			//kanał serw 7 skonfigurowany jako IO
    	}

    	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) && (chKonfigWyRC[KANAL_RC3] == SERWO_WS281X))
    	{
    	    if (AktualizujWS281xDMA(NAPELNIJ_BUF2_CH3, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
    	       	   HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
    	}
    }

    if(htim->Instance == TIM3)
    {
    	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) && (chKonfigWyRC[KANAL_RC4] == SERWO_WS281X))	//kanał 4 serw
    	{
    	    if (AktualizujWS281xDMA(NAPELNIJ_BUF2_CH4, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
    	    	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
    	}

    	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) && (chKonfigWyRC[KANAL_RC5] == SERWO_WS281X))	//kanał 5 serw
    	{
    	    if (AktualizujWS281xDMA(NAPELNIJ_BUF2_CH5, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
    	    	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
    	}
    }

	if(htim->Instance == TIM8)
    {
    	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) && (chKonfigWyRC[KANAL_RC6] == SERWO_WS281X))
    	{
    		if (AktualizujWS281xDMA(NAPELNIJ_BUF2_CH6, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
    			HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_1);
    		//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);			//kanał serw 7 skonfigurowany jako IO
    	}

    	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) && (chKonfigWyRC[KANAL_RC8] == SERWO_WS281X))
    	{
    		if (AktualizujWS281xDMA(NAPELNIJ_BUF2_CH8, nKolorWS281x, LICZBA_LED_WS281X, &cWskaznikLed) == BLAD_NIC_DO_ROBOTY)
    			HAL_TIMEx_PWMN_Stop_DMA(&htim8, TIM_CHANNEL_3);		//specjalna funkcja dla kanału zanegownego
    		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);	//kanał serw 1 skonfigurowany jako IO
    	}
    }
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje dane wysyłane na wyjścia RC 1..8 w zależności od przypisanej funkcji
// Parametry: *dane - wskaźnik na strukturę zawierajacą wartości wyjścia regulatorów PID
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktualizujWyjsciaRC(stWymianyCM4_t *daneCM4)
{
	uint8_t cBłąd = BLAD_OK;
	//uint32_t nWyjście;
	uint16_t sWyjście;

	//ponieważ odświeżanie LED-ów trwa długo, więc rób je co DZIELNIK_AKTUALIZACJI_LED okresów
	if (cDzielnikAktualizacjiLED)
		cDzielnikAktualizacjiLED--;

	//aktualizuj pierwsze 8 kanałów wyjściowych RC sterownych swobodnymi kanałami timera
	for (uint8_t n=0; n<KANALY_MIKSERA; n++)
	{
		//nWyjście = PobierzWartoscWyjsciaRC(chFunkcjaWyjscRC[n], daneCM4);
		//daneCM4->sWyjscieRC[n] = (uint16_t)nWyjście;
		sWyjście = PobierzWartoscWyjsciaRC(chFunkcjaWyjscRC[n], daneCM4);
		daneCM4->sWyjscieRC[n] = sWyjście;

		//sprawdź rodzaj ustawionego protokołu wyjscia
		switch (chKonfigWyRC[n])
		{
		case SERWO_IO:		break;	//nie rób nic - kanał jest portem IO sterowanym niezależnie z poziomu kodu, zwykle jako debug
		case SERWO_PWM400:
			chRozmiarSekwencjiDMA[n] = 1;
			//nBuforTimDMA[n][0] = nWyjście;
			nBuforTimDMA[n][0] = (uint32_t)sWyjście;
			break;

		case SERWO_PWM200:	//co drugi w buforze jest kanał, pozostałe są zerami
			chRozmiarSekwencjiDMA[n] = 2;
			for (uint8_t m=0; m<2; m++)
			{
				//nBuforTimDMA[n][2*m+0] = nWyjście;
				nBuforTimDMA[n][2*m+0] = (uint32_t)sWyjście;
				nBuforTimDMA[n][2*m+1] = 0;
			}
			break;

		case SERWO_PWM100:	//co czwarty w buforze jest kanał, pozostałe są zerami
			chRozmiarSekwencjiDMA[n] = 4;
			for (uint8_t m=0; m<4; m++)
			{
				//nBuforTimDMA[n][4*m+0] = nWyjście;
				nBuforTimDMA[n][4*m+0] = (uint32_t)sWyjście;
				nBuforTimDMA[n][4*m+1] = 0;
				nBuforTimDMA[n][4*m+2] = 0;
				nBuforTimDMA[n][4*m+3] = 0;
			}
			break;

		case SERWO_PWM50:	//pierwszy w buforze jest kanał, pozostałe są zerami
			chRozmiarSekwencjiDMA[n] = 8;
			//nBuforTimDMA[n][0] = nWyjście;
			nBuforTimDMA[n][0] = (uint32_t)sWyjście;
			for (uint8_t m=1; m<KANALY_MIKSERA; m++)
				nBuforTimDMA[n][m] = 0;
			break;

		case SERWO_DSHOT150:	//wszystkie typy DShot mają wspólną obsługę
		case SERWO_DSHOT300:
		case SERWO_DSHOT600:
		//case SERWO_DSHOT1200:	cBłąd |= AktualizujDShotDMA(nWyjście, n);	break;
		case SERWO_DSHOT1200:	cBłąd |= AktualizujDShotDMA(sWyjście, daneCM4->cPolecenieDShot, n);	break;

		case SERWO_WS281X:
			if (cDzielnikAktualizacjiLED == 0)
			{
				cDzielnikAktualizacjiLED = DZIELNIK_AKTUALIZACJI_LED;
				AktualizujKolorLedWs821x();
			}
			break;

		default: cBłąd = BLAD_BRAK_KONFIG;
		}	//switch
	}	//for

	//starsze 8 wyjść jest aktualizowanych w obsłudze przerwania TIM1_CC_IRQHandler() w pliku stm32h7xx_it.c
	//więc tylko ustaw wyjścia na podstawie konfiguracji
	for (uint8_t n=KANALY_MIKSERA; n<KANALY_WYJSC_RC; n++)
		daneCM4->sWyjscieRC[n] = PobierzWartoscWyjsciaRC(chFunkcjaWyjscRC[n], daneCM4);

	//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);	//serwo kanał 7
	//HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_15);	//serwo kanał 8
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera wartość do wysterowania kanału wyjściowego zdefiniowaną funkcjonalnością
// Parametry:
//  chIndeksFunkcji - indeks funkcji dla bieżącego wyjścia
//  *chFunkcja - wskaźnik na tablicę zawierajacą
//  *daneCM4 - wskaźnik na strukturę danych rdzenia CM4
// Zwraca: wysterowanie kanału
////////////////////////////////////////////////////////////////////////////////
uint16_t PobierzWartoscWyjsciaRC(uint8_t chIndeksFunkcji, stWymianyCM4_t *daneCM4)
{
	uint16_t sWyjście;

	switch(chIndeksFunkcji)
	{
	case FWYRC_SILNIK1:
	case FWYRC_SILNIK2:
	case FWYRC_SILNIK3:
	case FWYRC_SILNIK4:
	case FWYRC_SILNIK5:
	case FWYRC_SILNIK6:
	case FWYRC_SILNIK7:
	case FWYRC_SILNIK8:
		switch (chFunkcjaSilnika[chIndeksFunkcji - FWYRC_SILNIK1])
		{
		case FSIL_NAPED:	sWyjście = daneCM4->sSilnik[chIndeksFunkcji - FWYRC_SILNIK1];	break;				//normalna praca silnika jako napęd
		case FSIL_AN_DRGAN:		//dane do silników pochodzą z analizatora drgań w rdzeniu CM7. Wytyczne do ich obliczenia są przekazywane przez strukturę unię uRozne
			 					//uDaneCM7.dane.sAdres - indeks etapu badania: 0..LICZBA_TESTOW_FFT
			 					//uDaneCM7.dane.uRozne.U8[1] - stKonfigFFT->chAktywnSilniki;
								//uDaneCM7.dane.uRozne.U8[2] - stKonfigFFT->chMaxWysterowanie; - procentowa wartość wysterowania względem maksimum
								//uDaneCM7.dane.uRozne.U8[3] - licznik opóźnienia przy wyhamowaniu
								//uDaneCM7.dane.uRozne.U16[4] - wartość wysterowania zmniejszająca się do zera
			if (uDaneCM7.dane.sAdres < LICZBA_TESTOW_FFT - 1)
			{
				uint16_t sWysterowanieMaxAD = (sWysterowanieMax - sWysterowanieMin) * uDaneCM7.dane.uRozne.U8[2] / 100;	//pula ograniczonego zakresu sterowania
				sWyjście = sWysterowanieMin + sWysterowanieMaxAD * uDaneCM7.dane.sAdres / LICZBA_TESTOW_FFT;		//bieżące wysterowanie
				uDaneCM7.dane.uRozne.U16[4] = sWyjście;
				uDaneCM7.dane.uRozne.U8[3] = LICZNIK_PREDKOSCI_HAMOWANIA;	//co tyle obiegów pętli glłównej zmniejsz wartość wysterowania
			}
			else	//po dojściu do maksymalnej wartości wyhamuj płynnie do zera
			{
				if (uDaneCM7.dane.uRozne.U8[3])
					uDaneCM7.dane.uRozne.U8[3]--;
				else
				{
					if (uDaneCM7.dane.uRozne.U16[4])
						uDaneCM7.dane.uRozne.U16[4]--;
					uDaneCM7.dane.uRozne.U8[3] = LICZNIK_PREDKOSCI_HAMOWANIA;
				}
				sWyjście = uDaneCM7.dane.uRozne.U16[4];
			}
			break;

		case FSIL_IDENTYFIKACJA:	sWyjście = sWysterowanieIdentSiln;	break;

		case FSIL_ZATRZYMANY:	//silnik jest zatrzymany, bo nie bierze udziału w analizie drgań
		default:	sWyjście = 0;	break;
		}
		break;

	case FWYRC_WE_RC1:
	case FWYRC_WE_RC2:
	case FWYRC_WE_RC3:
	case FWYRC_WE_RC4:
	case FWYRC_WE_RC5:
	case FWYRC_WE_RC6:
	case FWYRC_WE_RC7:
	case FWYRC_WE_RC8:
	case FWYRC_WE_RC9:
	case FWYRC_WE_RC10:
	case FWYRC_WE_RC11:
	case FWYRC_WE_RC12:
	case FWYRC_WE_RC13:
	case FWYRC_WE_RC14:
	case FWYRC_WE_RC15:
	case FWYRC_WE_RC16:	sWyjście = daneCM4->sKanalRC[chIndeksFunkcji - FWYRC_WE_RC1];	break;	//przepisanie wejścia na wyjście
	default:	sWyjście = 0;	break;
	}
	return sWyjście;
}



////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna zbiór nowych ekstremalnych wartości wszystkich kanalów z obu odbiorników RC
// Parametry: brak - funkcja do wywyołania z zewnątrz
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RozpocznijZbieranieEkstremowWejscRC(void)
{
	//inicjuj wartości ekstremalne skrajnymi
	for (uint16_t n=0; n<KANALY_ODB_RC; n++)
	{
		stRC1.sKanMin[n] = WE_RC_MAX;
		stRC1.sKanMax[n] = WE_RC_MIN;
		stRC2.sKanMin[n] = WE_RC_MAX;
		stRC2.sKanMax[n] = WE_RC_MIN;
	}

	//rozpocznij zbieranie gdy jeszcze nie jest rozpoczęte
	if ((stRC1.cFlagi & FRC_ZBIERAJ_EKSTR) != FRC_ZBIERAJ_EKSTR)
		stRC1.cFlagi |= FRC_ZBIERAJ_EKSTR;

	if ((stRC2.cFlagi & FRC_ZBIERAJ_EKSTR) != FRC_ZBIERAJ_EKSTR)
		stRC2.cFlagi |= FRC_ZBIERAJ_EKSTR;
}



////////////////////////////////////////////////////////////////////////////////
// Kończy zbiór ekstremalnych wartości wszystkich kanalów z obu odbiorników RC
// Wartości o ile zostały zebrane są zapisywane we FRAM a jezęli nie to przywracane są poprzednie wartości
// Parametry: brak - funkcja do wywołania z zewnątrz
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ZapiszEkstremaWejscRC(void)
{
	if (stRC1.cFlagi & FRC_ZEBRANO_EKSTR)
	{
		//odbiornik 1
		for (uint16_t n=0; n<KANALY_ODB_RC; n++)
		{
			if (stRC1.cFlagi & FRC_ZEBRANO_EKSTR)
			{
				ZapiszFramU16(FAU_WE_RC1_MIN + n*2, stRC1.sKanMin[n]);		//16*2U minimalna wartość sygnału RC dla każego kanału
				ZapiszFramU16(FAU_WE_RC1_MAX + n*2, stRC1.sKanMax[n]);		//16*2U maksymalna wartość sygnału RC dla każego kanału
			}
			else	//jeżeli nie zebrano danych, bo np. nie ma podłączonego odbiornika to przywróć ostatnio zapisane wartosci
			{
				stRC1.sKanMin[n] = CzytajFramU16(FAU_WE_RC1_MIN + n*2);
				stRC1.sKanMax[n] = CzytajFramU16(FAU_WE_RC1_MAX + n*2);
			}
		}
		stRC1.cFlagi &= ~(FRC_ZBIERAJ_EKSTR | FRC_ZEBRANO_EKSTR);
	}

	//odbiornik 2
	if (stRC2.cFlagi & FRC_ZEBRANO_EKSTR)
	{
		for (uint16_t n=0; n<KANALY_ODB_RC; n++)
		{
			if (stRC2.cFlagi & FRC_ZEBRANO_EKSTR)
			{
				ZapiszFramU16(FAU_WE_RC2_MIN + n*2, stRC2.sKanMin[n]);
				ZapiszFramU16(FAU_WE_RC2_MAX + n*2, stRC2.sKanMax[n]);
			}
			else
			{
				stRC2.sKanMin[n] = CzytajFramU16(FAU_WE_RC2_MIN + n*2);
				stRC2.sKanMax[n] = CzytajFramU16(FAU_WE_RC2_MAX + n*2);
			}
		}
		stRC2.cFlagi &= ~(FRC_ZBIERAJ_EKSTR | FRC_ZEBRANO_EKSTR);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera dane z 2 odbiorników i wybiera aktywny lub lepszy.
// Normalizuje dane, jeżeli właczone to zbiera ekstrema
// Finalne dane przepisuje do struktury danych odbiornika CM4
// Parametry:
// [we] *stRC1 - wskaźnik na strukturę danych odbiornika RC1
// [we] *stRC2 - wskaźnik na strukturę danych odbiornika RC2
// [we] cUzywajRC - zmienna mówiąca których odbiorników używać: ODB_RC1, ODB_RC2, ODB_OBA
// [wy] *psDaneCM4 - wskaźnik na strukturę danych CM4
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t DywersyfikacjaOdbiornikowRC(stRC_t *stRC1, stRC_t *stRC2, uint8_t cUzywajRC, stWymianyCM4_t *psDaneCM4)
{
	uint8_t cBłąd = BLAD_OK;
	uint32_t nCzasBiezacy = PobierzCzasT7();
	uint32_t nCzasRC1, nCzasRC2;
	uint16_t sRóżnicaMaxMin;

	//Sprawdź kiedy przyszły ostatnie dane RC
	nCzasRC1 = MinalCzas2T7(stRC1->nCzasOdOstatniejRamki, nCzasBiezacy);
	nCzasRC2 = MinalCzas2T7(stRC2->nCzasOdOstatniejRamki, nCzasBiezacy);

	if (nCzasRC1 < 2*OKRES_RAMKI_PPM_RC)	//działa odbiornik 1
	{
		for (uint16_t n=0; n<KANALY_ODB_RC; n++)
		{
			if (stRC1->sZdekodowaneKanaly & (1<<n))
			{
				//zbieranie ekstremów do obliczenia nowej normalizacji
				if (stRC1->cFlagi & FRC_ZBIERAJ_EKSTR)
				{
					if (stRC1->sKanaly[n] < stRC1->sKanMin[n])
						stRC1->sKanMin[n] = stRC1->sKanaly[n];
					if (stRC1->sKanaly[n] > stRC1->sKanMax[n])
						stRC1->sKanMax[n] = stRC1->sKanaly[n];
					if ((stRC1->sKanMin[n] < WE_RC_M90) && (stRC1->sKanMax[n] > WE_RC_P90))
						stRC1->cFlagi |= FRC_ZEBRANO_EKSTR;

					psDaneCM4->sKanalRC[n] = stRC1->sKanaly[n];	//podawaj surowe dane bez normalizacji
				}
				else
				//normalizacja danych
				if (cUzywajRC & ODB_RC1)
				{
					sRóżnicaMaxMin = stRC1->sKanMax[n] - stRC1->sKanMin[n];
					if (sRóżnicaMaxMin)
						psDaneCM4->sKanalRC[n] = (stRC1->sKanaly[n] - stRC1->sKanMin[n]) * (WE_RC_P100 - WE_RC_M100) / sRóżnicaMaxMin + WE_RC_M100; 	//przepisz znornalizowane kanały
					else
						psDaneCM4->sKanalRC[n] = stRC1->sKanaly[n];	//surowe dane bez normalizacji
				}
				stRC1->sZdekodowaneKanaly &= ~(1<<n);		//kasuj bit obrobionego kanału
			}
		}
		psDaneCM4->cJakoscDnLinkuRC = stRC2->cJakoscDnLinku;
		psDaneCM4->cJakoscUpLinkuRC1 = stRC2->cJakoscUpLinku;
	}
	else	//Odzyskiwanie synchronizacji: Jeżeli nie było nowych danych przez czas 2x trwania ramki to wymuś odbiór
	if (nCzasRC1 > 2*OKRES_RAMKI_PPM_RC)
	{
		WłączOdbiórUART4();	//ustawia odbiornik gotowy na przyjęcie danych
		psDaneCM4->cJakoscDnLinkuRC = 0;
		psDaneCM4->cJakoscUpLinkuRC1 = 0;
	}


	if (nCzasRC2 < 2*OKRES_RAMKI_PPM_RC)	//działa odbiornik 2
	{
		for (uint16_t n=0; n<KANALY_ODB_RC; n++)
		{
			if (stRC2->sZdekodowaneKanaly & (1<<n))
			{
				//zbieranie ekstremów do obliczenia nowej normalizacji
				if (stRC2->cFlagi & FRC_ZBIERAJ_EKSTR)
				{
					if (stRC2->sKanaly[n] < stRC2->sKanMin[n])
						stRC2->sKanMin[n] = stRC2->sKanaly[n];
					if (stRC2->sKanaly[n] > stRC2->sKanMax[n])
						stRC2->sKanMax[n] = stRC2->sKanaly[n];
					if ((stRC2->sKanMin[n] < WE_RC_M90) && (stRC2->sKanMax[n] > WE_RC_P90))
						stRC2->cFlagi |= FRC_ZEBRANO_EKSTR;
				}
				else
				//normalizacja danych
				if (cUzywajRC & ODB_RC2)
				{
					sRóżnicaMaxMin = stRC2->sKanMax[n] - stRC2->sKanMin[n];
					if (sRóżnicaMaxMin)
						psDaneCM4->sKanalRC[n] = (stRC2->sKanaly[n] - stRC2->sKanMin[n]) * (WE_RC_P100 - WE_RC_M100) / sRóżnicaMaxMin + WE_RC_M100; 	//przepisz znornalizowane kanały
					else
						psDaneCM4->sKanalRC[n] = stRC2->sKanaly[n];	//surowe dane bez normalizacji
				}
				stRC2->sZdekodowaneKanaly &= ~(1<<n);		//kasuj bit obrobionego kanału
			}
		}
		psDaneCM4->cJakoscDnLinkuRC = stRC2->cJakoscDnLinku;
		psDaneCM4->cJakoscUpLinkuRC2 = stRC2->cJakoscUpLinku;
	}
	else	//Odzyskiwanie synchronizacji: Jeżeli nie było nowych danych przez czas 2x trwania ramki to wymuś odbiór
	if (nCzasRC2 > 2*OKRES_RAMKI_PPM_RC)
	{
		WłączOdbiórUART2();	//ustawia odbiornik gotowy na przyjęcie danych
		psDaneCM4->cJakoscDnLinkuRC = 0;
		psDaneCM4->cJakoscUpLinkuRC2 = 0;
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Analizuje gotowy znormalizowany sygnał z odbiornika RC uruchamiając specjalne funkcje sterowane drążkami aparatury
// Parametry:
// [we] *psDaneCM4 - wskaźnik na strukturę danych CM4
// [wy] *psDaneCM7 - wskaźnik na strukturę danych CM7
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t AnalizujSygnalRC(stWymianyCM4_t *psDaneCM4, stWymianyCM7_t *psDaneCM7)
{
	uint8_t cBłąd = BLAD_OK;

	//Dodać  sprawdzenie czy przyszły nowe dane z odbiornika

	//aby nie sprawdzać wszystkich warunków za każdym przebiegiem, sprawdzaj kolejno od najmniej prawdopodobnego
	if (psDaneCM4->sKanalRC[chKanalDrazkaRC[POCH]] < WE_RC_M90)
	{
		if (psDaneCM4->sKanalRC[chKanalDrazkaRC[WYSO]] < WE_RC_M90)
		{
			//sprawdź warunek rozbrojenia silników, czyli oba drążki w dół na zewnątrz
			if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] < WE_RC_M90) &&	(psDaneCM4->sKanalRC[chKanalDrazkaRC[PRZE]] > WE_RC_P90))
			{
				RozbrojSilniki(psDaneCM4, psDaneCM7);
			}

			//sprawdź warunek uzbrojenia silników, czyli oba drążki w dół do środka
			if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] > WE_RC_P90) &&	(psDaneCM4->sKanalRC[chKanalDrazkaRC[PRZE]] < WE_RC_M90))
			{
				cBłąd = UzbrojSilniki(psDaneCM4, psDaneCM7);
			}
		}
	}

    //sprawdź warunek ..., czyli gaz na maksimum i kierunek w lewo
	if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[WYSO]] > WE_RC_P90) && (psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] < WE_RC_M90) && ((psDaneCM4->chTrybLotu & BTR_UZBROJONY) != BTR_UZBROJONY))
    {
        //obsługa ...
    }

    //sprawdź warunek zerowania zużycia energii, czyli gaz na maksimum i kierunek w prawo
    if ((psDaneCM4->sKanalRC[chKanalDrazkaRC[WYSO]] > WE_RC_P90) && (psDaneCM4->sKanalRC[chKanalDrazkaRC[ODCH]] > WE_RC_P90) && ((psDaneCM4->chTrybLotu & BTR_UZBROJONY) != BTR_UZBROJONY))
    {
        //obsługa resetownia licznika energii
    }


    //sprawdzenie stanu wyższych kanałów funkcyjnych
    for (uint8_t n=0; n< KANALY_FUNKCYJNE; n++)
    {
    	//wstępna eliminacja stanów ustalonych i drobnego szumu
    	if ((psDaneCM4->sKanalRC[n + LICZBA_DRAZKOW] > sPoprzedniStanKanaluRozszerzonego[n] + WE_RC_HISTEREZA) || (psDaneCM4->sKanalRC[n + LICZBA_DRAZKOW] < sPoprzedniStanKanaluRozszerzonego[n] - WE_RC_HISTEREZA))
    	{
			if ((psDaneCM4->sKanalRC[n + LICZBA_DRAZKOW] < WE_RC_M25 - WE_RC_HISTEREZA) ||	//sprawdź czy stan 3-położeniowego przełącznika  został przestawiony w dół
				(psDaneCM4->sKanalRC[n + LICZBA_DRAZKOW] > WE_RC_P25 - WE_RC_HISTEREZA))	//lub czy został przestawiony w górę
			{
				switch (chFunkcjaKanaluRC[n])
				{
				case FRC_WLACZ_OD1:		psDaneCM4->chWykonajPolecenie = POL4_WLACZ_OD1;		break;	//aktywuj wyjście otwarty dren 1
				case FRC_WLACZ_OD2:		psDaneCM4->chWykonajPolecenie = POL4_WLACZ_OD2;		break;	//aktywuj wyjście otwarty dren 2
				case FRC_MOW_WYSOKOSC:	psDaneCM4->chWykonajPolecenie = POL4_MOW_WYSOKOSC;	break;	//mów komunikat 1
				case FRC_MOW_NAPIECIE:	psDaneCM4->chWykonajPolecenie = POL4_MOW_NAPIECIE;	break;
				case FRC_MOW_PREDKOSC:	psDaneCM4->chWykonajPolecenie = POL4_MOW_PREDKOSC;	break;
				case FRC_MOW_KIERUNEK:	psDaneCM4->chWykonajPolecenie = POL4_MOW_KIERUNEK;	break;
				case FRC_MOW_TEMPERAT:	psDaneCM4->chWykonajPolecenie = POL4_MOW_TEMPERAT;	break;
				default:				break;
				}
			}
			else
			if ((psDaneCM4->sKanalRC[n + LICZBA_DRAZKOW] > WE_RC_M25 + WE_RC_HISTEREZA) && (psDaneCM4->sKanalRC[n + LICZBA_DRAZKOW] < WE_RC_P25 - WE_RC_HISTEREZA))	//czy wrócił do pozycji neutralnej
			{
				switch(chFunkcjaKanaluRC[n])
				{
				case FRC_WLACZ_OD1:		psDaneCM4->chWykonajPolecenie = POL4_WYLACZ_OD1;		break;	//wyłącz wyjście otwarty dren 1
				case FRC_WLACZ_OD2:		psDaneCM4->chWykonajPolecenie = POL4_WYLACZ_OD2;		break;	//wyłącz wyjście otwarty dren 2
				default:				break;
				}
			}
			sPoprzedniStanKanaluRozszerzonego[n] = psDaneCM4->sKanalRC[n + LICZBA_DRAZKOW];
    	}

    	//funkcjonalność liniowa
    	switch(chFunkcjaKanaluRC[n])
    	{
    	case FRC_STROJ_PID_PARAM1:	psDaneCM4->fStrojenie[0] = StrojeniePID_KanałemRC(&stStrojPID[0], n + LICZBA_DRAZKOW, stKonfigPID, psDaneCM4);	break;
    	case FRC_STROJ_PID_PARAM2:	psDaneCM4->fStrojenie[1] = StrojeniePID_KanałemRC(&stStrojPID[1], n + LICZBA_DRAZKOW, stKonfigPID, psDaneCM4);	break;

    	default:	break;
    	}
    }
    return cBłąd;
}



