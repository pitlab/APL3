//////////////////////////////////////////////////////////////////////////////
//
// Moduł komunikacji z regulatorami silników protokołem DShot
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "dshot.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch4;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern DMA_HandleTypeDef hdma_tim8_ch3;
uint32_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM1"))) nBuforTimDMA[KANALY_MIKSERA][DS_BITOW_LACZNIE] = {0};
stDShot_t stDShot;



////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje timer wartościami do generowania protokołu DShot
// Parametry: chProtokol - indeks protokołu (obecnie tylko DShot150 i DShot300
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawTrybDShot(uint8_t chProtokol, uint8_t chKanal)
{
	uint8_t chErr = BLAD_OK;
	uint32_t nDzielnik;
	TIM_OC_InitTypeDef sConfigOC = {0};

	//sprawdź czy zegar jest odpowiedni dla danej prędkosci protokołu
	uint32_t nHCLK = HAL_RCC_GetHCLKFreq();
	switch(chProtokol)
	{
	case PROTOKOL_DSHOT150:
		nDzielnik = nHCLK / DS150_ZEGAR;	//6000000 Hz
		if ((nDzielnik * DS150_ZEGAR) != nHCLK)	//jeżeli dzielnik nie jest liczbą całkowitą to wyjdź z kodem błędu
			return ERR_ZLY_STAN_PROT;

		//oblicz długość impulsów do ustawienia w CC timera: czas impulsu / okres zegara => czas impulsu * częstotliwość zegara
		stDShot.nBit = (uint32_t)(6666e-9 * DS150_ZEGAR + 0.5f);	//ns * MHz + zaokrąglenie
		stDShot.nT1H = (uint32_t)(5000e-9 * DS150_ZEGAR + 0.5f);
		stDShot.nT0H = (uint32_t)(2500e-9 * DS150_ZEGAR + 0.5f);
		break;

	case PROTOKOL_DSHOT300:
		nDzielnik = nHCLK / DS300_ZEGAR;	//12000000 Hz
		if ((nDzielnik * DS300_ZEGAR) != nHCLK)
			return ERR_ZLY_STAN_PROT;

		stDShot.nBit = (uint32_t)(3333e-9 * DS300_ZEGAR + 0.5f);	//ns * MHz + zaokrąglenie
		stDShot.nT1H = (uint32_t)(2500e-9 * DS300_ZEGAR + 0.5f);
		stDShot.nT0H = (uint32_t)(1250e-9 * DS300_ZEGAR + 0.5f);
		break;

	case PROTOKOL_DSHOT600:
		nDzielnik = nHCLK / DS600_ZEGAR;	//24000000 Hz
		if ((nDzielnik * DS600_ZEGAR) != nHCLK)
			return ERR_ZLY_STAN_PROT;

		stDShot.nBit = (uint32_t)(1666e-9 * DS600_ZEGAR + 0.5f);
		stDShot.nT1H = (uint32_t)(1250e-9 * DS600_ZEGAR + 0.5f);
		stDShot.nT0H = (uint32_t)(625e-9 * DS600_ZEGAR + 0.5f);
		break;

	case PROTOKOL_DSHOT1200:
		nDzielnik = nHCLK / DS1200_ZEGAR;	//48000000 Hz
		if ((nDzielnik * DS1200_ZEGAR) != nHCLK)
			return ERR_ZLY_STAN_PROT;

		stDShot.nBit = (uint32_t)(833e-9 * DS1200_ZEGAR + 0.5f);
		stDShot.nT1H = (uint32_t)(625e-9 * DS1200_ZEGAR + 0.5f);
		stDShot.nT0H = (uint32_t)(313e-9 * DS1200_ZEGAR + 0.5f);
		break;

	default:	return ERR_ZLE_POLECENIE;
	}

	//wspólna konfiguracja dla wszystkich kanałów
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = stDShot.nT1H;


	if ((chKanal == KANAL_RC2) || (chKanal == KANAL_RC3))	//timer 2 obsluguje kanały 2 i 3
	{
		htim2.Init.Prescaler = nDzielnik - 1;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = stDShot.nBit;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		//HAL_TIM_Base_Init(&htim2);
		chErr |= HAL_TIM_PWM_Init(&htim2);
		AktualizujDShotDMA(1000, chKanal);	//przelicz czas trwania bitów
		HAL_NVIC_DisableIRQ(TIM2_IRQn);
	}

	if (chKanal == KANAL_RC2)		//kanał serw 2 obsługiwany przez Timer2ch3
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

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
		HAL_DMA_Init(&hdma_tim2_ch3);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, &nBuforTimDMA[chKanal][0], DS_BITOW_DANYCH + DS_BITOW_PRZERWY);
	}

	if (chKanal == KANAL_RC3)		//kanał serw 3 obsługiwany przez Timer2ch1
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

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
		chErr |= HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &nBuforTimDMA[chKanal][0], DS_BITOW_DANYCH + DS_BITOW_PRZERWY);
	}


	if ((chKanal == KANAL_RC4) || (chKanal == KANAL_RC5))	//timer 3 obsluguje kanały 4 i 5
	{
		htim3.Instance = TIM3;
		htim3.Init.Prescaler = nDzielnik - 1;
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = stDShot.nBit;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr |= HAL_TIM_PWM_Init(&htim3);
		AktualizujDShotDMA(1000, chKanal);	//przelicz czas trwania bitów
		//HAL_NVIC_DisableIRQ(TIM3_IRQn);	//generowanie PWM dla DShot nie wymaga przerwań
	}

	if (chKanal == KANAL_RC4)		//kanał 4
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

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
		chErr |= HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, &nBuforTimDMA[chKanal][0], DS_BITOW_DANYCH + DS_BITOW_PRZERWY);
	}

	if (chKanal == KANAL_RC5)		//kanał 5
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

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
		chErr |= HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, &nBuforTimDMA[chKanal][0], DS_BITOW_DANYCH + DS_BITOW_PRZERWY);
	}


	if ((chKanal == KANAL_RC6) || (chKanal == KANAL_RC8))	//timer 8 obsluguje kanały 6 i 8
	{
		__HAL_RCC_C1_TIM8_CLK_ENABLE();

		//ponownie inicjuj podstawę timera ze zmienionym dzielnikiem i okresem bitu
		htim8.Instance = TIM8;
		htim8.Init.Prescaler = nDzielnik - 1;
		htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim8.Init.Period = stDShot.nBit;
		htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim8.Init.RepetitionCounter = 0;
		htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
		chErr |= HAL_TIM_PWM_Init(&htim8);
		AktualizujDShotDMA(1000, chKanal);	//przelicz czas trwania bitów

		HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);	//generowanie PWM dla DShot nie wymaga przerwań

		//wspólna konfiguracja dla kanałów 1 i 3
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	}

	if (chKanal == KANAL_RC6)		//kanał 6
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);

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
		chErr |= HAL_DMA_Init(&hdma_tim8_ch1);
		chErr |= HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, &nBuforTimDMA[chKanal][0], DS_BITOW_DANYCH + DS_BITOW_PRZERWY);

	}

	if (chKanal == KANAL_RC8)	//kanał 8
	{
		chErr |= HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);

		hdma_tim8_ch3.Instance = DMA2_Stream2;
		hdma_tim8_ch3.Init.Request = DMA_REQUEST_TIM8_CH3;
		hdma_tim8_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tim8_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tim8_ch3.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tim8_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim8_ch3.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim8_ch3.Init.Mode = DMA_CIRCULAR;
		hdma_tim8_ch3.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_tim8_ch3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		chErr |= HAL_DMA_Init(&hdma_tim8_ch3);
		chErr |= HAL_TIMEx_PWMN_Start_DMA(&htim8, TIM_CHANNEL_3, &nBuforTimDMA[chKanal][0], DS_BITOW_DANYCH + DS_BITOW_PRZERWY);	//specjalna funkcja dla kanału komplementarnego
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje dane wysyłane cyklicznie do regulatora
// Parametry: sWysterowanie - wartość jaka ma być wysłana do regulatora z zakresu 0..1999
// chKanal - numer kanału w jakim są aktualizowane dane
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktualizujDShotDMA(uint16_t sWysterowanie, uint8_t chKanal)
{
	uint8_t chErr = BLAD_OK;
	uint16_t sCRC;
	if (sWysterowanie > DS_MAX_DANE)
	{
		sWysterowanie = DS_MAX_DANE;
		chErr = ERR_ZLE_DANE;
	}

	sWysterowanie += DS_OFFSET_DANYCH;
	for (uint8_t n=0; n<11; n++)
	{
		if (sWysterowanie & 0x400)	//wysyłany jest najstarszy przodem z 11 bitów - sprawdzić
			nBuforTimDMA[chKanal][n] = stDShot.nT1H;	//wysyłany bit 1
		else
			nBuforTimDMA[chKanal][n] = stDShot.nT0H;	//wysyłany bit 0
		sWysterowanie <<= 1;		//wskaż kolejny bit
	}
	nBuforTimDMA[chKanal][11] = stDShot.nT0H;	//brak telemetrii = 0

	//CRC
	sCRC = sWysterowanie >> 4;
	sWysterowanie ^= sCRC;
	sCRC = sWysterowanie >> 8;
	sCRC ^= sWysterowanie;
	sCRC &= 0x000F;
	for (uint8_t n=0; n<4; n++)
	{
		if (sCRC & 0x0008)	//wysyłany jest najstarszy przodem - sprawdzić
			nBuforTimDMA[chKanal][n+12] = stDShot.nT1H;	//wysyłany bit 1
		else
			nBuforTimDMA[chKanal][n+12] = stDShot.nT0H;	//wysyłany bit 0
		sCRC <<= 1;		//wskaż kolejny bit
	}

	//zerami oznacz przerwę między ramkami
	for (uint8_t n=0; n<DS_BITOW_PRZERWY; n++)
		nBuforTimDMA[chKanal][n + DS_BITOW_DANYCH] = 0;


	//nBuforTimDMA[chKanal][DS_BITOW_LACZNIE - 1] = 0;	//przerwa między ramkami w ostatnim bicie

	//chErr = HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_1);
	//chErr = HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_3);

	/*sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	sConfigOC.Pulse = stDShot.nT1H;
	chErr = HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);

	sConfigOC.Pulse = stDShot.nT0H;
	chErr = HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);

	//chErr = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	//chErr = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);*/
	//chErr = HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, nBuforTimDMA, DS_BITOW_LACZNIE);
	//chErr = HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_3, nBuforTimDMA, DS_BITOW_LACZNIE);

	return chErr;
}



