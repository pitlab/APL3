//////////////////////////////////////////////////////////////////////////////
//
// Moduł komunikacji z regulatorami silników protokołem DShot
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "dshot.h"

extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern DMA_HandleTypeDef hdma_tim8_ch3;

ALIGN_32BYTES(uint32_t __attribute__((section(".SekcjaSRAM1")))	nBuforDShot[KANALY_MIKSERA][DS_BITOW_LACZNIE]);	//kolejne wartości bitów protokołu dla wszystkich kanałów
stDShot_t stDShot;



////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje timer wartościami do generowania protokołu DShot
// Parametry: chProtokol - indeks protokołu (obecnie tylko DShot150 i DShot300
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawTrybDShot(uint8_t chProtokol, uint8_t chKanal)
{
	uint8_t chErr;
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
		//htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
		htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chErr = HAL_TIM_PWM_Init(&htim8);
	}

	if (chKanal == KANAL_RC8)	//kanał 8
	{
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		sConfigOC.Pulse = stDShot.nT0H;
		chErr = HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);

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
		chErr = HAL_DMA_Init(&hdma_tim8_ch3);

		AktualizujDShotDMA(1000, chKanal);
		chErr = HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_3, &nBuforDShot[chKanal][0], DS_BITOW_LACZNIE);
	}

	if (chKanal == KANAL_RC6)		//kanał 6
	{
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		sConfigOC.Pulse = stDShot.nT1H;
		chErr = HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);

		//chErr = HAL_DMA_DeInit(&hdma_tim8_ch1);
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
		chErr = HAL_DMA_Init(&hdma_tim8_ch1);

		AktualizujDShotDMA(1000, chKanal);
		chErr = HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, &nBuforDShot[chKanal][0], DS_BITOW_LACZNIE);
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
	uint8_t chErr = ERR_OK;
	uint16_t sCRC;
	//TIM_OC_InitTypeDef sConfigOC = {0};
	if (sWysterowanie > DS_MAX_DANE)
	{
		sWysterowanie = DS_MAX_DANE;
		chErr = ERR_ZLE_DANE;
	}

	sWysterowanie += DS_OFFSET_DANYCH;
	for (uint8_t n=0; n<11; n++)
	{
		if (sWysterowanie & 0x400)	//wysyłany jest najstarszy przodem z 11 bitów - sprawdzić
			nBuforDShot[chKanal][n] = stDShot.nT1H;	//wysyłany bit 1
		else
			nBuforDShot[chKanal][n] = stDShot.nT0H;	//wysyłany bit 0
		sWysterowanie <<= 1;		//wskaż kolejny bit
	}
	nBuforDShot[chKanal][11] = stDShot.nT0H;	//brak telemetrii = 0

	//CRC
	sCRC = sWysterowanie >> 4;
	sWysterowanie ^= sCRC;
	sCRC = sWysterowanie >> 8;
	sCRC ^= sWysterowanie;
	sCRC &= 0x000F;
	for (uint8_t n=0; n<4; n++)
	{
		if (sCRC & 0x0008)	//wysyłany jest najstarszy przodem - sprawdzić
			nBuforDShot[chKanal][n+12] = stDShot.nT1H;	//wysyłany bit 1
		else
			nBuforDShot[chKanal][n+12] = stDShot.nT0H;	//wysyłany bit 0
		sCRC <<= 1;		//wskaż kolejny bit
	}
	//nBuforDShot[chKanal][DS_BITOW_LACZNIE - 1] = 0;	//przerwa między ramkami w ostatnim bicie

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
	//chErr = HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, nBuforDShot, DS_BITOW_LACZNIE);
	//chErr = HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_3, nBuforDShot, DS_BITOW_LACZNIE);

	return chErr;
}



