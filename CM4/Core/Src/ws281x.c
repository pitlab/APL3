//////////////////////////////////////////////////////////////////////////////
//
// Moduł sterowania programowalnymi LED WS2811 i WS2813
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "ws281x.h"
#include "dshot.h"
#include "WeWyRC.h"
#include "fram.h"

//Timer wysyła przez DMA sekwencję bitóe dla segmentu 4 LEDów po 24 bity, Trwa to 1,064us/bit czyli 102,144us na segment
//Sekwencję resetu majacą trwać minimum 280us generuję przez 3 puste sekwencje danych

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch4;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern DMA_HandleTypeDef hdma_tim8_ch3;
extern unia_wymianyCM4_t uDaneCM4;
extern uint16_t sFlagiNapelnieniaBuforow;
uint32_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM1"))) nBuforTimDMA_WS281X[WS_BITOW_LACZNIE];
stDShot_t stWS281x;
uint32_t nKolorWS281x[LICZBA_LED_WS281X];
float fZakresDolny[LICZBA_LED_WS281X];
float fZakresGorny[LICZBA_LED_WS281X];
uint8_t chWskaznikLed;
uint8_t chTypLed;	//indeks typu układu scalonego: 0=WS8211, 1=WS8513
stWskaznikLed_t stWskaznikLed[LICZBA_WSKAZNIKOW_LED];



////////////////////////////////////////////////////////////////////////////////
// Inicjuje strukturę zmiennych definiującą kolor LEDów
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujKoloryWS281x(void)
{
	uint8_t chBłąd = BLAD_OK;
	uint8_t chDane[ROZMIAR_WSKAZNIKA_LED - 8];

	CzytajBuforFRAM(FAU_WSKLED_TYP_LED, &chTypLed, 1);

	for (uint8_t n=0; n<LICZBA_WSKAZNIKOW_LED; n++)
	{
		CzytajBuforFRAM(FAU_WSKLED1_NUM_ZMIENNEJ + n*ROZMIAR_WSKAZNIKA_LED, chDane, ROZMIAR_WSKAZNIKA_LED - 8);
		stWskaznikLed[n].chNumZmiennej = chDane[0];
		stWskaznikLed[n].chSzerokoscWskaznika =  chDane[1];
		stWskaznikLed[n].chDzielnikJasnosciTla = chDane[2];
		stWskaznikLed[n].chCzerMin = chDane[3];
		stWskaznikLed[n].chCzerMax = chDane[4];
		stWskaznikLed[n].chNiebMin = chDane[5];
		stWskaznikLed[n].chNiebMax = chDane[6];
		stWskaznikLed[n].chZielMax = chDane[7];
		stWskaznikLed[n].chZielMin = chDane[8];
		stWskaznikLed[n].chLiczbaLed = chDane[9];

		stWskaznikLed[n].fWartoscMin = CzytajFramFloat(FAU_WSKLED1_MIN_ZMIENNEJ + n*ROZMIAR_WSKAZNIKA_LED);
		stWskaznikLed[n].fWartoscMax = CzytajFramFloat(FAU_WSKLED1_MAX_ZMIENNEJ + n*ROZMIAR_WSKAZNIKA_LED);

	}

	chBłąd = UstawKolorWS281x(nKolorWS281x, stWskaznikLed, uDaneCM4.dane.stBSP.fKatIMU[0]);
	return chBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Aktualizuje kolor palety w zależnosci od wartosci wizualizowanej zmiennej
// Parametry: fZmienna - wartość podlegająca wizualizacji na LED-ach
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktualizujKolorLedWs821x(float fZmienna)
{
	return UstawKolorWS281x(nKolorWS281x, stWskaznikLed, fZmienna);
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje timery wartościami do generowania protokołu komunikacyjnego ze sterowanikiem WS281x
// Parametry: chKanal - indeks indeks kanału serwa użytego do sterowania LEDami
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawTrybWS281x(uint8_t chKanal)
{
	uint8_t chBłąd = BLAD_OK;
	TIM_OC_InitTypeDef sConfigOC = {0};

	//oblicz długość impulsów do ustawienia w CC timera: czas impulsu / okres zegara => czas impulsu * częstotliwość zegara
	stWS281x.nBit = (uint32_t)(800e-9 * ZEGAR_WS281X + 0.5f);		//ns * MHz + zaokrąglenie
	stWS281x.nT1H = (uint32_t)(580e-9 * ZEGAR_WS281X + 0.5f);
	stWS281x.nT0H = (uint32_t)(220e-9 * ZEGAR_WS281X + 0.5f);

	AktualizujWS281xDMA(&sFlagiNapelnieniaBuforow, nKolorWS281x, LICZBA_LED_WS281X, &chWskaznikLed);	//przelicz czas trwania bitów

	//wspólna konfiguracja dla wszystkich kanałów
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = stWS281x.nT1H;

	if ((chKanal == KANAL_RC2) || (chKanal == KANAL_RC3))	//timer 2 obsluguje kanały 2 i 3
	{
		htim2.Init.Prescaler = DZIELNIK_WS281X - 1;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = stWS281x.nBit;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chBłąd |= HAL_TIM_PWM_Init(&htim2);
		HAL_NVIC_DisableIRQ(TIM2_IRQn);
	}

	if (chKanal == KANAL_RC2)		//kanał serw 2 obsługiwany przez Timer2ch3
	{
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;		//wyjście przechodzi przez inwerter, więc wymaga dodatkowego odwrócenia sygnału aby finalnie było niezmienione
		chBłąd |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
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
		chBłąd |= HAL_DMA_Init(&hdma_tim2_ch3);
		chBłąd |= HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, nBuforTimDMA_WS281X, WS_BITOW_LACZNIE);
	}

	if (chKanal == KANAL_RC3)		//kanał serw 3 obsługiwany przez Timer2ch1
	{
		chBłąd |= HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
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
		chBłąd |= HAL_DMA_Init(&hdma_tim2_ch1);
		chBłąd |= HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, nBuforTimDMA_WS281X, WS_BITOW_LACZNIE);
	}

	if ((chKanal == KANAL_RC4) || (chKanal == KANAL_RC5))	//timer 3 obsluguje kanały 4 i 5
	{
		htim3.Instance = TIM3;
		htim3.Init.Prescaler = DZIELNIK_WS281X - 1;
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = stWS281x.nBit;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		chBłąd |= HAL_TIM_PWM_Init(&htim3);
		HAL_NVIC_DisableIRQ(TIM3_IRQn);	//generowanie PWM dla DShot nie wymaga przerwań
	}

	if (chKanal == KANAL_RC4)		//kanał 4
	{
		chBłąd |= HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
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
		chBłąd |= HAL_DMA_Init(&hdma_tim3_ch3);
		chBłąd |= HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, nBuforTimDMA_WS281X, WS_BITOW_LACZNIE);
	}

	if (chKanal == KANAL_RC5)		//kanał 5
	{
		chBłąd |= HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
		hdma_tim3_ch4.Init.Request = DMA_REQUEST_TIM3_CH4;
		hdma_tim3_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tim3_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tim3_ch4.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tim3_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim3_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_tim3_ch4.Init.Mode = DMA_CIRCULAR;
		hdma_tim3_ch4.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_tim3_ch4.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		chBłąd |= HAL_DMA_Init(&hdma_tim3_ch4);
		chBłąd |= HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, nBuforTimDMA_WS281X, WS_BITOW_LACZNIE);
	}


	if ((chKanal == KANAL_RC6) || (chKanal == KANAL_RC8))	//timer 8 obsluguje kanały 6 i 8
	{
		__HAL_RCC_C1_TIM8_CLK_ENABLE();

		//ponownie inicjuj podstawę timera ze zmienionym dzielnikiem i okresem bitu
		htim8.Instance = TIM8;
		htim8.Init.Prescaler = DZIELNIK_WS281X - 1;
		htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim8.Init.Period = stWS281x.nBit;
		htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim8.Init.RepetitionCounter = 0;
		htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
		chBłąd |= HAL_TIM_PWM_Init(&htim8);
		HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);	//generowanie PWM dla DShot nie wymaga przerwań

		//wspólna konfiguracja dla kanałów 1 i 3
		//sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		//sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		//sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	}

	if (chKanal == KANAL_RC6)		//kanał 6
	{
		chBłąd |= HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
		hdma_tim8_ch1.Instance = DMA2_Stream3;
		hdma_tim8_ch1.Init.Request = DMA_REQUEST_TIM8_CH1;
		hdma_tim8_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tim8_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tim8_ch1.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tim8_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim8_ch1.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_tim8_ch1.Init.Mode = DMA_NORMAL;
		hdma_tim8_ch1.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_tim8_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		chBłąd |= HAL_DMA_Init(&hdma_tim8_ch1);
		HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
		__HAL_DMA_ENABLE_IT(&hdma_tim8_ch1, DMA_IT_HT);
		chBłąd |= HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, nBuforTimDMA_WS281X, WS_BITOW_LACZNIE);
	}

	if (chKanal == KANAL_RC8)	//kanał 8
	{
		chBłąd |= HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);
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
		chBłąd |= HAL_DMA_Init(&hdma_tim8_ch3);
		HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		__HAL_DMA_ENABLE_IT(&hdma_tim8_ch3, DMA_IT_HT);
		chBłąd |= HAL_TIMEx_PWMN_Start_DMA(&htim8, TIM_CHANNEL_3, nBuforTimDMA_WS281X, WS_BITOW_LACZNIE);	//specjalna funkcja dla kanału komplementarnego N
	}
	return chBłąd;
}





////////////////////////////////////////////////////////////////////////////////
// Funkcja ładuje stan kolejnych LEDów do bufora nBuforTimDMA_WS281X. Pracuje z podwójnym buforowaniem, więc
// w obsłudze przerwania opróżnienia polowy bufora ładuje pierwszą połowę segmentu czyli 2 LEDy a w obsłudze
// opróżnienia całego bufora ładuje kolejne 2 LED. Licznik *chWskLED odlicza kolejne LEDy.
// Na końcu danych generowany jest sygnał resetu trwający 3 pełne cykle DMA.
// Parametry:
//	*sFlagi - wskaźnik na flagi konieczności napełnienia buforów
//  *nTabKoloru - tablica kolorów kolejnych LED
//  chRozmiar - liczba sterowanych LED
//  *chWskLED - wskaźnik na kolejny LED, inkrementowane po 2 czyli po połowie bufora DMA
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktualizujWS281xDMA(uint16_t *sFlagi, uint32_t *nTabKoloru, uint8_t chRozmiar, uint8_t *chWskLED)
{
	uint8_t chBłąd = BLAD_OK;
	uint32_t nKolorLED;
	uint8_t chIndeksBitu;

	if (*sFlagi & NAPELNIJ_BUF1_CH8)	//napełnij pierwszą połowę bufora
	{
		if (*chWskLED < chRozmiar)
		{
			for (uint8_t m=0; m<WS_LEDY_SEGMENTU / 2; m++)		//iteracja po LED-ach w pierwszej połowie segmentu
			{
				nKolorLED = *(nTabKoloru + *chWskLED + m);
				chIndeksBitu = m * WS_BITOW_KOLORU;
				for (uint8_t n=0; n<WS_BITOW_KOLORU; n++)	//iteracja po bitach koloru
				{
					if (nKolorLED & 0x800000)	//wysyłany jest najstarszy przodem z 24 bitów
						nBuforTimDMA_WS281X[chIndeksBitu + n] = stWS281x.nT1H;	//wysyłany bit 1
					else
						nBuforTimDMA_WS281X[chIndeksBitu + n] = stWS281x.nT0H;	//wysyłany bit 0
					nKolorLED <<= 1;		//wysuń na pozycję kolejny bit
				}
			}
		}
		else	//generuj sygnał resetu, czyli same zera
		{
			for (uint8_t n=0; n<WS_BITOW_LACZNIE / 2; n++)
				nBuforTimDMA_WS281X[n] = 0;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);	//serwo kanał 1
		}
		*sFlagi &= ~NAPELNIJ_BUF1_CH8;
	}

	if (*sFlagi & NAPELNIJ_BUF2_CH8)	//napełnij drugą połowę bufora
	{
		if (*chWskLED < chRozmiar)
		{
			for (uint8_t m=WS_LEDY_SEGMENTU / 2; m<WS_LEDY_SEGMENTU; m++)		//iteracja po LED-ach w drugiej połowie segmentu
			{
				nKolorLED = *(nTabKoloru + *chWskLED + m);
				chIndeksBitu = m * WS_BITOW_KOLORU;
				for (uint8_t n=0; n<WS_BITOW_KOLORU; n++)	//iteracja po bitach koloru
				{
					if (nKolorLED & 0x800000)	//wysyłany jest najstarszy przodem z 24 bitów
						nBuforTimDMA_WS281X[chIndeksBitu + n] = stWS281x.nT1H;	//wysyłany bit 1
					else
						nBuforTimDMA_WS281X[chIndeksBitu + n] = stWS281x.nT0H;	//wysyłany bit 0
					nKolorLED <<= 1;		//wysuń na pozycję kolejny bit
				}
			}
		}
		else	//generuj sygnał resetu, czyli same zera
		{
			for (uint8_t n=WS_BITOW_LACZNIE / 2; n<WS_BITOW_LACZNIE; n++)
				nBuforTimDMA_WS281X[n] = 0;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);	//serwo kanał 1
		}
		*sFlagi &= ~NAPELNIJ_BUF2_CH8;

	//wskaż na następne LED do obsługi w kolejnym cyklu
	*chWskLED += 4;
	if (*chWskLED > chRozmiar + WS_CZAS_RESETU)
		*chWskLED = 0;
	}

	return chBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja ustawia kolory wszystkich sterowanych LED liniowo w zależności od zdefiniowanych wartosci skrajnych
// Oblicza też wartość dzielnika jasności rozjaśniającego grupę LEDów formujących plamkę wskaźnika
// Parametry: *nKolor - tablica kolorów kolejnych LED
// chRozmiar - liczba sterowanych LED
// *chWskSegmentu - wskaźnik na kolejny segment 4 LEDów
// *stPaleta - wskaźnik na strukturę zawierającą definicje do zbudowania kolorów skali mierzonej wartosci
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawKolorWS281x(uint32_t *nKolor, stWskaznikLed_t *stWskaznikLed, float fPomiar)
{
	float fDeltaCzer, fDeltaZiel, fDeltaNieb, fDeltaPomiaru;
	uint8_t chBłąd = BLAD_OK;
	uint8_t chIndeksTabKoloru = 0;
	float fZakres;
	float fRoznica;
	float fDzielnik;
	float fTrojkat;
	float fWysokoscTrojkata = 20;

	for (uint8_t m=0; m<LICZBA_WSKAZNIKOW_LED; m++)
	{
		fDeltaCzer = (float)(stWskaznikLed[m].chCzerMax - stWskaznikLed[m].chCzerMin) / stWskaznikLed[m].chLiczbaLed;
		fDeltaZiel = (float)(stWskaznikLed[m].chZielMax - stWskaznikLed[m].chZielMin) / stWskaznikLed[m].chLiczbaLed;
		fDeltaNieb = (float)(stWskaznikLed[m].chNiebMax - stWskaznikLed[m].chNiebMin) / stWskaznikLed[m].chLiczbaLed;
		fDeltaPomiaru = (stWskaznikLed[m].fWartoscMax - stWskaznikLed[m].fWartoscMin) / stWskaznikLed[m].chLiczbaLed;

		for (uint8_t n=0; n<stWskaznikLed[m].chLiczbaLed; n++)
		{
			fZakres = stWskaznikLed[m].fWartoscMin + (n+1) * fDeltaPomiaru;
			fRoznica = fabs(fPomiar - fZakres + fDeltaPomiaru / 2);
			fTrojkat = fWysokoscTrojkata * fmaxf(0, 1.0 - 2*fRoznica / (fDeltaPomiaru * stWskaznikLed[m].chSzerokoscWskaznika));
			if (fTrojkat < (float)stWskaznikLed[m].chDzielnikJasnosciTla)
				fDzielnik = (float)stWskaznikLed[m].chDzielnikJasnosciTla - fTrojkat;
			else
				fDzielnik = (float)stWskaznikLed[m].chDzielnikJasnosciTla;

			fZakresDolny[n] = fTrojkat;		//test
			fZakresGorny[n] = fDzielnik;	//test

			switch (chTypLed)
			{
			case WS2811:	//RGB
			*(nKolor + chIndeksTabKoloru) =
				((uint32_t)((stWskaznikLed[m].chCzerMin + fDeltaCzer * n) / fDzielnik) << 16) +
				((uint32_t)((stWskaznikLed[m].chZielMin + fDeltaZiel * n) / fDzielnik) << 8) +
				 (uint32_t)((stWskaznikLed[m].chNiebMin + fDeltaNieb * n) / fDzielnik);	break;

			case WS2813:	//GRB
			*(nKolor + chIndeksTabKoloru) =
				((uint32_t)((stWskaznikLed[m].chZielMin + fDeltaZiel * n) / fDzielnik) << 16) +
				((uint32_t)((stWskaznikLed[m].chCzerMin + fDeltaCzer * n) / fDzielnik) << 8) +
				 (uint32_t)((stWskaznikLed[m].chNiebMin + fDeltaNieb * n) / fDzielnik);	break;
			}
			chIndeksTabKoloru++;
		}
	}
	return chBłąd;
}

