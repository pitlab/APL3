//////////////////////////////////////////////////////////////////////////////
//
// Moduł komunikacji z regulatorami silników protokołem DShot
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "dshot.h"

extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim8_ch3;
uint32_t nBuforDShot[DS_BITOW_LACZNIE];	//kolejne wartości bitów protokołu
stDShot_t stDShot;



////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje sposób sterowania regulatorami silników ustawiając PWM lub DShot
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujESC(void)
{
	//odczytaj konfigurację z FRAM
	return UstawTrybDShot(PROTOKOL_DSHOT300);
}


////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje timer wartościami do generowania protokołu DShot
// Parametry: chProtokol - indeks protokołu (obecnie tylko DShot150 i DShot300
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawTrybDShot(uint8_t chProtokol)
{
	uint8_t chErr;
	uint32_t nDzielnik;

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

	htim8.Init.Period = stDShot.nBit;
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = nDzielnik - 1;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	chErr = HAL_TIM_Base_Init(&htim8);

	hdma_tim8_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_tim8_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_tim8_ch3.Init.MemInc = DMA_MINC_ENABLE;
	hdma_tim8_ch3.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_tim8_ch3.Init.Mode = DMA_CIRCULAR;
	chErr = HAL_DMA_Init(&hdma_tim8_ch3);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje dane wysyłane cyklicznie do regulatora
// Parametry: sWysterowanie - wartość jaka ma być wysłana do regulatora z zakresu 0..1999
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktualizujDShotDMA(uint16_t sWysterowanie)
{
	uint8_t chErr;
	uint16_t sCRC;

	sWysterowanie += DS_OFFSET_DANYCH;
	for (uint8_t n=0; n<11; n++)
	{
		if (sWysterowanie & 0x400)	//wysyłany jest najstarszy przodem z 11 bitów - sprawdzić
			nBuforDShot[n] = stDShot.nT1H;	//wysyłany bit 1
		else
			nBuforDShot[n] = stDShot.nT0H;	//wysyłany bit 0
		sWysterowanie <<= 1;		//wskaż kolejny bit
	}
	nBuforDShot[11] = stDShot.nT0H;	//brak telemetrii = 0

	//CRC
	sCRC = sWysterowanie >> 4;
	sWysterowanie ^= sCRC;
	sCRC = sWysterowanie >> 8;
	sCRC ^= sWysterowanie;
	sCRC &= 0x000F;
	for (uint8_t n=0; n<4; n++)
	{
		if (sCRC & 0x0008)	//wysyłany jest najstarszy przodem - sprawdzić
			nBuforDShot[n+12] = stDShot.nT1H;	//wysyłany bit 1
		else
			nBuforDShot[n+12] = stDShot.nT0H;	//wysyłany bit 0
		sCRC <<= 1;		//wskaż kolejny bit
	}
	nBuforDShot[DS_BITOW_LACZNIE - 1] = 50;	//przerwa między ramkami w ostatnim bicie

	chErr = HAL_TIM_OC_Stop_DMA(&htim8, TIM_CHANNEL_3);
	chErr = HAL_TIM_OC_Start_DMA(&htim8, TIM_CHANNEL_3, nBuforDShot, DS_BITOW_LACZNIE);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje dane wysyłane na wyjścia RC
// Parametry: *dane - wskaźnik na strukturę zawierajacą wartości wyjścia regulatorów PID
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktualizujWyjsciaRC(stWymianyCM4_t *dane)
{
	uint16_t sWysterowanie[KANALY_MIKSERA];

	//aktualizuj młodsze 8 kanałów serw/ESC. Starsza mogą być tylko PWM
	for (uint8_t n=0; n<KANALY_MIKSERA / 2; n++)
	{
		//sprawdź rodzaj ustawionego protokołu wyjscia

		//if DSHot
		sWysterowanie[n] = dane->sSerwo[n];
	}

	//if DSHot
	return AktualizujDShotDMA(sWysterowanie[0]);
}
