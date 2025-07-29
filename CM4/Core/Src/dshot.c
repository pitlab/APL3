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

////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje timer wartościami do generowania protokołu DShot
// Parametry: chProtokol - indeks protokołu (obecnie tylko DShot150 i DShot300
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawTrybDShot(uint8_t chProtokol)
{
	uint8_t chErr;

	htim8.Instance = TIM8;
	htim8.Init.Prescaler = PODZIELNIK_TIMERA - 1;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	switch(chProtokol)
	{
	case PROTOKOL_DSHOT150: htim8.Init.Period = DS150_OKRESOW_BIT;	break;
	case PROTOKOL_DSHOT300: htim8.Init.Period = DS300_OKRESOW_BIT;	break;
	default:	return ERR_ZLE_POLECENIE;
	}

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
		if (sWysterowanie & 0x8000)	//wysyłany jest najstarszy przodem - sprawdzić
			nBuforDShot[n] = DS150_OKRESOW_T1H;	//wysyłany bit 1
		else
			nBuforDShot[n] = DS150_OKRESOW_T0H;	//wysyłany bit 0
		sWysterowanie <<= 1;		//wskaż kolejny bit
	}
	nBuforDShot[11] = DS150_OKRESOW_T0H;	//brak telemetrii

	//CRC
	sCRC = sWysterowanie >> 4;
	sWysterowanie ^= sCRC;
	sCRC = sWysterowanie >> 8;
	sCRC ^= sWysterowanie;
	sCRC &= 0x000F;
	for (uint8_t n=0; n<4; n++)
	{
		if (sCRC & 0x0008)	//wysyłany jest najstarszy przodem - sprawdzić
			nBuforDShot[n+12] = DS150_OKRESOW_T1H;	//wysyłany bit 1
		else
			nBuforDShot[n+12] = DS150_OKRESOW_T0H;	//wysyłany bit 0
		sCRC <<= 1;		//wskaż kolejny bit
	}
	nBuforDShot[DS_BITOW_LACZNIE - 1] = 100;	//przerwa między ramkami w ostatnim bicie

	chErr = HAL_TIM_OC_Stop_DMA(&htim8, TIM_CHANNEL_3);
	chErr = HAL_TIM_OC_Start_DMA(&htim8, TIM_CHANNEL_3, nBuforDShot, DS_BITOW_LACZNIE);
	return chErr;
}

