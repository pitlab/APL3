//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi generowania i rejestracji dźwiękóww
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "audio.h"


extern SAI_HandleTypeDef hsai_BlockB2;
extern uint8_t chPorty_exp_wysylane[];

uint8_t InicjujAudio(void)
{
	//chPorty_exp_wysylane[1] |= EXP13_AUDIO_IN_SD;	//AUDIO_IN_SD - włącznika ShutDown mikrofonu
	chPorty_exp_wysylane[1] |= EXP14_AUDIO_OUT_SD;	//AUDIO_OUT_SD - włączniek ShutDown wzmacniacza audio

	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Odtwarza komunikat głosowy zapisany we flash
// Parametry: chNrKomunikatu - numer komunikatu okreslający pozycję w spisie komunikatów
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t GenerujAudio(uint8_t chNrKomunikatu)
{
	uint32_t nAdres, nRozmiar, nOdczytano = 0;
	uint8_t chErr, chBufor[256];

	nAdres   = *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrKomunikatu * ROZM_WPISU_AUDIO);
	nRozmiar = *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrKomunikatu * ROZM_WPISU_AUDIO + 4);
	do
	{
		for (uint16_t n=0; n<256; n++)
			chBufor[n] = *(uint8_t*)nAdres++;
		chErr = HAL_SAI_Transmit(&hsai_BlockB2, chBufor, 256, 10);
		nOdczytano += 256;
	}
	while (nOdczytano < nRozmiar);
	return chErr;
}



uint8_t RejestrujAudio(void)
{
	//HAL_StatusTypeDef HAL_SAI_Receive(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	return ERR_OK;
}
