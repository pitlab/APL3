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
extern void Error_Handler(void);

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
	uint8_t chErr, chBufor[512];
	uint16_t sTemp, sBufor[256] ;

	hsai_BlockB2.Instance = SAI2_Block_B;
	hsai_BlockB2.Init.AudioMode = SAI_MODEMASTER_TX;
	hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
	hsai_BlockB2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockB2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
	hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB2.Init.MonoStereoMode = SAI_MONOMODE;
	hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	//if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
	if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_LSBJUSTIFIED, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
	{
		Error_Handler();
	}
//#define SAI_I2S_STANDARD         0U
//#define SAI_I2S_MSBJUSTIFIED     1U
//#define SAI_I2S_LSBJUSTIFIED     2U

	for (uint16_t n=0; n<256; n++)
	{
		sTemp = (uint16_t)(0xFFF * (sin(2 * M_PI * 500 * ((float)n / 16000)) + 1.0) / 2.0);
		sBufor[n] = sTemp;
		chBufor[2*n+1] = sTemp & 0xFF;
		chBufor[2*n+0] = sTemp>>8;

	}


	nAdres   = *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrKomunikatu * ROZM_WPISU_AUDIO);
	nRozmiar = *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrKomunikatu * ROZM_WPISU_AUDIO + 4);
	do
	{
		//for (uint16_t n=0; n<512; n++)
			//chBufor[n] = *(uint8_t*)nAdres++;

		chErr = HAL_SAI_Transmit(&hsai_BlockB2, chBufor, 512, 100);
		//nOdczytano += 256;
		nOdczytano += 2048;
	}
	while (nOdczytano < nRozmiar);
	return chErr;
}



uint8_t RejestrujAudio(void)
{
	uint8_t chErr, chIlosc, chBufor[512];

	hsai_BlockB2.Instance = SAI2_Block_B;
	hsai_BlockB2.Init.AudioMode = SAI_MODEMASTER_RX;
	hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
	hsai_BlockB2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockB2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
	hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB2.Init.MonoStereoMode = SAI_MONOMODE;
	hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
	{
		Error_Handler();
	}
	for (uint16_t n=0; n<512; n++)
		chBufor[n] = n;

	chIlosc = 0;
	do
	{
		chErr = HAL_SAI_Receive(&hsai_BlockB2, chBufor, 256, 100);
		chIlosc++;
	}
	while (chIlosc < 50);
	return chErr;
}
