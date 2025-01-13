//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi generowania i rejestracji dźwiękóww
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "audio.h"



static volatile int16_t sBuforAudioWy[ROZMIAR_BUFORA_AUDIO];	//bufor komunikatów wychodzących
//static int16_t sBuforTonu[ROZMIAR_BUFORA_AUDIO/2];				//przechowuje wygenerowany ton
static uint32_t nAdresKomunikatu;		//adres w pamieci flash skąd pobierany jest kolejny fragment komunikatu
static int32_t nRozmiarKomunikatu;		//pozostały do pobrania rozmiar komunikatu
static uint8_t chGlosnosc;		//regulacja głośności odtwarzania komunikatów w zakresie 0..SKALA_GLOSNOSCI



extern SAI_HandleTypeDef hsai_BlockB2;
extern uint8_t chPorty_exp_wysylane[];
extern void Error_Handler(void);

uint8_t InicjujAudio(void)
{
	//chPorty_exp_wysylane[1] |= EXP13_AUDIO_IN_SD;	//AUDIO_IN_SD - włącznika ShutDown mikrofonu
	chPorty_exp_wysylane[1] |= EXP14_AUDIO_OUT_SD;	//AUDIO_OUT_SD - włączniek ShutDown wzmacniacza audio
	chGlosnosc = 50;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Odtwarza komunikat głosowy zapisany we flash
// Parametry: chNrKomunikatu - numer komunikatu okreslający pozycję w spisie komunikatów
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t GenerujAudio(uint8_t chNrKomunikatu)
{
	uint32_t nRozmiar;
	uint8_t chErr;

	  hsai_BlockB2.Instance = SAI2_Block_B;
	  hsai_BlockB2.Init.AudioMode = SAI_MODEMASTER_TX;
	  hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
	  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
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

	nAdresKomunikatu   = *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrKomunikatu * ROZM_WPISU_AUDIO + 0);
	nRozmiarKomunikatu = *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrKomunikatu * ROZM_WPISU_AUDIO + 4);

	if (nRozmiarKomunikatu > ROZMIAR_BUFORA_AUDIO)
		nRozmiar = ROZMIAR_BUFORA_AUDIO;
	else
		nRozmiar = nRozmiarKomunikatu;

	//napełnij pierwszy bufor
	for (uint32_t n=0; n<nRozmiar; n++)
	{
		sBuforAudioWy[n] = (*(uint16_t*)nAdresKomunikatu * chGlosnosc) / SKALA_GLOSNOSCI;
		nAdresKomunikatu++;
	}
	nRozmiarKomunikatu -= nRozmiar;
	chErr = HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)sBuforAudioWy, 2*ROZMIAR_BUFORA_AUDIO);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Odtwarza komunikat głosowy zapisany we flash
// Parametry: chNrKomunikatu - numer komunikatu okreslający pozycję w spisie komunikatów
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t GenerujTonAudio(float fCzestotliwosc, uint16_t sGlosnosc)
{
	uint8_t chErr;

	hsai_BlockB2.Instance = SAI2_Block_B;
	hsai_BlockB2.Init.AudioMode = SAI_MODEMASTER_TX;
	hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
	hsai_BlockB2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	//hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_FULL;
	hsai_BlockB2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
	hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB2.Init.MonoStereoMode = SAI_MONOMODE;
	hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;

	hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
	{
		Error_Handler();
	}

	for (uint16_t n=0; n<ROZMIAR_BUFORA_AUDIO; n++)
	{
		sBuforAudioWy[n] = (int16_t)(sGlosnosc * sin(2 * M_PI * fCzestotliwosc * (float)n / SAI_AUDIO_FREQUENCY_16K));
	}

	nRozmiarKomunikatu = ROZMIAR_BUFORA_AUDIO;

	//nRozmiar = 2048;
	for (uint8_t n=0; n<100; n++)
	{
		//chErr = HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)sBuforAudioWy, ROZMIAR_BUFORA_AUDIO);
		//chErr = HAL_SAI_Transmit(&hsai_BlockB2, (uint8_t*)sBuforAudioWy, 2*ROZMIAR_BUFORA_AUDIO, 1000);
		chErr = HAL_SAI_Transmit(&hsai_BlockB2, (uint8_t*)sBuforAudioWy, ROZMIAR_BUFORA_AUDIO, 1000);
	}

	return chErr;
}





uint8_t GenerujTonAudio2(float fCzestotliwosc, uint16_t sGlosnosc)
{
	uint8_t chErr;

	hsai_BlockB2.Instance = SAI2_Block_B;
	hsai_BlockB2.Init.AudioMode = SAI_MODEMASTER_TX;
	hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
	hsai_BlockB2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	//hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_FULL;
	hsai_BlockB2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
	hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB2.Init.MonoStereoMode = SAI_MONOMODE;
	hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;

	hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
	{
		Error_Handler();
	}

	for (uint16_t n=0; n<ROZMIAR_BUFORA_AUDIO; n++)
	{
		sBuforAudioWy[n] = (int16_t)(sGlosnosc * sin(2 * M_PI * fCzestotliwosc * (float)n / SAI_AUDIO_FREQUENCY_16K));
	}

	nRozmiarKomunikatu = ROZMIAR_BUFORA_AUDIO;

	//nRozmiar = 2048;
	//for (uint8_t n=0; n<100; n++)
	{
		chErr = HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)sBuforAudioWy, ROZMIAR_BUFORA_AUDIO);
		//chErr = HAL_SAI_Transmit(&hsai_BlockB2, (uint8_t*)sBuforAudioWy, ROZMIAR_BUFORA_AUDIO, 1000);
	}

	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Callback opróżnienia połowy bufora
////////////////////////////////////////////////////////////////////////////////
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	uint32_t nRozmiar;

	if (nRozmiarKomunikatu > ROZMIAR_BUFORA_AUDIO/2)
		nRozmiar = ROZMIAR_BUFORA_AUDIO/2;
	else
		nRozmiar = nRozmiarKomunikatu;

	//napełnij pierwszą połowę bufora
	for (uint32_t n=0; n<nRozmiar; n++)
	{
		sBuforAudioWy[n] = (*(uint16_t*)nAdresKomunikatu * chGlosnosc) / SKALA_GLOSNOSCI;
		nAdresKomunikatu++;
	}
	nRozmiarKomunikatu -= nRozmiar;
	if (nRozmiarKomunikatu <= 0)
		HAL_SAI_DMAStop(&hsai_BlockB2);
}



////////////////////////////////////////////////////////////////////////////////
// Callback opróżnienia całego bufora
////////////////////////////////////////////////////////////////////////////////
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	uint32_t nRozmiar;

	if (nRozmiarKomunikatu > ROZMIAR_BUFORA_AUDIO/2)
		nRozmiar = ROZMIAR_BUFORA_AUDIO/2;
	else
		nRozmiar = nRozmiarKomunikatu;

	//napełnij drugą połowę bufora
	for (uint32_t n=0; n<nRozmiar; n++)
	{
		sBuforAudioWy[n+ROZMIAR_BUFORA_AUDIO/2] = (*(uint16_t*)nAdresKomunikatu * chGlosnosc) / SKALA_GLOSNOSCI;
		nAdresKomunikatu++;
	}
	nRozmiarKomunikatu -= nRozmiar;
	if (nRozmiarKomunikatu <= 0)
		HAL_SAI_DMAStop(&hsai_BlockB2);
}



////////////////////////////////////////////////////////////////////////////////
// Odtwarza komunikat głosowy zapisany we flash
// Parametry: chNrKomunikatu - numer komunikatu okreslający pozycję w spisie komunikatów
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t GenerujAudio2(uint8_t chNrKomunikatu)
{
	uint32_t nAdres, nRozmiar;

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
	if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
	{
		Error_Handler();
	}

	nAdres   = *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrKomunikatu * ROZM_WPISU_AUDIO);
	nRozmiar = *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrKomunikatu * ROZM_WPISU_AUDIO + 4);

	if (nRozmiar > 0xFFFF)
		nRozmiar = 0xFFFF;
	return HAL_SAI_Transmit(&hsai_BlockB2, (uint8_t*)nAdres, (uint16_t)nRozmiar, 10000);
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera dane z mokrofonu
// Parametry: nic
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
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
