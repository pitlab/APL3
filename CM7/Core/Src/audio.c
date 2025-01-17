//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi generowania i rejestracji dźwiękóww
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "audio.h"


static volatile uint16_t sBuforAudioWy[ROZMIAR_BUFORA_AUDIO];	//bufor komunikatów wychodzących
static volatile uint16_t sBuforAudioWe[ROZMIAR_BUFORA_AUDIO];	//bufor komunikatów przychodzących
static uint32_t nAdresKomunikatu;		//adres w pamieci flash skąd pobierany jest kolejny fragment komunikatu
static int32_t nRozmiarKomunikatu;		//pozostały do pobrania rozmiar komunikatu
uint8_t chGlosnosc;		//regulacja głośności odtwarzania komunikatów w zakresie 0..SKALA_GLOSNOSCI

extern SAI_HandleTypeDef hsai_BlockB2;
extern uint8_t chPorty_exp_wysylane[];
extern void Error_Handler(void);


////////////////////////////////////////////////////////////////////////////////
// Wykonuje inicjalizację zasobów dotwarzania dźwięku. Uruchamiane przy starcie
// Parametry: nic
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujAudio(void)
{
	//domyslnie ustaw wejscia ShutDown tak aby aktywny był wzmacniacz
	//chPorty_exp_wysylane[1] |= EXP13_AUDIO_IN_SD;	//AUDIO_IN_SD - włącznika ShutDown mikrofonu
	chPorty_exp_wysylane[1] |= EXP14_AUDIO_OUT_SD;	//AUDIO_OUT_SD - włączniek ShutDown wzmacniacza audio
	chGlosnosc = 45;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Odtwarza komunikat głosowy obecny w spisie komunikatów
// Parametry: chNrKomunikatu - numer komunikatu okreslający pozycję w spisie
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t OdtworzProbkeAudioZeSpisu(uint8_t chNrKomunikatu)
{
	return OdtworzProbkeAudio( *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrKomunikatu * ROZM_WPISU_AUDIO + 0), *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrKomunikatu * ROZM_WPISU_AUDIO + 4) / 2);
}



////////////////////////////////////////////////////////////////////////////////
// Odtwarza pojedynczy komunikat głosowy zapisany w adresowalnym obszarze pamięci poprzez DMA pracujące z buforem kołowym
// Parametry:
// 	nAdres - adres początku komunikatu
//	nRozmiar - rozmiar komunikat w bajtach
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t OdtworzProbkeAudio(uint32_t nAdres, uint32_t nRozmiar)
{
	nAdresKomunikatu   = nAdres;	//przepisz do zmiennych globalnych
	nRozmiarKomunikatu = nRozmiar;

	//napełnij pierwszy cały bufor, reszta będzie dopełniana połówkami w callbackach od opróżnienia połowy i całego bufora
	for (uint32_t n=0; n<ROZMIAR_BUFORA_AUDIO; n++)
	{
		//sBuforAudioWy[n] =  *(int16_t*)nAdresKomunikatu;
		sBuforAudioWy[n] = (*(int16_t*)nAdresKomunikatu * chGlosnosc) / SKALA_GLOSNOSCI;
		nAdresKomunikatu += 2;
	}
	nRozmiarKomunikatu -= ROZMIAR_BUFORA_AUDIO;

	return HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)sBuforAudioWy, (uint16_t)ROZMIAR_BUFORA_AUDIO);
}



////////////////////////////////////////////////////////////////////////////////
// Callback opróżnienia połowy bufora
// Parametry: *hsai - wskaźnik na strukturę opisującą sprzęt
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	uint32_t nRozmiar;
	//int32_t nGlosnosc = chGlosnosc;
	//int32_t nTemp;

	if (nRozmiarKomunikatu > ROZMIAR_BUFORA_AUDIO/2)
		nRozmiar = ROZMIAR_BUFORA_AUDIO/2;
	else
		nRozmiar = nRozmiarKomunikatu;

	//napełnij pierwszą połowę bufora
	for (uint32_t n=0; n<nRozmiar; n++)
	{
		/*/poniższy kod działa poprawnie, zajmuje 20 cykli maszynowych
		nTemp = *(int16_t*)nAdresKomunikatu;
		nTemp *= nGlosnosc;
		nTemp /= SKALA_GLOSNOSCI;
		sBuforAudioWy[n] = nTemp;*/

		//też działa dobrze,  zajmuje 15 cykli maszynowych
		//sBuforAudioWy[n] = (int16_t)((*(int16_t*)nAdresKomunikatu * nGlosnosc) / SKALA_GLOSNOSCI);	//OK
		//sBuforAudioWy[n] = (int16_t)((*(int16_t*)nAdresKomunikatu * chGlosnosc) / SKALA_GLOSNOSCI);	//OK
		sBuforAudioWy[n] = (*(int16_t*)nAdresKomunikatu * chGlosnosc) / SKALA_GLOSNOSCI;	//OK
		nAdresKomunikatu += 2;
	}
	nRozmiarKomunikatu -= nRozmiar;
	if (nRozmiarKomunikatu <= 0)
		HAL_SAI_DMAStop(&hsai_BlockB2);
}



////////////////////////////////////////////////////////////////////////////////
// Callback opróżnienia całego bufora
// Parametry: *hsai - wskaźnik na strukturę opisującą sprzęt
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	uint32_t nRozmiar;
	//int32_t nGlosnosc = chGlosnosc;

	if (nRozmiarKomunikatu > ROZMIAR_BUFORA_AUDIO/2)
		nRozmiar = ROZMIAR_BUFORA_AUDIO/2;
	else
		nRozmiar = nRozmiarKomunikatu;

	//napełnij drugą połowę bufora
	for (uint32_t n=0; n<nRozmiar; n++)
	{
		//sBuforAudioWy[n+ROZMIAR_BUFORA_AUDIO/2] = (int16_t)((*(int16_t*)nAdresKomunikatu * nGlosnosc) / SKALA_GLOSNOSCI);
		//sBuforAudioWy[n+ROZMIAR_BUFORA_AUDIO/2] = (int16_t)((*(int16_t*)nAdresKomunikatu * chGlosnosc) / SKALA_GLOSNOSCI);
		sBuforAudioWy[n+ROZMIAR_BUFORA_AUDIO/2] = (*(int16_t*)nAdresKomunikatu * chGlosnosc) / SKALA_GLOSNOSCI;
		nAdresKomunikatu += 2;
	}
	nRozmiarKomunikatu -= nRozmiar;
	if (nRozmiarKomunikatu <= 0)
		HAL_SAI_DMAStop(&hsai_BlockB2);
}




////////////////////////////////////////////////////////////////////////////////
// Pobiera dane z mikrofonu
// Parametry: nic
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t RejestrujAudio(void)
{
	uint8_t chErr;

	//Włącza mikrofon wyłącza wzmacniacz
	chPorty_exp_wysylane[1] |= EXP13_AUDIO_IN_SD;	//AUDIO_IN_SD - włącznika ShutDown mikrofonu, aktywny niski
	chPorty_exp_wysylane[1] &= ~EXP14_AUDIO_OUT_SD;	//AUDIO_OUT_SD - włączniek ShutDown wzmacniacza audio, aktywny niski


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
	hsai_BlockB2.Init.TriState = SAI_OUTPUT_RELEASED;
	if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
	{
		Error_Handler();
	}
	for (uint16_t n=0; n<ROZMIAR_BUFORA_AUDIO; n++)
		sBuforAudioWe[n] = n;

	chErr = HAL_SAI_Receive(&hsai_BlockB2, (uint8_t*)sBuforAudioWe, ROZMIAR_BUFORA_AUDIO, 1000);
	return chErr;
}
