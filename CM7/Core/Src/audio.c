//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi generowania i rejestracji dźwięków
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "audio.h"

static volatile int16_t sBuforAudioWy[ROZMIAR_BUFORA_AUDIO];	//bufor komunikatów wychodzących
static volatile int16_t sBuforAudioWe[ROZMIAR_BUFORA_AUDIO];	//bufor komunikatów przychodzących
static volatile int16_t sBuforTonuWario[ROZMIAR_BUFORA_TONU];	//bufor do przechowywania podstawowego tonu wario
static volatile int16_t sBuforNowegoTonuWario[ROZMIAR_BUFORA_TONU];	//bufor nowego tonu wario, który ma się zsynchronizować z podstawowym buforem w chwili przejścia przez zero aby uniknąć zakłóceń
static uint8_t chJestNowyTon;			//flaga informująca o tym że pojawił się nowy ton i trzeba go synchronicznie przepisać to podstawowego bufora tonu
static uint16_t sWskTonu;				//wskazuje na bieżącą próbkę w tablicy tonu
static uint16_t sRozmiarTonu;			//długość tablicy pełnego sinusa
static uint16_t sRozmiarNowegoTonu;		//długość tablicy nowego tonu do zsynchronizowania się z sRozmiarTonu w chwili przejscia przez zero

static uint16_t sAmpl1Harm = AMPLITUDA_1HARM;			//amplituda pierwszej harmonicznej sygnału
static uint16_t sAmpl3Harm = AMPLITUDA_3HARM;			//amplituda trzeciej harmonicznej sygnału


//uint8_t chGenerujTonWario = 0;			//flaga właczajaca generowanie tonu
uint8_t chNumerTonu;
static uint32_t nAdresKomunikatu;		//adres w pamieci flash skąd pobierany jest kolejny fragment komunikatu
int32_t nRozmiarKomunikatu;		//pozostały do pobrania rozmiar komunikatu
uint8_t chGlosnosc;		//regulacja głośności odtwarzania komunikatów w zakresie 0..SKALA_GLOSNOSCI_AUDIO

extern SAI_HandleTypeDef hsai_BlockB2;
extern uint8_t chPorty_exp_wysylane[];
extern void Error_Handler(void);




////////////////////////////////////////////////////////////////////////////////
// Wykonuje inicjalizację zasobów dotwarzania dźwięku. Uruchamiane przy starcie
// Parametry: nic
// Zwraca: kod błędu
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
// Zwraca: kod błędu
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
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t OdtworzProbkeAudio(uint32_t nAdres, uint32_t nRozmiar)
{
	nAdresKomunikatu   = nAdres;	//przepisz do zmiennych globalnych
	nRozmiarKomunikatu = nRozmiar;

	//napełnij pierwszy cały bufor, reszta będzie dopełniana połówkami w callbackach od opróżnienia połowy i całego bufora
	for (uint32_t n=0; n<ROZMIAR_BUFORA_AUDIO; n++)
	{
		//sBuforAudioWy[n] =  *(int16_t*)nAdresKomunikatu;
		sBuforAudioWy[n] = (*(int16_t*)nAdresKomunikatu * chGlosnosc) / SKALA_GLOSNOSCI_AUDIO;
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

	if (chNumerTonu < LICZBA_TONOW_WARIO)		//jeżeli generuje ton
		nRozmiar = ROZMIAR_BUFORA_AUDIO/2;		//to zawsze pracuj na pełnym buforze
	else
	{
		if (nRozmiarKomunikatu > ROZMIAR_BUFORA_AUDIO/2)
			nRozmiar = ROZMIAR_BUFORA_AUDIO/2;
		else
			nRozmiar = nRozmiarKomunikatu;
	}

	//napełnij pierwszą połowę bufora
	for (uint16_t n=0; n<nRozmiar; n++)
	{
		//sBuforAudioWy[n] = (*(int16_t*)nAdresKomunikatu * chGlosnosc) / SKALA_GLOSNOSCI_AUDIO;	//OK
		sBuforAudioWy[n] = sBuforTonuWario[sWskTonu];
		sWskTonu++;
		if (sWskTonu >= sRozmiarTonu)
			sWskTonu = 0;
		nAdresKomunikatu += 2;
	}
	if (nRozmiarKomunikatu > 0)
		nRozmiarKomunikatu -= nRozmiar;

	if ((nRozmiarKomunikatu <= 0) && (chNumerTonu >= LICZBA_TONOW_WARIO))
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

	if (chNumerTonu < LICZBA_TONOW_WARIO)		//jeżeli generuje ton
		nRozmiar = ROZMIAR_BUFORA_AUDIO/2;		//to zawsze pracuj na pełnym buforze
	else
	{
		if (nRozmiarKomunikatu > ROZMIAR_BUFORA_AUDIO/2)
			nRozmiar = ROZMIAR_BUFORA_AUDIO/2;
		else
			nRozmiar = nRozmiarKomunikatu;
	}

	//napełnij drugą połowę bufora
	for (uint16_t n=0; n<nRozmiar; n++)
	{
		//sBuforAudioWy[n+ROZMIAR_BUFORA_AUDIO/2] = (*(int16_t*)nAdresKomunikatu * chGlosnosc) / SKALA_GLOSNOSCI_AUDIO;
		sBuforAudioWy[n+ROZMIAR_BUFORA_AUDIO/2] = sBuforTonuWario[sWskTonu];
		sWskTonu++;
		if (sWskTonu >= sRozmiarTonu)
		{
			sWskTonu = 0;
			if (chJestNowyTon)	//jeżeli pojawił się nowy ton, to przepisz go synchronicznie do bufora tonu teraz kiedy jesteśmy w zerze na początku okresu
			{
				chJestNowyTon = 0;
				for (uint16_t x=0; x<sRozmiarNowegoTonu; x++)
					sBuforTonuWario[x] = sBuforNowegoTonuWario[x];

				sRozmiarTonu = sRozmiarNowegoTonu;
			}
		}
		nAdresKomunikatu += 2;
	}
	if (nRozmiarKomunikatu > 0)
		nRozmiarKomunikatu -= nRozmiar;

	if ((nRozmiarKomunikatu <= 0) && (chNumerTonu >= LICZBA_TONOW_WARIO))
		HAL_SAI_DMAStop(&hsai_BlockB2);
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera dane z mikrofonu
// Parametry: nic
// Zwraca: kod błędu
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



////////////////////////////////////////////////////////////////////////////////
// Funkcja generuje tablicę próbek tonu akustycznego, które będą miksowane z ewentualnymi komunikatami audio i wypuszczane na wzmacniacz
// Prędkość odtwarzania jest stała i wynosi 16kHz, wiec aby zmienić ton, należy zmieniać długość tablicy z jednym okresem sinusa
// Ton jest jednym okresem sinusa o podstawowej częstotliwości naniesionym na nośną o częstotliwości 3 krotnie wyjższej
// Parametry:
// 	chNrTonu - numer kolejnego tonu jaki jest generowany w zakresie 0..LICZBA_TONOW_WARIO
//	chGlosnosc - amplituda sygnału w zakresie 0..255
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void UstawTon(uint8_t chNrTonu, uint8_t chGlosnosc)
{
	uint16_t sWskBufora = 0;
	//ustaw zmienne globalne
	sRozmiarNowegoTonu = MIN_OKRES_TONU + SKOK_TONU * chNrTonu;
	chJestNowyTon = 1;

	for (uint16_t n=0; n<sRozmiarNowegoTonu; n++)
	{
		//Są dwa sinusy: częstotliwości podstawowej i trzeciej harmonicznej. Mają niezależnie ustawianą amplitudę
		//sBuforNowegoTonuWario[n] = (int16_t)((chGlosnosc * (AMPLITUDA_1HARM * sinf(2 * M_PI * n / sRozmiarNowegoTonu) + AMPLITUDA_3HARM * (sinf(6 * M_PI * n / sRozmiarNowegoTonu))))/ SKALA_GLOSNOSCI_TONU);
		sBuforNowegoTonuWario[n] = (int16_t)((chGlosnosc * (sAmpl1Harm * sinf(2 * M_PI * n / sRozmiarNowegoTonu) + sAmpl3Harm * (sinf(6 * M_PI * n / sRozmiarNowegoTonu))))/ SKALA_GLOSNOSCI_TONU);
	}

	if (chNumerTonu >= LICZBA_TONOW_WARIO)		//jeżeli wcześniej nie generował tonu to napełnij obie połowy bufora audio, bo nie wiemy od której zacznie się odtwarzanie
	{
		for (uint16_t n=0; n<ROZMIAR_BUFORA_AUDIO/2; n++)
		{
			sBuforAudioWy[n+ROZMIAR_BUFORA_AUDIO/2] = sBuforAudioWy[n] = sBuforNowegoTonuWario[sWskBufora];
			sWskBufora++;
			if (sWskBufora >= sRozmiarTonu)
				sWskBufora = 0;
		}
		//wypełnij też podstawowy bufor tonu
		for (uint16_t n=0; n<sRozmiarNowegoTonu; n++)
			sBuforTonuWario[n] = sBuforNowegoTonuWario[n];

	}
	chNumerTonu = chNrTonu;
	HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)sBuforAudioWy, (uint16_t)ROZMIAR_BUFORA_AUDIO);
}



////////////////////////////////////////////////////////////////////////////////
// Zatrzymuje generowanie tonu
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ZatrzymajTon(void)
{
	chNumerTonu = 255;
}
