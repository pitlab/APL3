//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi generowania i rejestracji dźwięków
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "audio.h"

//Słowniczek:
//próbka audio - pojedynczy plik wave zapisany we flash
//komunikat - grupa próbek składająca się w zdanie do wypowiedzenia
//ton - dźwięk generowany przez odtwarzanie przebiegu sinusiudy

static volatile int16_t sBuforAudioWy[ROZMIAR_BUFORA_AUDIO];	//bufor komunikatów wychodzących
static volatile int16_t sBuforAudioWe[ROZMIAR_BUFORA_AUDIO];	//bufor komunikatów przychodzących
static volatile int16_t sBuforTonuWario[ROZMIAR_BUFORA_TONU];	//bufor do przechowywania podstawowego tonu wario
static volatile int16_t sBuforNowegoTonuWario[ROZMIAR_BUFORA_TONU];	//bufor nowego tonu wario, który ma się zsynchronizować z podstawowym buforem w chwili przejścia przez zero aby uniknąć zakłóceń
static uint8_t chJestNowyTon;			//flaga informująca o tym że pojawił się nowy ton i trzeba go synchronicznie przepisać to podstawowego bufora tonu
static uint8_t chGlosnikJestZajęty;		//flaga informująca że zasób "głośnika" jest zajety odtwarzaniem próbki
static uint16_t sWskTonu;				//wskazuje na bieżącą próbkę w tablicy tonu
static uint16_t sRozmiarTonu;			//długość tablicy pełnego sinusa
static uint16_t sRozmiarNowegoTonu;		//długość tablicy nowego tonu do zsynchronizowania się z sRozmiarTonu w chwili przejscia przez zero
static uint32_t nPrzerywaczTonu;		//robi przerwy w dźwięku sygnalizacji wznoszenia
static uint16_t sAmpl1Harm = AMPLITUDA_1HARM;			//amplituda pierwszej harmonicznej sygnału
static uint16_t sAmpl3Harm = AMPLITUDA_3HARM;			//amplituda trzeciej harmonicznej sygnału
uint8_t chKolejkaKomunkatow[ROZM_KOLEJKI_KOMUNIKATOW];	//bufor kołowy do przechowywania próbek głosu do wymówienia
static uint8_t chWskNapKolKom, chWskOprKolKom;		//wskaźniki napełniania i opróżniania kolejki komunikatów audio
uint8_t chNumerTonu = 0xFF;		//ton domyślnie wyłączony
static uint32_t nAdresProbki;	//adres w pamieci flash skąd pobierany jest kolejny fragment próbki audio
int32_t nRozmiarProbki;			//pozostały do pobrania rozmiar próbki audio
uint8_t chGlosnosc;				//regulacja głośności odtwarzania komunikatów w zakresie 0..SKALA_GLOSNOSCI_AUDIO

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
	chWskNapKolKom = chWskOprKolKom = 0;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja procesowana w pętli głównej. Sprawdza czy jest coś do wymówienia i gdy przetwornik jest wolny to umiejszcza w nim kolejną próbkę
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaWymowyKomunikatu(void)
{
	uint8_t chErr;

	if ((chWskNapKolKom == chWskOprKolKom) || chGlosnikJestZajęty)
		return ERR_OK;		//nie ma nic do wymówienia

	//pobierz kolejną próbkę do wymówienia i zacznij wymowę
	chErr = OdtworzProbkeAudioZeSpisu(chKolejkaKomunkatow[chWskOprKolKom++]);
	if (chWskOprKolKom >= ROZM_KOLEJKI_KOMUNIKATOW)
		chWskOprKolKom = 0;

	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Odtwarza próbkę głosową obecną w spisie próbek
// Parametry: chNrKomunikatu - numer komunikatu okreslający pozycję w spisie
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t OdtworzProbkeAudioZeSpisu(uint8_t chNrProbki)
{
	return OdtworzProbkeAudio( *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrProbki * ROZM_WPISU_AUDIO + 0), *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrProbki * ROZM_WPISU_AUDIO + 4) / 2);
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
	extern volatile uint8_t chCzasSwieceniaLED[LICZBA_LED];	//czas świecenia liczony w kwantach 0,1s jest zmniejszany w przerwaniu TIM17_IRQHandler

	if ((nAdres < ADR_POCZATKU_KOM_AUDIO) || (nAdres > ADR_KONCA_KOM_AUDIO))
	{
		chCzasSwieceniaLED[LED_CZER] = 20;	//włącz czerwoną na 2 sekundy
		if (nAdres > 0x081FFFFF)	//jeżeli we flash programu to tylko sygnalizuj
			return ERR_BRAK_KOM_AUDIO;
	}

	chGlosnikJestZajęty = 1;		//zajęcie zasobu "głośnika"
	nAdresProbki   = nAdres;	//przepisz do zmiennych globalnych
	nRozmiarProbki = nRozmiar;


	//napełnij pierwszy cały bufor, reszta będzie dopełniana połówkami w callbackach od opróżnienia połowy i całego bufora
	for (uint32_t n=0; n<ROZMIAR_BUFORA_AUDIO; n++)
	{
		//sBuforAudioWy[n] =  *(int16_t*)nAdresProbki;
		sBuforAudioWy[n] = (*(int16_t*)nAdresProbki * chGlosnosc) / SKALA_GLOSNOSCI_AUDIO;
		nAdresProbki += 2;
	}
	nRozmiarProbki -= ROZMIAR_BUFORA_AUDIO;

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
	int16_t sProbka;

	if (chNumerTonu < LICZBA_TONOW_WARIO)		//jeżeli generuje ton
		nRozmiar = ROZMIAR_BUFORA_AUDIO/2;		//to zawsze pracuj na pełnym buforze
	else
	{
		if (nRozmiarProbki > ROZMIAR_BUFORA_AUDIO/2)
			nRozmiar = ROZMIAR_BUFORA_AUDIO/2;
		else
			nRozmiar = nRozmiarProbki;
	}

	//napełnij pierwszą połowę bufora
	for (uint16_t n=0; n<nRozmiar; n++)
	{
		sProbka = 0;
		if (nRozmiarProbki > 0)					//czy jest komunikat
		{
			sProbka += (*(int16_t*)nAdresProbki * chGlosnosc) / SKALA_GLOSNOSCI_AUDIO;
			nAdresProbki += 2;
			nRozmiarProbki--;
		}

		if (chNumerTonu < LICZBA_TONOW_WARIO)		//czy jest ton
		{
			if (chNumerTonu < LICZBA_TONOW_WARIO/2)	//przerywanie dźwięku dla wyższych tonów
				nPrzerywaczTonu += LICZBA_TONOW_WARIO - chNumerTonu;
			else
				nPrzerywaczTonu = 0;

			if (nPrzerywaczTonu > PRZERWA_TONU_WZNOSZ)				//czy ma być przerwa
			{
				if (nPrzerywaczTonu > 2 * PRZERWA_TONU_WZNOSZ)		//warunek wyjścia z przerwy
					nPrzerywaczTonu = 0;
			}
			else
			{
				sProbka += sBuforTonuWario[sWskTonu];
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
			}
		}
		sBuforAudioWy[n] = sProbka;		//suma komunikatu i tonu do bufora
	}

	//gdy nie ma nic do roboty to wyłącz
	if (nRozmiarProbki <= 0)
	{
		chGlosnikJestZajęty = 0;		//zwolnienie zasobu
		if (chNumerTonu >= LICZBA_TONOW_WARIO)
			HAL_SAI_DMAStop(&hsai_BlockB2);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Callback opróżnienia całego bufora
// Parametry: *hsai - wskaźnik na strukturę opisującą sprzęt
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	uint32_t nRozmiar;
	int16_t sProbka;

	if (chNumerTonu < LICZBA_TONOW_WARIO)		//jeżeli generuje ton
		nRozmiar = ROZMIAR_BUFORA_AUDIO/2;		//to zawsze pracuj na pełnym buforze
	else
	{
		if (nRozmiarProbki > ROZMIAR_BUFORA_AUDIO/2)
			nRozmiar = ROZMIAR_BUFORA_AUDIO/2;
		else
			nRozmiar = nRozmiarProbki;
	}

	//napełnij drugą połowę bufora
	for (uint16_t n=0; n<nRozmiar; n++)
	{
		sProbka = 0;
		if (nRozmiarProbki > 0)					//czy jest komunikat
		{
			sProbka += (*(int16_t*)nAdresProbki * chGlosnosc) / SKALA_GLOSNOSCI_AUDIO;
			nAdresProbki += 2;
			nRozmiarProbki--;
		}

		if (chNumerTonu < LICZBA_TONOW_WARIO)		//czy jest ton
		{
			if (chNumerTonu < LICZBA_TONOW_WARIO/2)	//przerywanie dźwięku dla wyższych tonów
				nPrzerywaczTonu += LICZBA_TONOW_WARIO - chNumerTonu;
			else
				nPrzerywaczTonu = 0;

			if (nPrzerywaczTonu > PRZERWA_TONU_WZNOSZ)
			{
				if (nPrzerywaczTonu > 2 * PRZERWA_TONU_WZNOSZ)
					nPrzerywaczTonu = 0;
			}
			else
			{
				sProbka += sBuforTonuWario[sWskTonu];
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
			}
		}
		sBuforAudioWy[n+ROZMIAR_BUFORA_AUDIO/2] = sProbka;		//suma komunikatu i tonu do bufora
	}

	//gdy nie ma nic do roboty to wyłącz
	if (nRozmiarProbki <= 0)
	{
		chGlosnikJestZajęty = 0;		//zwolnienie zasobu
		if (chNumerTonu >= LICZBA_TONOW_WARIO)
			HAL_SAI_DMAStop(&hsai_BlockB2);
	}
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



////////////////////////////////////////////////////////////////////////////////
// Wymawia komunikat słowny dotyczący jednego z predefiniowanych parametrów
// Parametry: chTypKomunikatu - predefiniowany typ: 1=wysokość, 2=napięcie, 3=temperatura, 4=prędkość
// fWartosc - liczba do wymówienia
// chPrezyzja - okresla ile miejsc po przecinku należy wymówić. Obecnie 0 lub 1
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t PrzygotujKomunikat(uint8_t chTypKomunikatu, float fWartosc)
{
	uint8_t chErr;
	float fLiczba;
	uint8_t chCyfra;
	uint8_t chFormaGramatyczna = 0;

	if (fWartosc > 900000)
			return ERR_ZLE_DANE;	//nie obsługuję wymowy większych liczb

	//dodaj nagłówek komunikatu
	switch(chTypKomunikatu)
	{
	case KOMG_WYSOKOSC:		chErr = DodajProbkeDoKolejki(PRGA_WYSOKOSC);	break;
	case KOMG_NAPIECIE:		chErr = DodajProbkeDoKolejki(PRGA_NAPIECIE);	break;
	case KOMG_TEMPERATURA:	chErr = DodajProbkeDoKolejki(PRGA_TEMPERATURA);	break;
	case KOMG_PREDKOSC:		chErr = DodajProbkeDoKolejki(PRGA_PREDKOSC);	break;
	default:	break;
	}

	//dodaj znak jeżeli liczba ujemna
	if (fWartosc < 0.0)
	{
		fWartosc *= -1.0f;		//zamień na liczbę dodatnią
		DodajProbkeDoKolejki(PRGA_MINUS);
	}

	//dodaj kolejne cyfry składajace się na liczbę
	if (fWartosc >= 100000)		//setki tysięcy
	{
		fLiczba = floorf(fWartosc / 100000);
		chCyfra = (uint8_t)fLiczba;
		chErr = DodajProbkeDoKolejki(PRGA_100 + chCyfra - 1);
		fWartosc -= chCyfra * 100000;
		chFormaGramatyczna = 3;		//użyj trzeciej formy: tysięcy
	}

	if (fWartosc >= 20000)		//dziesiątki tysięcy >=20k
	{
		fLiczba = floorf(fWartosc / 10000);
		chCyfra = (uint8_t)fLiczba;
		chErr = DodajProbkeDoKolejki(PRGA_20 + chCyfra - 2);
		fWartosc -= chCyfra * 10000;
		chFormaGramatyczna = 3;		//użyj trzeciej formy: tysięcy
	}

	if (fWartosc >= 10000)		//kilkanaście tysięcy
	{
		fLiczba = floorf(fWartosc / 10000);
		chCyfra = (uint8_t)fLiczba;
		chErr = DodajProbkeDoKolejki(PRGA_10 + chCyfra - 1);
		fWartosc -= chCyfra * 10000;
		chFormaGramatyczna = 3;		//użyj trzeciej formy: tysięcy
	}

	if (fWartosc >= 1000)		//jednostki tysięcy
	{
		fLiczba = floorf(fWartosc / 1000);
		chCyfra = (uint8_t)fLiczba;
		chErr = DodajProbkeDoKolejki(PRGA_01 + chCyfra - 1);
		fWartosc -= chCyfra * 1000;
		if (chCyfra >= 5)
			chFormaGramatyczna = 3;		//użyj trzeciej formy: tysięcy
		else
		if (chCyfra > 1)
			chFormaGramatyczna = 2;		//użyj drugiej formy: tysiące
		else
			chFormaGramatyczna = 1;		//użyj pierwszej formy: tysiąc
	}

	if (chFormaGramatyczna)
	{
		chErr = DodajProbkeDoKolejki(PRGA_TYSIAC + chCyfra - 1);	//dodaj słowo tysiąc w odpowiedniej formie
		chFormaGramatyczna = 0;
	}

	if (fWartosc >= 100)		//setki
	{
		fLiczba = floorf(fWartosc / 100);
		chCyfra = (uint8_t)fLiczba;
		chErr = DodajProbkeDoKolejki(PRGA_100 + chCyfra - 1);
		fWartosc -= chCyfra * 100;
		chFormaGramatyczna = 4;		//jednostka w liczbie >=5: woltów, metrów
	}

	if (fWartosc >= 20)		//dziesiątki >=20
	{
		fLiczba = floorf(fWartosc / 10);
		chCyfra = (uint8_t)fLiczba;
		chErr = DodajProbkeDoKolejki(PRGA_20 + chCyfra - 2);
		fWartosc -= chCyfra * 10;
		chFormaGramatyczna = 4;		//jednostka w liczbie >=5: woltów, metrów
	}

	if (fWartosc >= 10)		//kilkanascie
	{
		chCyfra = (uint8_t)fWartosc;
		chErr = DodajProbkeDoKolejki(PRGA_10 + chCyfra - 10);
		fWartosc -= chCyfra;
		chFormaGramatyczna = 4;		//jednostka w liczbie >=5: woltów, metrów
	}

	if (fWartosc >= 1.0f)		//jednostki
	{
		chCyfra = (uint8_t)fWartosc;
		chErr = DodajProbkeDoKolejki(PRGA_01 + chCyfra - 1);
		fWartosc -= chCyfra;
		if (chCyfra >= 5)
			chFormaGramatyczna = 4;		//jednostka w liczbie >=5 woltów, metrów
		else
			if (chCyfra > 1)
				chFormaGramatyczna = 3;		//jednostka w liczbie >=5: wolty, metry
			else
				chFormaGramatyczna = 2;		//jednostka w liczbie >=5: wolt, metr
	}

	if (fWartosc >= 0.1f)		//dziesiąte części
	{
		uint8_t chFormaDziesiatych;

		chErr = DodajProbkeDoKolejki(PRGA_I);
		fLiczba = floorf(fWartosc * 10);
		chCyfra = (uint8_t)fLiczba;
		chErr = DodajProbkeDoKolejki(PRGA_01 + chCyfra - 1);
		if (chCyfra >= 5)
			chFormaDziesiatych = 2;		//jednostka w liczbie >=5: dziesiatych
		else
			if (chCyfra > 1)
				chFormaDziesiatych = 1;		//jednostka w liczbie >1: dziesiąte
			else
				chFormaDziesiatych = 0;		//jednostka w liczbie == 1: dziesiąta

		chErr = DodajProbkeDoKolejki(PRGA_DZIESIATA + chFormaDziesiatych);
		chFormaGramatyczna = 1;		//jednostka w liczbie <1: wolta, metra
	}

	//dodaj jednostkę
	switch(chTypKomunikatu)
	{
	case KOMG_WYSOKOSC:		chErr = DodajProbkeDoKolejki(PRGA_METRA + chFormaGramatyczna - 1);	break;
	case KOMG_NAPIECIE:		chErr = DodajProbkeDoKolejki(PRGA_WOLTA + chFormaGramatyczna - 1);	break;
	case KOMG_TEMPERATURA:	chErr = DodajProbkeDoKolejki(PRGA_STOPNIA + chFormaGramatyczna - 1);	break;
	case KOMG_PREDKOSC:		//chErr = DodajProbkeDoKolejki(PRGA_METRA);
			chErr += DodajProbkeDoKolejki(PRGA_NA_SEKUNDE + chFormaGramatyczna - 1);	break;
	default:	break;
	}


	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Dodaje kolejną próbkę do kolejki komunikatów
// Parametry: chTypKomunikatu - predefiniowany typ: 1=wysokość, 2=napięcie, 3=temperatura, 4=prędkość
// fWartosc - liczba do wymówienia
// chPrezyzja - okresla ile miejsc po przecinku należy wymówić. Obecnie 0 lub 1
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t DodajProbkeDoKolejki(uint8_t chNumerProbki)
{
	uint8_t chNapelnianie = chWskNapKolKom;

	//sprawdź czy jest miejsce w kolejce
	chNapelnianie++;
	if (chNapelnianie >= ROZM_KOLEJKI_KOMUNIKATOW)
		chNapelnianie = 0;
	if (chNapelnianie == chWskOprKolKom)
		return ERR_PELEN_BUF_KOM;

	chKolejkaKomunkatow[chWskNapKolKom++] = chNumerProbki;
	if (chWskNapKolKom >= ROZM_KOLEJKI_KOMUNIKATOW)
			chWskNapKolKom = 0;

	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Tworzy losowy komunikat do wymówienia
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void TestKomunikatow(void)
{
	extern RNG_HandleTypeDef hrng;
	uint32_t nRrandom32;
	uint8_t chTypKomunikatu;
	float fWartosc;

	HAL_RNG_GenerateRandomNumber(&hrng, &nRrandom32);
	chTypKomunikatu = KOMG_WYSOKOSC + (nRrandom32 & 0x03);

	HAL_RNG_GenerateRandomNumber(&hrng, &nRrandom32);
	fWartosc = (float)(nRrandom32 & 0x8FFFFF) / 10.0;

	PrzygotujKomunikat(chTypKomunikatu, fWartosc);
}
