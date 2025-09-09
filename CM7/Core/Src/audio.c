//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi generowania i rejestracji dźwięków
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <RPi35B_480x320.h>
#include "audio.h"
#include "LCD.h"
#include <stdio.h>
#include "moduly_SPI.h"
#include "cmsis_os.h"
#include "rysuj.h"

//Słowniczek:
//próbka audio - pojedynczy plik wave zapisany we flash
//komunikat - grupa próbek składająca się w zdanie do wypowiedzenia
//ton - dźwięk generowany przez odtwarzanie przebiegu sinusiudy

//int16_t __attribute__ ((aligned (16))) __attribute__((section(".SekcjaDRAM"))) sBuforPapuga[ROZMIAR_BUFORA_PAPUGI];
int16_t __attribute__ ((aligned (16))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforPapuga[ROZMIAR_BUFORA_PAPUGI];
int16_t __attribute__ ((aligned (16))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforAudioWe[2][2*ROZMIAR_BUFORA_AUDIO_WE];

static volatile int16_t sBuforAudioWy[ROZMIAR_BUFORA_AUDIO];	//bufor komunikatów wychodzących
//int16_t sBuforAudioWe[2][2*ROZMIAR_BUFORA_AUDIO_WE];	//bufor komunikatów przychodzących
uint8_t chWskaznikBuforaAudio;
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
uint32_t nRozmiarProbki;			//pozostały do pobrania rozmiar próbki audio
uint8_t chGlosnosc;				//regulacja głośności odtwarzania komunikatów w zakresie 0..SKALA_GLOSNOSCI_AUDIO

extern SAI_HandleTypeDef hsai_BlockB2;
extern const uint8_t chAdres_expandera[LICZBA_EXP_SPI_ZEWN];
extern uint8_t chPort_exp_wysylany[];
extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
uint32_t nLicznikSAI_DMA;	//test


////////////////////////////////////////////////////////////////////////////////
// Wykonuje inicjalizację zasobów dotwarzania dźwięku. Uruchamiane przy starcie
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujAudio(void)
{
	uint8_t chErr = BLAD_OK;

	//domyslnie ustaw wejscia ShutDown tak aby aktywny był wzmacniacz
	//chPort_exp_wysylany[1] |= EXP13_AUDIO_IN_SD;	//AUDIO_IN_SD - włącznika ShutDown mikrofonu
	chPort_exp_wysylany[1] |= EXP14_AUDIO_OUT_SD;	//AUDIO_OUT_SD - włączniek ShutDown wzmacniacza audio
	chGlosnosc = 45;
	chWskNapKolKom = chWskOprKolKom = 0;

	/*chErr = OdtworzProbkeAudioZeSpisu(PRGA_GOTOWY_SLUZYC);	//komunikat powitalny, sprawdzajacy czy audio działa
	if (chErr == BLAD_OK)*/
		nZainicjowanoCM7 |= INIT_AUDIO;
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja procesowana w pętli głównej. Sprawdza czy jest coś do wymówienia i gdy przetwornik jest wolny to umiejszcza w nim kolejną próbkę
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaWymowyKomunikatu(void)
{
	uint8_t chErr;

	if ((chWskNapKolKom == chWskOprKolKom) || chGlosnikJestZajęty || ((nZainicjowanoCM7 & INIT_AUDIO) != INIT_AUDIO))
		return BLAD_OK;		//nie ma nic do wymówienia lub nie skonfigurowane

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
	if (chNrProbki >= PRGA_MAX_PROBEK)
		return ERR_BRAK_PROBKI_AUDIO;
	return OdtworzProbkeAudio( *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrProbki * ROZM_WPISU_AUDIO + 0), *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrProbki * ROZM_WPISU_AUDIO + 4) / 2);
}




////////////////////////////////////////////////////////////////////////////////
// Przepisuję próbkę głosową z Flash do DRAM i odtwarza ją
// Parametry: chNrKomunikatu - numer komunikatu okreslający pozycję w spisie
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PrzepiszProbkeDoDRAM(uint8_t chNrProbki)
{
	uint32_t *sAdres;
	uint32_t nRozmiar;
	if (chNrProbki >= PRGA_MAX_PROBEK)
		return ERR_BRAK_PROBKI_AUDIO;

	sAdres = (uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrProbki * ROZM_WPISU_AUDIO + 0);
	nRozmiar = *(uint32_t*)(ADR_SPISU_KOM_AUDIO + chNrProbki * ROZM_WPISU_AUDIO + 4) / 2;

	for (uint32_t n=0; n<nRozmiar; n++)
		sBuforPapuga[n] = (uint16_t)*(sAdres + n);
	return OdtworzProbkeAudio((uint32_t)sBuforPapuga, nRozmiar);
}



////////////////////////////////////////////////////////////////////////////////
// Odtwarza pojedynczy komunikat głosowy zapisany w adresowalnym obszarze pamięci poprzez DMA pracujące z buforem kołowym
// Parametry:
// 	nAdres - adres początku komunikatu
//	nRozmiar - rozmiar komunikatu w bajtach
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t OdtworzProbkeAudio(uint32_t nAdres, uint32_t nRozmiar)
{
	uint8_t chErr;
	extern volatile uint8_t chCzasSwieceniaLED[LICZBA_LED];	//czas świecenia liczony w kwantach 0,1s jest zmniejszany w przerwaniu TIM17_IRQHandler

	nLicznikSAI_DMA = 0;	//test
	if ((nAdres < ADR_POCZATKU_KOM_AUDIO) || (nAdres > ADR_KONCA_KOM_AUDIO))
	{
		chCzasSwieceniaLED[LED_CZER] += 20;	//włącz czerwoną na 2 sekundy
		if ((nAdres < POCZATEK_FLASH) || (nAdres > KONIEC_FLASH))	//jeżeli we flash programu to tylko sygnalizuj
			return ERR_BRAK_PROBKI_AUDIO;
	}

	chGlosnikJestZajęty = 1;		//zajęcie zasobu "głośnika"
	nAdresProbki   = nAdres;	//przepisz do zmiennych globalnych
	nRozmiarProbki = nRozmiar;
	if (nRozmiar > ROZMIAR_BUFORA_AUDIO)
		nRozmiar = ROZMIAR_BUFORA_AUDIO;

	//napełnij pierwszy cały bufor, reszta będzie dopełniana połówkami w callbackach od opróżnienia połowy i całego bufora
	for (uint32_t n=0; n<nRozmiar; n++)
	{
		//sBuforAudioWy[n] =  *(int16_t*)nAdresProbki;
		sBuforAudioWy[n] = (*(int16_t*)nAdresProbki * chGlosnosc) / SKALA_GLOSNOSCI_AUDIO;
		nAdresProbki += 2;
	}
	nRozmiarProbki -= nRozmiar;

	chErr = HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)sBuforAudioWy, (uint16_t)ROZMIAR_BUFORA_AUDIO);
	return chErr;
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
	for (uint8_t n=0; n<nRozmiar; n++)
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
	nLicznikSAI_DMA++;	//test
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
// Zmienia konfigurację systemu audio na odtwarzanie próbek
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujOdtwarzanieDzwieku(void)
{
	uint8_t chErr;

	if (chPort_exp_wysylany[1] & EXP13_AUDIO_IN_SD)		//jeżeli aktywny jest mikrofon
	{
		//Włącza wzmacniacz, wyłącza mikrofon
		chPort_exp_wysylany[1] |= EXP14_AUDIO_OUT_SD;	//AUDIO_OUT_SD - ShutDown wzmacniacza audio, aktywny niski
		chPort_exp_wysylany[1] &= ~EXP13_AUDIO_IN_SD;	//AUDIO_IN_SD  - ShutDown mikrofonu, aktywny niskii
		chErr = WyslijDaneExpandera(chAdres_expandera[1], chPort_exp_wysylany[1]);
	}

	hsai_BlockB2.Instance = SAI2_Block_B;
	hsai_BlockB2.Init.AudioMode = SAI_MODEMASTER_TX;
	hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockB2.Init.NoDivider = SAI_MCK_OVERSAMPLING_DISABLE;
	hsai_BlockB2.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
	hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockB2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
	hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB2.Init.MonoStereoMode = SAI_MONOMODE;
	hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockB2.Init.TriState = SAI_OUTPUT_RELEASED;
	chErr = HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2);
	return chErr;
}


////////////////////////////////////////////////////////////////////////////////
// Zmienia konfigurację systemu audio na rejestracje próbek
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujRejestracjeDzwieku(void)
{
	uint8_t chErr;

	if (chPort_exp_wysylany[1] & EXP14_AUDIO_OUT_SD)	//jeżeli aktywny jest wzmacniacz
	{
		//Włącza mikrofon wyłącza wzmacniacz
		chPort_exp_wysylany[1] |= EXP13_AUDIO_IN_SD;	//AUDIO_IN_SD - ShutDown mikrofonu, aktywny niski
		chPort_exp_wysylany[1] &= ~EXP14_AUDIO_OUT_SD;	//AUDIO_OUT_SD - ShutDown wzmacniacza audio, aktywny niski
		chErr = WyslijDaneExpandera(chAdres_expandera[1], chPort_exp_wysylany[1]);
	}
	osDelay(10);	//czas na stabilizcję napięcia na mikrofonie

	hsai_BlockB2.Instance = SAI2_Block_B;
	hsai_BlockB2.Init.AudioMode = SAI_MODEMASTER_RX;
	hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockB2.Init.NoDivider = SAI_MCK_OVERSAMPLING_DISABLE;
	hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_FULL;
	hsai_BlockB2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
	hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB2.Init.MonoStereoMode = SAI_MONOMODE;
	hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
	chErr = HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BITEXTENDED, 2);	//dane w 2 słowach
	//chErr = HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2);
	return chErr;
}



uint8_t NapelnijBuforDzwieku(int16_t *sBufor, uint16_t sRozmiar)
{
	uint8_t chErr;
	chErr = HAL_SAI_Receive(&hsai_BlockB2, (uint8_t*)sBufor, sRozmiar, 100+sRozmiar/16);
	return chErr;
}
//void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)




////////////////////////////////////////////////////////////////////////////////
// Funkcja generuje tablicę próbek tonu akustycznego, które będą miksowane z ewentualnymi komunikatami audio i wypuszczane na wzmacniacz
// Prędkość odtwarzania jest stała i wynosi 16kHz, wiec aby zmienić ton, należy zmieniać długość tablicy z jednym okresem sinusa
// Ton jest jednym okresem sinusa o podstawowej częstotliwości naniesionym na nośną o częstotliwości 3 krotnie wyższej
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

	if (fWartosc > 999999)
			return ERR_ZLE_DANE;	//nie obsługuję wymowy większych liczb

	//dodaj nagłówek komunikatu
	switch(chTypKomunikatu)
	{
	case KOMG_WYSOKOSC:		chErr = DodajProbkeDoKolejki(PRGA_WYSOKOSC);	break;
	case KOMG_NAPIECIE:		chErr = DodajProbkeDoKolejki(PRGA_NAPIECIE);	break;
	case KOMG_TEMPERATURA:	chErr = DodajProbkeDoKolejki(PRGA_TEMPERATURA);	break;
	case KOMG_PREDKOSC:		chErr = DodajProbkeDoKolejki(PRGA_PREDKOSC);	break;
	case KOMG_KIERUNEK:		chErr = DodajProbkeDoKolejki(PRGA_KIERUNEK);	break;
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
		fLiczba = floorf(fWartosc / 1000);
		chCyfra = (uint8_t)fLiczba;
		chErr = DodajProbkeDoKolejki(PRGA_10 + chCyfra - 10);
 		fWartosc -= chCyfra * 1000;
		chFormaGramatyczna = 3;		//użyj trzeciej formy: tysięcy
	}

	if (fWartosc >= 1000)		//jednostki tysięcy
	{
		fLiczba = floorf(fWartosc / 1000);
		chCyfra = (uint8_t)fLiczba;
		if ((chFormaGramatyczna) || (chCyfra > 1))	//nie dodawaj "jeden" przed "tysiąc", ale tylko gdy nie ma starszych cyfr
			chErr = DodajProbkeDoKolejki(PRGA_01 + chCyfra - 1);
		fWartosc -= chCyfra * 1000;
		if (chCyfra >= 5)
			chFormaGramatyczna = 3;		//użyj trzeciej formy: tysięcy
		else
		if (chCyfra > 1)
			chFormaGramatyczna = 2;		//użyj drugiej formy: tysiące
		else
			if (!chFormaGramatyczna)		//jezeli są starsze cyfry to forma odnosi się do nich, jeżeli nie to jest specyficzna dla jednego tysiaca
				chFormaGramatyczna = 1;		//użyj pierwszej formy: tysiąc
	}

	if (chFormaGramatyczna)		//jeżeli chFormaGramatyczna jest niezerowa to wystąpiły tysiace i trzeba je wymówić w odpowiedniej formie
	{
		chErr = DodajProbkeDoKolejki(PRGA_TYSIAC + chFormaGramatyczna - 1);	//dodaj słowo tysiąc w odpowiedniej formie
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
				if (!chFormaGramatyczna)	//jeżeli nie było starszych cyfr dla których okreslono formę to użyj formy dla jedynki
					chFormaGramatyczna = 2;		//jednostka w liczbie >=5: wolt, metr
	}

	if (fWartosc >= 0.1f)		//dziesiąte części
	{
		uint8_t chFormaDziesiatych;

		chErr = DodajProbkeDoKolejki(PRGA_I);
		fLiczba = roundf(fWartosc * 10);	//ponieważ to ostatnia znacząca cyfra więc potrzebne zaokrąglenie a nie obcięcie
		chCyfra = (uint8_t)fLiczba;
		if (chCyfra > 2)
			chErr = DodajProbkeDoKolejki(PRGA_01 + chCyfra - 1);
		else
			chErr = DodajProbkeDoKolejki(PRGA_JEDNA + chCyfra - 1);	//specyficzna wymowa dla jedna i dwie dziesiate

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
	case KOMG_KIERUNEK:
	case KOMG_TEMPERATURA:	chErr = DodajProbkeDoKolejki(PRGA_STOPNIA + chFormaGramatyczna - 1);	break;
	case KOMG_PREDKOSC:		chErr = DodajProbkeDoKolejki(PRGA_METRA + chFormaGramatyczna - 1);
							chErr += DodajProbkeDoKolejki(PRGA_NA_SEKUNDE);	break;
	default:	break;
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Dodaje kolejną próbkę do dużej kolejki komunikatów o rozmiarze ROZM_KOLEJKI_KOMUNIKATOW
// Parametry: chNumerProbki - identyfikator próbki w spisie treści
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

	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Dodaje kolejną próbkę do małej kolejki komunikatów o definiowanym
// Chodzi o to aby generujac dużą liczbę komunikatów do wymówienia nie zapychać kolejki na długi czas. Nadmiarowe komunikaty nie będą wypowiedziane
// Parametry: chNumerProbki - identyfikator próbki w spisie treści
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t DodajProbkeDoMalejKolejki(uint8_t chNumerProbki, uint8_t chRozmiarKolejki)
{
	uint8_t chNapelnianie = chWskNapKolKom;

	//sprawdź czy jest miejsce w kolejce
	chNapelnianie++;
	if (chNapelnianie >= chRozmiarKolejki)
		chNapelnianie = 0;
	if (chNapelnianie == chWskOprKolKom)
		return ERR_PELEN_BUF_KOM;

	chKolejkaKomunkatow[chWskNapKolKom++] = chNumerProbki;
	if (chWskNapKolKom >= chRozmiarKolejki)
			chWskNapKolKom = 0;
	for (uint8_t n=chRozmiarKolejki; n<ROZM_KOLEJKI_KOMUNIKATOW; n++)	//pozostałą część kolejki wypełnij komunikatami nie do wymówienia
		chKolejkaKomunkatow[n] = PRGA_PUSTE_MIEJSCE;

	return BLAD_OK;
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
	uint8_t chDlugosc;
	float fWartosc;
	extern uint8_t chRysujRaz;
	extern char chNapis[60];
	char chKomunikat[13];
	char chJednostka[4];
	uint16_t n;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Generator komunikatow");

		setColor(GRAY80);
		sprintf(chNapis, "Rozmiar kolejki komunikat%cw:", ó);
		RysujNapis(chNapis, 10, 60);
		sprintf(chNapis, "Wymawiam:");
		RysujNapis(chNapis, 10, 40);

		setColor(GRAY60);
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		RysujNapis(chNapis, CENTER, 300);
	}

	setColor(WHITE);
	chDlugosc = DlugoscKolejkiKomunikatow();
	sprintf(chNapis, "%d/%d  ",chDlugosc, ROZM_KOLEJKI_KOMUNIKATOW);
	RysujNapis(chNapis, 10+29*FONT_SL, 60);
	if (!chDlugosc)	//gdy kolejka komunikatów się opróżni
	{

		do		//losuj typ komunikatu -1, bo indeksy zaczynają się od 1 a nie od zera
		{
			HAL_RNG_GenerateRandomNumber(&hrng, &nRrandom32);
			chTypKomunikatu = KOMG_WYSOKOSC + (nRrandom32 & 0x07);
		} while (chTypKomunikatu > KOMG_MAX_KOMUNIKAT - 1);
		chTypKomunikatu++;	//konwersja z 0..KOMG_MAX_KOMUNIKAT-1 na 1..KOMG_MAX_KOMUNIKAT

		switch (chTypKomunikatu)
		{
		case KOMG_WYSOKOSC:		sprintf(chKomunikat, "Wysoko%c%c", ś, ć);		sprintf(chJednostka, "m");		break;
		case KOMG_NAPIECIE:		sprintf(chKomunikat, "Napi%ccie", ę);			sprintf(chJednostka, "V");		break;
		case KOMG_TEMPERATURA:	sprintf(chKomunikat, "Temperatura");			sprintf(chJednostka, "%cC", ZNAK_STOPIEN);	break;
		case KOMG_PREDKOSC:		sprintf(chKomunikat, "Pr%cdko%c%c", ę, ś, ć);	sprintf(chJednostka, "m/s");	break;
		case KOMG_KIERUNEK:		sprintf(chKomunikat, "Kierunek");				sprintf(chJednostka, "%c", ZNAK_STOPIEN);	break;
		}

		HAL_RNG_GenerateRandomNumber(&hrng, &nRrandom32);
		fWartosc = (float)(nRrandom32 & 0x7FFFFF) / 10.0;
		if (!(nRrandom32 & 0x1800000))	//te 2 bity będzie wylosowanym znakiem z prawdopodobieństwem 1/4 dla minusa
			fWartosc *= -1.0;

		chDlugosc = sprintf(chNapis, "%s %.1f%s", chKomunikat, fWartosc, chJednostka);
		for (n=chDlugosc; n<50; n++)
			chNapis[n] = ' ';	//dopełnij spacjami aby zamazać wczesniejsze napisy
		chNapis[n] = 0;
		RysujNapis(chNapis, 10+10*FONT_SL, 40);

		PrzygotujKomunikat(chTypKomunikatu, fWartosc);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Liczy długość kolejki komunikatów
// Parametry: nic
// Zwraca: długość kolejki
////////////////////////////////////////////////////////////////////////////////
uint8_t DlugoscKolejkiKomunikatow(void)
{
	if (chWskNapKolKom >= chWskOprKolKom)
		return chWskNapKolKom - chWskOprKolKom;
	else
		return ROZM_KOLEJKI_KOMUNIKATOW + chWskNapKolKom - chWskOprKolKom;
}



////////////////////////////////////////////////////////////////////////////////
// normalizuje dźwięk w buforze do ustalonej głośności
// Parametry: s*Bufor - wskaźnik na bufor z danymi
//   nRozmiar - liczba próbek w buforze
//   chGlosnosc - docelowa głosność w zakresie 0..100
// Zwraca: długość kolejki
////////////////////////////////////////////////////////////////////////////////
void NormalizujDzwiek(int16_t* sBufor, uint32_t nRozmiar, uint8_t chGlosnosc)
{
	int16_t sMin, sMax;
	int16_t sSrednia, sRob;
	float fWzmocnienie;
	int64_t lSuma = 0;

	sMin = 0x4FFF;
	sMax = -0x4FFF;
	for (uint32_t n=0; n<nRozmiar; n++)
	{
		if (*(sBufor+n) > sMax)
			sMax = *(sBufor+n);
		if (*(sBufor+n) < sMin)
			sMin = *(sBufor+n);
		lSuma += *(sBufor+n);
	}
	sSrednia = lSuma / nRozmiar;

	uint16_t sAmplituda = sMax - sMin;
	fWzmocnienie = 32768.0f * chGlosnosc / (100 * sAmplituda);

	for (uint32_t n=0; n<nRozmiar; n++)
	{
		sRob = *(sBufor+n) - sSrednia;	//ustaw sygnał w środku zakresu
		sRob *= fWzmocnienie;
		*(sBufor+n) = sRob;
	}
}
