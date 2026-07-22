//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi przetworników ADC i pomiarów analogowych
//
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "ADC.h"
#include "WymianaCM4.h"
#include "PetlaGlowna.h"
#include "fram.h"
#include "ModulyWew.h"


extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern unia_wymianyCM4_t uDaneCM4;
uint16_t sObiegowPetliCzekaniaNaADC[LICZBA_POMIAROW_ADC3];
float fNapiecieIdModuluWewn[4];
float fMnożnikCzujnWeAnalog[4] = {1.0f, 1.0f, 1.0f, 1.0f};		//mnożniki do obliczenia finalnej wartości czujników odłaczonych do wejsć analogoweych
float fSkładnikCzujnWeAnalog[4];	//składniki dodawane do iloczynu wyniki poniaru napiecia czujnika ADC i mnożnika ab uzyskać właściwy wynik
uint8_t chIndeksPomiaruADC;
uint16_t sTS_CAL1, sTS_CAL2;	//wspólczynniki kalibracji czujnika temperatury odczytywane w CM7 i przekazywane poleceniem
uint8_t chWykonanoPomiarADC;	//pole bitowe wykonania pomiarów bit0 = ADC2, bit1 = ADC3
ADC_ChannelConfTypeDef sConfigADC3 = {0};
uint16_t sDzielnikCzasuPomiarowWewn = LICZBA_POMIAROW_ADC3;	//odlicza czas do okresowego wykonania pomiarów wewnętrznych wymagających rekonfiguracji przetwornika




////////////////////////////////////////////////////////////////////////////////
// Inicjuje pracę przetworników ADC2 i ADC3
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujADC(void)
{
	uint8_t cBłąd = BLAD_OK;

	for (uint8_t n=0; n<ILOSC_ZEWN_WE_ANALOG; n++)
	{
		cBłąd |= CzytajFramFloatZWalidacja(FAG_MNOZNIK_CZUJ_ZEWN + 4*n, &fMnożnikCzujnWeAnalog[n], VMIN_MNOZNIK_WE_ADC, VMAX_MNOZNIK_WE_ADC, VDOM_MNOZNIK_WE_ADC);	//4*4F współczynnik mnożenia analogowego napęcia czujnika zewnętrznego
		cBłąd |= CzytajFramFloatZWalidacja(FAG_SKLADNIK_CZUJ_ZEWN + 4*n, &fSkładnikCzujnWeAnalog[n], VMIN_SKLADNIK_WE_ADC, VMAX_SKLADNIK_WE_ADC, VDOM_SKLADNIK_WE_ADC);	//4*4F współczynnik dodawany do analogowego napęcia czujnika zewnętrznego
	}

	//wyślij polecenie do CM7 odczytania współczynników kalibracyjnych temperatury, gdyż tylko CM7 ma do nich dostęp
	uDaneCM4.dane.cWykonajPolecenie = POL4_CZYTAJ_KALIBR_TEMP;

	cBłąd  = HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	cBłąd |= HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	//konfiguracjia ADC3 będzie zmieniana, więc wstępnie ustaw ją tutaj aby nie budować od nowa przy każdym pomiarze
	sConfigADC3.Channel = ADC_CHANNEL_6;
	sConfigADC3.Rank = ADC_REGULAR_RANK_1;
	sConfigADC3.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;	//pomiar co 28us, wystarczajaco szybko, nie trzeba czekać na pomiar
	sConfigADC3.SingleDiff = ADC_SINGLE_ENDED;
	sConfigADC3.OffsetNumber = ADC_OFFSET_NONE;
	sConfigADC3.Offset = 0;
	sConfigADC3.OffsetSignedSaturation = DISABLE;

	cBłąd |= HAL_ADC_Start_IT(&hadc2);
	cBłąd |= HAL_ADC_Start_IT(&hadc3);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj pomiar przetwornikami ADC2 i ADC3 na bieżącym kanale zewnętrznego multipleksera
// Dla zegara ADC = 7MHz i ustawionego BOOST = 1, maksymalny czas próbkowania jest wciąż wystarczająco szybki aby nie czekać na zakończenia pomiaru
// Pomiary ADC2 wykonuje na kanałach 0..7, pomiary na ADC3 na kanałach 0..9 gdyż na starszych pozycjach są VBat i TempSens
// Aby zoptymalizować powolną zmianę kanału, pomiary VBat i TempSens są wykonywane co DZIELNIK_CZASU_POMIAROW_WEWN
// Przełacz mutiplekser na kolejny kanał
// Parametry: chKanal - indeks slotu czasowego odpowiadajacy indeksowi kanału multipleksera
// Zwraca: kod błędu HAL
// Czas wykonania: 5..6ms - strasznie długo. Same 2x HAL_ADC_Start_IT zajmują ok 4600us
////////////////////////////////////////////////////////////////////////////////
uint8_t PomiarADC(uint8_t cKanal, uint8_t cBityPozwoleniaNaPomiar)
{
	uint8_t cBłąd = BLAD_OK;

	chIndeksPomiaruADC = cKanal;	//ustaw w zmiennej numer kanału aby w przerwaniu wiedziało gdzie przypisać wynik pomiaru

	if (sDzielnikCzasuPomiarowWewn < LICZBA_POMIAROW_ADC3 + 1)
	{
		//ustaw ADC3 na właściwy kanał, ale tylko wtedy kiedy trzeba, gdyż konfiguracja zajmuje duuużo czasu, ok. 3000us
		if ((cKanal == 0) || (cKanal >= LICZBA_POMIAROW_ADC2))
		{
			//przetwornik ADC3 może również mierzyć czujniki wewnętrzne: Temp, VBat na kanałach 8..9
			switch (cKanal)	//ustawienia ADC3 na czujniki wewnętrzne
			{
			case 8:		sConfigADC3.Channel = ADC_CHANNEL_VREFINT;		break;
			case 9: 	sConfigADC3.Channel = ADC_CHANNEL_TEMPSENSOR;	break;
			case 10:	sConfigADC3.Channel = ADC_CHANNEL_VBAT;			break;
			default: 	sConfigADC3.Channel = ADC_CHANNEL_6;		//ustawienie ADC3 na wyjscie multipleksera
			}

			//ADC3 powinien wyjść ustawiony na ADC_CHANNEL_6, więc jeżeli licznik kończy się w okolicy 8..9 to zwiększ go aby wyszedł z bezpiecznymi ustawieniami
			if ((cKanal == 8) && (sDzielnikCzasuPomiarowWewn < 3))
				sDzielnikCzasuPomiarowWewn += 3;

			HAL_ADC_ConfigChannel(&hadc3, &sConfigADC3);

			//wykonaj pomiary wewnętrzne
			if (cKanal >= LICZBA_POMIAROW_ADC2)
				cBłąd |= HAL_ADC_Start_IT(&hadc3);
		}
	}
	sDzielnikCzasuPomiarowWewn--;
	if (sDzielnikCzasuPomiarowWewn == 0)
		sDzielnikCzasuPomiarowWewn = DZIELNIK_CZASU_POMIAROW_WEWN;

	//zmierz napięcia na zewnętrznych wejściach ADC dla 8 pierwszych kanałów
	if ((cKanal < LICZBA_POMIAROW_ADC2) && (cBityPozwoleniaNaPomiar == (1 << cKanal)))
	{
		cBłąd |= HAL_ADC_Start_IT(&hadc2);
		cBłąd |= HAL_ADC_Start_IT(&hadc3);
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj pomiary przetwornikami ADC2 i ADC3 wętli głównej
// Startuje nowy kanał, przełacza dekoder modułów i odbiera wyniki z poprzedniego pomiaru
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t ObsługaDekoderaiADC(uint8_t cOdcinekCzasu, uint8_t cBityPozwoleniaNaPomiar)
{
	uint8_t cBłąd = BLAD_OK;

	//ustaw dekoder adresów i jednocześnie multiplekser analogowy na zadany kanał w zakresie 0..7
	if (cOdcinekCzasu < LICZBA_POMIAROW_ADC2)
		cBłąd |= UstawDekoderModulow(cOdcinekCzasu);

	chWykonanoPomiarADC = 0;
	cBłąd |= PomiarADC(cOdcinekCzasu, cBityPozwoleniaNaPomiar);

	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Callback błędu ADC
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{

}



////////////////////////////////////////////////////////////////////////////////
// Callback błędu ADC
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{

}



////////////////////////////////////////////////////////////////////////////////
// Callback końca konwersji ADC
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC2)
	{
		uint32_t nOdczytADC = HAL_ADC_GetValue(&hadc2);
		float fNapiecie = nOdczytADC * VREF / 0xFFFF;
		if (chIndeksPomiaruADC < 4)
			fNapiecieIdModuluWewn[chIndeksPomiaruADC] = fNapiecie;
		else
		{
			if (chIndeksPomiaruADC < 8)
				uDaneCM4.dane.fNapCzujZewn[chIndeksPomiaruADC - 4] = fNapiecie * fMnożnikCzujnWeAnalog[chIndeksPomiaruADC - 4] + fSkładnikCzujnWeAnalog[chIndeksPomiaruADC - 4];
		}
		chWykonanoPomiarADC |= WYKONANO_POMIAR_ADC2;
	}

	if (hadc->Instance == ADC3)
	{
		uint32_t nOdczytADC = HAL_ADC_GetValue(&hadc3);
		float fNapiecie = nOdczytADC * VREF / 0xFFFF;
		switch (chIndeksPomiaruADC)
		{
		case 0:	//Uwe1
		case 1:	uDaneCM4.dane.fNapiecieWej[chIndeksPomiaruADC] = fNapiecie * DZIELNIK_UWE_ZASIL;	break;	//Uwe2
		case 2:	uDaneCM4.dane.fNapiecieSerw = fNapiecie * DZIELNIK_USERWO;		break;	//Userwo
		case 3:	uDaneCM4.dane.fNapiecieUSB = fNapiecie * DZIELNIK_UWE_ZASIL;	break;	//UUSB
		case 4:	uDaneCM4.dane.fNapiecieAku[0] = fNapiecie * DZIELNIK_UCZUJNIK;	break;	//Uczujn1
		case 5:	uDaneCM4.dane.fPradAku[0] = fNapiecie * DZIELNIK_ICZUJNIK;		break;	//Iczujn1
		case 6:	uDaneCM4.dane.fNapiecieAku[1] = fNapiecie * DZIELNIK_UCZUJNIK;	break;	//Uczujn2
		case 7:	uDaneCM4.dane.fPradAku[1] = fNapiecie * DZIELNIK_ICZUJNIK;		break;	//Iczujn2

		case 8:	uDaneCM4.dane.fNapiecieBatRTC = fNapiecie * DZIELNIK_VBAT;	break;	//Vbat/4
		case 9:
			if (sTS_CAL1 && sTS_CAL2)	//jest dzielenie przez te zmienne, więc nie mogą być zerowe
			{
				//int32_t nSkorygowanaTemparatura = nOdczytADC * VREFINT_CAL_VREF / (VREF * 1000);
				int32_t nSkorygowanaTemparatura = nOdczytADC * (VREF * 1000) / VREFINT_CAL_VREF;
				uDaneCM4.dane.fTemperCPU = (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (sTS_CAL2 - sTS_CAL1)  * (nSkorygowanaTemparatura - sTS_CAL1) + TEMPSENSOR_CAL1_TEMP;	//Vsense - temperatura
			}
			else
				uDaneCM4.dane.cWykonajPolecenie = POL4_CZYTAJ_KALIBR_TEMP;
			break;
		default: break;
		}
		chWykonanoPomiarADC |= WYKONANO_POMIAR_ADC3;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Callback połowy konwersji ADC
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{

}
