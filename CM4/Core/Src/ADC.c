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


extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern unia_wymianyCM4_t uDaneCM4;
extern uint32_t nCzasStartuADC;
float fNapiecieIdModuluWewn[4];
float fMnożnikCzujnWeAnalog[4] = {1.0f, 1.0f, 1.0f, 1.0f};		//mnożniki do obliczenia finalnej wartości czujników odłaczonych do wejsć analogoweych
float fSkładnikCzujnWeAnalog[4];	//składniki dodawane do iloczynu wyniki poniaru napiecia czujnika ADC i mnożnika ab uzyskać właściwy wynik
uint8_t chIndeksPomiaruADC;
uint16_t sTS_CAL1, sTS_CAL2;	//wspólczynniki kalibracji czujnika temperatury odczytywane w CM7 i przekazywane poleceniem
uint8_t chWykonanoPomiarADC;	//pole bitowe wykonania pomiarów bit0 = ADC2, bit1 = ADC3



////////////////////////////////////////////////////////////////////////////////
// Inicjuje pracę przetworników ADC2 i ADC3
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujADC(void)
{
	uint8_t cBłąd = BLAD_OK;

	//LL_ADC_DisableDeepPowerDown(ADC3);
	//LL_ADC_EnableInternalRegulator(ADC3);
	//HAL_Delay(1);

	for (uint8_t n=0; n<ILOSC_ZEWN_WE_ANALOG; n++)
	{
		cBłąd |= CzytajFramFloatZWalidacja(FAG_MNOZNIK_CZUJ_ZEWN + 4*n, &fMnożnikCzujnWeAnalog[n], VMIN_MNOZNIK_WE_ADC, VMAX_MNOZNIK_WE_ADC, VDOM_MNOZNIK_WE_ADC);	//4*4F współczynnik mnożenia analogowego napęcia czujnika zewnętrznego
		cBłąd |= CzytajFramFloatZWalidacja(FAG_SKLADNIK_CZUJ_ZEWN + 4*n, &fSkładnikCzujnWeAnalog[n], VMIN_SKLADNIK_WE_ADC, VMAX_SKLADNIK_WE_ADC, VDOM_SKLADNIK_WE_ADC);	//4*4F współczynnik dodawany do analogowego napęcia czujnika zewnętrznego
	}

	//wyślij polecenie odczytania współczynników kalibracyjnych temperatury
	uDaneCM4.dane.chWykonajPolecenie = POL4_CZYTAJ_KALIBR_TEMP;

	cBłąd  = HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	cBłąd |= HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	cBłąd |= HAL_ADC_Start_IT(&hadc2);
	cBłąd |= HAL_ADC_Start_IT(&hadc3);
	nCzasStartuADC = PobierzCzas();
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj pomiar przetwornikami ADC2 i ADC3 na bieżącym kanale zewnętrznego multipleksera
// Pomiary ADC2 wykonuje na kanałach 0..7, pomiary na ADC3 na kanałach 0..9 gdyż na starszych pozycjach są vbat i Tempsens
// Przełacz mutiplekser na kolejny kanał
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t PomiarADC(uint8_t chKanal)
{
	uint8_t cBłąd = BLAD_OK;
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;	//pomiar co 28us
	//sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;	//pomiar co 170us
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;

	chIndeksPomiaruADC = chKanal;	//ustaw w zmiennej numer kanału aby w przerwaniu wiedziało gdzie przypisać wynik pomiaru

	//zmierz napięcia na zewnętrznych wejściach ADC
	if (chKanal < 8)
	{
		cBłąd |= HAL_ADC_Start_IT(&hadc2);		//pomiar ADC2 tylko dla 8 pierwszych kanałów
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//serwo kanał 1
	}

	//przetwornik ADC3 może mierzyć czujniki wewnętrzne: Temp, VBat na kanałach 8..9
	switch (chKanal)	//ustawienia ADC3 na czujniki wewnętrzne
	{
	case 8:		sConfig.Channel = ADC_CHANNEL_VBAT;			break;
	case 9: 	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;	break;
	case 10: 	sConfig.Channel = ADC_CHANNEL_VREFINT;		break;
	default: sConfig.Channel = ADC_CHANNEL_6;		//ustawienie ADC3 na wyjscie multipleksera
	}
	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	cBłąd |= HAL_ADC_Start_IT(&hadc3);
	//HAL_GPIO_WritePin(GPIOI, GPIO_PIN_10, GPIO_PIN_SET);	//serwo kanał 7
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
			uDaneCM4.dane.fNapCzujZewn[chIndeksPomiaruADC - 4] = fNapiecie * fMnożnikCzujnWeAnalog[chIndeksPomiaruADC - 4] + fSkładnikCzujnWeAnalog[chIndeksPomiaruADC - 4];

		chWykonanoPomiarADC |= WYKONANO_POMIAR_ADC2;
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);	//serwo kanał 1
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
		case 9:	if (sTS_CAL1 && sTS_CAL2)	//jest dzielenie przez te zmienne, więc nie mogą być zerowe
			{
				//int32_t nSkorygowanaTemparatura = nOdczytADC * VREFINT_CAL_VREF / (VREF * 1000);
				int32_t nSkorygowanaTemparatura = nOdczytADC * (VREF * 1000) / VREFINT_CAL_VREF;
				uDaneCM4.dane.fTemperCPU = (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (sTS_CAL2 - sTS_CAL1)  * (nSkorygowanaTemparatura - sTS_CAL1) + TEMPSENSOR_CAL1_TEMP;	//Vsense - temperatura
			}
			break;
		default: break;
		}
		chWykonanoPomiarADC |= WYKONANO_POMIAR_ADC3;
		//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);	//serwo kanał 7
		//HAL_GPIO_WritePin(GPIOI, GPIO_PIN_10, GPIO_PIN_RESET);	//serwo kanał 7
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
