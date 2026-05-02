//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi przetworników ADC i pomiarów analogowych
//
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "adc.h"
#include "wymiana_CM4.h"
#include "petla_glowna.h"

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern unia_wymianyCM4_t uDaneCM4;
extern uint32_t nCzasStartuADC;
float fNapiecieIdModuluWewn[4];
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
	uint8_t chBłąd = BLAD_OK;

	//wyślij polecenie odczytania współczynników kalibracyjnych temperatury
	uDaneCM4.dane.chWykonajPolecenie = POL4_CZYTAJ_KALIBR_TEMP;

	chBłąd  = HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	chBłąd |= HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	chBłąd |= HAL_ADC_Start_IT(&hadc2);
	chBłąd |= HAL_ADC_Start_IT(&hadc3);
	nCzasStartuADC = PobierzCzas();
	return chBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj pomiar przetwornikami ADC2 i ADC3 na bieżącym kanale zewnętrznego multipleksera
// Przełacz mutiplekser na kolejny kanał
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t PomiarADC(uint8_t chKanal)
{
	uint8_t chBłąd = BLAD_OK;
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Rank = ADC_REGULAR_RANK_1;
	//sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;	//pomiar co 22,8us
	//sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;	//pomiar co 28us
	sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;	//pomiar co 170us
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;

	chIndeksPomiaruADC = chKanal;	//ustaw w zmiennej numer kanału aby w przerwaniu wiedziało gdzie przypisać wynik pomiaru
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);	//serwo kanał 1

	//zmierz napięcia na zewnętrznych wejściach ADC
	if (chKanal < 8)
	{
		sConfig.Channel = ADC_CHANNEL_6;
		HAL_ADC_ConfigChannel(&hadc3, &sConfig);
		chBłąd |= HAL_ADC_Start_IT(&hadc2);
	}
	else	//przetwornika ADC3 może mierzyć czujniki wewnętrzne: Temp, VBat. Mierz je na kanałach 8..9
	{
		switch (chKanal)
		{
		case 8:		sConfig.Channel = ADC_CHANNEL_VBAT;		break;
		case 9: 	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;	break;
		default: return chBłąd;
		}
		HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	}
	chBłąd |= HAL_ADC_Start_IT(&hadc3);
	return chBłąd;
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
		float fNapiecie = nOdczytADC * VREF / 0x10000;
		if (chIndeksPomiaruADC < 4)
			fNapiecieIdModuluWewn[chIndeksPomiaruADC] = fNapiecie;
		else
			uDaneCM4.dane.fNapCzujZewn[chIndeksPomiaruADC - 4] = fNapiecie;
		chWykonanoPomiarADC |= WYKONANO_POMIAR_ADC2;
	}

	if (hadc->Instance == ADC3)
	{
		uint32_t nOdczytADC = HAL_ADC_GetValue(&hadc3);
		float fNapiecie = nOdczytADC * VREF / 0x10000;
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
			uDaneCM4.dane.fTemperCPU = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (sTS_CAL2 - sTS_CAL1)  * (nOdczytADC - sTS_CAL1) + TEMPSENSOR_CAL1_TEMP;	//Vsense - temperatura
			break;
		default: break;
		}
		chWykonanoPomiarADC |= WYKONANO_POMIAR_ADC3;
		HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);	//serwo kanał 7
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
