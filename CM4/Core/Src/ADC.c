//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi przetworników ADC
//
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "adc.h"

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

float fNapieciaZasilania[8];
float fNapiecieCzujnikaZewn[4];
float fNapiecieIdModuluWewn[4];
uint8_t chKanalADC;


////////////////////////////////////////////////////////////////////////////////
// Inicjuje pracę przetworników ADC2 i ADC3
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujADC(void)
{
	uint8_t chBłąd;

	HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	chBłąd  = HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	chBłąd |= HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	//chBłąd |= HAL_ADC_Start(&hadc2);
	//chBłąd |= HAL_ADC_Start(&hadc3);

	//LL_ADC_DisableDeepPowerDown(&hadc2);
	//LL_ADC_DisableDeepPowerDown(&hadc3);

	//HAL_ADCEx_DisableVoltageRegulator(&hadc2);

	HAL_Delay(1);
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
	//uint32_t nOdczytADC2 = 0, nOdczytADC3 = 0;
	uint8_t chBłąd = BLAD_OK;

	chKanalADC = chKanal;	//ustaw w zmiennej numer kanału aby w przerwaniu wiedziało gdzie przypisać wynik pomiaru
	chBłąd |= HAL_ADC_Start_IT(&hadc2);
	chBłąd |= HAL_ADC_Start_IT(&hadc3);

	//chBłąd = HAL_ADC_PollForConversion(&hadc2, 200);
	//chBłąd = HAL_ADC_PollForConversion(&hadc3, 200);

	//nOdczytADC2 = HAL_ADC_GetValue(&hadc2);
	//nOdczytADC3 = HAL_ADC_GetValue(&hadc3);

	//chBłąd = HAL_ADC_Stop(&hadc2);
	//chBłąd = HAL_ADC_Stop(&hadc3);
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
		fNapieciaZasilania[chKanalADC] = nOdczytADC * VREF / 0x10000;
	}

	if (hadc->Instance == ADC3)
	{
		uint32_t nOdczytADC = HAL_ADC_GetValue(&hadc2);
		switch (chKanalADC)
		{
		case 0:
		case 1:
		case 2:
		case 3:	fNapiecieIdModuluWewn[chKanalADC] = nOdczytADC * VREF / 0x10000;	break;
		case 4:
		case 5:
		case 6:
		case 7:	fNapiecieCzujnikaZewn[chKanalADC - 4] = nOdczytADC * VREF / 0x10000;	break;
		default: break;
		}
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
