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



////////////////////////////////////////////////////////////////////////////////
// Inicjuje pracę przetworników ADC2 i ADC3
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujADC(void)
{
	uint8_t chErr;

	chErr  = HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	chErr |= HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	chErr |= HAL_ADC_Start(&hadc2);
	chErr |= HAL_ADC_Start(&hadc3);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj pomiar przetwornikami ADC2 i ADC3 na bieżącym kanale zewnętrznego multipleksera
// Przełacz mutiplekser na kolejny kanał
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t PomiarADC(uint8_t chKanal)
{
	uint32_t nOdczytADC2, nOdczytADC3;
	uint8_t chErr = BLAD_OK;

	nOdczytADC2 = HAL_ADC_GetValue(&hadc2);
	nOdczytADC3 = HAL_ADC_GetValue(&hadc3);

	switch (chKanal)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		fNapieciaZasilania[chKanal] = nOdczytADC2 * VREF / 0x10000;
		fNapiecieIdModuluWewn[chKanal] = nOdczytADC3 * VREF / 0x10000;
		break;

	case 4:
	case 5:
	case 6:
	case 7:
		fNapieciaZasilania[chKanal] = nOdczytADC2 * VREF / 0x10000;
		fNapiecieCzujnikaZewn[chKanal - 4] = nOdczytADC3 * VREF / 0x10000;
		break;
	default: chErr = ERR_ZLY_ADRES;	break;
	}
	return chErr;
}
