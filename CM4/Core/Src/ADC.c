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
float fNapiecieCzujnikaWewn[3];		//VRef, Temp, VBat
uint8_t chIndeksPomiaruADC;


////////////////////////////////////////////////////////////////////////////////
// Inicjuje pracę przetworników ADC2 i ADC3
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujADC(void)
{
	uint8_t chBłąd = BLAD_OK;

	chBłąd  = HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	//chBłąd |= HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	chBłąd |= HAL_ADC_Start(&hadc2);	//wystartuj raz continous mode
	//chBłąd |= HAL_ADC_Start(&hadc3);

	//LL_ADC_DisableDeepPowerDown(&hadc2);
	//LL_ADC_DisableDeepPowerDown(&hadc3);

	//HAL_ADCEx_DisableVoltageRegulator(&hadc2);

	//HAL_Delay(1);
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
	sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;

	chIndeksPomiaruADC = chKanal;	//ustaw w zmiennej numer kanału aby w przerwaniu wiedziało gdzie przypisać wynik pomiaru

	//zmierz napięcia na zewnętrznych wejściach ADC
	if (chKanal < 8)
	{
		sConfig.Channel = ADC_CHANNEL_6;
		HAL_ADC_ConfigChannel(&hadc3, &sConfig);
		//chBłąd |= HAL_ADC_Start_IT(&hadc2);	//continous mode
		chBłąd |= HAL_ADC_Start_IT(&hadc3);
	}
	else	//przetwornika ADC3 może mierzyć czujniki wewnętrzne: VRef, Temp, VBat. Mierz je na kanałach 8..10
	{
		switch (chKanal)
		{
		case 8:		sConfig.Channel = ADC_CHANNEL_VREFINT;		break;
		case 9: 	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;	break;
		case 10:	sConfig.Channel = ADC_CHANNEL_VBAT;			break;
		default: return chBłąd;
		}
		HAL_ADC_ConfigChannel(&hadc3, &sConfig);
		chBłąd |= HAL_ADC_Start_IT(&hadc3);
	}
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
			fNapiecieCzujnikaZewn[chIndeksPomiaruADC - 4] = fNapiecie;
	}


	if (hadc->Instance == ADC3)
	{
		uint32_t nOdczytADC = HAL_ADC_GetValue(&hadc3);
		float fNapiecie = nOdczytADC * VREF / 0x10000;
		uint16_t TS_CAL1 = *TEMPSENSOR_CAL1_ADDR;	//Internal temperature sensor, address of parameter TS_CAL1: On STM32H7, temperature sensor ADC raw data acquired at temperature  30 DegC (tolerance: +-5 DegC), Vref+ = 3.3 V (tolerance: +-10 mV).
		uint16_t TS_CAL2 = *TEMPSENSOR_CAL2_ADDR;

		switch (chIndeksPomiaruADC)
		{
		case 0:	//Uwe1
		case 1:	fNapieciaZasilania[chIndeksPomiaruADC] = fNapiecie * DZIELNIK_UWE_ZASIL;	break;	//Uwe2
		case 2:	fNapieciaZasilania[chIndeksPomiaruADC] = fNapiecie * DZIELNIK_USERWO;		break;	//Userwo
		case 3:	fNapieciaZasilania[chIndeksPomiaruADC] = fNapiecie * DZIELNIK_UWE_ZASIL;	break;	//UUSB
		case 4:	fNapieciaZasilania[chIndeksPomiaruADC] = fNapiecie * DZIELNIK_UCZUJNIK;		break;	//Uczujn1
		case 5:	fNapieciaZasilania[chIndeksPomiaruADC] = fNapiecie * DZIELNIK_ICZUJNIK;		break;	//Iczujn1
		case 6:	fNapieciaZasilania[chIndeksPomiaruADC] = fNapiecie * DZIELNIK_UCZUJNIK;		break;	//Uczujn2
		case 7:	fNapieciaZasilania[chIndeksPomiaruADC] = fNapiecie * DZIELNIK_ICZUJNIK;		break;	//Iczujn2
		case 8:	fNapiecieCzujnikaWewn[chIndeksPomiaruADC - 8] = fNapiecie * DZIELNIK_VBAT;	break;	//Vbat/4
		case 9:	fNapiecieCzujnikaWewn[chIndeksPomiaruADC - 8] = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (TS_CAL2 - TS_CAL1)  * (nOdczytADC - TS_CAL1) + TEMPSENSOR_CAL1_TEMP;	//Vsense - temperatura
		case 10: fNapiecieCzujnikaWewn[chIndeksPomiaruADC - 8] = fNapiecie;	break;	//Vrefin
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
