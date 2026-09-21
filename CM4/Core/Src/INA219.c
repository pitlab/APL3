//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika prądu INA219
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <INA219.h>
#include "WymianaCM4.h"
#include "Fram.h"
#include "KonfigFram.h"


extern I2C_HandleTypeDef hi2c3;
static uint8_t cBuforINA219[4];
static uint8_t cDzielnikOperacjiINA219;
extern volatile unia_wymianyCM4_t uDaneCM4;


////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika prądu INA219
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObsługaNA219(void)
{
	uint8_t cBłąd = BLAD_OK;

	if ((uDaneCM4.dane.nZainicjowano & INIT_INA219) != INIT_INA219)
	{
		cBłąd = InicjujINA219();
		if (cBłąd)
			return cBłąd;
		else
			uDaneCM4.dane.nZainicjowano |= INIT_INA219;
	}

	cDzielnikOperacjiINA219 &= 0x01;
	switch (cDzielnikOperacjiINA219)
	{
	case 0:		cBłąd = ZmierzNapięcieINA219((float*)&uDaneCM4.dane.fNapiecieAku[0]);	break;
	case 1: 	cBłąd = ZmierzPrądINA219((float*)&uDaneCM4.dane.fPradAku[0]);	break;
	default:	cBłąd = BLAD_NIC_DO_ROBOTY;	break;
	}
	cDzielnikOperacjiINA219++;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika prądu INA219
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujINA219(void)
{
	uint8_t cBłąd;
	uint16_t sRejestr;

	cBuforINA219[0] = R219_KONFIGURACJA;
	cBuforINA219[1] = 	(0 << 7)|	//RST - reset
						(1 << 5)|	//BRNG Bus Voltage range: 0=16V FSR, 1=32V FSR
						(3 << 3)|	//PG1..0 PGA Gain & range: 0= /1 +-40mV, 1= /2 +-80mV, 2= /4 +-160mV, 3= /8 +-320mV
						(1 << 6);	//BADC Bus ADC Resolution Averaging 3..1: 001 = 12-bit
	cBuforINA219[2] = 	(1 << 7)|	//BADC Bus ADC Resolution Averaging 0: 1 = 12-bit
						(9 << 3)|	//SADC Shunt ADC Averaging 3..0: 9=2 sample, 10=4 sample, 11=8 sampli, ..., 15=128 sampli
						(7 << 0);	//MODE 2..0: 0=Power down, 1=shunt voltage triggered, 2=bus voltage triggered, 3=shunt and bus triggered, 4=ADC off, 5=shunt voltage continous, 6=bus viltage continous, 7 shunt and bus continous
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_INA219, cBuforINA219, 3, TIMEOUT_INA219);

	sRejestr = (uint16_t)WARTOSC_KALIB_INA219;
	cBuforINA219[0] = R219_KALIBRACJA;
	cBuforINA219[1] = (uint8_t)(sRejestr >> 8);
	cBuforINA219[2] = (uint8_t)(sRejestr & 0xFF);
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_INA219, cBuforINA219, 3, TIMEOUT_INA219);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj pomiar napiecia
// Parametry: *fNapiecie - wskaźnik na zwracaną wartość pomiaru
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZmierzNapięcieINA219(float *fNapiecie)
{
	uint8_t cBłąd;
	uint16_t sNapiecie;

	cBuforINA219[0] = R219_NAP_OBWODU;
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_INA219, cBuforINA219, 1, TIMEOUT_INA219);
	cBłąd = HAL_I2C_Master_Receive(&hi2c3, ADRES_I2C_INA219, cBuforINA219, 2, TIMEOUT_INA219);
	sNapiecie = (uint16_t)cBuforINA219[0] * 0x100 + cBuforINA219[1];
	*fNapiecie = (float)(sNapiecie >> INA219_PRZESUN_NAP) * INA219_LSB_NAPIECIA;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj pomiar prądu
// Parametry: *fPrad - wskaźnik na zwracaną wartość pomiaru
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZmierzPrądINA219(float *fPrad)
{
	uint8_t cBłąd;
	int16_t sPrad;

	cBuforINA219[0] = R219_PRAD;
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_INA219, cBuforINA219, 1, TIMEOUT_INA219);
	cBłąd = HAL_I2C_Master_Receive(&hi2c3, ADRES_I2C_INA219, cBuforINA219, 2, TIMEOUT_INA219);
	sPrad = (uint16_t)cBuforINA219[0] * 0x100 + cBuforINA219[1];
	*fPrad = (float)sPrad * INA219_LSB_PRADU;
	return cBłąd;
}

