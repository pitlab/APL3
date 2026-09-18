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
uint8_t cBuforINA219[6];
extern volatile unia_wymianyCM4_t uDaneCM4;


////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika prądu INA219
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObsługaNA219(void)
{
	uint8_t cBłąd;

	if ((uDaneCM4.dane.nZainicjowano & INIT_INA219) != INIT_INA219)
	{
		cBłąd = InicjujINA219();
		if (cBłąd == BLAD_OK)
			uDaneCM4.dane.nZainicjowano |= INIT_INA219;
	}

	cBłąd = ZmierzINA219(&uDaneCM4.dane.fPradAku[0], &uDaneCM4.dane.fNapiecieAku[0]);
	return cBłąd;
}


////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika prądu INA219
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujINA219(void)
{
	uint8_t cBłąd;

	//cBłąd |= CzytajFramFloatZWalidacja(FAH_MAGN3_SKLADNIK_X + 4*n, &fPrzesMagn3[n], VMIN_SKLADNIK_MAGN, VMAX_SKLADNIK_MAGN, VDOM_SKLADNIK_MAGN);

	cBuforINA219[0] = I219_KONFIGURACJA;
	cBuforINA219[1] = 	(0 << 7)|	//RST - reset
						(1 << 5)|	//BRNG Bus Voltage range: 0=16V FSR, 1=32V FSR
						(0 << 3)|	//PG1..0 PGA Gain & range: 0= /1 +-40mV, 1= /2 +-80mV, 2= /4 +-160mV, 3= /8 +-320mV
						(1 << 6);	//BADC Bus ADC Resolution Averaging 3..1: 001 = 12-bit
	cBuforINA219[2] = 	(1 << 7)|	//BADC Bus ADC Resolution Averaging 0: 1 = 12-bit
						(9 << 3)|	//SADC Shunt ADC Averaging 3..0: 9=2 sample, 10=4 sample, 11=8 sampli, ..., 15=128 sampli
						(1 << 0);	//MODE 2..0: 0=Power down, 1=shunt voltage triggered, 2=bus voltage triggered, 3=shunt and bus triggered, 4=ADC off, 5=shunt voltage continous, 6=bus viltage continous, 7 shunt and bys continous
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, I219_ADRES_I2C, cBuforINA219, 3, I219_TIMEOUT);
	return cBłąd;
}




////////////////////////////////////////////////////////////////////////////////
// Wykonaj pomiar prądu, napiecia i mocy
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ZmierzINA219(float *fPrad, float *fNapiecie)
{
	uint8_t cBłąd;
	int16_t sPrad, sNapiecie;

	cBuforINA219[0] = I219_NAP_OBWODU;
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, I219_ADRES_I2C, cBuforINA219, 1, I219_TIMEOUT);
	cBłąd = HAL_I2C_Master_Receive(&hi2c3, I219_ADRES_I2C, cBuforINA219, 4, I219_TIMEOUT);
	sNapiecie = (uint16_t)cBuforINA219[0] * 0x100 + cBuforINA219[1];
	*fNapiecie = (float)sNapiecie * 0.004;
	//*sMoc 		= (uint16_t)cBuforINA219[2] * 0x100 + cBuforINA219[3];


	cBuforINA219[0] = I219_PRAD;
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, I219_ADRES_I2C, cBuforINA219, 1, I219_TIMEOUT);
	cBłąd = HAL_I2C_Master_Receive(&hi2c3, I219_ADRES_I2C, cBuforINA219, 2, I219_TIMEOUT);
	sPrad = (uint16_t)cBuforINA219[0] * 0x100 + cBuforINA219[1];
	*fPrad = (float)sPrad * 10e-6;
	return cBłąd;
}

