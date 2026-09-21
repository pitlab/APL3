//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika prądu INA226
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <INA226.h>
#include "WymianaCM4.h"
#include "Fram.h"
#include "KonfigFram.h"


extern I2C_HandleTypeDef hi2c3;
static uint8_t cBuforINA226[4];
static uint8_t cDzielnikOperacjiINA226;
extern volatile unia_wymianyCM4_t uDaneCM4;
uint8_t cAdresINA226 = ADRES_I2C_INA226;

////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika prądu INA226
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObsługaNA226(void)
{
	uint8_t cBłąd = BLAD_OK;

	if ((uDaneCM4.dane.nZainicjowano & INIT_INA226) != INIT_INA226)
	{
		cBłąd = InicjujINA226();
		if (cBłąd)
			return cBłąd;
		else
			uDaneCM4.dane.nZainicjowano |= INIT_INA226;
	}

	cDzielnikOperacjiINA226 &= 0x01;
	switch (cDzielnikOperacjiINA226)
	{
	case 0:		cBłąd = ZmierzNapięcieINA226((float*)&uDaneCM4.dane.fNapiecieAku[0]);	break;
	case 1: 	cBłąd = ZmierzPrądINA226((float*)&uDaneCM4.dane.fPradAku[0]);	break;
	default:	cBłąd = BLAD_NIC_DO_ROBOTY;	break;
	}
	cDzielnikOperacjiINA226++;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika prądu INA219
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujINA226(void)
{
	uint8_t cBłąd;
	uint16_t sRejestr;

	//sprawdź obecność układu
	cBuforINA226[0] = R226_ID_PRODUCENTA;
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_INA226, cBuforINA226, 1, TIMEOUT_INA226);
	if (cBłąd)
		return cBłąd;

	cBłąd = HAL_I2C_Master_Receive(&hi2c3, ADRES_I2C_INA226, cBuforINA226, 2, TIMEOUT_INA226);
	if (cBłąd || (cBuforINA226[0] != INA226_ID_PRODUCENTA_H) || (cBuforINA226[1] != INA226_ID_PRODUCENTA_L))
		return BLAD_BRAK_CZUJNIKA;

	cBuforINA226[0] = R226_KONFIGURACJA;
	sRejestr = 	(0 << 15)|	//RST - reset
				(0 << 9)|	//AVG Averaging Mode: 0=1, 1=4, 2=16, 3=64, 4=128, 5=256, 6=512, 7=1024 próbek
				(0 << 6)|	//VBUSCT2..0 Bus Voltage Conversion Time: 0=140us, 1=204us, 2=332us, 3=588us, 4=1,1ms, 5=2,116ms, 6=4,156ms, 7=8,244ms
	 			(0 << 3)|	//VSHCT Shunt Voltage Conversion Time: 0=140us, 1=204us, 2=332us, 3=588us, 4=1,1ms, 5=2,116ms, 6=4,156ms, 7=8,244ms
				(7 << 0);	//MODE 2..0: 0=Power down, 1=shunt voltage triggered, 2=bus voltage triggered, 3=shunt and bus triggered, 4=ADC off, 5=shunt voltage continous, 6=bus viltage continous, 7 shunt and bus continous
	cBuforINA226[1] = (uint8_t)(sRejestr >> 8);
	cBuforINA226[2] = (uint8_t)(sRejestr & 0xFF);
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_INA226, cBuforINA226, 3, TIMEOUT_INA226);

	sRejestr = (uint16_t)WARTOSC_KALIB_INA226;
	cBuforINA226[0] = R226_KALIBRACJA;
	cBuforINA226[1] = (uint8_t)(sRejestr >> 8);
	cBuforINA226[2] = (uint8_t)(sRejestr & 0xFF);
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_INA226, cBuforINA226, 3, TIMEOUT_INA226);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj pomiar napiecia
// Parametry: *fNapiecie - wskaźnik na zwracaną wartość pomiaru
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZmierzNapięcieINA226(float *fNapiecie)
{
	uint8_t cBłąd;
	uint16_t sNapiecie;

	cBuforINA226[0] = R226_NAP_OBWODU;
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_INA226, cBuforINA226, 1, TIMEOUT_INA226);
	cBłąd = HAL_I2C_Master_Receive(&hi2c3, ADRES_I2C_INA226, cBuforINA226, 2, TIMEOUT_INA226);
	sNapiecie = (uint16_t)cBuforINA226[0] * 0x100 + cBuforINA226[1];
	*fNapiecie = (float)sNapiecie  * INA226_LSB_NAPIECIA;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj pomiar prądu
// Parametry: *fPrad - wskaźnik na zwracaną wartość pomiaru
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZmierzPrądINA226(float *fPrad)
{
	uint8_t cBłąd;
	int16_t sPrad;

	cBuforINA226[0] = R226_PRAD;
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_INA226, cBuforINA226, 1, TIMEOUT_INA226);
	cBłąd = HAL_I2C_Master_Receive(&hi2c3, ADRES_I2C_INA226, cBuforINA226, 2, TIMEOUT_INA226);
	sPrad = (uint16_t)cBuforINA226[0] * 0x100 + cBuforINA226[1];
	*fPrad = (float)sPrad * INA226_LSB_PRADU;
	return cBłąd;
}

