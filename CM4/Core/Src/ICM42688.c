//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika żyroskopów i akcelerometrów ICM-42688-V na magistrali SPI
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "ICM42688.h"
#include "wymiana_CM4.h"
#include "petla_glowna.h"
#include "main.h"
#include "petla_glowna.h"
#include "modul_IiP.h"
#include "spi.h"

extern volatile unia_wymianyCM4_t uDaneCM4;

////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z EEPROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujICM42688(void)
{
	uint8_t chDane;

	chDane = CzytajSPIu8(PICM4268_WHO_I_AM);		//sprawdź obecność układu
	if (chDane != 0xDB)
		return ERR_BRAK_ICM42688;

	uDaneCM4.dane.nZainicjowano |= INIT_ICM42688;
	return ERR_OK;
}




////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika do wywołania w wyższej warstwie
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaICM42688(void)
{
	uint8_t chErr = ERR_OK;

	if ((uDaneCM4.dane.nZainicjowano & INIT_ICM42688) != INIT_ICM42688)	//jeżeli czujnik nie jest zainicjowany
	{
		chErr = InicjujICM42688();
		if (chErr)
			return chErr;
	}
	else
	{
		uDaneCM4.dane.fTemper[2] = CzytajSPIu16mp(PICM4268_TEMP_DATA1);
		uDaneCM4.dane.fAkcel1[0] = CzytajSPIu16mp(PICM4268_ACCEL_DATA_X1);
		uDaneCM4.dane.fAkcel1[1] = CzytajSPIu16mp(PICM4268_ACCEL_DATA_Y1);
		uDaneCM4.dane.fAkcel1[2] = CzytajSPIu16mp(PICM4268_ACCEL_DATA_Z1);
		uDaneCM4.dane.fZyros1[0] = CzytajSPIu16mp(PICM4268_GYRO_DATA_X1);
		uDaneCM4.dane.fZyros1[1] = CzytajSPIu16mp(PICM4268_GYRO_DATA_Y1);
		uDaneCM4.dane.fZyros1[2] = CzytajSPIu16mp(PICM4268_GYRO_DATA_Z1);
	}
	return chErr;
}
