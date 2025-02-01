//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika żyroskopów i akcelerometrów LSM6DSV na magistrali SPI
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "LSM6DSV.h"
#include "wymiana_CM4.h"
#include "petla_glowna.h"
#include "main.h"
#include "modul_IiP.h"
#include "spi.h"

extern volatile unia_wymianyCM4_t uDaneCM4;



////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z EEPROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLSM6DSV(void)
{
	uint8_t chDane;

	chDane = CzytajSPIu8(PLSC6DSV_SPI2_WHO_AM_I);		//sprawdź obecność układu
	if (chDane != 0x7F)
		return ERR_BRAK_LSM6DSV;

	uDaneCM4.dane.nZainicjowano |= INIT_LSM6DSV;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika do wywołania w wyższej warstwie
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaLSM6DSV(void)
{
	uint8_t chErr = ERR_OK;

	if ((uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV) != INIT_LSM6DSV)	//jeżeli czujnik nie jest zainicjowany
	{
		chErr = InicjujLSM6DSV();
		if (chErr)
			return chErr;
	}
	else
	{
		uDaneCM4.dane.fTemper[3] = CzytajSPIu16mp(PLSC6DSV_SPI2_OUT_TEMP_L);
		uDaneCM4.dane.fAkcel2[0] = CzytajSPIu16mp(PLSC6DSV_SPI2_OUTX_L_A_OIS);
		uDaneCM4.dane.fAkcel2[1] = CzytajSPIu16mp(PLSC6DSV_SPI2_OUTY_L_A_OIS);
		uDaneCM4.dane.fAkcel2[2] = CzytajSPIu16mp(PLSC6DSV_SPI2_OUTZ_L_A_OIS);
		uDaneCM4.dane.fZyros2[0] = CzytajSPIu16mp(PLSC6DSV_SPI2_OUTX_L_G_OIS);
		uDaneCM4.dane.fZyros2[1] = CzytajSPIu16mp(PLSC6DSV_SPI2_OUTY_L_G_OIS);
		uDaneCM4.dane.fZyros2[2] = CzytajSPIu16mp(PLSC6DSV_SPI2_OUTZ_L_G_OIS);
	}
	return chErr;
}
