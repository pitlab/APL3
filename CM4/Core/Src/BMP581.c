//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika ciśnienia BMP581 na magistrali SPI
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "BMP581.h"
#include "wymiana_CM4.h"
#include "petla_glowna.h"
#include "spi.h"
#include "petla_glowna.h"

// Dopuszczalna prędkość magistrali 1..12MHz

extern volatile unia_wymianyCM4_t uDaneCM4;
static uint8_t chProporcjaPomiarow;




////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z EEPROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujBMP581(void)
{
	uint8_t chDane;

	chDane = CzytajSPIu8(PBMP5_CHIP_ID);	//sprawdź obecność układu
	if (chDane != 0x50)
		return ERR_BRAK_BMP581;

    /*unsigned short sCnt = 100;
    do
    {
    sAC1 = (short)BMP085_Read16bit(0xAA);
    sAC2 = (short)BMP085_Read16bit(0xAC);
    sAC3 = (short)BMP085_Read16bit(0xAE);
    sAC4 = BMP085_Read16bit(0xB0);
    sAC5 = BMP085_Read16bit(0xB2);
    sAC6 = BMP085_Read16bit(0xB4);
    sB1 = (short)BMP085_Read16bit(0xB6);
    sB2 = (short)BMP085_Read16bit(0xB8);
    sMB = (short)BMP085_Read16bit(0xBA);
    sMC = (short)BMP085_Read16bit(0xBC);
    sMD = (short)BMP085_Read16bit(0xBE);
    sCnt--;
    }
    while ((!sAC1 | !sAC2 | !sAC3 | !sAC4 | !sAC5 | !sAC6 | !sB1 | !sB2 | !sMB | !sMC | !sMD) & sCnt);
    
    if (sCnt)
        return ERR_OK;
    else
        return ERR_TIMEOUT;*/

	uDaneCM4.dane.nZainicjowano |= INIT_BMP581;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika do wywołania w wyższej warstwie
// Wykonuje w pętli 1 pomiar temepratury i 7 pomiarów ciśnienia
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaBMP581(void)
{
	//uint32_t nKonwersja;
	uint8_t chErr;

	if ((uDaneCM4.dane.nZainicjowano & INIT_BMP581) != INIT_BMP581)	//jeżeli czujnik nie jest zainicjowany
	{
		chErr = InicjujBMP581();
		if (chErr)
			return chErr;
	}
	else
	{
		switch (chProporcjaPomiarow)
		{
		case 0:
			uDaneCM4.dane.fTemper[1] = (float)(CzytajSPIs24mp(PBMP5_TEMP_DATA_XLSB) / 65536);
			break;

		default:
			uDaneCM4.dane.fCisnie[1] = (float)(CzytajSPIs24mp(PBMP5_PRESS_DATA_XLSB) / 64);
			break;
		}
		chProporcjaPomiarow++;
		chProporcjaPomiarow &= 0x07;
	}
	return chErr;
}
