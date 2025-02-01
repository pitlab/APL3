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
#include "main.h"

#include "petla_glowna.h"

// Dopuszczalna prędkość magistrali 1..12MHz

extern SPI_HandleTypeDef hspi2;
extern volatile unia_wymianyCM4_t uDaneCM4;
static uint8_t chRdWrBufBMP[4];
static uint8_t chProporcjaPomiarow;





////////////////////////////////////////////////////////////////////////////////
// Odczytuje 8 bitów z czujnika
// Parametry: 
// chAddress - adres wewnętrznego rejestru lub komórki EEPROM
// Zwraca: odczytana wartość
// Czas wykonania: 
////////////////////////////////////////////////////////////////////////////////
uint8_t BMP581_Read8bit(unsigned char chAdres)
{
	chRdWrBufBMP[0] = chAdres | PBMP5_READ;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chRdWrBufBMP, chRdWrBufBMP, 2, 5);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	return chRdWrBufBMP[1];
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje 24 bitów z czujnika z 3 kolejnych rejestrów
// Parametry:
// chAddress - adres najmłodszego rejestru
// Zwraca: odczytana wartość
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
int32_t BMP581_Read24bit(unsigned char chAdres)
{
	uint32_t nWynik;
	chRdWrBufBMP[0] = chAdres;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chRdWrBufBMP, chRdWrBufBMP, 4, 5);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1

	nWynik = (int32_t)chRdWrBufBMP[3];
	nWynik <<= 8;
	nWynik += chRdWrBufBMP[2];
	nWynik <<= 8;
	nWynik += chRdWrBufBMP[1];
	return nWynik;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z EEPROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujBMP581(void)
{
	uint8_t chDane;

	chDane = BMP581_Read8bit(PBMP5_CHIP_ID);		//sprawdź obecność układu
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
		//else
			//StartKonwersjiMS5611(PMS_CONV_D2_OSR1024);	//uruchom konwersję temperatury nie trwajacą dłużej niż obieg pętli, czymi max 2048
	}
	else
	{
		switch (chProporcjaPomiarow)
		{
		case 0:
			uDaneCM4.dane.fTemperatura[1] = (float)(BMP581_Read24bit(PBMP5_TEMP_DATA_XLSB) / 65536);
			break;

		default:
			uDaneCM4.dane.fWysokosc[1] = (float)(BMP581_Read24bit(PBMP5_PRESS_DATA_XLSB) / 64);
			break;
		}
		chProporcjaPomiarow++;
		chProporcjaPomiarow &= 0x07;
	}
	return chErr;
}
