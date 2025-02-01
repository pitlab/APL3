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


extern uint8_t chRdWrBufIIP[5];	//bufor transmisji SPI układów
extern SPI_HandleTypeDef hspi2;
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

	chDane = ICM42688_Read8bit(PICM42688_WHO_I_AM);		//sprawdź obecność układu
	if (chDane != 0xDB)
		return ERR_BRAK_ICM42688;

	uDaneCM4.dane.nZainicjowano |= INIT_ICM42688;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje 8 bitów z czujnika
// Parametry:
// chAddress - adres wewnętrznego rejestru lub komórki EEPROM
// Zwraca: odczytana wartość
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ICM42688_Read8bit(unsigned char chAdres)
{
	chRdWrBufIIP[0] = chAdres | ICM4_READ;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chRdWrBufIIP, chRdWrBufIIP, 2, 5);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	return chRdWrBufIIP[1];
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

	}
	return chErr;
}
