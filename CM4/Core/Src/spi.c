//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Funkcje dotyczące SPI
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "spi.h"
#include "main.h"

static uint8_t chBufSPI[5];	//bufor transmisji SPI układów
extern SPI_HandleTypeDef hspi2;


////////////////////////////////////////////////////////////////////////////////
// Odczytuje 8 bitów bez znaku
// Parametry: chAdres - adres wewnętrznego rejestru
// Zwraca: odczytana wartość
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajSPIu8(unsigned char chAdres)
{
	chBufSPI[0] = chAdres | READ_SPI;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chBufSPI, chBufSPI, 2, 5);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	return chBufSPI[1];
}



////////////////////////////////////////////////////////////////////////////////
// Zapisuje 8 bitów bez znaku
// Parametry: chAdres - adres wewnętrznego rejestru
// Zwraca: kod  błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
void ZapiszSPIu8(unsigned char chAdres)
{
	chBufSPI[0] = chAdres;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_Transmit(&hspi2, chBufSPI, 1, 5);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje 16 bitów bez znaku, młodszy przodem
// Parametry: chAdres - adres wewnętrznego rejestru
// Zwraca: odczytana wartość
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint16_t CzytajSPIu16mp(uint8_t chAdres)
{
	uint16_t sWartosc;

	chBufSPI[0] = chAdres;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chBufSPI, chBufSPI, 3, 5);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	sWartosc = ((uint16_t)chBufSPI[1] <<8) + chBufSPI[2];
	return sWartosc;
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje 24 bity ze znakiem, starszy przodem
// Parametry:
// chAddress - adres najmłodszego rejestru
// Zwraca: odczytana wartość
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
int32_t CzytajSPIs24sp(uint8_t chAdres)
{
	int32_t nWartosc;

	chBufSPI[0] = chAdres;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chBufSPI, chBufSPI, 4, 5);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	nWartosc = ((uint32_t)chBufSPI[1] <<16) + ((uint32_t)chBufSPI[2] <<8) + chBufSPI[3];
	return nWartosc;
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje 24 bity ze znakiem, młodszy przodem
// Parametry:
// chAddress - adres najmłodszego rejestru
// Zwraca: odczytana wartość
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
int32_t CzytajSPIs24mp(uint8_t chAdres)
{
	int32_t nWartosc;
	chBufSPI[0] = chAdres;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chBufSPI, chBufSPI, 4, 5);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1

	nWartosc = (int32_t)chBufSPI[3];
	nWartosc <<= 8;
	nWartosc += chBufSPI[2];
	nWartosc <<= 8;
	nWartosc += chBufSPI[1];
	return nWartosc;
}
