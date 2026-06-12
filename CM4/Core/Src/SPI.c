//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Funkcje dotyczące SPI
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "SPI.h"
#include "main.h"

static uint8_t chBufSPIWy[35], chBufSPIWe[5];	//bufor transmisji SPI układów
extern SPI_HandleTypeDef hspi2;


////////////////////////////////////////////////////////////////////////////////
// Odczytuje 8 bitów bez znaku
// Parametry: chAdres - adres wewnętrznego rejestru
// Zwraca: odczytana wartość
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajSPIu8(uint8_t chAdres)
{
	chBufSPIWy[0] = chAdres | READ_SPI;
	chBufSPIWy[1] = 0;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chBufSPIWy, chBufSPIWe, 2, TOUT_SPI);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	return chBufSPIWe[1];
}



////////////////////////////////////////////////////////////////////////////////
// Zapisuje 8 bitów bez znaku
// Parametry: chAdres - adres wewnętrznego rejestru
// Zwraca: kod  błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
void ZapiszSPIu8(uint8_t *chDane, uint8_t chIlosc)
{
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_Transmit(&hspi2, chDane, chIlosc, TOUT_SPI);
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

	chBufSPIWy[0] = chAdres | READ_SPI;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chBufSPIWy, chBufSPIWe, 3, TOUT_SPI);
	//HAL_SPI_Transmit(&hspi2, chBufSPIWy, 32, SPI_DELAY);
	//HAL_SPI_Receive(&hspi2, &chBufSPIWe[1], 2, SPI_DELAY);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1

	sWartosc = ((uint16_t)chBufSPIWe[1] <<8) + chBufSPIWe[2];
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

	chBufSPIWy[0] = chAdres | READ_SPI;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chBufSPIWy, chBufSPIWe, 4, TOUT_SPI);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	nWartosc = ((uint32_t)chBufSPIWe[1] <<16) + ((uint32_t)chBufSPIWe[2] <<8) + chBufSPIWe[3];
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
	chBufSPIWy[0] = chAdres | READ_SPI;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chBufSPIWy, chBufSPIWe, 4, TOUT_SPI);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1

	nWartosc = (int32_t)chBufSPIWe[3];
	nWartosc <<= 8;
	nWartosc += chBufSPIWe[2];
	nWartosc <<= 8;
	nWartosc += chBufSPIWe[1];
	return nWartosc;
}
