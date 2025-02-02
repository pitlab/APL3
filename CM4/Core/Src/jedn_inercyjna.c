//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obliczenia jednostki inercyjnej (IMU)
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "jedn_inercyjna.h"
#include "wymiana.h"

extern volatile unia_wymianyCM4_t uDaneCM4;


////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia jednostki inercyjnej
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObliczeniaJednostkiInercujnej(void)
{
	uint8_t chErr = ERR_OK;


	//oblicz kąt odchylenia w radianach z danych magnetometru
	uDaneCM4.dane.fKatIMU[2] = atan2f((float)uDaneCM4.dane.sMagne3[1], (float)uDaneCM4.dane.sMagne3[0]);
	return chErr;
}
