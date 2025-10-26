//////////////////////////////////////////////////////////////////////////////
//
// Funkcje wyświetlania bezpośrednio do pamięci dla wyświetlacza
//  przyjmującego kolor w formacie RGB666 lub RGB888
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "LCD_mem.h"
#include "display.h"
#include <math.h>
#include <stdio.h>
#include <string.h>


uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforLCD[DISP_X_SIZE * DISP_Y_SIZE * 3];	//pamięć obrazu wyświetlacza w formacie RGB888

////////////////////////////////////////////////////////////////////////////////
// Zapełnij cały bufor wyświetlacza podanym kolorem
// Parametry: chBufor* - wskaźnik na bufor pamieci ekranu
// nKolorARGB - kolor w formacie ARGB 8888
// Zwraca: nic
// Czas wykonania: 59 us
////////////////////////////////////////////////////////////////////////////////
void WypelnijBuforEkranu(uint8_t *chBufor, uint32_t nKolorARGB)
{
	for (uint16_t y=0; y<DISP_Y_SIZE; y++)
	{
		for (uint16_t x=0; x<DISP_X_SIZE; x++)
		{
			*(chBufor + y*DISP_X_SIZE*3 + x*3 + 0) = (uint8_t)(nKolorARGB >> 16);
			*(chBufor + y*DISP_X_SIZE*3 + x*3 + 1) = (uint8_t)(nKolorARGB >> 8);
			*(chBufor + y*DISP_X_SIZE*3 + x*3 + 2) = (uint8_t)(nKolorARGB & 0xFF);
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuj prostokąt wypełniony kolorem w buforze wyświetlacza
// Parametry:
// sStartX, sStartY - współrzędne lewego, górnego rogu prostokąta
// sSzerokosc, sWysokosc - rozmiary prostokąta
// nKolorARGB - kolor w formacie ARGB 8888
// Zwraca: nic
// Czas wykonania: ?
////////////////////////////////////////////////////////////////////////////////
void RysujProstokatWypelnionywBuforze(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *chBufor, uint32_t nKolorARGB)
{
	for (uint16_t y=0; y<sWysokosc; y++)
	{
		for (uint16_t x=0; x<sSzerokosc; x++)
		{
			*(chBufor + (y + sStartY) * DISP_X_SIZE * 3 + (x + sStartX) * 3 + 0) = (uint8_t)(nKolorARGB >> 16);
			*(chBufor + (y + sStartY) * DISP_X_SIZE * 3 + (x + sStartX) * 3 + 1) = (uint8_t)(nKolorARGB >> 8);
			*(chBufor + (y + sStartY) * DISP_X_SIZE * 3 + (x + sStartX) * 3 + 2) = (uint8_t)(nKolorARGB & 0xFF);
		}
	}
}



