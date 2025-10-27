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
// Parametry:
//	chBufor* - wskaźnik na bufor pamieci ekranu
//  nKolorARGB - kolor w formacie ARGB 8888
//  Zwraca: nic
// Czas wykonania: 46 us
////////////////////////////////////////////////////////////////////////////////
void WypelnijBuforEkranu(uint8_t *chBufor, uint32_t nKolorARGB)
{
	uint32_t nOffset;

	for (uint16_t y=0; y<DISP_Y_SIZE; y++)
	{
		for (uint16_t x=0; x<DISP_X_SIZE; x++)
		{
			nOffset = y*DISP_X_SIZE*3 + x*3;
			*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
			*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
			*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuj prostokąt wypełniony kolorem w buforze wyświetlacza
// Parametry:
//  sStartX, sStartY - współrzędne lewego, górnego rogu prostokąta
//  sSzerokosc, sWysokosc - rozmiary prostokąta
//  chBufor* - wskaźnik na bufor pamieci ekranu
//  nKolorARGB - kolor w formacie ARGB 8888
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujProstokatWypelnionywBuforze(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *chBufor, uint32_t nKolorARGB)
{
	uint32_t nOffset;

	for (uint16_t y=0; y<sWysokosc; y++)
	{
		for (uint16_t x=0; x<sSzerokosc; x++)
		{
			nOffset = (y + sStartY) * DISP_X_SIZE * 3 + (x + sStartX) * 3;
			*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
			*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
			*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);
		}
	}
}


////////////////////////////////////////////////////////////////////////////////
// Rysuj linię o podanym kolorze w buforze wyświetlacza korzystajac z algorytmu Bresenhama
// https://pl.wikipedia.org/wiki/Algorytm_Bresenhama
// Parametry:
//  x1, y1 - współrzędne początku
//  x2, y2 - współrzędne końca
//  chBufor* - wskaźnik na bufor pamieci ekranu
//  nKolorARGB - kolor w formacie ARGB 8888
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujLiniewBuforze(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t *chBufor, uint32_t nKolorARGB)
{
	int16_t d;		//zmienna decyzyjna
	int16_t dx, dy; //przyrosty
	int16_t xi, yi;	//wartości zmiany
	int16_t ai, bi;
	int16_t x = x1, y = y1;
	uint32_t nOffset;

	// ustalenie kierunku rysowania
	if (x1 < x2)
	{
		xi = 1;
		dx = x2 - x1;
	}
	else
	{
		xi = -1;
		dx = x1 - x2;
	}

	if (y1 < y2)
	{
		yi = 1;
		dy = y2 - y1;
	}
	else
	{
		yi = -1;
		dy = y1 - y2;
	}

	 //pierwszy piksel
	nOffset = y * DISP_X_SIZE * 3 + x * 3;
	*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
	*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
	*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

	// oś wiodąca to OX
	if (dx > dy)
	{
		ai = (dy - dx) * 2;
		bi = dy * 2;
		d = bi - dx;
		// pętla po kolejnych x
		while (x != x2)
		{
			// test współczynnika
			if (d >= 0)
			{
				 x += xi;
				 y += yi;
				 d += ai;
			}
			else
			{
				 d += bi;
				 x += xi;
			}
			nOffset = y * DISP_X_SIZE * 3 + x * 3;
			*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
			*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
			*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);
		}
	}
	else	// oś wiodąca to OY
	{
		ai = ( dx - dy ) * 2;
		bi = dx * 2;
		d = bi - dy;
		// pętla po kolejnych y
		while (y != y2)
		{
			// test współczynnika
			if (d >= 0)
			{
				x += xi;
				y += yi;
				d += ai;
			}
			else
			{
				d += bi;
				y += yi;
			}
			nOffset = y * DISP_X_SIZE * 3 + x * 3;
			*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
			*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
			*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuj poziomą linię o podanym kolorze w buforze wyświetlacza
// Parametry:
//  x1, x2 - współrzędne x początku i końca
//  y - niezmienna współrzędna y
//  chBufor* - wskaźnik na bufor pamieci ekranu
//  nKolorARGB - kolor w formacie ARGB 8888
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujLiniePoziomawBuforze(uint16_t x1, uint16_t x2, uint16_t y, uint8_t *chBufor, uint32_t nKolorARGB)
{
	uint32_t nOffset;

	for (uint16_t x=x1; x<x2; x++)
	{
		nOffset = y * DISP_X_SIZE * 3 + x * 3;
		*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
		*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
		*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuj pionową linię o podanym kolorze w buforze wyświetlacza
// Parametry:
//  x - niezmienna współrzędna x
//  y1, y2 - współrzędne y początku i końca
//  chBufor* - wskaźnik na bufor pamieci ekranu
//  nKolorARGB - kolor w formacie ARGB 8888
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujLiniePionowawBuforze(uint16_t x, uint16_t y1, uint16_t y2, uint8_t *chBufor, uint32_t nKolorARGB)
{
	uint32_t nOffset;

	for (uint16_t y=y1; y<y2; y++)
	{
		nOffset = y * DISP_X_SIZE * 3 + x * 3;
		*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
		*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
		*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);
	}
}

////////////////////////////////////////////////////////////////////////////////
// Rysuj okrąg o podanym kolorze w buforze wyświetlacza korzystajac z algorytmu Bresenhama
// Parametry:
//  x, y - współrzdne środka
//  promien - promień
//  chBufor* - wskaźnik na bufor pamieci ekranu
//  nKolorARGB - kolor w formacie ARGB 8888
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujOkragwBuforze(uint16_t x, uint16_t y, uint16_t promien, uint8_t *chBufor, uint32_t nKolorARGB)
{
	int16_t f = 1 - promien;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * promien;
	int16_t x1 = 0;
	int16_t y1 = promien;

	//jalizacja:Rozpocznij od punktu \((0,r)\), gdzie \(r\) to promień okręgu.Oblicz wartość początkową parametru decyzyjnego, często oznaczanego jako \(d\) lub \(e\), zgodnie z formułą \(d=1-r\).Generowanie pierwszej ósemki okręgu:Iteruj, zwiększając \(x\) od \(0\) do momentu, gdy \(x=y\) (czyli do \(45^{\circ }\)).W każdym kroku wykonaj następujące działania:Wyświetl punkty symetryczne w 8 ćwiartkach okręgu (np. \((x,y),(x,-y),(-x,y),(-x,-y),(y,x),(y,-x),(-y,x),(-y,-x)\)).Sprawdź wartość parametru decyzyjnego \(d\).Jeśli \(d\le 0\), wybierz następny piksel na wschód (\(x+1,y\)). Zaktualizuj parametr decyzyjny za pomocą formuły \(d=d+2x+1\).Jeśli \(d>0\), wybierz następny piksel na południowy-wschód (\(x+1,y-1\)). Zaktualizuj parametr decyzyjny za pomocą formuły \(d=d+2x-2y+1\).Zwiększ \(x\) o 1. 



	/*/cbi(P_CS, B_CS);
	setXY(x, y + promien, x, y + promien);
	LCD_WrData(chKolor666, 3);

	setXY(x, y - promien, x, y - promien);
	LCD_WrData(chKolor666, 3);

	setXY(x + promien, y, x + promien, y);
	LCD_WrData(chKolor666, 3);

	setXY(x - promien, y, x - promien, y);
	LCD_WrData(chKolor666, 3);

	while(x1 < y1)
	{
		if(f >= 0)
		{
			y1--;
			ddF_y += 2;
			f += ddF_y;
		}
		x1++;
		ddF_x += 2;
		f += ddF_x;
		setXY(x + x1, y + y1, x + x1, y + y1);
		LCD_WrData(chKolor666, 3);

		setXY(x - x1, y + y1, x - x1, y + y1);
		LCD_WrData(chKolor666, 3);

		setXY(x + x1, y - y1, x + x1, y - y1);
		LCD_WrData(chKolor666, 3);

		setXY(x - x1, y - y1, x - x1, y - y1);
		LCD_WrData(chKolor666, 3);

		setXY(x + y1, y + x1, x + y1, y + x1);
		LCD_WrData(chKolor666, 3);

		setXY(x - y1, y + x1, x - y1, y + x1);
		LCD_WrData(chKolor666, 3);

		setXY(x + y1, y - x1, x + y1, y - x1);
		LCD_WrData(chKolor666, 3);

		setXY(x - y1, y - x1, x - y1, y - x1);
		LCD_WrData(chKolor666, 3);
	}
	//sbi(P_CS, B_CS);
	clrXY();*/
}




