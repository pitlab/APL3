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
extern struct current_font cfont;
extern uint8_t _transparent;	//flaga określająca czy mamy rysować tło czy rysujemy na istniejącym

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

	// ustalenie kierunku rysowania w osi X
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

	//i w osi Y
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

	//jezeli oś wiodąca X
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
	else	// oś wiodąca Y
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
	uint32_t nOffset;

	nOffset = (y + promien) * DISP_X_SIZE * 3 + x * 3;
	*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
	*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
	*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

	nOffset = (y - promien) * DISP_X_SIZE * 3 + x * 3;
	*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
	*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
	*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

	nOffset = y * DISP_X_SIZE * 3 + (x + promien) * 3;
	*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
	*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
	*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

	nOffset = y * DISP_X_SIZE * 3 + (x - promien) * 3;
	*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
	*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
	*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

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
		nOffset = (y + y1) * DISP_X_SIZE * 3 + (x + x1) * 3;
		*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
		*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
		*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

		nOffset = (y + y1) * DISP_X_SIZE * 3 + (x - x1) * 3;
		*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
		*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
		*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

		nOffset = (y - y1) * DISP_X_SIZE * 3 + (x + x1) * 3;
		*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
		*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
		*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

		nOffset = (y - y1) * DISP_X_SIZE * 3 + (x - x1) * 3;
		*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
		*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
		*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

		nOffset = (y + x1) * DISP_X_SIZE * 3 + (x + y1) * 3;
		*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
		*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
		*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

		nOffset = (y + x1) * DISP_X_SIZE * 3 + (x - y1) * 3;
		*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
		*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
		*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

		nOffset = (y - x1) * DISP_X_SIZE * 3 + (x + y1) * 3;
		*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
		*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
		*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);

		nOffset = (y - x1) * DISP_X_SIZE * 3 + (x - y1) * 3;
		*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
		*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
		*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Pisze znak w buforze na miejscu o podanych współrzędnych
// Parametry:
//	chZnak - znak;
//	x, y - współrzędne
//  chBufor* - wskaźnik na bufor pamieci ekranu
//  nKolorARGB, nTloARGB - kolory znaku i tła w formacie ARGB 8888
//	chTransparent - okresla czy wolne przestrzenie w znaku są przezroczyste czy wypełnione kolorem tła
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujZnakwBuforze(uint8_t chZnak, uint16_t x, uint16_t y, uint8_t *chBufor, uint32_t nKolorARGB, uint32_t nTloARGB, uint8_t chPrzezroczystosc)
{
	uint8_t i, ch;
	uint16_t j;
	uint16_t temp;
	int16_t zz;
	uint32_t nOffset;

	temp = ((chZnak - cfont.offset) * ((cfont.x_size / 8) * cfont.y_size)) + 4;

	if (!chPrzezroczystosc)
	{
		for(j=0;j<((cfont.x_size/8)*cfont.y_size);j+=(cfont.x_size/8))
		{
			//setXY(x,y+(j/(cfont.x_size/8)),x+cfont.x_size-1,y+(j/(cfont.x_size/8)));
			for (zz=(cfont.x_size / 8)-1; zz>=0; zz--)
			{
				ch=cfont.font[temp + zz];
				for(i=0; i<8; i++)
				{
					nOffset = (y + (j / (cfont.x_size / 8))) * DISP_X_SIZE * 3 + (x + i + zz * 8) * 3;

					if((ch & (1<<i)) != 0)
					{
						*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
						*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
						*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);
					}
					else
					{
						*(chBufor + nOffset + 0) = (uint8_t)(nTloARGB >> 16);
						*(chBufor + nOffset + 1) = (uint8_t)(nTloARGB >> 8);
						*(chBufor + nOffset + 2) = (uint8_t)(nTloARGB & 0xFF);
					}
				}
			}
			temp += (cfont.x_size / 8);
		}
	}
	else
	{
		for(j=0;j<cfont.y_size;j++)
		{
			for (zz=0; zz<(cfont.x_size/8); zz++)
			{
				ch = cfont.font[temp + zz];
				for(i=0; i<8; i++)
				{
					if((ch&(1<<(7-i)))!=0)
					{
						//setXY(x+i+(zz*8),y+j,x+i+(zz*8)+1,y+j+1);
						nOffset = (y + j) * DISP_X_SIZE * 3 + (x + i + zz * 8) * 3;
						*(chBufor + nOffset + 0) = (uint8_t)(nKolorARGB >> 16);
						*(chBufor + nOffset + 1) = (uint8_t)(nKolorARGB >> 8);
						*(chBufor + nOffset + 2) = (uint8_t)(nKolorARGB & 0xFF);
					}
				}
			}
			temp+=(cfont.x_size/8);
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Pisze napis w buforze na miejscu o podanych współrzędnych
// Parametry:
//	*chNapis - wskaźnik na napis;
//	x, y - współrzędne, gdzie x może przybierać formy szczegółne: RIGHT napis od lewego brzegu ekranu, CENTER - napis wyjustowany na środku
//  chBufor* - wskaźnik na bufor pamieci ekranu
//  nKolorARGB, nTloARGB - kolory znaku i tła w formacie ARGB 8888
//	chTransparent - okresla czy wolne przestrzenie w znaku są przezroczyste czy wypełnione kolorem tła
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujNapiswBuforze(char *chNapis, uint16_t x, uint16_t y, uint8_t *chBufor, uint32_t nKolorARGB, uint32_t nTloARGB, uint8_t chPrzezroczystosc)
{
	uint16_t sDlugosc;

	sDlugosc = strlen((char*)chNapis);

	if (x == RIGHT)
		x = (DISP_X_SIZE + 1) - (sDlugosc * cfont.x_size);
	if (x == CENTER)
		x = ((DISP_X_SIZE + 1) - (sDlugosc * cfont.x_size)) / 2;

	for (uint16_t n=0; n<sDlugosc; n++)
		RysujZnakwBuforze(*chNapis++, x + (n * cfont.x_size), y, chBufor, nKolorARGB, nTloARGB, chPrzezroczystosc);
}

