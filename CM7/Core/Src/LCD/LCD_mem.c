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


uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforLCD[SZER_BUFORA * WYS_BUFORA * 3];	//pamięć obrazu wyświetlacza w formacie RGB888
extern struct current_font cfont;
extern uint8_t _transparent;	//flaga określająca czy mamy rysować tło czy rysujemy na istniejącym



////////////////////////////////////////////////////////////////////////////////
// Zapełnij cały bufor wyświetlacza podanym kolorem
// Parametry:
//	chBufor* - wskaźnik na bufor pamieci ekranu
//  chKolor* - wskaźnik na kolor w dowolnym formacie
//	chRozmKoloru -rozmiar koloru w bajtach
//  Zwraca: nic
// Czas wykonania: 46 us
////////////////////////////////////////////////////////////////////////////////
void PostawPikselwBuforze(uint16_t x, uint16_t y, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru)
{
	uint32_t nOffset;
	nOffset = y * SZER_BUFORA * chRozmKoloru + x * chRozmKoloru;
	for (uint8_t n=0; n<chRozmKoloru; n++)
		*(chBufor + nOffset + n) = (uint8_t)(*(chKolor + n));
}



////////////////////////////////////////////////////////////////////////////////
// Zapełnij cały bufor wyświetlacza podanym kolorem
// Parametry:
//	chBufor* - wskaźnik na bufor pamieci ekranu
//  nKolorARGB - kolor w formacie ARGB 8888
//  Zwraca: nic
// Czas wykonania: 46 us
////////////////////////////////////////////////////////////////////////////////
void WypelnijEkranwBuforze(uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru)
{
	uint32_t nOffset;

	for (uint16_t y=0; y<WYS_BUFORA; y++)
	{
		for (uint16_t x=0; x<SZER_BUFORA; x++)
		{
			nOffset = y * SZER_BUFORA * chRozmKoloru + x * chRozmKoloru;
			for (uint8_t n=0; n<chRozmKoloru; n++)
				*(chBufor + nOffset + n) = (uint8_t)(*(chKolor + n));
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
void RysujProstokatWypelnionywBuforze(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru)
{
	for (uint16_t y=0; y<sWysokosc; y++)
	{
		for (uint16_t x=0; x<sSzerokosc; x++)
			PostawPikselwBuforze(sStartX + x, sStartY + y, chBufor, chKolor, chRozmKoloru);
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
void RysujLiniewBuforze(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru)
{
	int16_t d;		//zmienna decyzyjna
	int16_t dx, dy; //przyrosty
	int16_t xi, yi;	//wartości zmiany
	int16_t ai, bi;
	int16_t x = x1, y = y1;
	//uint32_t nOffset;

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
	PostawPikselwBuforze(x, y, chBufor, chKolor, chRozmKoloru);

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
			PostawPikselwBuforze(x, y, chBufor, chKolor, chRozmKoloru);
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
			PostawPikselwBuforze(x, y, chBufor, chKolor, chRozmKoloru);
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
void RysujLiniePoziomawBuforze(uint16_t x1, uint16_t x2, uint16_t y, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru)
{
	for (uint16_t x=x1; x<x2; x++)
		PostawPikselwBuforze(x, y, chBufor, chKolor, chRozmKoloru);
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
void RysujLiniePionowawBuforze(uint16_t x, uint16_t y1, uint16_t y2, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru)
{
	for (uint16_t y=y1; y<y2; y++)
		PostawPikselwBuforze(x, y, chBufor, chKolor, chRozmKoloru);
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
void RysujOkragwBuforze(uint16_t x, uint16_t y, uint16_t promien, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru)
{
	int16_t f = 1 - promien;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * promien;
	int16_t x1 = 0;
	int16_t y1 = promien;

	PostawPikselwBuforze(x, y + promien, chBufor, chKolor, chRozmKoloru);
	PostawPikselwBuforze(x, y - promien, chBufor, chKolor, chRozmKoloru);
	PostawPikselwBuforze(x + promien, y, chBufor, chKolor, chRozmKoloru);
	PostawPikselwBuforze(x - promien, y, chBufor, chKolor, chRozmKoloru);

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
		PostawPikselwBuforze(x + x1, y + y1, chBufor, chKolor, chRozmKoloru);
		PostawPikselwBuforze(x - x1, y + y1, chBufor, chKolor, chRozmKoloru);
		PostawPikselwBuforze(x + x1, y - y1, chBufor, chKolor, chRozmKoloru);
		PostawPikselwBuforze(x - x1, y - y1, chBufor, chKolor, chRozmKoloru);
		PostawPikselwBuforze(x + y1, y + x1, chBufor, chKolor, chRozmKoloru);
		PostawPikselwBuforze(x - y1, y + x1, chBufor, chKolor, chRozmKoloru);
		PostawPikselwBuforze(x + y1, y - x1, chBufor, chKolor, chRozmKoloru);
		PostawPikselwBuforze(x - y1, y - x1, chBufor, chKolor, chRozmKoloru);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Pisze znak w buforze na miejscu o podanych współrzędnych
// Parametry:
//	chZnak - znak;
//	x, y - współrzędne
//  chBufor* - wskaźnik na bufor pamieci ekranu
//  chKolor*, chTlo* - wskaźniki na kolory znaku i tła . Zerowe tło oznacza że nie jest ono rysowane
//	chRozmKoloru - ilość bajtów okreslające kolor
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujZnakwBuforze(uint8_t chZnak, uint16_t x, uint16_t y, uint8_t *chBufor, uint8_t *chKolor, uint8_t *chTlo, uint8_t chRozmKoloru)
{
	uint8_t i, ch;
	uint16_t j;
	uint16_t temp;
	int16_t zz;

	temp = ((chZnak - cfont.offset) * ((cfont.x_size / 8) * cfont.y_size)) + 4;

	if (chTlo != 0)
	{
		for(j=0;j<cfont.y_size;j++)
		{
			for (zz=0; zz<(cfont.x_size/8); zz++)
			{
				ch=cfont.font[temp + zz];
				for(i=0; i<8; i++)
				{
					if((ch & (1 << (7 - i))) != 0)
						PostawPikselwBuforze(x + i + zz * 8, y +j, chBufor, chKolor, chRozmKoloru);
					else
						PostawPikselwBuforze(x + i + zz * 8, y +j, chBufor, chTlo, chRozmKoloru);
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
						PostawPikselwBuforze(x + i + zz * 8, y + j, chBufor, chKolor, chRozmKoloru);
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
//	x, y - współrzędne, gdzie x może przybierać formy szczegółne: RIGHT napis od prawego brzegu ekranu, CENTER - napis wyjustowany na środku
//  chBufor* - wskaźnik na bufor pamieci ekranu
//  nKolorARGB, nTloARGB - kolory znaku i tła w formacie ARGB 8888
//	chTransparent - okresla czy wolne przestrzenie w znaku są przezroczyste czy wypełnione kolorem tła
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujNapiswBuforze(char *chNapis, uint16_t x, uint16_t y, uint8_t *chBufor, uint8_t *chKolor, uint8_t *chTlo, uint8_t chRozmKoloru)
{
	uint16_t sDlugosc;

	sDlugosc = strlen((char*)chNapis);

	if (x == RIGHT)
		x = (SZER_BUFORA + 1) - (sDlugosc * cfont.x_size);
	if (x == CENTER)
		x = ((SZER_BUFORA + 1) - (sDlugosc * cfont.x_size)) / 2;

	for (uint16_t n=0; n<sDlugosc; n++)
		RysujZnakwBuforze(*chNapis++, x + (n * cfont.x_size), y, chBufor, chKolor, chTlo, chRozmKoloru);
}

