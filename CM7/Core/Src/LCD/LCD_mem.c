//////////////////////////////////////////////////////////////////////////////
//
// Funkcje wyświetlania bezpośrednio do pamięci obrazu
//  przyjmującego kolor w formacie RGB888
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "LCD/LCD_mem.h"
#include "Ekran.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "OSD.h"
#include "Rysuj.h"
#include "AnalizaObrazu.h"

uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) cBuforLCD[DISP_X_SIZE * DISP_Y_SIZE * 3];	//pamięć obrazu wyświetlacza w formacie RGB888

extern struct current_font stCzcionka;
extern uint8_t cTransparent;	//flaga określająca czy mamy rysować tło czy rysujemy na istniejącym



////////////////////////////////////////////////////////////////////////////////
// Zapełnij cały bufor wyświetlacza podanym kolorem
// Parametry:
// 	x, y - współrzędne piksela
//	sSzerokosc - szerokość obrazu
//	cBufor* - wskaźnik na bufor pamieci ekranu
//  cKolor* - wskaźnik na kolor w dowolnym formacie
//	cRozmKoloru - rozmiar koloru w bajtach
//  Zwraca: nic
// Czas wykonania: 46 us
////////////////////////////////////////////////////////////////////////////////
void PostawPikselwBuforze(uint16_t x, uint16_t y, uint16_t sSzerokosc, uint8_t *cBufor, uint8_t *cKolor, uint8_t cRozmKoloru)
{
	uint32_t nOffset;

	if ((x < sSzerokosc) && (y < sSzerokosc))	//zabezpieczenie przed współrzędnymi ujemnymi
	{
		nOffset = y * sSzerokosc * cRozmKoloru + x * cRozmKoloru;
		for (uint8_t n=0; n<cRozmKoloru; n++)
			*(cBufor + nOffset + n) = (uint8_t)(*(cKolor + n));
	}
}



/*///////////////////////////////////////////////////////////////////////////////
// Zapełnij cały bufor wyświetlacza podanym kolorem
// Parametry:
//	cBufor* - wskaźnik na bufor pamieci ekranu
//  cKolor* - wskaźnik na kolor w dowolnym formacie
//	cRozmKoloru - rozmiar koloru w bajtach
//  Zwraca: nic
// Czas wykonania: 46 us
////////////////////////////////////////////////////////////////////////////////
void WypelnijEkranwBuforze1(uint8_t *cBufor, uint8_t *cKolor, uint8_t cRozmKoloru)
{
	uint32_t nOffset;

	for (uint16_t y=0; y<WYS_BUFORA; y++)
	{
		for (uint16_t x=0; x<SZER_BUFORA; x++)
		{
			nOffset = y * SZER_BUFORA * chRozmKoloru + x * hRozmKoloru;
			for (uint8_t n=0; n<hRozmKoloru; n++)
				*(cBufor + nOffset + n) = (uint8_t)(*(cKolor + n));
		}
	}
}*/

////////////////////////////////////////////////////////////////////////////////
// Zapełnij cały bufor wyświetlacza podanym kolorem
// Parametry:
// 	sSzerokosc, sWysokosc - rozmiary ekranu
//	chBufor* - wskaźnik na bufor pamieci ekranu
//  sKolor - kolor w formacie RGB565
//  Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WypelnijEkranwBuforze(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *cBufor, uint16_t sKolor)
{
	uint32_t nOffset;

	for (uint16_t y=0; y<sWysokosc; y++)
	{
		for (uint16_t x=0; x<sSzerokosc; x++)
		{
			nOffset = y * sSzerokosc * ROZMIAR_KOLORU_OSD + x * ROZMIAR_KOLORU_OSD;
			*(cBufor + nOffset + 0) = (uint8_t)sKolor;
			*(cBufor + nOffset + 1) = (uint8_t)(sKolor >> 8);
		}
	}
}


////////////////////////////////////////////////////////////////////////////////
// Rysuj prostokąt wypełniony kolorem w buforze wyświetlacza
// Parametry:
//  sStartX, sStartY - współrzędne lewego, górnego rogu prostokąta
//  sSzerokosc, sWysokosc - rozmiary prostokąta
//  cBufor* - wskaźnik na bufor pamieci ekranu
//  cKolor* - wskaźnik na kolor w dowolnym formacie
//	cRozmKoloru - rozmiar koloru w bajtach
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujProstokatWypelnionywBuforze(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *cBufor, uint8_t *cKolor, uint8_t cRozmKoloru)
{
	for (uint16_t y=0; y<sWysokosc; y++)
	{
		for (uint16_t x=0; x<sSzerokosc; x++)
			PostawPikselwBuforze(sStartX + x, sStartY + y, DISP_X_SIZE, cBufor, cKolor, cRozmKoloru);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuj linię o podanym kolorze w buforze wyświetlacza korzystajac z algorytmu Bresenhama
// https://pl.wikipedia.org/wiki/Algorytm_Bresenhama
// Parametry:
//  x1, y1 - współrzędne początku
//  x2, y2 - współrzędne końca
//	sSzerokosc - szerokość ekranu
//  cBufor* - wskaźnik na bufor pamieci ekranu
//  cKolor* - wskaźnik na kolor w dowolnym formacie
//	cRozmKoloru - rozmiar koloru w bajtach
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujLiniewBuforze(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t sSzerokosc, uint8_t *cBufor, uint8_t *cKolor, uint8_t cRozmKoloru)
{
	int16_t d;		//zmienna decyzyjna
	int16_t dx, dy; //przyrosty
	int16_t xi, yi;	//wartości zmiany
	int16_t ai, bi;
	int16_t x = x1, y = y1;
	//uint32_t nOffset;

	if ((x1 > sSzerokosc) || (x2 > sSzerokosc) || (y1 > sSzerokosc) || (y2 > sSzerokosc))	//zabezpieczenie przed współrzędnymi ujemnymi. Zakłada że szerokość jest zawsze większa niż nieznanz tutaj wysokość
		return;

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
	PostawPikselwBuforze(x, y, sSzerokosc, cBufor, cKolor, cRozmKoloru);

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
			PostawPikselwBuforze(x, y, sSzerokosc, cBufor, cKolor, cRozmKoloru);
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
			PostawPikselwBuforze(x, y, sSzerokosc, cBufor, cKolor, cRozmKoloru);
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuj poziomą linię o podanym kolorze w buforze wyświetlacza
// Parametry:
//  x1, x2 - współrzędne x początku i końca
//  y - niezmienna współrzędna y
//  cBufor* - wskaźnik na bufor pamieci ekranu
//  cKolor* - wskaźnik na kolor w dowolnym formacie
//	cRozmKoloru - rozmiar koloru w bajtach
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujLiniePoziomawBuforze(uint16_t x1, uint16_t x2, uint16_t y, uint16_t sSzerokosc, uint8_t *cBufor, uint8_t *cKolor, uint8_t cRozmKoloru)
{
	if ((x1 > sSzerokosc) || (x2 > sSzerokosc) || (y > sSzerokosc))	//zabezpieczenie przed współrzędnymi ujemnymi. Zakłada że szerokość jest zawsze większa niż nieznanz tutaj wysokość
		return;

	for (uint16_t x=x1; x<x2; x++)
		PostawPikselwBuforze(x, y, sSzerokosc, cBufor, cKolor, cRozmKoloru);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuj pionową linię o podanym kolorze w buforze wyświetlacza
// Parametry:
//  x - niezmienna współrzędna x
//  y1, y2 - współrzędne y początku i końca
//  cBufor* - wskaźnik na bufor pamieci ekranu
//  cKolor* - wskaźnik na kolor w dowolnym formacie
//	cRozmKoloru - rozmiar koloru w bajtach
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujLiniePionowawBuforze(uint16_t x, uint16_t y1, uint16_t y2, uint16_t sSzerokosc, uint8_t *cBufor, uint8_t *cKolor, uint8_t cRozmKoloru)
{
	if ((x > sSzerokosc) || (y1 > sSzerokosc) || (y2 > sSzerokosc))	//zabezpieczenie przed współrzędnymi ujemnymi. Zakłada że szerokość jest zawsze większa niż nieznana tutaj wysokość
		return;

	for (uint16_t y=y1; y<y2; y++)
		PostawPikselwBuforze(x, y, sSzerokosc, cBufor, cKolor, cRozmKoloru);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuj okrąg o podanym kolorze w buforze wyświetlacza korzystajac z algorytmu Bresenhama
// Parametry:
//  x, y - współrzdne środka
//  promien - promień
//  cBufor* - wskaźnik na bufor pamieci ekranu
//  cKolor* - wskaźnik na kolor w dowolnym formacie
//	cRozmKoloru - rozmiar koloru w bajtach
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujOkragwBuforze(uint16_t x, uint16_t y, uint16_t promien, uint16_t sSzerokosc, uint8_t *cBufor, uint8_t *cKolor, uint8_t cRozmKoloru)
{
	int16_t f = 1 - promien;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * promien;
	int16_t x1 = 0;
	int16_t y1 = promien;

	PostawPikselwBuforze(x, y + promien, sSzerokosc, cBufor, cKolor, cRozmKoloru);
	PostawPikselwBuforze(x, y - promien, sSzerokosc, cBufor, cKolor, cRozmKoloru);
	PostawPikselwBuforze(x + promien, y, sSzerokosc, cBufor, cKolor, cRozmKoloru);
	PostawPikselwBuforze(x - promien, y, sSzerokosc, cBufor, cKolor, cRozmKoloru);

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
		PostawPikselwBuforze(x + x1, y + y1, sSzerokosc, cBufor, cKolor, cRozmKoloru);
		PostawPikselwBuforze(x - x1, y + y1, sSzerokosc, cBufor, cKolor, cRozmKoloru);
		PostawPikselwBuforze(x + x1, y - y1, sSzerokosc, cBufor, cKolor, cRozmKoloru);
		PostawPikselwBuforze(x - x1, y - y1, sSzerokosc, cBufor, cKolor, cRozmKoloru);
		PostawPikselwBuforze(x + y1, y + x1, sSzerokosc, cBufor, cKolor, cRozmKoloru);
		PostawPikselwBuforze(x - y1, y + x1, sSzerokosc, cBufor, cKolor, cRozmKoloru);
		PostawPikselwBuforze(x + y1, y - x1, sSzerokosc, cBufor, cKolor, cRozmKoloru);
		PostawPikselwBuforze(x - y1, y - x1, sSzerokosc, cBufor, cKolor, cRozmKoloru);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Pisze znak w buforze na miejscu o podanych współrzędnych
// Parametry:
//	cZnak - znak;
//	x, y - współrzędne
//  cBufor* - wskaźnik na bufor pamieci ekranu
//  cKolor*, cTlo* - wskaźniki na kolory znaku i tła . Zerowe tło oznacza że nie jest ono rysowane
//	cRozmKoloru - ilość bajtów okreslające kolor
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujZnakwBuforze(uint8_t cZnak, uint16_t x, uint16_t y, uint16_t sSzerokosc, uint8_t *cBufor, uint8_t *cKolor, uint8_t *cTlo, uint8_t cRozmKoloru)
{
	uint8_t i, ch;
	uint16_t j;
	uint16_t temp;
	int16_t zz;

	temp = ((cZnak - stCzcionka.offset) * ((stCzcionka.x_size / 8) * stCzcionka.y_size)) + 4;

	if (cTlo != 0)
	{
		for(j=0;j<stCzcionka.y_size;j++)
		{
			for (zz=0; zz<(stCzcionka.x_size/8); zz++)
			{
				ch = stCzcionka.font[temp + zz];
				for(i=0; i<8; i++)
				{
					if((ch & (1 << (7 - i))) != 0)
						PostawPikselwBuforze(x + i + zz * 8, y +j, sSzerokosc, cBufor, cKolor, cRozmKoloru);
					else
						PostawPikselwBuforze(x + i + zz * 8, y +j, sSzerokosc, cBufor, cTlo, cRozmKoloru);
				}
			}
			temp += (stCzcionka.x_size / 8);
		}
	}
	else
	{
		for(j=0;j<stCzcionka.y_size;j++)
		{
			for (zz=0; zz<(stCzcionka.x_size/8); zz++)
			{
				ch = stCzcionka.font[temp + zz];
				for(i=0; i<8; i++)
				{
					if((ch & (1<<(7-i))) != 0)
						PostawPikselwBuforze(x + i + zz * 8, y + j, sSzerokosc, cBufor, cKolor, cRozmKoloru);
				}
			}
			temp+=(stCzcionka.x_size/8);
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Pisze napis w buforze na miejscu o podanych współrzędnych
// Parametry:
//	*cNapis - wskaźnik na napis;
//	x, y - współrzędne, gdzie x może przybierać formy szczegółne: RIGHT napis od prawego brzegu ekranu, CENTER - napis wyjustowany na środku
//	sSzerokosc -
//  cBufor* - wskaźnik na bufor pamieci ekranu
//  *cKolor, *cTlo - wskażniki na struktury koloru znaku i tła w formacie ARGB 8888
//  cRozmKoloru - rozmiar struktur kolorui tła
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujNapiswBuforze(char *cNapis, uint16_t x, uint16_t y, uint16_t sSzerokosc, uint8_t *cBufor, uint8_t *cKolor, uint8_t *cTlo, uint8_t cRozmKoloru)
{
	uint16_t sDlugosc;

	sDlugosc = strlen((char*)cNapis);

	if (x == RIGHT)
		x = (SZER_BUFORA + 1) - (sDlugosc * stCzcionka.x_size);
	if (x == CENTER)
		x = ((SZER_BUFORA + 1) - (sDlugosc * stCzcionka.x_size)) / 2;

	for (uint16_t n=0; n<sDlugosc; n++)
		RysujZnakwBuforze(*cNapis++, x + (n * stCzcionka.x_size), y, sSzerokosc, cBufor, cKolor, cTlo, cRozmKoloru);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje histogram obrazu kolorowego RGB565 w prawej dolnej części ekranu
// Aby nie trzeba było zamazwywać całego tła za każdym razem, histogra jest rysowany w dwu etapach:
// W pierwszym jako widoczny pasek, nastepnie jako przezroczysta reszta, zamazując poprzednią zawartość
// Parametry:
//	*cOSD - wskaźnik na obraz OSD w którym ma być rysowany histogram
//	*histR, *histG, *histB - wskaźniki na składowe histogramu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void RysujHistogramOSD_RGB32(uint8_t *cOSD, uint8_t *histR, uint8_t *histG, uint8_t *histB)
{
	uint16_t sPoczatekX;	//współrzędna X początku kolejnego histogramu
	uint16_t sWysokosc;		//współrzędna Y wierzchołka paska histogramu
	uint16_t sKolorR = KOLOSD_CZER0 + PRZEZR_60;
	uint16_t sKolorG = KOLOSD_ZIEL0 + PRZEZR_60;
	uint16_t sKolorB = KOLOSD_NIEB0 + PRZEZR_60;
	uint16_t sKolorTla = PRZEZR_100;

	for (uint8_t x=0; x<ROZMIAR_HIST_KOLOR; x++)
	{
		sPoczatekX = (x * SZER_PASKA_HISTOGRAMU) + (DISP_X_SIZE - (3 * ROZMIAR_HIST_KOLOR * SZER_PASKA_HISTOGRAMU));
		sWysokosc = *(histR + x);
		RysujProstokatWypelnionywBuforze(sPoczatekX, DISP_Y_SIZE - sWysokosc, SZER_PASKA_HISTOGRAMU, sWysokosc, cOSD, (uint8_t*)&sKolorR, ROZMIAR_KOLORU_OSD);	//pasek
		RysujProstokatWypelnionywBuforze(sPoczatekX, DISP_Y_SIZE - 255,  SZER_PASKA_HISTOGRAMU, 255 - sWysokosc, cOSD, (uint8_t*)&sKolorTla, ROZMIAR_KOLORU_OSD);			//dopełnienie

		sPoczatekX = (x * SZER_PASKA_HISTOGRAMU) + (DISP_X_SIZE - (3 * ROZMIAR_HIST_KOLOR * SZER_PASKA_HISTOGRAMU)) + (ROZMIAR_HIST_KOLOR * SZER_PASKA_HISTOGRAMU);
		sWysokosc = *(histG + x);
		RysujProstokatWypelnionywBuforze(sPoczatekX, DISP_Y_SIZE - sWysokosc, SZER_PASKA_HISTOGRAMU, sWysokosc, cOSD, (uint8_t*)&sKolorG, ROZMIAR_KOLORU_OSD);
		RysujProstokatWypelnionywBuforze(sPoczatekX, DISP_Y_SIZE - 255, SZER_PASKA_HISTOGRAMU, 255 - sWysokosc, cOSD, (uint8_t*)&sKolorTla, ROZMIAR_KOLORU_OSD);

		sPoczatekX = (x * SZER_PASKA_HISTOGRAMU) + (DISP_X_SIZE - (3 * ROZMIAR_HIST_KOLOR * SZER_PASKA_HISTOGRAMU)) + (2 * ROZMIAR_HIST_KOLOR * SZER_PASKA_HISTOGRAMU);
		sWysokosc = *(histB + x);
		RysujProstokatWypelnionywBuforze(sPoczatekX, DISP_Y_SIZE - sWysokosc, SZER_PASKA_HISTOGRAMU,sWysokosc, cOSD, (uint8_t*)&sKolorB, ROZMIAR_KOLORU_OSD);
		RysujProstokatWypelnionywBuforze(sPoczatekX, DISP_Y_SIZE - 255, SZER_PASKA_HISTOGRAMU, 255 - sWysokosc, cOSD, (uint8_t*)&sKolorTla, ROZMIAR_KOLORU_OSD);
	}
}
