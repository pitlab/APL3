/*
 * jpeg.h
 *
 *  Created on: Sep 19, 2025
 *      Author: PitLab
 */

#ifndef INC_JPEG_H_
#define INC_JPEG_H_

#include "sys_def_CM7.h"

#define ROZM_BUF_WY_JPEG	8192	//16 sektorów, 1 jednostka alokacji FAT32
//#define ROZM_BUF_WY_JPEG	1024	//2 sektory
#define ILOSC_BUF_JPEG		4		//buforowanie x4
#define MASKA_LICZBY_BUF	0x03	//maska do przycinania wskaźnika buforów
#define SZEROKOSC_BLOKU		8
#define WYSOKOSC_BLOKU		8
#define ROZMIAR_BLOKU		(SZEROKOSC_BLOKU * WYSOKOSC_BLOKU)
#define ROZMIAR_MCU420		(6 * ROZMIAR_BLOKU)
#define ILOSC_BUF_WE_MCU	12		//liczba podzielna przez 6 (UVY420), przez 4(UVY422), przez 3 (UVY444) i przez 1 (Y8)
//#define MASKA_BUF_MCU		0x07	//maska do przycinania wskaźnika buforów

//flagi ustawiane w calbackach określające stan enkodera
#define KOMPR_PUSTE_WE		1	//na wejście enkodera można podać nowe dane
#define KOMPR_MCU_GOTOWY	2	//podany MCU jest już skompresowany
#define KOMPR_OBR_GOTOWY	4	//podany obraz jest już skompresowany

#define ROZMIAR_NAGL_JPEG	20
#define ROZMIAR_ZNACZ_xOI	2
#define ROZMIAR_EXIF		44


uint8_t InicjalizujJpeg(void);
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chTypKoloru, uint8_t chTypChrominancji, uint8_t chJakoscObrazu);
uint8_t CzekajNaKoniecPracyJPEG(void);
uint8_t KompresujY8(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujYUV444(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujYUV420(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t* ZnajdzZnacznikJpeg(uint8_t *chDaneSkompresowane, uint8_t chZnacznik);
#endif /* INC_JPEG_H_ */
