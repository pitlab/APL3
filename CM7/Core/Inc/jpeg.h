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
#define ILOSC_BUF_JPEG		4		//buforowanie x4
#define MASKA_LICZBY_BUF	0x03	//maska do przycinania wskaźnika buforów
#define SZEROKOSC_BLOKU		8
#define WYSOKOSC_BLOKU		8
#define ROZMIAR_BLOKU		(SZEROKOSC_BLOKU * WYSOKOSC_BLOKU)
#define ROZMIAR_MCU_Y8		ROZMIAR_BLOKU
#define ROZMIAR_MCU422		(4 * ROZMIAR_BLOKU)
#define ROZMIAR_MCU420		(6 * ROZMIAR_BLOKU)
#define ILOSC_BUF_WE_MCU	24		//liczba podzielna przez 2*6 (UVY420), przez 2*4(UVY422), przez 2*3 (UVY444) i przez 2*1 (Y8) - podwójne buforowane
#define LICZBA_KOLOR_RGB888	3		//liczba bajtów koloru
#define ROZMIAR_NAGL_JPEG	20
#define ROZMIAR_ZNACZ_xOI	2


uint8_t InicjalizujJpeg(void);
uint8_t KonfigurujKompresjeJpeg(JPEG_ConfTypeDef *stKonfJpeg, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chTypKoloru, uint8_t chTypChrominancji, uint8_t chJakoscObrazu);
uint8_t CzekajNaKoniecPracyJPEG(void);
uint8_t KompresujY8(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujYUV444(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujYUV420(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujRGB888doY8(uint8_t *obrazRGB888, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chJakosc);
uint8_t KompresujRGB888doYUV444(uint8_t *obrazRGB888, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chJakosc);
uint8_t KompresujRGB888doYUV422(uint8_t *obrazRGB888, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chJakosc);
uint8_t KompresujRGB888doYUV420(uint8_t *obrazRGB888, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chJakosc);
uint8_t TestKonwersjiRGB888doYCbCr(uint8_t *obrazRGB888, uint8_t *buforYCbCr, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t* ZnajdzZnacznikJpeg(uint8_t *chDaneSkompresowane, uint8_t chZnacznik);


#endif /* INC_JPEG_H_ */
