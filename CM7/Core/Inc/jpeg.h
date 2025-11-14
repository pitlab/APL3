/*
 * jpeg.h
 *
 *  Created on: Sep 19, 2025
 *      Author: PitLab
 */

#ifndef INC_JPEG_H_
#define INC_JPEG_H_

#include "sys_def_CM7.h"
#include "kamera.h"
#include "wymiana.h"

#define ROZM_BUF_WY_JPEG	8192	//16 sektorów, 1 jednostka alokacji FAT32
//#define ROZM_BUF_WY_JPEG	1024	//2 sektory
#define ILOSC_BUF_JPEG		4		//buforowanie x4
#define MASKA_LICZBY_BUF	0x03	//maska do przycinania wskaźnika buforów
#define SZEROKOSC_BLOKU		8
#define WYSOKOSC_BLOKU		8
#define ROZMIAR_BLOKU		(SZEROKOSC_BLOKU * WYSOKOSC_BLOKU)
#define ROZMIAR_MCU422		(4 * ROZMIAR_BLOKU)
#define ROZMIAR_MCU420		(6 * ROZMIAR_BLOKU)
#define ILOSC_BUF_WE_MCU	12		//liczba podzielna przez 6 (UVY420), przez 4(UVY422), przez 3 (UVY444) i przez 1 (Y8)
//#define MASKA_BUF_MCU		0x07	//maska do przycinania wskaźnika buforów

//flagi ustawiane w calbackach określające stan enkodera
#define KOMPR_PUSTE_WE		1	//na wejście enkodera można podać nowe dane
#define KOMPR_MCU_GOTOWY	2	//podany MCU jest już skompresowany
#define KOMPR_OBR_GOTOWY	4	//podany obraz jest już skompresowany

#define ROZMIAR_NAGL_JPEG	20
#define ROZMIAR_ZNACZ_xOI	2
#define LICZBA_TAGOW_EXIF	10
#define ROZMIAR_TAGU_EXIF	12
#define ROZMIAR_EXIF		200

//definicje typów tagów Exif
#define EXIF_TYPE_BYTE			0x01   	//BYTE
#define EXIF_TYPE_ASCII			0x02  	//ASCII
#define EXIF_TYPE_SHORT			0x03   	//SHORT
#define EXIF_TYPE_LONG			0x04   	//LONG
#define EXIF_TYPE_RATIONAL 		0x05	//RATIONAL: 2x LONG. Pierwszy numerator, drugi denominator

//definicje tagów Exif
#define EXTAG_IMAGE_WIDTH		0x100
#define EXTAG_IMAGE_HEIGHT		0x101
#define EXTAG_BITS_PER_SAMPLE	0x102
#define EXTAG_IMAGE_INPUT_MANUF	0x10F
#define EXTAG_EQUIPMENT_MODEL	0x110
#define EXTAG_ORIENTATION		0x112
#define EXTAG_DATE_TIME			0x132
#define EXTAG_OFFSET2JPEG_SOI	0x201
#define EXTAG_BYTES_JPEG_DATA	0x202
#define EXTAG_YCBCR_SUBSAMPL	0x212



uint8_t InicjalizujJpeg(void);
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chTypKoloru, uint8_t chTypChrominancji, uint8_t chJakoscObrazu);
uint8_t CzekajNaKoniecPracyJPEG(void);
uint8_t KompresujY8(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujYUV444(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujYUV420(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujRGB888(uint8_t *obrazRGB888, uint8_t *buforYCbCr, uint8_t *chDaneSkompresowane, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t* ZnajdzZnacznikJpeg(uint8_t *chDaneSkompresowane, uint8_t chZnacznik);
uint32_t PrzygotujExif(stKonfKam_t *stKonf, volatile stWymianyCM4_t *stDane, RTC_DateTypeDef stData, RTC_TimeTypeDef stCzas);
void PrzygotujTag(uint8_t** chWskTaga, uint16_t sTagID, uint16_t sTyp, uint8_t *chDane, uint8_t chRozmiar, uint8_t** chWskDanych);
#endif /* INC_JPEG_H_ */
