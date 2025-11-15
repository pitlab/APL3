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
#define LICZBA_TAGOW_IFD0	7	//EXTAG_IMAGE_DESCRIPTION + EXTAG_IMAGE_INPUT_MAKE + EXTAG_EQUIPMENT_MODEL + EXTAG_ORIENTATION + EXTAG_DATE_TIME + EXTAG_EXIF_IFD + EXTAG_GPS_IFD
#define LICZBA_TAGOW_EXIF	4	//EXTAG_TEMPERATURE + EXTAG_PRESSURE + EXTAG_CAM_ELEVATION + EXTAG_DIGITAL_ZOOM
#define LICZBA_TAGOW_GPS	11
#define ROZMIAR_TAGU		12
#define ROZMIAR_EXIF		208

//definicje typów tagów Exif
#define EXIF_TYPE_BYTE			0x01   	//BYTE
#define EXIF_TYPE_ASCII			0x02  	//ASCII
#define EXIF_TYPE_SHORT			0x03   	//SHORT
#define EXIF_TYPE_LONG			0x04   	//LONG 32-bit
#define EXIF_TYPE_RATIONAL 		0x05	//RATIONAL: 2x LONG. Pierwszy numerator, drugi denominator
#define EXIF_TYPE_SLONG			0x09  	//signed integer 32-bit
#define EXIF_TYPE_SRATIONAL 	0x0A	//STRTAIONAL: 2x SLONG

//definicje tagów
#define EXTAG_IMAGE_WIDTH		0x100
#define EXTAG_IMAGE_HEIGHT		0x101
#define EXTAG_BITS_PER_SAMPLE	0x102
#define EXTAG_IMAGE_DESCRIPTION	0x10E	//ASCII
#define EXTAG_IMAGE_INPUT_MAKE	0x10F	//ASCII
#define EXTAG_EQUIPMENT_MODEL	0x110	//ASCII
#define EXTAG_ORIENTATION		0x112
#define EXTAG_DATE_TIME			0x132
#define EXTAG_OFFSET2JPEG_SOI	0x201
#define EXTAG_BYTES_JPEG_DATA	0x202
#define EXTAG_YCBCR_SUBSAMPL	0x212

#define EXTAG_EXIF_IFD			0x8796
#define EXTAG_GPS_IFD			0x8825

//definicje tagów Exif
#define EXTAG_EXPOSURE_TIME		0x829A	//RATIONAL
#define EXTAG_EXPOSURE_MODE		0xA402	//SHORT
#define EXTAG_DIGITAL_ZOOM		0xA404	//RATIONAL
#define EXTAG_GAIN_CONTROL		0xA407	//RATIONAL
#define EXTAG_CONTRAST			0xA408	//SHORT
#define EXTAG_SATURATION		0xA409	//SHORT

#define EXTAG_TEMPERATURE		0x9400	//SRATIONAL x1
#define EXTAG_PRESSURE			0x9402	//RATIONAL x1
#define EXTAG_CAM_ELEVATION		0x9405	//SRATIONAL x1
#define EXTAG_BODY_SERIAL_NUM	0xA435	//ASCII

//definicje tagów GPS
#define EXTAG_GPS_TAG_VERSION	0x00	//BYTE x4
#define EXTAG_GPS_NS_LATI_REF	0x01	//ASII x2
#define EXTAG_GPS_LATITUDE		0x02	//RATIONAL x3
#define EXTAG_GPS_EW_LONGI_REF	0x03	//ASCII x2
#define EXTAG_GPS_LONGITUDE		0x04	//RATIONAL x3
#define EXTAG_GPS_ALTITUDE_REF	0x05	//BYTE x1
#define EXTAG_GPS_ALTITUDE		0x06	//RATIONAL x1
#define EXTAG_GPS_TIME_STAMP	0x07	//RATIONAL x3
#define EXTAG_GPS_SATS			0x08	//ASCII
#define EXTAG_GPS_STATUS		0x09	//ASCII
#define EXTAG_GPS_SPEED_REF		0x0C	//ASCII
#define EXTAG_GPS_SPEED			0x0D	//RATIONAL x1
#define EXTAG_GPS_DATE_STAMP	0x1D	//ASCII x11

uint8_t InicjalizujJpeg(void);
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chTypKoloru, uint8_t chTypChrominancji, uint8_t chJakoscObrazu);
uint8_t CzekajNaKoniecPracyJPEG(void);
uint8_t KompresujY8(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujYUV444(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujYUV420(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujRGB888(uint8_t *obrazRGB888, uint8_t *buforYCbCr, uint8_t *chDaneSkompresowane, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t* ZnajdzZnacznikJpeg(uint8_t *chDaneSkompresowane, uint8_t chZnacznik);
uint32_t PrzygotujExif(stKonfKam_t *stKonf, volatile stWymianyCM4_t *stDane, RTC_DateTypeDef stData, RTC_TimeTypeDef stCzas);
//void PrzygotujTag(uint8_t** chWskTaga, uint16_t sTagID, uint16_t sTyp, uint8_t *chDane, uint8_t chRozmiar, uint8_t** chWskDanych);
void PrzygotujTag(uint8_t** chWskTaga, uint16_t sTagID, uint16_t sTyp, uint8_t *chDane, uint32_t nRozmiar, uint8_t** chWskDanych, uint8_t *chPoczatekTIFF);
#endif /* INC_JPEG_H_ */
