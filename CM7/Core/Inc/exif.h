/*
 * exif.h
 *
 *  Created on: 3 gru 2025
 *      Author: PitLab
 */

#ifndef INC_EXIF_H_
#define INC_EXIF_H_

#include "sys_def_CM7.h"
#include "kamera.h"
#include "wymiana.h"



#define LICZBA_TAGOW_IFD0	10	//EXTAG_IMAGE_DESCRIPTION + EXTAG_IMAGE_INPUT_MAKE + EXTAG_EQUIPMENT_MODEL + EXTAG_SOFTWARE + EXTAG_ARTIST + EXTAG_ORIENTATION + EXTAG_DATE_TIME + EXTAG_COPYRIGT + EXTAG_EXIF_IFD + EXTAG_GPS_IFD
#define LICZBA_TAGOW_EXIF	4	//EXTAG_TEMPERATURE + EXTAG_PRESSURE + EXTAG_CAM_ELEVATION + EXTAG_DIGITAL_ZOOM
#define LICZBA_TAGOW_GPS	10
#define ROZMIAR_TAGU		12
#define ROZMIAR_EXIF		560		//rozmiar wyrównany tak aby kolena sekcja jpeg zaczynała się od adresu będącego wielokrotnością 4
#define ROZMIAR_INTEROPER	2

//definicje typów tagów Exif
#define EXIF_TYPE_BYTE			0x01   	//BYTE
#define EXIF_TYPE_ASCII			0x02  	//ASCII
#define EXIF_TYPE_SHORT			0x03   	//SHORT
#define EXIF_TYPE_LONG			0x04   	//LONG 32-bit
#define EXIF_TYPE_RATIONAL 		0x05	//RATIONAL: 2x LONG. Pierwszy numerator, drugi denominator
#define EXIF_TYPE_SLONG			0x09  	//Signed LONG 32-bit
#define EXIF_TYPE_SRATIONAL 	0x0A	//Signed RATIONAL: 2x SLONG

//definicje tagów
#define EXTAG_IMAGE_WIDTH		0x100
#define EXTAG_IMAGE_HEIGHT		0x101
#define EXTAG_BITS_PER_SAMPLE	0x102
#define EXTAG_IMAGE_DESCRIPTION	0x10E	//ASCII
#define EXTAG_IMAGE_INPUT_MAKE	0x10F	//ASCII
#define EXTAG_EQUIPMENT_MODEL	0x110	//ASCII
#define EXTAG_SOFTWARE			0x131	//ASCII
#define EXTAG_ARTIST			0x13B	//ASCII
#define EXTAG_COPYRIGT			0x8298	//ASCII
#define EXTAG_ORIENTATION		0x112
#define EXTAG_DATE_TIME			0x132
#define EXTAG_OFFSET2JPEG_SOI	0x201
#define EXTAG_BYTES_JPEG_DATA	0x202
#define EXTAG_YCBCR_SUBSAMPL	0x212

#define EXTAG_EXIF_IFD			0x8769
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

uint32_t PrzygotujExif(JPEG_ConfTypeDef *stKonfJpeg, stKonfKam_t *stKonfKam, volatile stWymianyCM4_t *stDane, RTC_DateTypeDef *stData, RTC_TimeTypeDef *stCzas);
void PrzygotujTag(uint8_t** chWskTaga, uint16_t sTagID, uint16_t sTyp, uint8_t *chDane, uint32_t nRozmiar, uint8_t** chWskDanych, uint8_t *chPoczatekTIFF);


#endif /* INC_EXIF_H_ */
