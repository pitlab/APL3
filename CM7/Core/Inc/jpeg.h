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
#define ROZMIAR_MCU_Y8		ROZMIAR_BLOKU
#define ROZMIAR_MCU422		(4 * ROZMIAR_BLOKU)
#define ROZMIAR_MCU420		(6 * ROZMIAR_BLOKU)
#define ILOSC_BUF_WE_MCU	24		//liczba podzielna przez 2*6 (UVY420), przez 2*4(UVY422), przez 2*3 (UVY444) i przez 2*1 (Y8) - podwójne buforowane
#define LICZBA_KOLOR_RGB888	3		//liczba bajtów koloru

//flagi ustawiane w calbackach określające stan enkodera
#define KOMPR_PUSTE_WE		1	//na wejście enkodera można podać nowe dane
#define KOMPR_MCU_GOTOWY	2	//podany MCU jest już skompresowany
#define KOMPR_OBR_GOTOWY	4	//podany obraz jest już skompresowany

#define ROZMIAR_NAGL_JPEG	20
#define ROZMIAR_ZNACZ_xOI	2
#define LICZBA_TAGOW_IFD0	10	//EXTAG_IMAGE_DESCRIPTION + EXTAG_IMAGE_INPUT_MAKE + EXTAG_EQUIPMENT_MODEL + EXTAG_SOFTWARE + EXTAG_ARTIST + EXTAG_ORIENTATION + EXTAG_DATE_TIME + EXTAG_COPYRIGT + EXTAG_EXIF_IFD + EXTAG_GPS_IFD
#define LICZBA_TAGOW_EXIF	4	//EXTAG_TEMPERATURE + EXTAG_PRESSURE + EXTAG_CAM_ELEVATION + EXTAG_DIGITAL_ZOOM
#define LICZBA_TAGOW_GPS	10
#define ROZMIAR_TAGU		12
#define ROZMIAR_EXIF		560		//rozmiar wyrównany tak aby kolena sekcja jpeg zaczynała się od adresu będącego wielokrotnością 4
#define ROZMIAR_INTEROPER	2


//z przykładu
#define JPEG_BYTES_PER_PIXEL     3
#define JPEG_GREEN_OFFSET        8       /* Offset of the GREEN color in a pixel         */
#define YCBCR_422_BLOCK_SIZE       256


#if (JPEG_SWAP_RB == 0)
#define JPEG_RED_OFFSET        11      /* Offset of the RED color in a pixel           */
#define JPEG_BLUE_OFFSET       0       /* Offset of the BLUE color in a pixel          */
#define JPEG_RGB565_RED_MASK   0xF800  /* Mask of Red component in RGB565 Format       */
#define JPEG_RGB565_BLUE_MASK  0x001F  /* Mask of Blue component in RGB565 Format      */
#else
#define JPEG_RED_OFFSET        0       /* Offset of the RED color in a pixel           */
#define JPEG_BLUE_OFFSET       11      /* Offset of the BLUE color in a pixel          */
#define JPEG_RGB565_RED_MASK   0x001F  /* Mask of Red component in RGB565 Format       */
#define JPEG_RGB565_BLUE_MASK  0xF800  /* Mask of Blue component in RGB565 Format      */
#endif /* JPEG_SWAP_RB */

typedef struct __JPEG_MCU_RGB_ConvertorTypeDef
{
  uint32_t ColorSpace;
  uint32_t ChromaSubsampling;

  uint32_t ImageWidth;
  uint32_t ImageHeight;
  uint32_t ImageSize_Bytes;

  uint32_t LineOffset;
  uint32_t BlockSize;

  uint32_t H_factor;
  uint32_t V_factor;

  uint32_t WidthExtend;
  uint32_t ScaledWidth;

  uint32_t MCU_Total_Nb;

  uint16_t *Y_MCU_LUT;
  uint16_t *Cb_MCU_LUT;
  uint16_t *Cr_MCU_LUT;
  uint16_t *K_MCU_LUT;

}JPEG_MCU_RGB_ConvertorTypeDef;


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



uint8_t InicjalizujJpeg(void);
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chTypKoloru, uint8_t chTypChrominancji, uint8_t chJakoscObrazu);
uint8_t CzekajNaKoniecPracyJPEG(void);
uint8_t KompresujY8(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujYUV444(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujYUV420(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujRGB888doYUV444(uint8_t *obrazRGB888, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujRGB888doYUV422(uint8_t *obrazRGB888, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujRGB888doY8(uint8_t *obrazRGB888, uint16_t sSzerokosc, uint16_t sWysokosc);	//testowo
uint8_t TestKonwersjiRGB888doYCbCr(uint8_t *obrazRGB888, uint8_t *buforYCbCr, uint16_t sSzerokosc, uint16_t sWysokosc);


uint8_t* ZnajdzZnacznikJpeg(uint8_t *chDaneSkompresowane, uint8_t chZnacznik);
uint32_t PrzygotujExif(stKonfKam_t *stKonf, volatile stWymianyCM4_t *stDane, RTC_DateTypeDef stData, RTC_TimeTypeDef stCzas);
//void PrzygotujTag(uint8_t** chWskTaga, uint16_t sTagID, uint16_t sTyp, uint8_t *chDane, uint8_t chRozmiar, uint8_t** chWskDanych);
void PrzygotujTag(uint8_t** chWskTaga, uint16_t sTagID, uint16_t sTyp, uint8_t *chDane, uint32_t nRozmiar, uint8_t** chWskDanych, uint8_t *chPoczatekTIFF);

uint8_t KompresujPrzyklad(uint8_t *obrazRGB888,  uint8_t *chMCU, uint16_t sSzerokosc, uint16_t sWysokosc);
void JPEG_Init_MCU_LUT(void);
uint32_t JPEG_ARGB_MCU_YCbCr422_ConvertBlocks (uint8_t *pInBuffer, uint8_t *pOutBuffer, uint32_t BlockIndex, uint32_t DataCount, uint32_t *ConvertedDataCount);
//HAL_StatusTypeDef JPEG_GetEncodeColorConvertFunc(JPEG_ConfTypeDef *pJpegInfo, JPEG_RGBToYCbCr_Convert_Function *pFunction, uint32_t *ImageNbMCUs);
//static uint32_t JPEG_ARGB_MCU_YCbCr422_ConvertBlocks (uint8_t *pInBuffer, uint8_t *pOutBuffer, uint32_t BlockIndex, uint32_t DataCount, uint32_t *ConvertedDataCount);
#endif /* INC_JPEG_H_ */
