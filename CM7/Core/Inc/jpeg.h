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
#define ROZMIAR_MCU_Y8		ROZMIAR_BLOKU
#define ROZMIAR_MCU422		(4 * ROZMIAR_BLOKU)
#define ROZMIAR_MCU420		(6 * ROZMIAR_BLOKU)
#define ILOSC_BUF_WE_MCU	24		//liczba podzielna przez 2*6 (UVY420), przez 2*4(UVY422), przez 2*3 (UVY444) i przez 2*1 (Y8) - podwójne buforowane
#define LICZBA_KOLOR_RGB888	3		//liczba bajtów koloru

//flagi ustawiane w calbackach określające stan enkodera
//#define KOMPR_PUSTE_WE		1	//na wejście enkodera można podać nowe dane
//#define KOMPR_OBR_GOTOWY	4	//podany obraz jest już skompresowany
#define ROZMIAR_NAGL_JPEG	20
#define ROZMIAR_ZNACZ_xOI	2


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

/*typedef struct __JPEG_MCU_RGB_ConvertorTypeDef
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
*/




uint8_t InicjalizujJpeg(void);
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chTypKoloru, uint8_t chTypChrominancji, uint8_t chJakoscObrazu);
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
