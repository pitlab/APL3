/*
 * ili9488.h
 *
 *  Created on: Aug 21, 2025
 *      Author: PitLab
 */

#ifndef INC_LCD_ILI9488_H_
#define INC_LCD_ILI9488_H_

#include "stm32h7xx_hal.h"


#define ILI9488_TFTWIDTH  320
#define ILI9488_TFTHEIGHT 480

#define ILI9488_NOP     0x00
#define ILI9488_SWRESET 0x01
#define ILI9488_RDDID   0x04
#define ILI9488_RDDST   0x09

#define ILI9488_SLPIN   0x10
#define ILI9488_SLPOUT  0x11
#define ILI9488_PTLON   0x12
#define ILI9488_NORON   0x13

#define ILI9488_RDMODE  0x0A
#define ILI9488_RDMADCTL  0x0B
#define ILI9488_RDPIXFMT  0x0C
#define ILI9488_RDIMGFMT  0x0D
#define ILI9488_RDSELFDIAG  0x0F

#define ILI9488_INVOFF  0x20
#define ILI9488_INVON   0x21
#define ILI9488_GAMMASET 0x26
#define ILI9488_DISPOFF 0x28
#define ILI9488_DISPON  0x29

#define ILI9488_CASET   0x2A
#define ILI9488_PASET   0x2B
#define ILI9488_RAMWR   0x2C
#define ILI9488_RAMRD   0x2E

#define ILI9488_PTLAR   0x30
#define ILI9488_MADCTL  0x36
#define ILI9488_PIXFMT  0x3A

#define ILI9488_FRMCTR1 0xB1
#define ILI9488_FRMCTR2 0xB2
#define ILI9488_FRMCTR3 0xB3
#define ILI9488_INVCTR  0xB4
#define ILI9488_DFUNCTR 0xB6

#define ILI9488_PWCTR1  0xC0
#define ILI9488_PWCTR2  0xC1
#define ILI9488_PWCTR3  0xC2
#define ILI9488_PWCTR4  0xC3
#define ILI9488_PWCTR5  0xC4
#define ILI9488_VMCTR1  0xC5
#define ILI9488_VMCTR2  0xC7

#define ILI9488_RDID1   0xDA
#define ILI9488_RDID2   0xDB
#define ILI9488_RDID3   0xDC
#define ILI9488_RDID4   0xDD

#define ILI9488_GMCTRP1 0xE0
#define ILI9488_GMCTRN1 0xE1


uint8_t InicjujLCD_ILI9488(void);
void OrientacjaEkranu(uint8_t orientacja);
void WypelnijEkran(uint16_t sKolor565);
void setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void clrXY(void);
uint16_t getColor(void);
void setColor(uint16_t color);
void setColorRGB(uint8_t r, uint8_t g, uint8_t b);
uint16_t getBackColor(void);
void setBackColor(uint16_t color);
void setBackColorRGB(uint8_t r, uint8_t g, uint8_t b);
void RysujProstokatWypelniony(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t kolor);
void RysujLiniePozioma(int16_t x, int16_t y, int16_t len);
void RysujLiniePionowa(int16_t x, int16_t y, int16_t len);
void RysujLinie(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void RysujPixel(uint16_t x, uint16_t y);
void RysujBitmape(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data);
void RysujOkrag(uint16_t x, uint16_t y, uint16_t promien);
void RysujZnak(uint8_t c, uint16_t x, uint16_t y);
#endif /* INC_LCD_ILI9488_H_ */
