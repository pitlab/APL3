/*
 * RPi35B_480x320.h
 *
 *  Created on: Nov 16, 2024
 *      Author: PitLab
 */

#ifndef RPI35B_480X320_H_
#define RPI35B_480X320_H_
#include "stm32h7xx_hal.h"

uint8_t InicjujLCD_35B_16bit(void);
uint8_t InicjujLCD_35C_16bit(void);
uint8_t InicjujLCD_35C_8bit(void);
void OrientacjaEkranu(uint8_t orient);
void WypelnijEkran(uint16_t sKolor565);
void setColorRGB(uint8_t r, uint8_t g, uint8_t b);
void setColor(uint16_t color);
uint16_t getColor(void);
void setBackColorRGB(uint8_t r, uint8_t g, uint8_t b);
void setBackColor(uint16_t color);
uint16_t getBackColor(void);
//void LCD_rect(uint16_t col, uint16_t row, uint16_t width, uint16_t height, uint16_t color);
void RysujProstokatWypelniony(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t kolor);
void RysujLiniePozioma(int16_t x, int16_t y, int16_t len);
void RysujLiniePionowa(int16_t x, int16_t y, int16_t len);
void RysujLinie(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void clrXY(void);
void drawPixel(uint16_t x, uint16_t y);
void RysujZnak(uint8_t c, uint16_t x, uint16_t y);
//void fillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

//void drawCircle(uint16_t x, uint16_t y, uint16_t radius);
void RysujOkrag(uint16_t x, uint16_t y, uint16_t promien);
void fillCircle(uint16_t x, uint16_t y, uint16_t radius);
void RysujBitmape(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data);
void drawBitmap2(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, uint16_t* data);
void drawBitmap3(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data);
void drawBitmap4(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data);
void CzytajPamiecObrazu(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t* bufor);
#endif /* RPI35B_480X320_H_ */



