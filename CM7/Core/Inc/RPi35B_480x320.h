/*
 * RPi35B_480x320.h
 *
 *  Created on: Nov 16, 2024
 *      Author: PitLab
 */

#ifndef RPI35B_480X320_H_
#define RPI35B_480X320_H_
#include "stm32h7xx_hal.h"
uint8_t LCD_write_command8(uint8_t chDane);
uint8_t LCD_write_command16(uint8_t chDane);
uint8_t LCD_WrData(uint8_t* chDane, uint8_t chIlosc);
uint8_t LCD_WrDataDMA(uint8_t* chDane, uint16_t sIlosc);
uint8_t LCD_write_data16(uint8_t chDane1, uint8_t chDane2);
uint8_t LCD_write_dat_jed8(uint8_t chDane);
uint8_t LCD_write_dat_jed16(uint8_t chDane);
uint8_t LCD_write_dat_pie8(uint8_t chDane);
uint8_t LCD_write_dat_pie16(uint8_t chDane);
void LCD_write_dat_sro8(uint8_t chDane);
void LCD_write_dat_sro16(uint8_t chDane);
void LCD_write_dat_ost8(uint8_t chDane);
void LCD_write_dat_ost16(uint8_t chDane);
void LCD_data_read(uint8_t *chDane, uint8_t chIlosc);
uint8_t InicjujLCD_35B_16bit(void);
uint8_t InicjujLCD_35C_16bit(void);
void LCD_Orient(uint8_t orient);
void LCD_clear(uint16_t kolor);
//void LCD_rect(uint16_t col, uint16_t row, uint16_t width, uint16_t height, uint16_t color);
void LCD_ProstokatWypelniony(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t kolor);
void drawHLine(int16_t x, int16_t y, int16_t len);
void drawVLine(int16_t x, int16_t y, int16_t len);
void setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void clrXY(void);
void setColorRGB(uint8_t r, uint8_t g, uint8_t b);
void setColor(uint16_t color);
uint16_t getColor(void);
void setBackColorRGB(uint8_t r, uint8_t g, uint8_t b);
void setBackColor(uint16_t color);
uint16_t getBackColor(void);
void drawPixel(uint16_t x, uint16_t y);
void drawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void drawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
//void fillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void drawRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void printChar(uint8_t c, uint16_t x, uint16_t y);
void setFont(uint8_t* font);
uint8_t GetFontX(void);
uint8_t GetFontY(void);
void print(char *st, uint16_t x, uint16_t y);
void printRamka(char *str, uint16_t x, uint16_t y, uint16_t sx, uint16_t sy);
void drawCircle(uint16_t x, uint16_t y, uint16_t radius);
void fillCircle(uint16_t x, uint16_t y, uint16_t radius);
void drawBitmap(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data);
void drawBitmap2(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, uint16_t* data);
void drawBitmap3(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data);
void drawBitmap4(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data);
void CzytajPamiecObrazu(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t* bufor);
#endif /* RPI35B_480X320_H_ */



