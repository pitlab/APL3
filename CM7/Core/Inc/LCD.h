/*
 * LCD.h
 *
 *  Created on: Nov 16, 2024
 *      Author: PitLab
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_
#include "stm32h7xx_hal.h"



void RysujEkran(void);
void FraktalTest(unsigned char chTyp);
void FraktalDemo(void);
void GenerateJulia(unsigned short size_x, unsigned short size_y, unsigned short offset_x, unsigned short offset_y, unsigned short zoom, unsigned short * buffer);
void GenerateMandelbrot(float centre_X, float centre_Y, float Zoom, unsigned short IterationMax, unsigned short * buffer);
void HSV2RGB(float hue, float sat, float val, float *red, float *grn, float *blu);
unsigned int MinalCzas(unsigned int nStart);
void InitFraktal(unsigned char chTyp);

#endif /* INC_LCD_H_ */
