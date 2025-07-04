/*
 * fraktale.h
 *
 *  Created on: Jun 1, 2025
 *      Author: PitLab
 */

#ifndef INC_FRAKTALE_H_
#define INC_FRAKTALE_H_
#include "sys_def_CM7.h"
#include "display.h"


void InitFraktal(unsigned char chTyp);
void FraktalDemo(void);
void FraktalTest(unsigned char chTyp);
void GenerateJulia(unsigned short size_x, unsigned short size_y, unsigned short offset_x, unsigned short offset_y, unsigned short zoom, unsigned short * buffer);
void GenerateMandelbrot(float centre_X, float centre_Y, float Zoom, unsigned short IterationMax, unsigned short * buffer);

#endif /* INC_FRAKTALE_H_ */
