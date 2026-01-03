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

#define	START_FRAKTAL	1

void InitFraktal(unsigned char chTyp);
void FraktalDemo(void);
void FraktalTest(unsigned char chTyp);
//void GenerateJulia(uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t sZoom, float fZespRzecz, float fZespUroj, uint8_t chPaleta, uint8_t* chBufor);
void GenerateJulia(uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t sZoom, float fZespRzecz, float fZespUroj, uint16_t sPaleta, uint8_t* chBufor);
//void GenerateMandelbrot(float fSrodekX, float fSrodekY, float Zoom, uint16_t sMaxIteracji, uint8_t chPaleta, uint8_t* chBufor);
void GenerateMandelbrot(float fSrodekX, float fSrodekY, float Zoom, uint16_t sMaxIteracji, uint16_t sPaleta, uint8_t* chBufor);

#endif /* INC_FRAKTALE_H_ */
