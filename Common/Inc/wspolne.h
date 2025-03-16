/*
 * wspolne.h
 *
 *  Created on: Mar 11, 2025
 *      Author: PitLab
 */

#ifndef INC_WSPOLNE_H_
#define INC_WSPOLNE_H_
#include "stm32h7xx_hal.h"


typedef struct
{
int16_t sX;
int16_t sY;
} stPunkt2D16_t;

typedef struct
{
int16_t sX;
int16_t sY;
int16_t sZ;
} stPunkt3D16_t;

typedef struct
{
int32_t nA;
int32_t nB;
int32_t nC;
int32_t nD;
int32_t nDlug;	//pierwiastek z sumy kwadratów ABC
} stPlas_t;	//Współczynniki równania płaszczyzny


void ObrocWektor(float *fWektorWe, float *fWektorWy, float *fKat);

#endif /* INC_WSPOLNE_H_ */
