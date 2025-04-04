/*
 * jedn_inercyjna.h
 *
 *  Created on: Dec 29, 2024
 *      Author: PitLab
 */

#ifndef INC_JEDN_INERCYJNA_H_
#define INC_JEDN_INERCYJNA_H_

#include "sys_def_CM4.h"



//struktura danych do przechowywania parametrów do kalibracji magnetometru
typedef struct
{
	float fMin[3];
	float fMax[3];
} magn_t;

uint8_t InicjujJednostkeInercyjna(void);
void ObliczeniaJednostkiInercujnej(uint8_t chGniazdo);
void ObrocWektor(float *fWektor);
void ZapiszOffsetMagnetometru(uint8_t chMagn);
void KalibracjaZeraMagnetometru(float *fMag);
#endif /* INC_JEDN_INERCYJNA_H_ */
