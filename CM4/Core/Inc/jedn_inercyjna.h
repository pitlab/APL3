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
	int16_t sMin[3];
	int16_t sMax[3];
} magn_t;

void InicjujJednostkeInercyjna(void);
void ObliczeniaJednostkiInercujnej(uint8_t chGniazdo);
void ObrocWektor(float *fWektor);
void KalibracjaZeraMagnetometru(int16_t *sMag);

#endif /* INC_JEDN_INERCYJNA_H_ */
