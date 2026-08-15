/*
 * KalmanWysokosci4D.h
 *
 *  Created on: 15 sie 2026
 *      Author: PitLab
 */

#ifndef INC_KALMANWYSOKOSCI4D_H_
#define INC_KALMANWYSOKOSCI4D_H_

#include "SysDefCM4.h"
#include "wymiana.h"
#include "arm_math.h"

#define WARIANCJA_ZRYWU_ACEL	6.0e-6f
#define WARIANCJA_DRYFTU_ACEL 	1.0e-6f;

uint8_t InicjujFiltrKalmanaWysokości4D(void);
uint8_t PredykcjaFiltraKalmanaWysokości4D(stWymianyCM4_t *dane);
uint8_t AktulizacjaWysokościiPrzyspieszeniaFiltraKalmanaWysokości4D(stWymianyCM4_t *dane);
uint8_t AktulizacjaPrzyspieszeniaFiltraKalmanaWysokości4D(stWymianyCM4_t *dane);

#endif /* INC_KALMANWYSOKOSCI4D_H_ */
