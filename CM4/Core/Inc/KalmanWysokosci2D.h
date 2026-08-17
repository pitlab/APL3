/*
 * KalmanWysokosci.h
 *
 *  Created on: 13 sie 2026
 *      Author: PitLab
 */

#ifndef INC_KALMANWYSOKOSCI2D_H_
#define INC_KALMANWYSOKOSCI2D_H_

#include "SysDefCM4.h"
#include "wymiana.h"
#include "arm_math.h"


uint8_t InicjujFiltrKalmanaWysokości2D(void);
uint8_t PredykcjaFiltraKalmanaWysokości2D(stWymianyCM4_t *dane);
uint8_t AktulizacjaFiltraKalmanaWysokości2D(stWymianyCM4_t *dane);
#endif /* INC_KALMANWYSOKOSCI2D_H_ */
