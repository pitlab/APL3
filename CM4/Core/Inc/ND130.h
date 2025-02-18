/*
 * ND130.h
 *
 *  Created on: Feb 16, 2025
 *      Author: PitLab
 */

#ifndef INC_ND130_H_
#define INC_ND130_H_
#include "sys_def_CM4.h"


uint8_t InicjujND130(void);
uint8_t ObslugaND130(void);
float PredkoscRurkiPrantla(float fCisnRozn, float fCisnStat);
float PredkoscRurkiPrantla1(float fCisnRozn);

#endif /* INC_ND130_H_ */
