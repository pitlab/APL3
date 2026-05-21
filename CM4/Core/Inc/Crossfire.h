/*
 * Crossfire.h
 *
 *  Created on: 21 maj 2026
 *      Author: PitLab
 */

#ifndef INC_CROSSFIRE_H_
#define INC_CROSSFIRE_H_

#include "SysDefCM4.h"
#include "wymiana.h"

uint8_t InicjujCrossfire(void);
uint8_t AnalizujSygnalCrossfire(stWymianyCM4_t *psDaneCM4, stWymianyCM7_t *psDaneCM7);


#endif /* INC_CROSSFIRE_H_ */
