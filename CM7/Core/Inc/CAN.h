/*
 * CAN.h
 *
 *  Created on: May 6, 2025
 *      Author: PitLab
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_
#include "sys_def_CM7.h"


#define ID_MAGNETOMETR	210

void InicjujCAN(void);
uint8_t TestCanTx(void);
void FormatujMag2Can(float fMag, uint8_t* chWynik);

#endif /* INC_CAN_H_ */
