/*
 * OSD.h
 *
 *  Created on: Oct 28, 2025
 *      Author: PitLab
 */

#ifndef INC_OSD_H_
#define INC_OSD_H_
#include "sys_def_CM7.h"


uint8_t InicjujOSD(void);
void RysujOSD();
uint8_t PolaczBuforOSDzObrazem(uint8_t *chObrazKamery, uint8_t *chBuforOSD, uint8_t *chBuforWyjsciowy, uint16_t sSzerokosc, uint16_t sWysokosc);



#endif /* INC_OSD_H_ */
