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
uint8_t PolaczBuforOSDzObrazem(uint8_t *chObrazFront, uint8_t *chObrazTlo, uint8_t *chBuforWyjsciowy, uint16_t sSzerokosc, uint16_t sWysokosc);
void XferCpltCallback(DMA2D_HandleTypeDef *hdma2d);
void XferErrorCallback(DMA2D_HandleTypeDef *hdma2d);

#endif /* INC_OSD_H_ */
