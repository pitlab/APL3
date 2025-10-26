

#ifndef INC_LCD_MEM_H_
#define INC_LCD_MEM_H_
#include "stm32h7xx_hal.h"


void WypelnijBuforEkranu(uint8_t *chBufor, uint32_t nKolorARGB);
void RysujProstokatWypelnionywBuforze(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *chBufor, uint32_t nKolorARGB);

#endif /* INC_LCD_MEM_H_ */
