/*
 * dotyk.h
 *
 *  Created on: Nov 17, 2024
 *      Author: PitLab
 */

#ifndef INC_DOTYK_H_
#define INC_DOTYK_H_
#include "stm32h7xx_hal.h"



//definicje kanałów przetwornika
#define TPCHX	5
#define TPCHY	1
#define TPCHZ1	3
#define TPCHZ2	4

uint16_t CzytajKanalDotyku(uint8_t  chKanal);
void CzytajDotyk(void);

#endif /* INC_DOTYK_H_ */
