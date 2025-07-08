/*
 * adc.h
 *
 *  Created on: Jul 8, 2025
 *      Author: PitLab
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "sys_def_CM4.h"


#define VREF	3.0f	//napiecie referencyjne przetwornika a/c


uint8_t InicjujADC(void);
uint8_t PomiarADC(uint8_t chKanal);

#endif /* INC_ADC_H_ */
