/*
 * wymiana_miedzyrdzeniowa_CM7.h
 *
 *  Created on: Dec 5, 2024
 *      Author: PitLab
 */

#ifndef INC_WYMIANA_CM7_H_
#define INC_WYMIANA_CM7_H_

#include "wymiana.h"

#define ROZMIAR_BUF_NAPISOW_CM4		4*ROZMIAR_BUF_NAPISU_WYMIANY


uint8_t PobierzDaneWymiany_CM4(void);
uint8_t UstawDaneWymiany_CM7(void);

#endif /* INC_WYMIANA_CM7_H_ */
