/*
 * konfiguracja.h
 *
 *  Created on: Jun 4, 2024
 *      Author: PitLab
 */

#ifndef INC_KONFIGURACJA_H_
#define INC_KONFIGURACJA_H_

#include "sys_def_CM7.h"

//konfiguracja kamery
struct st_KonfKam
{
	uint16_t sSzerWe;
	uint16_t sWysWe;
	uint16_t sSzerWy;
	uint16_t sWysWy;
	uint8_t chTrybDiagn;
	uint8_t chFlagi;
};

//typedef struct st_KonfKam typKonfKam;
//extern struct st_KonfKam strKonfKam;


#endif /* INC_KONFIGURACJA_H_ */
