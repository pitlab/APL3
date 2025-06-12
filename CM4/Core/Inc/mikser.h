/*
 * mikser.h
 *
 *  Created on: Jun 12, 2025
 *      Author: PitLab
 */

#include "pid_kanaly.h"
#include "sys_def_CM4.h"
#include "wymiana.h"

#ifndef INC_MIKSER_H_
#define INC_MIKSER_H_

//struktura danych miksera
typedef struct
{
	float fPrze;
	float fPoch;
	float fOdch;
}stMikser_t;


uint8_t InicjujMikser(stMikser_t* mikser);
uint8_t LiczMikser(stMikser_t* mikser, stWymianyCM4_t *dane);


#endif /* INC_MIKSER_H_ */
