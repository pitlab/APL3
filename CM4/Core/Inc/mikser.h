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
	float fPrze;		//składowa przechylenia na każdy silnik
	float fPoch;		//składowa pochylenia na każdy silnik
	float fOdch;		//składowa odchylenia na każdy silnik

}stMikser_t;


//uint8_t InicjujMikser(stMikser_t* mikser);
uint8_t InicjujMikser(void);
uint8_t LiczMikser(stMikser_t *mikser, stWymianyCM4_t *dane, stKonfPID_t *konfig);


#endif /* INC_MIKSER_H_ */
