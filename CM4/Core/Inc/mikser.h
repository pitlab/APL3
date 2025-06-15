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
	uint8_t chSilnikow;					//liczba sterowanych silnikow napedowych
	float fPrze[KANALY_MIKSERA];		//składowa przechylenia na każdy silnik
	float fPoch[KANALY_MIKSERA];		//składowa pochylenia na każdy silnik
	float fOdch[KANALY_MIKSERA];		//składowa odchylenia na każdy silnik

}stMikser_t;


//uint8_t InicjujMikser(stMikser_t* mikser);
uint8_t InicjujMikser(void);
uint8_t LiczMikser(stMikser_t* mikser, stWymianyCM4_t *dane);


#endif /* INC_MIKSER_H_ */
