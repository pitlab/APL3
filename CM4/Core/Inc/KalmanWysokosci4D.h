/*
 * KalmanWysokosci4D.h
 *
 *  Created on: 15 sie 2026
 *      Author: PitLab
 */

#ifndef INC_KALMANWYSOKOSCI4D_H_
#define INC_KALMANWYSOKOSCI4D_H_

#include "SysDefCM4.h"
#include "wymiana.h"
#include "arm_math.h"

//#define WARIANCJA_ZRYWU_ACEL	6.0e-5f
//#define WARIANCJA_ZRYWU_ACEL	3.0e-4f		//wyraźnie lepiej
#define WARIANCJA_ZRYWU_ACEL	3.0e-3f		//
//#define WARIANCJA_DRYFTU_ACEL 	1.0e-8f;
//#define WARIANCJA_DRYFTU_ACEL 	1.0e-6f;	//widoczne w X[3] skoki przyspieszenia na poczatku i końcu ruchu
//#define WARIANCJA_DRYFTU_ACEL 	1.0e-2f;	//widoczny w X[3] szum pomiaru
#define WARIANCJA_DRYFTU_ACEL 	1.0e-7f;

#define LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI		128

uint8_t InicjujFiltrKalmanaWysokości4D(stWymianyCM4_t *dane);
uint8_t PredykcjaFiltraKalmanaWysokości4D(stWymianyCM4_t *dane);
uint8_t AktulizacjaWysokościiPrzyspieszeniaFiltraKalmanaWysokości4D(stWymianyCM4_t *dane);
uint8_t AktulizacjaPrzyspieszeniaFiltraKalmanaWysokości4D(stWymianyCM4_t *dane);

#endif /* INC_KALMANWYSOKOSCI4D_H_ */
