/*
 * KalmanWysokosci4D.h
 *
 *  Created on: 15 sie 2026
 *      Author: PitLab
 */

#ifndef INC_KALMANWYSOKOSCI4X2Z_H_
#define INC_KALMANWYSOKOSCI4X2Z_H_

#include "SysDefCM4.h"
#include "wymiana.h"
#include "arm_math.h"

//#define WARIANCJA_ZRYWU_ACEL	6.0e-5f
//#define WARIANCJA_ZRYWU_ACEL	3.0e-4f		//wyraźnie lepiej
//#define WARIANCJA_ZRYWU_ACEL	3.0e-3f		//mniejsze przeregulowanie na estymacji prędkosci i przyspieszenia
//#define WARIANCJA_ZRYWU_ACEL	3.0e-2f		//mniejsze przeregulowanie na estymacji prędkosci i przyspieszenia
#define WARIANCJA_ZRYWU_ACEL	5.0e-1f		//
//#define WARIANCJA_DRYFTU_ACEL 	1.0e-8f;
//#define WARIANCJA_DRYFTU_ACEL 	1.0e-6f;	//widoczne w X[3] skoki przyspieszenia na poczatku i końcu ruchu
//#define WARIANCJA_DRYFTU_ACEL 	1.0e-2f;	//widoczny w X[3] szum pomiaru, ząbki przyspieszenia windy na poziomie 0,8
//#define WARIANCJA_DRYFTU_ACEL 	1.0e-7f;	//widoczne w X[3] ząbki od przyspieszenia windy na poziomie 0,02
//#define WARIANCJA_DRYFTU_ACEL 	1.0e-9f;	//widoczne w X[3] ząbki od przyspieszenia windy na poziomie 0,006
//#define WARIANCJA_DRYFTU_ACEL 	1.0e-10f;	//nie widać ząbków przyspieszenia, pojawiła się szpilka na jednym zboczu
#define WARIANCJA_DRYFTU_ACEL 	5.0e-9f;		//jest dobrze. Widać tylko dryft, bez elementów dynamiki przy wariancji zrywu 5e-1

#define LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI		128

uint8_t InicjujFiltrKalmanaWysokości4X2Z(stWymianyCM4_t *dane);
uint8_t PredykcjaFiltraKalmanaWysokości4X2Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaWysokościiPrzyspieszeniaFiltraKalmanaWysokości4X2Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaPrzyspieszeniaFiltraKalmanaWysokości4X2Z(stWymianyCM4_t *dane);

#endif /* INC_KALMANWYSOKOSCI4X2Z_H_ */
