/*
 * KalmanWysokosci8X10Z.h
 *
 *  Created on: 11 wrz 2026
 *      Author: PitLab
 */

#ifndef INC_KALMANWYSOKOSCI8X10Z_H_
#define INC_KALMANWYSOKOSCI8X10Z_H_

#include "SysDefCM4.h"
#include "wymiana.h"
#include "arm_math.h"

#define KSTAN		8	//rozmiar wektora stanu
#define KPCIS		2	//rozmiar wektora pomiaru czujnika ciśnienia: wysokość i prędkość
#define KPACC		1	//rozmiar wektora pomiaru przyspieszenia
#define KPGNS		1	//rozmiar wektora pomiaru wysokości przez GNSS
#define KPLID		1	//rozmiar wektora pomiaru lidarem
#define KPMAP		1	//rozmiar wektora odczytu z mapy

#define WARIANCJA_ZRYWU_ACEL	5.0e-1f		//
#define WARIANCJA_DRYFTU_ACEL 	7.0e-9f;

#define LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI		128

uint8_t InicjujFiltrKalmanaWysokości8X10Z(stWymianyCM4_t *dane);
uint8_t PredykcjaFiltraKalmanaWysokości8X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaCzujnikiemCiśnienia1FiltraKalmanaWysokości8X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaCzujnikiemCiśnienia2FiltraKalmanaWysokości8X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaAkcelerometrem1FiltraKalmanaWysokości8X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaAkcelerometrem2FiltraKalmanaWysokości8X10Z(stWymianyCM4_t *dane);

#endif /* INC_KALMANWYSOKOSCI8X10Z_H_ */
