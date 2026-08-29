/*
 * KalmanWysokosci4X3Z.h
 *
 *  Created on: 15 sie 2026
 *      Author: PitLab
 */

#ifndef INC_KALMANWYSOKOSCI5X6Z_H_
#define INC_KALMANWYSOKOSCI5X6Z_H_

#include "SysDefCM4.h"
#include "wymiana.h"
#include "arm_math.h"

#define KSTAN		5	//rozmiar wektora stanu
#define KPOMIAR		6	//rozmiar wektora pomiaru
#define WARIANCJA_ZRYWU_ACEL	5.0e-1f		//
#define WARIANCJA_DRYFTU_ACEL 	5.0e-9f;	//

#define LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI		128

uint8_t InicjujFiltrKalmanaWysokości5X6Z(stWymianyCM4_t *dane);
uint8_t PredykcjaFiltraKalmanaWysokości5X6Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaCzujnikiemCiśnienia1FiltraKalmanaWysokości5X6Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaCzujnikiemCiśnienia2FiltraKalmanaWysokości5X6Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaAkcelerometrem1FiltraKalmanaWysokości5X6Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaAkcelerometrem2FiltraKalmanaWysokości5X6Z(stWymianyCM4_t *dane);

#endif /* INC_KALMANWYSOKOSCI5X6Z_H_ */
