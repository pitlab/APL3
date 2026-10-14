/*
 * KalmanKatow7X9Z.h
 *
 *  Created on: 22 wrz 2026
 *      Author: PitLab
 */

#ifndef INC_KALMANKATOW7X9Z_H_
#define INC_KALMANKATOW7X9Z_H_

#include "SysDefCM4.h"
#include "wymiana.h"
#include "arm_math.h"

#define KKSTAN		7	//rozmiar wektora stanu filtra katów
#define KWATER		4	//rozmiar kwaternionu
#define KKPOMR		3	//rozmiar wektora pomiaru filtra katów

#define WARIANCJA_PROCESU_ZYRO	1.0e-11f	//określa jak szybko może przesuwać sie poziom zera żyroskopu
#define WARIANCJA_SZUMU_ZYRO	5.0e-3f		//określa jak szybko może zmieniać sie pomiar żyroskopu
#define WARIANCJA_SZUMU_ACEL 	5.0e-3f		//określa jak szybko może zmieniać sie pomiar akcelerometru
#define WARIANCJA_SZUMU_MAGN 	5.0e-1f		//określa jak szybko może zmieniać sie pomiar magnetometru - uwaga: do obliczenia wariancji wektor musi być znormalizowany, bo w takiej formie występuje w filtrze



uint8_t InicjujFiltrKalmanaKątów7X9Z(stWymianyCM4_t *dane);
uint8_t PredykcjaFiltraKalmanaKątów7X9Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaAkcelerometremFiltraKalmanaKątów7X9Z(float *fAkcel);
uint8_t AktulizacjaMagnetometremFiltraKalmanaKątów7X9Z(float *fMagne);

#endif /* INC_KALMANKATOW7X9Z_H_ */
