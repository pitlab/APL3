/*
 * KalmanWysokosci10X10Z.h
 *
 *  Created on: 11 wrz 2026
 *      Author: PitLab
 */

#ifndef INC_KALMANWYSOKOSCI10X10Z_H_
#define INC_KALMANWYSOKOSCI10X10Z_H_

#include "SysDefCM4.h"
#include "wymiana.h"
#include "arm_math.h"

#define KSTAN		10	//rozmiar wektora stanu
#define KPCIS		2	//rozmiar wektora pomiaru czujnika ciśnienia: wysokość i prędkość
#define KPACC		1	//rozmiar wektora pomiaru przyspieszenia
#define KPGNS		1	//rozmiar wektora pomiaru wysokości przez GNSS
#define KPLID		1	//rozmiar wektora pomiaru lidarem
#define KPMAP		1	//rozmiar wektora odczytu z mapy

#define WYSOKOSC_MAPY	110.0f		//wysokość z mapy - tymczasowa zaślepka dopóki nie ma map wysokościowych


#define WARIANCJA_ZRYWU_ACEL	5.0e-1f		//okresla dynamikę procesu
#define WARIANCJA_DRYFTU_ACEL 	7.0e-9f;	//okresla jak szybko może zmieniać sie błąd akcelerometru
#define WARIANCJA_DRYFTU_BARO 	1.0e-6f;	//okresla jak szybko może zmieniać sie błąd wysokości z czujnika ciśnienia
#define WARIANCJA_DRYFTU_GNSS 	1.0e-4f;	//okresla jak szybko może zmieniać sie błąd wysokości z odbiornika GNSS
#define WARIANCJA_ZMIANY_WYSOKOSCI_MAPY		1e-4;

#define LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI		128

uint8_t InicjujFiltrKalmanaWysokości10X10Z(stWymianyCM4_t *dane);
uint8_t PredykcjaFiltraKalmanaWysokości10X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaCzujnikiemCiśnienia1FiltraKalmanaWysokości10X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaCzujnikiemCiśnienia2FiltraKalmanaWysokości10X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaAkcelerometrem1FiltraKalmanaWysokości10X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaAkcelerometrem2FiltraKalmanaWysokości10X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaGNSS1FiltraKalmanaWysokości10X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaGNSS2FiltraKalmanaWysokości10X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaLidaremFiltraKalmanaWysokości10X10Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaMapąFiltraKalmanaWysokości10X10Z(stWymianyCM4_t *dane);

#endif /* INC_KALMANWYSOKOSCI10X10Z_H_ */
