/*
 * Wyrzutnia.h
 *
 *  Created on: 9 paź 2026
 *      Author: Piotr
 */
#ifndef INC_WYRZUTNIA_H_
#define INC_WYRZUTNIA_H_
#include <Rysuj.h>
#include "SysDefCM7.h"
#include "Ekran.h"


#define	DROGA_CALKOWITA		9500U	//długość bieżni wyrzutni [mm]
#define	DROGA_ROZPEDZANIA	7500U	//odcinek wyrzutni na której rozpędza się BSP [mm]
#define OBWOD_ROLKI			200U	//droga jednego obrotu rolki [mm]
#define PRZYSP_STARTOWE		12.0f	//12-15 m/s^2, impuls startowy do 20 m/s^2
#define PRZYSP_HAMOWANIA	0.0f
#define CZUJNIKOW_NA_ROLCE	1U
#define LICZBA_POMIAROW		(DROGA_CALKOWITA / OBWOD_ROLKI * CZUJNIKOW_NA_ROLCE)
#define SKOK_X_WYKRESU		(DISP_X_SIZE / LICZBA_POMIAROW)
#define MASA_BSP			78.0f	//masa BSP [kg]
#define SILA_TLOKA			(MASA_BSP * PRZYSP_STARTOWE)	// Zakładam siłę napędzajacą jako iloczyn masy BSP i zmierzonego przyspieszenia [N]

#define KSTAN	5	//rozmiar wektora stanu
#define KPOM	2	//rozmiar wektora pomiaru: droga i prędkość

#define WARIANCJA_ZRYWUL	5.0e-1f		//
#define WARIANCJA_POSLIZGU 	7.0e-9f;

uint8_t TestFiltraPrędkościRolek(void);
uint8_t SymulujPrędkościRolek(float *fDrogaRolki1, float *fDrogaRolki2, float *fVRolki1, float *fVRolki2, float *fCzas1, float *fCzas2);
uint8_t InicjujFiltrKalmanaPrędkościRolek5X2Z(void);
uint8_t PredykcjaFiltraKalmanaPrędkościRolek5X2Z(float fDeltaCzasu);
uint8_t AktulizacjaRolką1FiltraKalmanaPrędkościRolek5X2Z(float fDroga, float fPredkość);
uint8_t AktulizacjaRolką2FiltraKalmanaPrędkościRolek5X2Z(float fDroga, float fPredkość);

#endif /* INC_WYRZUTNIA_H_ */
