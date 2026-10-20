/*
 * jedn_inercyjna.h
 *
 *  Created on: Dec 29, 2024
 *      Author: PitLab
 */

#ifndef INC_JEDN_INERCYJNA_H_
#define INC_JEDN_INERCYJNA_H_

#include "SysDefCM4.h"
#include "wymiana.h"

//Progi zaburzeń przyspieszenia wynikającego ze zmiany predkości
#define PROG_ACC_DOBRY	0.05f * AKCEL1G
#define PROG_ACC_ZLY	0.20f * AKCEL1G
#define PROG_MAG_DOBRY	0.05f * NOMINALNE_MAGN
#define PROG_MAG_ZLY	0.20f * NOMINALNE_MAGN

//wartości współczynników filtra komplementarnego
#define WSP_FILTRA_ADAPT	0.05f	//Dla niezakłóconego pomiaru przyspieszenia weź tyle danych z akcelerometru a resztę do 1.0 z modelu obracanego żyroskopem po to, aby filtrować szum z akcelerometru i kasować dryft żyroskopu


uint8_t InicjujJednostkeInercyjna(void);
uint8_t JednostkaInercyjnaTrygonometria(float *fŻyro, float *fAkcel, float *fMagn, stWymianyCM4_t *wyniki, uint32_t ndT);
uint8_t JednostkaInercyjnaKwaterniony(float *fŻyro, float *fAkcel, float *fMagn, uint32_t ndT);
float FiltrAdaptacyjnyAkc(float *fAkcel);
float FiltrAdaptacyjnyMag(float *fMag);
#endif /* INC_JEDN_INERCYJNA_H_ */
