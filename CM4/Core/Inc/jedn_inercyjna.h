/*
 * jedn_inercyjna.h
 *
 *  Created on: Dec 29, 2024
 *      Author: PitLab
 */

#ifndef INC_JEDN_INERCYJNA_H_
#define INC_JEDN_INERCYJNA_H_

#include "sys_def_CM4.h"

//Progi zaburzeń przyspieszenia wynikającego ze zmiany predkości
#define PROG_ACC_DOBRY	0.05f * AKCEL1G
#define PROG_ACC_ZLY	0.20f * AKCEL1G
#define PROG_MAG_DOBRY	0.05f * NOMINALNE_MAGN
#define PROG_MAG_ZLY	0.20f * NOMINALNE_MAGN

//wartości współczynników filtra komplementarnego
#define WSP_FILTRA_ADAPT	0.05f	//Dla niezakłóconego pomiaru przyspieszenia weź tyle danych z akcelerometru a resztę do 1.0 z modelu obracanego żyroskopem po to, aby filtrować szum z akcelerometru i kasować dryft żyroskopu

// stałe dla IMU Madgwicka
#define Kp 2.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f		// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.5f		// half the sample period


uint8_t InicjujJednostkeInercyjna(void);
uint8_t JednostkaInercyjna1Trygonometria(uint8_t chGniazdo);
uint8_t JednostkaInercyjna4Kwaterniony(uint8_t chGniazdo, float *fZyro, float *fAkcel, float *fMagn);
float FiltrAdaptacyjnyAkc(float *fAkcel);
float FiltrAdaptacyjnyMag(float *fMag);
void JednostkaInercyjnaMadgwick(void);
void ObrotWektora(uint8_t chGniazdo);
#endif /* INC_JEDN_INERCYJNA_H_ */
