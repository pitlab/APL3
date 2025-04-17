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
#define PROG_ACC_DOBRY	0.1f * AKCEL1G
#define PROG_ACC_ZLY	0.2f * AKCEL1G

//wartości współczynników filtra komplementarnego
#define WSP_FILTRA_ADAPT	0.25f	//Dla niezakłóconego pomiaru przyspieszenia weź tyle danych z akcelerometru a resztę z modelu obracanego żyroskopem po to aby filtrować szum z akcelerometru

// stałe dla IMU Madgwicka
#define Kp 2.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f		// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.5f		// half the sample period


uint8_t InicjujJednostkeInercyjna(void);
void ObliczeniaJednostkiInercujnej(uint8_t chGniazdo);
uint8_t JednostkaInercyjnaKwaterniony2(uint8_t chGniazdo);
uint8_t JednostkaInercyjnaKwaterniony3(uint8_t chGniazdo);
float FiltrAdaptacyjnyAcc(void);
void JednostkaInercyjnaMadgwick(void);
void ObrotWektora(uint8_t chGniazdo);
#endif /* INC_JEDN_INERCYJNA_H_ */
