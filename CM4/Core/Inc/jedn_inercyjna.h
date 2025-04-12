/*
 * jedn_inercyjna.h
 *
 *  Created on: Dec 29, 2024
 *      Author: PitLab
 */

#ifndef INC_JEDN_INERCYJNA_H_
#define INC_JEDN_INERCYJNA_H_

#include "sys_def_CM4.h"

#define Kp 2.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f		// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.5f		// half the sample period


uint8_t InicjujJednostkeInercyjna(void);
void ObliczeniaJednostkiInercujnej(uint8_t chGniazdo);
uint8_t JednostkaInercyjnaKwaterniony(uint8_t chGniazdo);
void JednostkaInercyjnaMadgwick(void);
void ObrotWektora(uint8_t chGniazdo);
void MnozenieKwaternionow(float a1, float b1, float c1, float d1, float a2, float b2, float c2, float d2, float *wynik);
void MnozenieKwaternionow2(float *q, float *p, float *wynik);

#endif /* INC_JEDN_INERCYJNA_H_ */
