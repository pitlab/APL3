/*
 * ICM42688.h
 *
 *  Created on: Feb 1, 2025
 *      Author: PitLab
 */

#ifndef INC_ICM42688_H_
#define INC_ICM42688_H_
#include "sys_def_CM4.h"


//definicje Poleceń czujnikaBMP581


//definicje rejestrów czujnika
#define PICM42688_WHO_I_AM		0x75


#define ICM4_READ				0x80



uint8_t InicjujICM42688(void);
uint8_t ICM42688_Read8bit(unsigned char chAdres);
uint8_t ObslugaICM42688(void);


#endif /* INC_ICM42688_H_ */
