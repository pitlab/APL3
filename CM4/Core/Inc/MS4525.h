/*
 * MS4525.h
 *
 *  Created on: Feb 17, 2025
 *      Author: PitLab
 */

#ifndef INC_MS4525_H_
#define INC_MS4525_H_
#include "sys_def_CM4.h"


//Adres I2C czujnika jest okreslony w nazwie symbolu 4525DO 5Ax gdzie x może być literą: I=0x28, J=0x36, K=0x46
#define MS2545_I2C_ADR	0x51	//0x28 + RD
#define PASKALI_NA_PSI	6894.7572932f		//1PSI to tyle Paskali
#define ZAKRES_POMIAROWY_CISNIENIA	1	//[psi]


uint8_t InicjujMS4525(void);
uint8_t ObslugaMS4525(void);
float CisnienieMS2545(uint8_t * dane);
float TemperaturaMS2545(uint8_t * dane);

#endif /* INC_MS4525_H_ */
