/*
 * petla_glowna.h
 *
 *  Created on: Dec 3, 2024
 *      Author: PitLab
 */

#ifndef SRC_PETLA_GLOWNA_H_
#define SRC_PETLA_GLOWNA_H_


#include "sys_def_CM4.h"


void PetlaGlowna(void);
uint32_t PobierzCzas(void);
uint32_t MinalCzas(uint32_t nPoczatek);
uint32_t MinalCzas2(uint32_t nPoczatek, uint32_t nCzasAkt);
uint8_t RozdzielniaOperacjiI2C(void);
uint8_t ObslugaCzujnikowI2C(uint8_t *chCzujniki);
#endif /* SRC_PETLA_GLOWNA_H_ */
