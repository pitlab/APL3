/*
 * petla_glowna.h
 *
 *  Created on: Dec 3, 2024
 *      Author: PitLab
 */

#ifndef SRC_PETLA_GLOWNA_H_
#define SRC_PETLA_GLOWNA_H_


#include "SysDefCM4.h"

#define MAX_DT	10000	//2 obiegi pętli głównej jest akceptowalną granicą czasu trwania pętli

void PetlaGlowna(void);
uint32_t PobierzCzasT7(void);
uint32_t MinalCzasT7(uint32_t nPoczatek);
uint32_t MinalCzas2T7(uint32_t nPoczatek, uint32_t nCzasAkt);
void WykonajPolecenieCM7(void);
uint8_t RozdzielniaOperacjiI2C(void);
uint8_t ObslugaCzujnikowI2C(uint8_t *chCzujniki);
#endif /* SRC_PETLA_GLOWNA_H_ */
