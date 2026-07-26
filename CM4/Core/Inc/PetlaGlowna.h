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
#define TIMEOUT_KASOWANIA_BLEDOW	60000;		//liczba obiegów pętli po której kasowany jest najstarszy błąd

void PetlaGlowna(void);
uint8_t WykonajPolecenieCM7(void);
uint8_t RozdzielniaOperacjiI2C(void);
uint8_t ObslugaCzujnikowI2C(uint8_t *chCzujniki);
void PrzechwyćBłąd(uint8_t cBłąd);
#endif /* SRC_PETLA_GLOWNA_H_ */
