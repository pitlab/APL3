/*
 * ND130.h
 *
 *  Created on: Feb 16, 2025
 *      Author: PitLab
 */

#ifndef INC_ND130_H_
#define INC_ND130_H_
#include "sys_def_CM4.h"

#define PASKALI_NA_INH2O					249.082f	//1 cal H2O to tyle Paskali
#define ZAKRES_POMIAROWY_CISNIENIA_ND130	30			//+-tyle [cali H2O]
#define LICZBA_PROBEK_USREDNIANIA_ND130		1500		//tyle trzeba aby filtr (127+1)/128 uzyskał dokładność 6 cyfr znaczących

uint8_t InicjujND130(void);
uint8_t ObslugaND130(void);
float PredkoscRurkiPrantla(float fCisnRozn, float fCisnStat);
float PredkoscRurkiPrantla1(float fCisnRozn);


#endif /* INC_ND130_H_ */
