/*
 * adc.h
 *
 *  Created on: Jul 8, 2025
 *      Author: PitLab
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "SysDefCM4.h"


#define VREF	3.0f	//napiecie referencyjne przetwornika a/c

#define WYKONANO_POMIAR_ADC2	1
#define WYKONANO_POMIAR_ADC3	2
#define ILOSC_ZEWN_WE_ANALOG	4

#define LICZBA_POMIAROW_ADC2	8
#define LICZBA_POMIAROW_ADC3	10

#define DZIELNIK_CZASU_POMIAROW_WEWN	1000	//co tyle cyklu pomiarów wykonywane są pomiary czujników wewnętrznych: VBat i TempSens

#define TIMEOUT_ADC		2500	//maksymalny czas w mikrosekundach potrzebny na wykonanie pomiaru ADC


//wartości dzielników napiecia używanych przy pomiarach analogowych
#define DZIELNIK_UWE_ZASIL	16.0f	//górny opornik 30k 1%, dolny 2k 1%
#define DZIELNIK_USERWO		3.9375f	//górny opornik 47k 1%, dolny 16k 1%
#define DZIELNIK_UCZUJNIK	1.0f	//
#define DZIELNIK_ICZUJNIK	1.0f	//
#define DZIELNIK_VBAT		4.0f	//wewnętrzny dzielnik /4

uint8_t InicjujADC(void);
uint8_t PomiarADC(uint8_t chKanal, uint8_t cBityPozwoleniaNaPomiar);
uint8_t ObsługaADC(uint8_t cOdcinekCzasu, uint8_t cBityPozwoleniaNaPomiar);

#endif /* INC_ADC_H_ */
