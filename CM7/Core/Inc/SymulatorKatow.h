/*
 * SymulatorKatow.h
 *
 *  Created on: 20 paź 2026
 *      Author: PitLab
 */

#ifndef INC_SYMULATORKATOW_H_
#define INC_SYMULATORKATOW_H_
#include "SysDefCM7.h"
//#include "PoleceniaKomunikacyjne.h"
//#include "WymianaCM7.h"


typedef struct _Sym
{
	float fAkcelerometr[3];		//składowe symulowanego wektora przyspieszenia
	float fMagnetometr[3];		//składowe symulowanego wektora magnetycznego
	float fPrędkośćKątowa[3];	//składowe prędkości kątowej obracajacej wektory
	float fCałkaKataObrotu[3];	//całka prędkości kątowej dająca bieżącą wartość kątów
} stSym_t;


uint8_t InicjujSymulatorKątów(void);

#endif /* INC_SYMULATORKATOW_H_ */
