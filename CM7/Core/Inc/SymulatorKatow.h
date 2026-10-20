/*
 * SymulatorKatow.h
 *
 *  Created on: 20 paź 2026
 *      Author: PitLab
 */

#ifndef INC_SYMULATORKATOW_H_
#define INC_SYMULATORKATOW_H_
#include "SysDefCM7.h"
#include "Wymiana.h"



uint8_t InicjujSymulatorKątów(stSym_t *stSym);
uint8_t SymulujKątyOrientacji(stSym_t *stSym);
uint8_t SymulacjaKątówOrientacji(stSym_t *stSym);

#endif /* INC_SYMULATORKATOW_H_ */
