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

#define MIN_WARTOSC	0.00001f
#define MAX_WARTOSC_KATA	0.5f
#define KROK_KATA			0.1f

#define PRZYCISKI_GORNE_Y	100		//górna krawędź przycisków górnego wiersza
#define PRZYCISKI_DOLNE_Y	200		//górna krawędź przycisków dolnego wiersza
#define PRZYCISKI_WYSOKOSC	100
#define PRZYCISKI_SZEROKOSC	160
#define PRZYCISKI_RZAD1X	(1*PRZYCISKI_SZEROKOSC)		//granica miedzy kolumną 1 i 2 przycisków
#define PRZYCISKI_RZAD2X	(2*PRZYCISKI_SZEROKOSC)		//granica miedzy kolumną 2 i 3 przycisków

uint8_t InicjujSymulatorKątów(stSym_t *stSym);
uint8_t InicjujKąty(stSym_t *stSym);
uint8_t SymulujKątyOrientacji(stSym_t *stSym);
uint8_t SymulacjaKątówOrientacji(stSym_t *stSym);

#endif /* INC_SYMULATORKATOW_H_ */
