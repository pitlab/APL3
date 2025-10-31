/*
 * bmp.h
 *
 *  Created on: Oct 16, 2025
 *      Author: PitLab
 */

#ifndef INC_BMP_H_
#define INC_BMP_H_
#include "sys_def_CM7.h"

#define ROZMIAR_NAGLOWKA_BMP	54
#define ROZMIAR_PALETY_BMP		1024
//#define OFFSET_DANYCH_BMP		(ROZMIAR_NAGLOWKA_BMP + ROZMIAR_PALETY_BMP)

//formaty obrazu
#define BMP_KOLOR_8		8
#define BMP_KOLOR_24	24


void ObslugaZapisuBmp(void);
uint8_t ZapiszPlikBmp(uint8_t *chObrazWe, uint8_t chFormatKoloru, uint16_t sSzerokosc, uint16_t sWysokosc);

#endif /* INC_BMP_H_ */
