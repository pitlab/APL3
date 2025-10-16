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

//formaty obrazu
#define BMP_KOLOR_Y8		8
#define BMP_KOLOR_RGB24		24


void ObslugaZapisuBmp(void);
uint8_t ZapiszPlikBmp(uint8_t *chObrazWe, uint8_t chFormatKoloru, uint16_t sSzerokosc, uint16_t sWysokosc);

#endif /* INC_BMP_H_ */
