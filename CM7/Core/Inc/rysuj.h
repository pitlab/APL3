/*
 * rysuj.h
 *
 *  Created on: Aug 17, 2025
 *      Author: PitLab
 */

#ifndef INC_RYSUJ_H_
#define INC_RYSUJ_H_
#include "sys_def_CM7.h"

#include "display.h"


void Menu(char *tytul, tmenu *menu, unsigned char *tryb);
void BelkaTytulu(char* chTytul);
uint8_t WyswietlZdjecie(uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t* sObraz);
void RysujPrzebieg(int16_t *sDaneKasowania, int16_t *sDaneRysowania, uint16_t sKolor);
uint8_t RysujHistogramRGB16(uint8_t *histR, uint8_t *histG, uint8_t *histB);



#endif /* INC_RYSUJ_H_ */
