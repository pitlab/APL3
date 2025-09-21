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

#define SZER_PASKA_HISTOGRAMU		2		//szerokość paska w pikselach

void Menu(char *tytul, tmenu *menu, unsigned char *tryb);
void BelkaTytulu(char* chTytul);
uint8_t WyswietlZdjecie(uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t* sObraz);
uint8_t WyswietlZdjecieRGB666(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t* chObraz);
void RysujPrzebieg(int16_t *sDaneKasowania, int16_t *sDaneRysowania, uint16_t sKolor);
void RysujHistogramRGB32(uint8_t *histR, uint8_t *histG, uint8_t *histB);
void RysujHistogramCB8(uint8_t *histCB8);
void RysujProstokat(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void RysujProstokatZaokraglony(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void UstawCzcionke(uint8_t* chCzcionka);
uint8_t GetFontX(void);
uint8_t GetFontY(void);
void RysujNapis(char *st, uint16_t x, uint16_t y);
void RysujNapiswRamce(char *str, uint16_t x, uint16_t y, uint16_t sx, uint16_t sy);
void RysujKolo(uint16_t x, uint16_t y, uint16_t promien);
#endif /* INC_RYSUJ_H_ */
