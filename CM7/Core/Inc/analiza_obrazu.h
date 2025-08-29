/*
 * analiza_obrazu.h
 *
 *  Created on: Jun 20, 2024
 *      Author: PitLab
 */

#ifndef INC_ANALIZA_OBRAZU_H_
#define INC_ANALIZA_OBRAZU_H_
#define DZIELNIK_HIST_CB	8
#define DZIELNIK_HIST_RGB	128


#include <stdint.h>


void KonwersjaRGB565doCB7(uint16_t *obrazRGB565, uint8_t *obrazCB, uint32_t rozmiar);
void KonwersjaCB7doRGB565(uint8_t *obrazCB, uint16_t *obrazCB565, uint32_t rozmiar);
void DetekcjaKrawedziRoberts(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog);
void DetekcjaKrawedziSobel(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog);
void HistogramCB7(uint8_t *obraz, uint8_t *hist, uint32_t rozmiar);
void HistogramRGB565(uint16_t *obrazRGB565, uint32_t rozmiar, uint8_t *histR, uint8_t *histG, uint8_t *histB);
void Progowanie(uint8_t *obraz, uint8_t prog, uint32_t rozmiar);
void Dylatacja(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog);
void Erozja(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog);
void Odszumianie(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog);

#endif /* INC_ANALIZA_OBRAZU_H_ */
