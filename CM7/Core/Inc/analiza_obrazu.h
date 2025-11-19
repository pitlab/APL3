/*
 * analiza_obrazu.h
 *
 *  Created on: Jun 20, 2024
 *      Author: PitLab
 */

#ifndef INC_ANALIZA_OBRAZU_H_
#define INC_ANALIZA_OBRAZU_H_
#define ROZMIAR_HIST_KOLOR	32
#define ROZMIAR_HIST_CB7	128
#define ROZMIAR_HIST_CB8	256
#define DZIELNIK_HIST_CB	8
#define DZIELNIK_HIST_RGB	(3*32)	//służy do normalizacji histogramu kolorowego 5-5-5


#include <stdint.h>


void KonwersjaRGB565doCB7(uint16_t *obrazRGB565, uint8_t *obrazCB, uint32_t rozmiar);
void KonwersjaCB7doRGB565(uint8_t *obrazCB, uint16_t *obrazCB565, uint32_t rozmiar);
void KonwersjaCB8doRGB666(uint8_t *obrazCB, uint8_t *obrazCB666, uint32_t rozmiar);
void KonwersjaRGB565doRGB666(uint16_t *obrazRG565, uint8_t *obrazRGB666, uint32_t rozmiar);
void KonwersjaRGB888doYCbCr(uint8_t chR, uint8_t chG, uint8_t chB, uint8_t *chY, uint8_t *chCb, uint8_t *chCr);
void DetekcjaKrawedziRoberts(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog);
void DetekcjaKrawedziSobel(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog);
void HistogramCB7(uint8_t *chObraz, uint32_t nRozmiar, uint8_t *chHist);
void HistogramCB8(uint8_t *chObraz, uint32_t nRozmiar, uint8_t *chHist);
void HistogramRGB565(uint16_t *obrazRGB565, uint32_t rozmiar, uint8_t *histR, uint8_t *histG, uint8_t *histB);
void Progowanie(uint8_t *obraz, uint8_t prog, uint32_t rozmiar);
void Dylatacja(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog);
void Erozja(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog);
void Odszumianie(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog);

#endif /* INC_ANALIZA_OBRAZU_H_ */
