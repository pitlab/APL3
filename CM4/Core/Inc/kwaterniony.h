/*
 * kwaterniony.h
 *
 *  Created on: Apr 13, 2025
 *      Author: PitLab
 */

#ifndef INC_KWATERNIONY_H_
#define INC_KWATERNIONY_H_

#include "sys_def_CM4.h"

void MnozenieKwaternionow(float *q, float *p, float *wynik);
void KwaternionNaMacierz(float *q, float *m);
void MacierzNaKwaternion(float  *m, float *q);
void MnozenieMacierzy4x4(float *a, float *b, float *m);
void ObrotWektoraKwaternionem(float *wektor_we, float *v, float *wektor_wy);
void WektorNaKwaternion(float *wektor, float *q);
void KwaternionNaWektor(float *q, float *wektor);
void KwaternionSprzezony(float *q, float *sprzezony);
//void KatyKwaterniona1(float *qA, float *qM, float *fKaty);
//void KatyKwaterniona2(float *qA, float *qM, float *fKaty);
void KatyKwaterniona(float *qA, float *qM, float *fKaty);
void Normalizuj(float * fWe, float *fWy, uint8_t chRozmiar);

#endif /* INC_KWATERNIONY_H_ */
