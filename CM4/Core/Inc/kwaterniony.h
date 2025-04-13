/*
 * kwaterniony.h
 *
 *  Created on: Apr 13, 2025
 *      Author: PitLab
 */

#ifndef INC_KWATERNIONY_H_
#define INC_KWATERNIONY_H_

#include "sys_def_CM4.h"

void MnozenieKwaternionow1(float a1, float b1, float c1, float d1, float a2, float b2, float c2, float d2, float *wynik);
void MnozenieKwaternionow2(float *q, float *p, float *wynik);
void KwaternionNaMacierz(float *q, float (* m)[4][4]);
void MacierzNaKwaternion(float  m[4][4], float *q);
//void MnozenieMacierzy4x4(float *a, float *b, float *m);
void MnozenieMacierzy4x4(float a[4][4], float b[4][4], float (*m)[4][4]);
void ObrotWektoraKwaternionem(float *wektor_we, float *v, float *wektor_wy);
void WektorNaKwaternion(float *wektor, float *q);
void KwaternionNaWektor(float *q, float *wektor);
void KwaternionSprzezony(float *q, float *sprzezony);

#endif /* INC_KWATERNIONY_H_ */
