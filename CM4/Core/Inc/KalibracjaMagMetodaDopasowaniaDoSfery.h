/*
 * KalibracjaMagMetodaDopasowaniaDoSfery.h
 *
 *  Created on: 4 paź 2026
 *      Author: PitLab
 */

#ifndef INC_KALIBRACJAMAGMETODADOPASOWANIADOSFERY_H_
#define INC_KALIBRACJAMAGMETODADOPASOWANIADOSFERY_H_
#include "SysDefCM4.h"
#include "wymiana.h"
#include "arm_math.h"



typedef struct
{
   float fATA[4][4];	//wynik operacji A * A^T
   float fATy[4];		//wynik operacji A^T * y
   uint16_t sLiczbaPróbek;
} stDopasowanieDoSfery_t;

typedef struct
{
	float fBias[3];	//obliczone biasy magnetometru
	float fNatężeniePolaMag;
	float fC;	//część równania zawierajaca sumę natężenie pola i biasów
} stWynikiDopasowania_t;

uint8_t InicjujKalibracje(void);
uint8_t ZbierajDaneMagDoKalibracji(stWymianyCM4_t *dane);
uint8_t ZbierajProbki(float *fMag, stDopasowanieDoSfery_t *stDopasowanieDoSfery);
uint8_t MetodaNajmniejszychKwadratow(stDopasowanieDoSfery_t *stDopasowanieDoSfery, stWynikiDopasowania_t *stWynikiDopasowania);


#endif /* INC_KALIBRACJAMAGMETODADOPASOWANIADOSFERY_H_ */
