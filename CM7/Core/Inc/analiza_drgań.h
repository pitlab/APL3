/*
 * analiza_drgań.h
 *
 *  Created on: 31 mar 2026
 *      Author: PitLab
 */

#ifndef INC_ANALIZA_DRGAŃ_H_
#define INC_ANALIZA_DRGAŃ_H_

#include "sys_def_CM7.h"
#include "wymiana.h"
#include "fft.h"

uint8_t RozpocznijAnalizęDrgań(stFFT_t *stKonfigFFT, uint8_t *chTrybPracy);
uint8_t KrokAnalizyDrgań(stFFT_t *stKonfigFFT, uint8_t *chTrybPracy);

#endif /* INC_ANALIZA_DRGAŃ_H_ */
