/*
 * czas.h
 *
 *  Created on: Feb 14, 2025
 *      Author: PitLab
 */

#ifndef INC_CZAS_H_
#define INC_CZAS_H_

#include "sys_def_CM7.h"
#include "wymiana.h"
#include "integer.h"

//definicje flag zmiennej chStanSynchronizacjiCzasu
#define SSC_CZAS_NIESYNCHR		0x01
#define SSC_DATA_NIESYNCHR		0x02



void SynchronizujCzasDoGNSS(stGnss_t *stGnss);
uint32_t PobierzCzasT6(void);
uint32_t MinalCzas(uint32_t nPoczatek);
uint32_t MinalCzas2(uint32_t nPoczatek, uint32_t nKoniec);
uint8_t CzekajNaZero(uint8_t chZajety, uint32_t nCzasOczekiwania);
DWORD PobierzCzasFAT(void);

#endif /* INC_CZAS_H_ */
