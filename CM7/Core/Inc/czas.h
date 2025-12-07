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
//#define SSC_CZAS_NIESYNCHR		0x01
//#define SSC_DATA_NIESYNCHR		0x02

#define SSC_GODZ_SYNCHR		0x01
#define SSC_MIN_SYNCHR		0x02
#define SSC_SEK_SYNCHR		0x04
#define SSC_MASKA_CZASU		0x0F
#define SSC_ROK_SYNCHR		0x10
#define SSC_MIES_SYNCHR		0x20
#define SSC_DZIEN_SYNCHR	0x40
#define SSC_MASKA_DATY		0xF0


uint8_t SynchronizujCzasDoGNSS(stGnss_t *stGnss);
uint8_t PobierzDateCzas(RTC_DateTypeDef *sData, RTC_TimeTypeDef *sCzas);
uint32_t PobierzCzasT6(void);
uint32_t MinalCzas(uint32_t nPoczatek);
uint32_t MinalCzas2(uint32_t nPoczatek, uint32_t nKoniec);
uint8_t CzekajNaZero(uint8_t chZajety, uint32_t nCzasOczekiwania);
DWORD PobierzCzasFAT(void);
void StartPomiaruCykli(void);
uint32_t WynikPomiaruCykli(uint8_t *CPI, uint8_t *EXC, uint8_t *SLEEP, uint8_t *LSU);
void WyswietlCykle(void);
#endif /* INC_CZAS_H_ */
