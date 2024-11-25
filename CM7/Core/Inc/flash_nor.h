/*
 * flash_nor.h
 *
 *  Created on: Nov 20, 2024
 *      Author: PitLab
 */

#ifndef SRC_FLASH_NOR_H_
#define SRC_FLASH_NOR_H_

#include "stm32h7xx_hal.h"

#define ADRES_NOR			0x68000000
#define LICZBA_SEKTOROW		256
#define ROZMIAR8_SEKTORA	(128*1024)
#define ROZMIAR16_SEKTORA	(64*1024)
#define ROZMIAR8_BUFORA		512
#define ROZMIAR16_BUFORA	256
#define ROZMIAR8_STRONY		32
#define ROZMIAR16_STRONY	16




uint8_t SprawdzObecnoscFlashNOR(void);
uint8_t KasujSektorFlashNOR(uint32_t nAdres);
uint8_t KasujFlashNOR(void);
uint8_t ZapiszDaneFlashNOR(uint32_t nAdres, uint16_t* sDane, uint32_t nIlosc);
uint8_t Test_Flash(void);
void TestPredkosciOdczytu(void);

#endif /* SRC_FLASH_NOR_H_ */
