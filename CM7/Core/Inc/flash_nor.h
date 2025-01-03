/*
 * flash_nor.h
 *
 *  Created on: Nov 20, 2024
 *      Author: PitLab
 */

#ifndef SRC_FLASH_NOR_H_
#define SRC_FLASH_NOR_H_

#include "sys_def_CM7.h"

#define ADRES_NOR			0x68000000
#define LICZBA_SEKTOROW		256
#define ROZMIAR8_SEKTORA	(128*1024)
#define ROZMIAR16_SEKTORA	(64*1024)
#define ROZMIAR8_BUFORA		512
#define ROZMIAR16_BUFORA	256
#define ROZMIAR8_STRONY		32
#define ROZMIAR16_STRONY	16
#define ROZMIAR8_EXT_SRAM	(4096*1024)
#define ROZMIAR16_EXT_SRAM	(2048*1024)
#define ROZMIAR16_BUF_SEKT	256


uint8_t InicjujFlashNOR(void);
uint8_t SprawdzObecnoscFlashNOR(void);
uint8_t KasujSektorFlashNOR(uint32_t nAdres);
uint8_t KasujFlashNOR(void);
uint8_t ZapiszDaneFlashNOR(uint32_t nAdres, uint16_t* sDane, uint32_t nIlosc);
uint8_t Test_Flash(void);
void TestPredkosciOdczytuNOR(void);
void TestPredkosciOdczytuRAM(void);

#endif /* SRC_FLASH_NOR_H_ */
