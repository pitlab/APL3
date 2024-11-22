/*
 * flash_nor.h
 *
 *  Created on: Nov 20, 2024
 *      Author: PitLab
 */

#ifndef SRC_FLASH_NOR_H_
#define SRC_FLASH_NOR_H_

#include "stm32h7xx_hal.h"

#define ADRES_NOR		0x68000000


uint8_t SprawdzObecnoscFlashNOR(void);
void Test_Flash(void);

#endif /* SRC_FLASH_NOR_H_ */
