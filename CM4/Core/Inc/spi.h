/*
 * spi.h
 *
 *  Created on: Feb 1, 2025
 *      Author: PitLab
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_
#include "sys_def_CM4.h"

#define READ_SPI		0x80


uint8_t  CzytajSPIu8(uint8_t chAdres);
void ZapiszSPIu8(uint8_t *chDane, uint8_t chIlosc);
uint16_t CzytajSPIu16mp(uint8_t chAdres);
 int32_t CzytajSPIs24mp(uint8_t chAdres);
 int32_t CzytajSPIs24sp(uint8_t chAdres);



#endif /* INC_SPI_H_ */
