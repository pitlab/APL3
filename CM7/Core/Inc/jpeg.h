/*
 * jpeg.h
 *
 *  Created on: Sep 19, 2025
 *      Author: PitLab
 */

#ifndef INC_JPEG_H_
#define INC_JPEG_H_

#include "sys_def_CM7.h"


#define ROZMIAR_BUF_JPEG	(320*480)
#define SZEROKOSC_MCU	8
#define WYSOKOSC_MCU	8
#define ROZMIAR_MCU		(SZEROKOSC_MCU * WYSOKOSC_MCU)


uint8_t InicjalizujJpeg(void);
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chKolor, uint8_t chJakoscKompresji);
uint8_t CzekajNaKoniecPracyJPEG(void);
uint8_t KompresujY8(uint8_t *chObrazY8, uint8_t *chBlokWyj, uint16_t sSzerokosc, uint16_t sWysokosc);

#endif /* INC_JPEG_H_ */
