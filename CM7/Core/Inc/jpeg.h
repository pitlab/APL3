/*
 * jpeg.h
 *
 *  Created on: Sep 19, 2025
 *      Author: PitLab
 */

#ifndef INC_JPEG_H_
#define INC_JPEG_H_

#include "sys_def_CM7.h"


#define ROZMIAR_BUF_JPEG	(480 * 320)
#define SZEROKOSC_BLOKU		8
#define WYSOKOSC_BLOKU		8
#define ROZMIAR_BLOKU		(SZEROKOSC_BLOKU * WYSOKOSC_BLOKU)
#define ROZMIAR_MCU420		(6 * ROZMIAR_BLOKU)

uint8_t InicjalizujJpeg(void);
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chKolor, uint8_t chJakoscKompresji);
uint8_t CzekajNaKoniecPracyJPEG(void);
uint8_t KompresujYUV420(uint8_t *chObrazY8, uint8_t *chBlokWyj, uint16_t sSzerokosc, uint16_t sWysokosc);


void prepareMCU(int mcuX, int mcuY, uint8_t *Yplane, uint8_t *Cbplane, uint8_t *Crplane, uint8_t *dstBuffer);
void copyBlock8x8(uint8_t *dst, const uint8_t *src, int stride, int startX, int startY);
void prepareImage(uint8_t *Yplane, uint8_t *Cbplane, uint8_t *Crplane, void (*feedMCU)(uint8_t *mcuData));
#endif /* INC_JPEG_H_ */
