/*
 * jpeg.h
 *
 *  Created on: Sep 19, 2025
 *      Author: PitLab
 */

#ifndef INC_JPEG_H_
#define INC_JPEG_H_

#include "sys_def_CM7.h"


#define ROZMIAR_BUF_JPEG	(480 * 320 / 4)	//na razie minimalna kompresja jest rzędu 7 razy
#define SZEROKOSC_BLOKU		8
#define WYSOKOSC_BLOKU		8
#define ROZMIAR_BLOKU		(SZEROKOSC_BLOKU * WYSOKOSC_BLOKU)
#define ROZMIAR_MCU420		(6 * ROZMIAR_BLOKU)

//flagi ustawiane w calbackach określające stan enkodera
#define KOMPR_PUSTE_WE		1	//na wejście enkodera można podać nowe dane
#define KOMPR_MCU_GOTOWY	2	//podany MCU jest już skompresowany
#define KOMPR_OBR_GOTOWY	4	//podany obraz jest już skompresowany



uint8_t InicjalizujJpeg(void);
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chKolor, uint8_t chWspKompresji);
uint8_t CzekajNaKoniecPracyJPEG(void);
uint8_t KompresujYUV420(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *chDaneSkompresowane, uint32_t nRozmiarBuforaJPEG);
uint8_t KompresujY8(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *chDaneSkompresowane, uint32_t nRozmiarBuforaJPEG);
uint8_t* ZnajdzZnacznikJpeg(uint8_t *chDaneSkompresowane, uint8_t chZnacznik);
#endif /* INC_JPEG_H_ */
