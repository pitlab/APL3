/*
 * konfiguracja.h
 *
 *  Created on: Jun 4, 2024
 *      Author: PitLab
 */

#ifndef INC_KONFIGURACJA_H_
#define INC_KONFIGURACJA_H_

#include "sys_def_CM7.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "ov5642_regs.h"

#define OV5642_I2C_ADR	0x78
#define OV5642_ID		0x5642
#define OV5640_ID		0x5640
#define KAMERA_TIMEOUT	1	//czas w milisekundach na wysłanie jednego polecenia do kamery. Nominalnie jest to ok 400us na adres i 3 bajty danych
#define KAMERA_ZEGAR	24000000	//kamera wymaga zegara 24MHz (6-27MHz)
//#define KAMERA_ZEGAR	20000000	//kamera wymaga zegara 24MHz (6-27MHz)





//konfiguracja kamery
struct st_KonfKam
{
	uint16_t sSzerWe;
	uint16_t sWysWe;
	uint16_t sSzerWy;
	uint16_t sWysWy;
	uint8_t chTrybDiagn;
	uint8_t chFlagi;
};

typedef struct st_KonfKam stKonfKam_t;
//extern struct st_KonfKam strKonfKam;

uint8_t InicjalizujKamere(void);
uint8_t Wyslij_I2C_Kamera(uint16_t rejestr, uint8_t dane);
uint8_t Czytaj_I2C_Kamera(uint16_t rejestr, uint8_t *dane);
uint8_t	SprawdzKamere(void);
uint8_t UstawKamere(stKonfKam_t *konf);
uint8_t RozpocznijPraceDCMI(uint8_t chAparat);
//uint8_t ZrobZdjecie(int16_t sSzerokosc, uint16_t sWysokosc);
uint8_t ZrobZdjecie(void);
uint8_t Wyslij_Blok_Kamera(const struct sensor_reg reglist[]);
uint8_t UstawRozdzielczoscKamery(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chZoom);

#endif /* INC_KONFIGURACJA_H_ */
