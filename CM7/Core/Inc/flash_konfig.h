/*
 * flash_konfig.h
 *
 *  Created on: 15 maj 2019
 *      Author: PitLab
 */

#ifndef FLASH_KONFIG_H_
#define FLASH_KONFIG_H_

#include "flash_nor.h"


#define ROZMIAR_PACZKI_KONF8	32
#define ROZMIAR_PACZKI_KONF16	16
#define ROZMIAR_PACZKI_KONF		ROZMIAR_PACZKI_KONF8
#define ROZMIAR_DANYCH_WPACZCE	30
#define ADRES_SEKTORA0			ADRES_NOR
#define ADRES_SEKTORA1			ADRES_SEKTORA0 + ROZMIAR_SEKTORA



//typy paczek danych z konfiguracją we flash
#define FKON_KALIBRACJA_DOTYKU		0x00	//współczynniki kalibracji panelu dotykowego
#define FKON_OKRES_TELEMETRI1		0x01	//okresy wysyłania zmiennych telemetrycznych
#define FKON_OKRES_TELEMETRI2		0x02	//okresy wysyłania zmiennych telemetrycznych
#define FKON_OKRES_TELEMETRI3		0x03	//okresy wysyłania zmiennych telemetrycznych
#define FKON_OKRES_TELEMETRI4		0x04	//okresy wysyłania zmiennych telemetrycznych

#define LICZBA_TYPOW_PACZEK	1




uint8_t InicjujKonfigFlash(void);
uint8_t ZapiszPaczkeKonfigu(uint8_t* chDane);
uint8_t ZapiszPaczkeAdr(uint32_t nAdrZapisu,  uint8_t* chDane);
uint8_t CzytajPaczkeKonfigu(uint8_t* chDane, uint8_t chIdent);
float KonwChar2Float(unsigned char *chData);
void KonwFloat2Char(float fData, unsigned char *chData);
unsigned short KonwChar2UShort(unsigned char *chData);
signed short KonwChar2SShort(unsigned char *chData);
void KonwUShort2Char(unsigned short sData, unsigned char *chData);
void KonwSShort2Char(signed short sData, unsigned char *chData);
unsigned char SprawdzPaczke(unsigned int nAdres);
uint8_t PrzepiszDane(void);

#endif /* FLASH_KONFIG_H_ */
