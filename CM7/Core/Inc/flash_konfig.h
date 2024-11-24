/*
 * flash_konfig.h
 *
 *  Created on: 15 maj 2019
 *      Author: PitLab
 */

#ifndef FLASH_KONFIG_H_
#define FLASH_KONFIG_H_

#include "flash_nor.h"

#define ROZMIAR_PACZKI_KONFIGU	32
#define ADRES_SEKTORA0	ADRES_NOR
#define ADRES_SEKTORA1	ADRES_SEKTORA0 + ROZMIAR8_SEKTORA



//typy paczek danych z konfiguracją we flash
#define FKON_KALIBRACJA_DOTYKU		0x00	//współczynniki kalibracji panelu dotykowego


#define LICZBA_TYPOW_PACZEK	1




void IncjujKonfigFlash(void);
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
