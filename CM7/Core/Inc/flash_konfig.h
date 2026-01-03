/*
 * flash_konfig.h
 *
 *  Created on: 15 maj 2019
 *      Author: PitLab
 */

#ifndef FLASH_KONFIG_H_
#define FLASH_KONFIG_H_

#include "flash_nor.h"


#define ROZMIAR_PACZKI_KONFIG	32
#define ROZMIAR_PACZKI_KONF16	16
#define ROZMIAR_DANYCH_WPACZCE	(ROZMIAR_PACZKI_KONFIG - 2)
#define ADRES_SEKTORA0			ADRES_NOR
#define ADRES_SEKTORA1			ADRES_SEKTORA0 + ROZMIAR_SEKTORA



//typy paczek danych z konfiguracją we flash
#define FKON_KALIBRACJA_DOTYKU		0x00	//współczynniki kalibracji panelu dotykowego
#define FKON_NAZWA_ID_BSP			0x01	//struktura identyfikacyjna BSP: ID, nazwa i numer IP
#define FKON_OKRES_TELEMETRI1		0x02	//okresy wysyłania zmiennych telemetrycznych 0
#define FKON_OKRES_TELEMETRI2		0x03	//okresy wysyłania zmiennych telemetrycznych 15
#define FKON_OKRES_TELEMETRI3		0x04	//okresy wysyłania zmiennych telemetrycznych 30
#define FKON_OKRES_TELEMETRI4		0x05	//okresy wysyłania zmiennych telemetrycznych 45
#define FKON_OKRES_TELEMETRI5		0x06	//okresy wysyłania zmiennych telemetrycznych 60
#define FKON_OKRES_TELEMETRI6		0x07	//okresy wysyłania zmiennych telemetrycznych 75
#define FKON_OKRES_TELEMETRI7		0x08	//okresy wysyłania zmiennych telemetrycznych 90
#define FKON_OKRES_TELEMETRI8		0x09	//okresy wysyłania zmiennych telemetrycznych 105
#define FKON_OKRES_TELEMETRI9		0x0A	//okresy wysyłania zmiennych telemetrycznych 120
#define FKON_OKRES_TELEMETRI10		0x0B	//okresy wysyłania zmiennych telemetrycznych 135
#define FKON_OKRES_TELEMETRI11		0x0C	//okresy wysyłania zmiennych telemetrycznych 150
#define FKON_OKRES_TELEMETRI12		0x0D	//okresy wysyłania zmiennych telemetrycznych 175
#define FKON_OKRES_TELEMETRI13		0x0E	//okresy wysyłania zmiennych telemetrycznych 180
#define FKON_OKRES_TELEMETRI14		0x0F	//okresy wysyłania zmiennych telemetrycznych 195
#define FKON_OKRES_TELEMETRI15		0x10	//okresy wysyłania zmiennych telemetrycznych 210
#define FKON_OKRES_TELEMETRI16		0x11	//okresy wysyłania zmiennych telemetrycznych 225
#define FKON_OKRES_TELEMETRI17		0x12	//okresy wysyłania zmiennych telemetrycznych 240
#define LICZBA_TYPOW_PACZEK			0x13


uint8_t InicjujKonfigFlash(void);
uint8_t ZapiszPaczkeKonfigu(uint8_t chIdPaczki, uint8_t* chDane);
uint8_t ZapiszPaczkeAdr(uint8_t chIdPaczki, uint8_t* chDane, uint32_t nAdres);
uint8_t CzytajPaczkeKonfigu(uint8_t* chDane, uint8_t chIdPaczki);
float KonwChar2Float(unsigned char *chData);
void KonwFloat2Char(float fData, unsigned char *chData);
unsigned short KonwChar2UShort(unsigned char *chData);
signed short KonwChar2SShort(unsigned char *chData);
void KonwUShort2Char(unsigned short sData, unsigned char *chData);
void KonwSShort2Char(signed short sData, unsigned char *chData);
unsigned char SprawdzPaczke(unsigned int nAdres);
uint8_t PrzepiszDane(void);

#endif /* FLASH_KONFIG_H_ */
