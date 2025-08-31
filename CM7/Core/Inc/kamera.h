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

#define ROZMIAR_STRUKTURY_REJESTROW_KAMERY 20
#define SKALA_POZIOMU_EKSPOZYCJI	96


//konfiguracja kamery
typedef struct st_KonfKam
{
	uint8_t chSzerWe;
	uint8_t chWysWe;
	uint8_t chSzerWy;
	uint8_t chWysWy;
	uint8_t chPrzesWyPoz;		//przesunięcie obrazu względem początku matrycy w poziomie
	uint8_t chPrzesWyPio;		//przesunięcie obrazu względem początku matrycy w pionie
//	uint8_t chTrybDiagn;
	uint8_t chObracanieObrazu;	//Timing Control: 0x3818
	uint8_t chFormatObrazu;		//Format Control 0x4300
	uint16_t sWzmocnienieR;		//AWB R Gain: 0x3400..01
	uint16_t sWzmocnienieG;		//AWB G Gain: 0x3402..03
	uint16_t sWzmocnienieB;		//AWB B Gain: 0x3404..05
	uint8_t chKontrBalBieli;	//AWB Manual: 0x3406
	uint16_t sCzasEkspozycji;	//AEC Long Channel Exposure [19:0]: 0x3500..02
	uint8_t chKontrolaExpo;		//AEC Manual Mode Control: 0x3503
	uint8_t chTrybyEkspozycji;	//AEC System Control 0: 0x3A00
	uint16_t sAGCLong;			//AEC PK Long Gain 0x3508..09
	uint16_t sAGCAdjust;		//AEC PK Long Gain 0x350A..0B
	uint16_t sAGCVTS;			//AEC PK VTS Output 0x350C..0D
	uint8_t chKontrolaISP0;		//0x5000
	uint8_t chKontrolaISP1;		//0x50001
	uint8_t chProgUsuwania;		//0x5080 Even CTRL 00 Treshold for even odd  cancelling
	uint8_t chNasycenie;		//SDE Control3 i 4: 0x5583 Saturation U i 5584 Saturation V
	uint8_t chPoziomEkspozycji;	//wpływa na grupę rejestrów: 0x3A0F..1F
} stKonfKam_t;



uint8_t InicjalizujKamere(void);
uint8_t Wyslij_I2C_Kamera(uint16_t rejestr, uint8_t dane);
uint8_t Czytaj_I2C_Kamera(uint16_t rejestr, uint8_t *dane);
uint8_t	SprawdzKamere(void);
uint8_t UstawKamere(stKonfKam_t *konf);
uint8_t UstawKamere2(stKonfKam_t *konf);
uint8_t RozpocznijPraceDCMI(uint8_t chAparat);
uint8_t ZrobZdjecie(void);
uint8_t Wyslij_Blok_Kamera(const struct sensor_reg reglist[]);
uint8_t UstawRozdzielczoscKamery(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chZoom);
uint8_t UtworzGrupeRejestrowKamery(uint8_t chNrGrupy, struct sensor_reg *stListaRejestrow, uint8_t chLiczbaRejestrow);
uint8_t UruchomGrupeRejestrowKamery(uint8_t chNrGrupy);


void UstawDomyslny(void);
void Ustaw1(void);
void Ustaw2(void);
void Ustaw3(void);


#endif /* INC_KONFIGURACJA_H_ */
