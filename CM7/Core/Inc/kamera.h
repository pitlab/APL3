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
#define ROZMIAR_STRUKTURY_REJESTROW_KAMERY 20
#define SKALA_POZIOMU_EKSPOZYCJI	0x96
#define KROK_ROZDZ_KAM	16		//najmnijeszy krok zmiany rozmiaru obrazu o tyle pikseli. Umożliwia wysłanie rozmiaru jako liczby 8-bitowej

//tryby pracy kamery
#define KAM_ZDJECIE	1
#define KAM_FILM	0

//formaty obrazu
#define OBR_Y8		0x10	//Y8		YYYY
#define OBR_YUV444	0x20	//YAU444	YUVYUV/GBRGBR
#define OBR_YUV422	0x30	//YAU422	YUYV
#define OBR_YUV420	0x40	//YUV420	YYYY/YUYV
#define OBR_RGB565 	0x6F	//RGB565	{g[2:0], b[4:0]}, {r[4:0], g[5:3]}

#define MASKA_BUFORA_KAMERY	0x07


//konfiguracja kamery
//typedef struct st_KonfKam
typedef struct
{
	uint8_t chSzerWe;			//szerokość patrzenia przetwornika obrazu / KROK_ROZDZ_KAM
	uint8_t chWysWe;			//wysokość patrzenia przetwornika obrazu / KROK_ROZDZ_KAM
	uint8_t chSzerWy;			//szerokość obrazu wychodzącego z kamery / KROK_ROZDZ_KAM
	uint8_t chWysWy;			//wysokość obrazu wychodzącego z kamery / KROK_ROZDZ_KAM
	uint8_t chPrzesWyPoz;		//przesunięcie obrazu względem początku matrycy w poziomie
	uint8_t chPrzesWyPio;		//przesunięcie obrazu względem początku matrycy w pionie
	uint8_t chTrybPracy;		//KAM_ZDJECIE lub KAM_FILM
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

//diagnostyka stanu kamery
typedef struct st_DiagKam
{
	uint16_t sPoczatOknaAVG_X;		//0x5680..01
	uint16_t sKoniecOknaAVG_X;		//0x5682..03
	uint16_t sPoczatOknaAVG_Y;		//0x5684..05
	uint16_t sKoniecOknaAVG_Y;		//0x5686..07
	uint16_t sPoczatOknaObrazu_X;	//0x3800..01
	uint16_t sPoczatOknaObrazu_Y;	//0x3802..03
	uint16_t sRozmiarOknaObrazu_X;	//0x3804..05
	uint16_t sRozmiarOknaObrazu_Y;	//0x3806..07
	uint8_t chTrybEAC_EAG;			//0x3503
	uint8_t chSredniaJasnoscAVG;	//0x5690
	uint8_t chProgAEC_H;			//0x3A0F
	uint8_t chProgAEC_L;			//0x3A10
	uint8_t chProgStabAEC_H;		//0x3A1B
	uint8_t chProgStabAEC_L;		//0x3A1E
	uint16_t sMaxCzasEkspoVTS;		//0x350C..0D
	uint16_t sRozmiarPoz_HTS;		//0x380C..0D
	uint16_t sRozmiarPio_VTS;		//0x380E..0F
} stDiagKam_t;

uint8_t InicjalizujKamere(void);
uint8_t Wyslij_I2C_Kamera(uint16_t rejestr, uint8_t dane);
uint8_t Czytaj_I2C_Kamera(uint16_t rejestr, uint8_t *dane);
uint8_t	SprawdzKamere(void);
uint8_t UstawKamere(stKonfKam_t *konf);
uint8_t UstawKamere2(stKonfKam_t *konf);
uint8_t RozpocznijPraceDCMI(stKonfKam_t *konfig, uint16_t* sBufor, uint32_t nRozmiarObrazu32bit);
uint8_t ZakonczPraceDCMI(void);
//uint8_t CzekajNaKoniecPracyDCMI(void);
uint8_t CzekajNaKoniecPracyDCMI(uint16_t sWysokoscZdjecia);
uint8_t ZrobZdjecie(uint16_t* sBufor, uint32_t nRozmiarObrazu32bit);
uint8_t Wyslij_Blok_Kamera(const struct sensor_reg reglist[]);
//uint8_t UstawRozdzielczoscKamery(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chZoom);
uint8_t UtworzGrupeRejestrowKamery(uint8_t chNrGrupy, struct sensor_reg *stListaRejestrow, uint8_t chLiczbaRejestrow);
uint8_t UruchomGrupeRejestrowKamery(uint8_t chNrGrupy);

//uint8_t UstawObrazKameryYUV420(uint16_t sSzerokosc, uint16_t sWysokosc);
//uint8_t UstawObrazKameryRGB565(uint16_t sSzerokosc, uint16_t sWysokosc);
//uint8_t UstawObrazKameryY8(uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t UstawObrazKamery(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chFormatObrazu, uint8_t chTrybPracy);
uint8_t KompresujObrazY8(uint8_t* chBufKamery, uint8_t* chBufLCD, uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t KompresujObrazYUV420(uint8_t* chBufKamery, uint8_t* chBufLCD, uint16_t sSzerokosc, uint16_t sWysokosc);
void CzyscBufory(void);
uint8_t WykonajDiagnostykeKamery(stDiagKam_t* stDiagKam);

void DCMI_DMAXferCplt(struct __DMA_HandleTypeDef * hdma);
void DCMI_DMAXferHalfCplt(DMA_HandleTypeDef * hdma);
void DCMI_DMAError(struct __DMA_HandleTypeDef * hdma);

#endif /* INC_KONFIGURACJA_H_ */
