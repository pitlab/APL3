/*
 * wymiana.h
 *
 *  Created on: Dec 5, 2024
 *      Author: PitLab
 */

#ifndef INC_WYMIANA_H_
#define INC_WYMIANA_H_
#include "stm32h7xx_hal.h"

#ifndef HSEM_CM4_TO_CM7
#define HSEM_CM4_TO_CM7 (1U)
#endif

#ifndef HSEM_CM7_TO_CM4
#define HSEM_CM7_TO_CM4 (2U)
#endif

#define ROWNAJ_DO32B(adres)				((adres + 3) & 0xFFFFFFFC)

#define ROZMIAR_BUF8_WYMIANY_CM4		ROWNAJ_DO32B(sizeof(stWymianyCM4_t))
#define ROZMIAR_BUF8_WYMIANY_CM7		ROWNAJ_DO32B(sizeof(stWymianyCM7_t))
#define ROZMIAR_BUF32_WYMIANY_CM4		ROZMIAR_BUF8_WYMIANY_CM4 / 4
#define ROZMIAR_BUF32_WYMIANY_CM7		ROZMIAR_BUF8_WYMIANY_CM7 / 4
#define ROZMIAR_BUF_NAPISU_WYMIANY		32
typedef struct _GNSS
{
	double dDlugoscGeo;
	double dSzerokoscGeo;
	float fHdop;
	float fPredkoscWzglZiemi;
	float fKurs;
	float fWysokoscMSL;
	uint8_t chLiczbaSatelit;
	uint8_t chFix;
	uint8_t chGodz, chMin, chSek;
	uint8_t chDzien, chMies, chRok;
} stGnss_t;


//definicja struktury wymiany danych wychodzących z rdzenia CM4
typedef struct _stWymianyCM4
{
	float fAkcel1[3];
	float fAkcel2[3];
	float fZyro1[3];
	float fZyro2[3];
	float fMagn1[3];
	float fMagn2[3];
	float fKatyIMU[3];
	float fWysokosc[2];
	uint16_t sSerwa[16];
	uint8_t chBledyPetliGlownej;
	uint32_t nZainicjowano;
	stGnss_t stGnss1;
	char chNapis[ROZMIAR_BUF_NAPISU_WYMIANY];
} stWymianyCM4_t;

//definicja struktury wymiany danych wychodzących z rdzenia CM7
typedef struct
{
	uint8_t chTrybPracy;
	uint16_t sTest;
} stWymianyCM7_t;

//unie do konwersji struktur na słowa 32-bitowe
typedef union
{
	stWymianyCM4_t dane;
	uint32_t nSlowa[ROZMIAR_BUF32_WYMIANY_CM4];
} unia_wymianyCM4_t;

typedef union
{
	stWymianyCM7_t dane;
	uint32_t nSlowa[ROZMIAR_BUF32_WYMIANY_CM7];
} unia_wymianyCM7_t;


#endif /* INC_WYMIANA_H_ */
