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

#define ROZMIAR_BUF8_WYMIANY_CM4		ROWNAJ_DO32B(sizeof(stWymianyCM4))
#define ROZMIAR_BUF8_WYMIANY_CM7		ROWNAJ_DO32B(sizeof(stWymianyCM7))
#define ROZMIAR_BUF32_WYMIANY_CM4		ROZMIAR_BUF8_WYMIANY_CM4 / 4
#define ROZMIAR_BUF32_WYMIANY_CM7		ROZMIAR_BUF8_WYMIANY_CM7 / 4



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
} stWymianyCM4;

//definicja struktury wymiany danych wychodzących z rdzenia CM7
typedef struct
{
	uint8_t chTrybPracy;
	uint16_t sTest;
} stWymianyCM7;

//unie do konwersji struktur na słowa 32-bitowe
typedef union
{
	stWymianyCM4 dane;
	uint32_t nSlowa[ROZMIAR_BUF32_WYMIANY_CM4];
} unia_wymianyCM4;

typedef union
{
	stWymianyCM7 dane;
	uint32_t nSlowa[ROZMIAR_BUF32_WYMIANY_CM7];
} unia_wymianyCM7;


#endif /* INC_WYMIANA_H_ */
