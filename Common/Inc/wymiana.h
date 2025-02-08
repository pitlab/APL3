/*
 * wymiana.h
 *
 *  Created on: Dec 5, 2024
 *      Author: PitLab
 */

#ifndef INC_WYMIANA_H_
#define INC_WYMIANA_H_
#include "stm32h7xx_hal.h"
#include "semafory.h"


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
	float fZyros1[3];
	float fZyros2[3];
	int16_t sMagne1[3];
	int16_t sMagne2[3];
	int16_t sMagne3[3];
	float fKatIMU[3];
	float fCisnie[2];
	float fWysoko[2];
	float fTemper[5];	//0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC
	uint16_t sSerwa[16];
	uint8_t chErrPetliGlownej;
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
