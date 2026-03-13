/*
 * ws281x.h
 *
 *  Created on: 12 mar 2026
 *      Author: PitLab
 */

#ifndef INC_WS281X_H_
#define INC_WS281X_H_
#include "sys_def_CM4.h"

#define LICZBA_LED_WS281X	20		//taśma ma 30 LED/m, więc 0,7m * 30 = 21. LEDy są w sekcjach po 4, więc ograniczam liczbę do 20
#define WS_BITOW_KOLORU		24
#define WS_LEDY_SEGMENTU	4
#define WS_BITOW_LACZNIE	(WS_BITOW_KOLORU * WS_LEDY_SEGMENTU + 1)


//timery sterowane są zegarem 240 MHz co daje czas cyklu 4,16ns
#define DZIELNIK_WS281X		64	//220..380 ns / 4,16 = 53..91; 64*4,16 = 266ns
#define ZEGAR_WS281X		3750000	//Hz
#define CZAS_WS281X_0H		1		//1 * 266ns
#define CZAS_WS281X_1H		3		//3 * 266ns = 799ns
#define CZAS_WS281X_BIT		4		//4 * 266ns = 1,064us
#define CZAS_WS281X_RESET	1088	//1088 * 266ns = 289,4us

typedef struct
{
	float fWartoscMax;
	uint8_t chCzerMax;
	uint8_t chZielMax;
	uint8_t chNiebMax;
	float fWartoscMin;
	uint8_t chCzerMin;
	uint8_t chZielMin;
	uint8_t chNiebMin;
	uint8_t chDzielnikJasnosciTla;
} stPaletaKolorow_t;

uint8_t UstawTrybWS281x(uint8_t chKanal);
uint8_t AktualizujWS281xDMA(uint32_t *nKolor, uint8_t chRozmiar, uint8_t *chWskSegmentu);

#endif /* INC_WS281X_H_ */
