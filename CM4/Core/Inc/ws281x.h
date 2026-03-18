/*
 * ws281x.h
 *
 *  Created on: 12 mar 2026
 *      Author: PitLab
 */

#ifndef INC_WS281X_H_
#define INC_WS281X_H_
#include "sys_def_CM4.h"

//typy układu. Różną się układem kolorów
#define WS2811		0	//RGB
#define WS2813		1	//GRB

#define LICZBA_LED_WS281X	40		//taśma ma 30 LED/m, więc 0,7m * 30 = 21. LEDy są w sekcjach po 4, więc ograniczam liczbę do 20 LEDów * 2 wskaźniki
#define WS_BITOW_KOLORU		24
#define WS_LEDY_SEGMENTU	4
#define WS_BITOW_LACZNIE	(WS_BITOW_KOLORU * WS_LEDY_SEGMENTU)
#define WS_CZAS_RESETU		12		//czas trwania cyklu resetu odpowiadajacy czasowi obsługi danej liczbie LED-ów

//timery sterowane są zegarem 240 MHz co daje czas cyklu 4,16ns
#define DZIELNIK_WS281X		64	//220..380 ns / 4,16 = 53..91; 64*4,16 = 266ns
#define ZEGAR_WS281X		3750000	//Hz
#define CZAS_WS281X_0H		1		//1 * 266ns
#define CZAS_WS281X_1H		3		//3 * 266ns = 799ns
#define CZAS_WS281X_BIT		4		//4 * 266ns = 1,064us
#define CZAS_WS281X_RESET	1088	//1088 * 266ns = 289,4us

//Definicje wizualizowanych zmiennych
#define WLZ_POCHYLENIE		0
#define WLZ_PRZECHYLENIE	1
#define WLZ_ODCHYLENIE		2
#define WLZ_WYSOKOSC_AGL	3
#define WLZ_NAPIECIE_BAT	4


typedef struct
{
	float fWartoscMin;
	float fWartoscMax;
	uint8_t chNumZmiennej;
	uint8_t chSzerokoscWskaznika;
	uint8_t chDzielnikJasnosciTla;
	uint8_t chCzerMin;
	uint8_t chCzerMax;
	uint8_t chZielMin;
	uint8_t chZielMax;
	uint8_t chNiebMin;
	uint8_t chNiebMax;
	uint8_t chLiczbaLed;
} stWskaznikLed_t;


uint8_t InicjujKoloryWS281x(void);
uint8_t AktualizujKolorLedWs821x(void);
uint8_t UstawTrybWS281x(uint8_t chKanal);
uint8_t AktualizujWS281xDMA(uint16_t *sFlagi, uint32_t *nTabKoloru, uint8_t chRozmiar, uint8_t *chWskLED);
uint8_t UstawKolorWS281x(uint32_t *nKolor, stWskaznikLed_t *stWskaznikLed);


#endif /* INC_WS281X_H_ */
