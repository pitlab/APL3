/*
 * dshot.h
 *
 *  Created on: Jul 29, 2025
 *      Author: Piotr
 */

#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_
#include "sys_def_CM4.h"
#include "wymiana.h"
#include "konfig_fram.h"


/*/definicje timingu
#define ZEGAR_TIMEROW		216000000	//MHz
#define PODZIELNIK_TIMERA	34
#define ZEGAR_TIMERA		(ZEGAR_TIMEROW / PODZIELNIK_TIMERA)		//6 MHz
#define CZAS_OKRESU			(1.0f / ZEGAR_TIMERA)		//0,16667 us*/

#define DS150_ZEGAR			6000000
#define DS300_ZEGAR			12000000
#define DS600_ZEGAR			24000000
#define DS1200_ZEGAR		48000000

/*#define DS150_CZAS_T0H		2.5
#define DS150_CZAS_T1H		5.0
#define DS150_CZAS_BIT		6.6667
#define DS150_OKRESOW_T0H	(DS150_CZAS_T0H / (CZAS_OKRESU * 1000000))	//15
#define DS150_OKRESOW_T1H	(DS150_CZAS_T1H / (CZAS_OKRESU * 1000000))	//30
#define DS150_OKRESOW_BIT	(DS150_CZAS_BIT / (CZAS_OKRESU * 1000000))	//40

#define DS300_CZAS_T0H		1.25
#define DS300_CZAS_T1H		2.5
#define DS300_CZAS_BIT		3.3333
#define DS300_OKRESOW_T0H	(DS300_CZAS_T0H / (CZAS_OKRESU * 1000000))	//7
#define DS300_OKRESOW_T1H	(DS300_CZAS_T1H / (CZAS_OKRESU * 1000000))	//15
#define DS300_OKRESOW_BIT	(DS300_CZAS_BIT / (CZAS_OKRESU * 1000000))	//20*/

//definicje protokołu DShot
#define DS_BITOW_GAZU		11
#define DS_BITOW_TELE		1
#define DS_BITOW_CRC		4
#define DS_BITOW_LACZNIE	17
#define DS_OFFSET_DANYCH	48		//wartości przesyłane protokołem są powiększone o taką wartość

//definicje protokołów
#define PROTOKOL_DSHOT150	1
#define PROTOKOL_DSHOT300	2
#define PROTOKOL_DSHOT600	3
#define PROTOKOL_DSHOT1200	4

typedef struct
{
	uint32_t nT0H;
	uint32_t nT1H;
	uint32_t nBit;
} stDShot_t;


//deklaraje funkcji
uint8_t UstawTrybDShot(uint8_t chProtokol, uint8_t chKanal);
uint8_t AktualizujDShotDMA(uint16_t sWysterowanie, uint8_t chKanal);


#endif /* INC_DSHOT_H_ */
