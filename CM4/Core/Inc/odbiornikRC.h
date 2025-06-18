/*
 * odbiornikRC.h
 *
 *  Created on: Jun 15, 2025
 *      Author: PitLab
 */

#ifndef INC_ODBIORNIKRC_H_
#define INC_ODBIORNIKRC_H_

#include "sys_def_CM4.h"
#include "wymiana.h"


#define CZAS_RAMKI_PPM_RC	30000	//Czas między kolejnymi ramkami PPM odbiornika RC [us]
#define ROZMIAR_BUF_ODB_SBUS	16

typedef struct
{
	volatile int16_t sOdb1[KANALY_ODB_RC];	//dane z odbiornika RC1
	volatile int16_t sOdb2[KANALY_ODB_RC];	//dane z odbiornika RC2
	volatile uint8_t chStatus1;				//określaja stan odbioru RC1
	volatile uint8_t chStatus2;				//określaja stan odbioru RC2
	uint8_t chNrKan1;						//wskaźnik numeru kanału odbiornika 1
	uint8_t chNrKan2;						//wskaźnik numeru kanału odbiornika 2
	uint32_t nCzasWe1;						//czas początku dekodowania  ostatniego pakietu danych z odbiornika 1
	uint32_t nCzasWe2;						//czas początku dekodowania  ostatniego pakietu danych z odbiornika 2
	uint16_t sZdekodowaneKanaly1;			//bity zdekodowanych kanałów odbiornika 1
	uint16_t sZdekodowaneKanaly2;			//bity zdekodowanych kanałów odbiornika 2
	volatile uint16_t sPoprzedniaWartoscTimera1;	//zapamiętuje wartość timera wejścia odbiornika 1 dla poprzedniego zbocza aby z tego policzyć czas
	volatile uint16_t sPoprzedniaWartoscTimera2;	//zapamiętuje wartość timera wejścia odbiornika 2 dla poprzedniego zbocza aby z tego policzyć czas
} stRC_t;


//deklaracja funkcji
uint8_t InicjujOdbiornikiRC(void);
uint8_t DywersyfikacjaOdbiornikowRC(stRC_t* psRC,  stWymianyCM4_t* psDaneCM4);


#endif /* INC_ODBIORNIKRC_H_ */
