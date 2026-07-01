/*
 * analiza_drgań.h
 *
 *  Created on: 31 mar 2026
 *      Author: PitLab
 */

#ifndef INC_ANALIZA_DRGAN_H_
#define INC_ANALIZA_DRGAN_H_

#include <FFT.h>
#include "SysDefCM7.h"
#include "Wymiana.h"

typedef struct
{
	uint16_t sWysterowanie;
	uint16_t sCzasIdent;
	uint32_t nCzasPoprzedniegoEtapu;
	uint8_t cLiczbaSilnikow;
	uint8_t cNumerEtapu;
	//float fKatSilnika[KANALY_MIKSERA];	//kąt między osią X a ramieniem silnika [rad]
	float fSkładowaPochylenia[KANALY_MIKSERA];
	float fSkładowaPrzechylenia[KANALY_MIKSERA];
} stIdentyfikacjaSilnikow_t;

uint8_t RozpocznijAnalizęDrgań(stFFT_t *stKonfigFFT, uint8_t *chTrybPracy);
uint8_t KrokAnalizyDrgań(stFFT_t *stKonfigFFT, uint8_t *chTrybPracy);
uint8_t RozpocznijIdentyfikacjęSilników(stIdentyfikacjaSilnikow_t *stIdentSiln, uint8_t *chTrybPracy);
uint8_t IdentyfikacjaSilników(stIdentyfikacjaSilnikow_t *stIdentSiln);

#endif /* INC_ANALIZA_DRGAN_H_ */
