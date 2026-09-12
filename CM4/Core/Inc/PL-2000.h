/*
 * PL-2000.h
 *
 *  Created on: 12 wrz 2026
 *      Author: PitLab
 */

#ifndef INC_PL_2000_H_
#define INC_PL_2000_H_
#include "SysDefCM4.h"
#include "wymiana.h"


#define fff								(1 / 298.257222101)
#define POLOS_WIELKA_ELIPSOIDY_GRS80	6378137.0	//we wzorach to a [m]

#define MIMOSROD_GRS80_E2	0.00669438002290	//kwadrat mimośrodu elipsy GRS80
#define MIMOSROD_GRS80_E4	0.00004481472389	//mimośród elipsy GRS80 do potęgi 4
#define MIMOSROD_GRS80_E6	0.00000030000679	//mimośród elipsy GRS80 do potęgi 6
#define MIMOSROD_GRS80_E8	0.00000000200836	//mimośród elipsy GRS80 do potęgi 8
#define MIMOSROD_GRS80_EP2	(MIMOSROD_GRS80_E2 / (1.0 - MIMOSROD_GRS80_E2))	//kwadrat drugiego mimośrodu elipsy GRS80

//współczynniki do obliczenia długości łuku południka
#define WSP_A0 	(1.0 - (MIMOSROD_GRS80_E2 / 4) - (3 * MIMOSROD_GRS80_E4 / 64) - (5*  MIMOSROD_GRS80_E6 / 256) - (175 * MIMOSROD_GRS80_E8 / 16384))
#define WSP_A2	(3.0 / 8.0 * (MIMOSROD_GRS80_E2 + MIMOSROD_GRS80_E4 / 4 + 15 * MIMOSROD_GRS80_E6 / 128 - 455 * MIMOSROD_GRS80_E8 / 4096))
#define WSP_A4	(15.0 / 256.0 * (MIMOSROD_GRS80_E4 + 3 * MIMOSROD_GRS80_E6 / 4 + 35 * MIMOSROD_GRS80_E8 / 64))
#define WSP_A6	(35.0 / 3072.0 * (MIMOSROD_GRS80_E6 - 5 * MIMOSROD_GRS80_E8 / 4))
#define WSP_A8	(315 * MIMOSROD_GRS80_E8 / 131072)

#define SZEROKOSC_STREFY_PL2000	3.0f
#define SKALA_ODWZOROWANIA_M0	0.999923


uint8_t ZamieńGNSSnaPL_2000d(stGnss_t *stGnss, stBSP_t *stBSP);
uint8_t ZamieńGNSSnaPL_2000f(stGnss_t *stGnss, stBSP_t *stBSP);

#endif /* INC_PL_2000_H_ */
