/*
 * czas.h
 *
 *  Created on: Feb 14, 2025
 *      Author: PitLab
 */

#ifndef INC_CZAS_H_
#define INC_CZAS_H_

#include "sys_def_CM7.h"
#include "wymiana.h"


//definicje flag zmiennej chStanSynchronizacjiCzasu
#define SSC_CZAS_NIESYNCHR		0x01
#define SSC_DATA_NIESYNCHR		0x02



void SynchronizujCzasDoGNSS(stGnss_t *stGnss);

#endif /* INC_CZAS_H_ */
