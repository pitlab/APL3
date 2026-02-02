/*
 * kontroler_lotu.h
 *
 *  Created on: 24 sty 2026
 *      Author: PitLab
 */

#ifndef INC_KONTROLER_LOTU_H_
#define INC_KONTROLER_LOTU_H_
#include "sys_def_CM4.h"
#include "wymiana.h"

uint8_t InicjujKontrolerLotu(void);
uint8_t KontrolerLotu(uint8_t *chTrybRegulatora, uint32_t ndT, stWymianyCM4_t *dane, stKonfPID_t *konfig);
uint8_t UzbrojSilniki(stWymianyCM4_t *dane);
void RozbrojSilniki(stWymianyCM4_t *dane);


#endif /* INC_KONTROLER_LOTU_H_ */
