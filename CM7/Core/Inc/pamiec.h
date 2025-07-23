/*
 * pamiec.h
 *
 *  Created on: Jul 23, 2025
 *      Author: PitLab
 */

#ifndef INC_PAMIEC_H_
#define INC_PAMIEC_H_

#include "sys_def_CM7.h"

void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command);
void TestPredkosciOdczytuNOR(void);
void TestPredkosciOdczytuRAM(void);
uint8_t TestPredkosciZapisuNOR(void);
void InicjujMDMA(void);
void MDMA_TransferCompleteCallback(MDMA_HandleTypeDef *hmdma);
void MDMA_TransferErrorCallback(MDMA_HandleTypeDef *hmdma);
uint8_t SprawdzMagistrale(void);

#endif /* INC_PAMIEC_H_ */
