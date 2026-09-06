/*
 * VL53L1.h
 *
 *  Created on: 5 wrz 2026
 *      Author: PitLab
 */

#ifndef INC_VL53L1_H_
#define INC_VL53L1_H_
#include "SysDefCM4.h"

#define TIMEOUT_VL53C1		10	//timerout operacji na I2C

uint8_t InicjujVL53L1(void);
uint8_t ObsługaVL53L1(void);

int32_t TOF_I2C_Init(void);
int32_t TOF_I2C_DeInit(void);
int32_t TOF_GetTick(void);
int32_t TOF_WriteReg(uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t TOF_ReadReg(uint16_t Reg, uint8_t *pData, uint16_t Length);


#endif /* INC_VL53L1_H_ */
