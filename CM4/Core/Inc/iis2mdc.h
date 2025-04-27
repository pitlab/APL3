/*
 * iis2mdc.h
 *
 *  Created on: Feb 2, 2025
 *      Author: PitLab
 */

#ifndef INC_IIS2MDC_H_
#define INC_IIS2MDC_H_
#include "sys_def_CM4.h"

#define IIS2MDC_I2C_ADR		0x3C
#define READ				0x01


#define CZULOSC_IIS2MDC	1.5e-7	//1 bit odpowiada 1,5 mGauss a 1 mGauss to 100nT

//Hard-iron registers
#define PIIS2MDS_OFFSET_X_REG_L 0x45
#define PIIS2MDS_OFFSET_X_REG_H 0x46
#define PIIS2MDS_OFFSET_Y_REG_L 0x47
#define PIIS2MDS_OFFSET_Y_REG_H 0x48
#define PIIS2MDS_OFFSET_Z_REG_L 0x49
#define PIIS2MDS_OFFSET_Z_REG_H 0x4A
#define PIIS2MDS_WHO_AM_I		0x4F		//zwraca 0x40

//Configuration registers
#define PIIS2MDS_CFG_REG_A 		0x60
#define PIIS2MDS_CFG_REG_B 		0x61
#define PIIS2MDS_CFG_REG_C 		0x62
#define PIIS2MDS_INT_CRTL_REG 	0x63

//Interrupt configuration registers
#define PIIS2MDS_INT_SOURCE_REG 0x64
#define PIIS2MDS_INT_THS_L_REG 	0x65
#define PIIS2MDS_INT_THS_H_REG 	0x66
#define PIIS2MDS_STATUS_REG 	0x67


//Output registers
#define PIIS2MDS_OUTX_L_REG 	0x68
#define PIIS2MDS_OUTX_H_REG 	0x69
#define PIIS2MDS_OUTY_L_REG 	0x6A
#define PIIS2MDS_OUTY_H_REG 	0x6B
#define PIIS2MDS_OUTZ_L_REG 	0x6C
#define PIIS2MDS_OUTZ_H_REG 	0x6D
#define PIIS2MDS_TEMP_OUT_L_REG 0x6E
#define PIIS2MDS_TEMP_OUT_H_REG 0x6F


uint8_t InicjujIIS2MDC(void);
uint8_t ObslugaIIS2MDC(void);
uint8_t StartujOdczytIIS2MDC(uint8_t chRejestr);
uint8_t CzytajIIS2MDC(void);

#endif /* INC_IIS2MDC_H_ */
