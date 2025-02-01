/*
 * mmc3416x.h
 *
 *  Created on: Feb 1, 2025
 *      Author: PitLab
 */

#ifndef INC_MMC3416X_H_
#define INC_MMC3416X_H_
#include "sys_def_CM4.h"

#define MMC34160_I2C_ADR	0x60
#define READ				0x01

#define PMMC3416_XOUT_L 	0x00 //Xout LSB
#define PMMC3416_XOUT_H		0x01 //Xout MSB
#define PMMC3416_YOUT_L 	0x02 //Yout LSB
#define PMMC3416_YOUT_H 	0x03 //Yout MSB
#define PMMC3416_ZOUT_L 	0x04 //Zout LSB
#define PMMC3416_ZOUT_H 	x005 //Zout MSB
#define PMMC3416_STATUS		0x06 //Device status
#define PMMC3416_INT_CTRL0 	0x07 //Control register 0
#define PMMC3416_INT_CTRL1 	0x08 //Control register 1
#define PMMC3416_R0 		0x1B //Factor used register
#define PMMC3416_R1 		0x1C //Factory used register
#define PMMC3416_R2 		0x1D //Factory used register
#define PMMC3416_R3 		0x1E //Factory used register
#define PMMC3416_R4 		0x1F //Factory used register
#define PMMC3416_PRODUCT_ID 0x20 //Product ID = 0x06


uint8_t InicjujMMC3416x(void);
uint8_t StartujPomiarMMC3416x(void);
uint8_t StartujOdczytMMC3416x(void);
uint8_t CzytajMMC3416x(void);

#endif /* INC_MMC3416X_H_ */
