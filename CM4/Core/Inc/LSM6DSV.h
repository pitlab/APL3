/*
 * LSM6DSV.h
 *
 *  Created on: Feb 1, 2025
 *      Author: PitLab
 */

#ifndef INC_LSM6DSV_H_
#define INC_LSM6DSV_H_
#include "sys_def_CM4.h"

//definicje rejestrów
#define PLSC6DSV_SPI2_WHO_AM_I 		0x0F		//odpowiedź 0x7F
#define PLSC6DSV_SPI2_STAT_REG_OIS  0x1E
#define PLSC6DSV_SPI2_OUT_TEMP_L 	0x20
#define PLSC6DSV_SPI2_OUT_TEMP_H  	0x21
#define PLSC6DSV_SPI2_OUTX_L_G_OIS  0x22
#define PLSC6DSV_SPI2_OUTX_H_G_OIS  0x23
#define PLSC6DSV_SPI2_OUTY_L_G_OIS  0x24
#define PLSC6DSV_SPI2_OUTY_H_G_OIS  0x25
#define PLSC6DSV_SPI2_OUTZ_L_G_OIS  0x26
#define PLSC6DSV_SPI2_OUTZ_H_G_OIS  0x27
#define PLSC6DSV_SPI2_OUTX_L_A_OIS  0x28
#define PLSC6DSV_SPI2_OUTX_H_A_OIS  0x29
#define PLSC6DSV_SPI2_OUTY_L_A_OIS  0x2A
#define PLSC6DSV_SPI2_OUTY_H_A_OIS  0x2B
#define PLSC6DSV_SPI2_OUTZ_L_A_OIS  0x2C
#define PLSC6DSV_SPI2_OUTZ_H_A_OIS  0x2D
#define PLSC6DSV_SPI2_HANDSH_CTRL 	0x6E
#define PLSC6DSV_SPI2_INT_OIS		0x6F
#define PLSC6DSV_SPI2_CTRL1_OIS		0x70
#define PLSC6DSV_SPI2_CTRL2_OIS		0x71
#define PLSC6DSV_SPI2_CTRL3_OIS		0x72


uint8_t InicjujLSM6DSV(void);
uint8_t ObslugaLSM6DSV(void);

#endif /* INC_LSM6DSV_H_ */
