/*
 * modul_IiP.h
 *
 *  Created on: Jan 31, 2025
 *      Author: PitLab
 */

#ifndef INC_MODUL_IIP_H_
#define INC_MODUL_IIP_H_
#include "sys_def_CM4.h"
#include <IIS2MDC.h>
#include <MMC3416x.h>

//definicje adresów na module
#define ADR_MIIP_ICM42688	0
#define ADR_MIIP_LSM6DSV	1
#define ADR_MIIP_MS5611		2
#define ADR_MIIP_BMP581		3
#define ADR_MIIP_ND130		4
#define ADR_MIIP_RES_ND130	5
#define ADR_MIIP_GRZALKA	6

//definicje do obliczenia wysokości
#define STALA_GAZOWA_R		 	8.31446261815324f	//J·mol−1·K−1.
#define MASA_MOLOWA_POWIETRZA	0.0289644f			//kg/mol
#define PRZYSPIESZENIE_ZIEMSKIE	9.80665f			//m/s^2
#define KELWIN					273.15f

#define USREDNIONO_POMIAROW	128	//z tylu pomiarow jest uśredniana wartośc cinienie P0 (wartośc 256 i wyżej powoduje utratę wartości liczonej sumy na liczbach float)

uint8_t ObslugaModuluIiP(uint8_t gniazdo);
float WysokoscBarometryczna(float fP, float fP0, float fTemp);

#endif /* INC_MODUL_IIP_H_ */
