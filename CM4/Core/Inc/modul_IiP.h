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
//#define KELWIN					273.15f

#define LICZBA_PROBEK_USREDNIANIA		1500	//tyle trzeba aby filtr (127+1)/128 uzyskał dokładność 6 cyfr znaczących

//definicje rodzajów temepratur
#define ZIM		0
#define POK		1
#define GOR		2


typedef struct
{
	float fAzim[3];		//współczynnik A równania dla "zimnej" temperatury
	float fBzim[3];
	float fAgor[3];
	float fBgor[3];		//Współczynnik B równanie dla "gorącej" temperatury
	float fTempPok;		//temperatura pokojowa jako punkt przecięcia prostych
} WspRownProstej3_t;

typedef struct
{
	float fAzim;		//współczynnik A równania dla "zimnej" temperatury
	float fBzim;
	float fAgor;
	float fBgor;		//Współczynnik B równanie dla "gorącej" temperatury
	float fTempPok;		//temperatura pokojowa jako punkt przecięcia prostych
} WspRownProstej1_t;


uint8_t InicjujModulI2P(void);
uint8_t ObslugaModuluI2P(uint8_t gniazdo);
float WysokoscBarometryczna(float fP, float fP0, float fTemp);
uint8_t RozpocznijKalibracjeZeraZyroskopu(uint8_t chRodzajKalib);
uint8_t KalibrujZeroZyroskopu(void);
uint8_t KalibracjaWzmocnieniaZyro(uint8_t chRodzajKalib);
void ObliczRownanieFunkcjiTemperatury(float fOffset1, float fOffset2, float fTemp1, float fTemp2, float *fA, float *fB);
void ObliczOffsetTemperaturowy3(WspRownProstej3_t stWsp, float fTemp, float *fOffset);
float ObliczOffsetTemperaturowy1(WspRownProstej1_t stWsp, float fTemp);
uint8_t KalibrujCisnienie(float fCisnienie1, float fCisnienie2, float fTemp, uint16_t sLicznik, uint8_t chPrzebieg);

#endif /* INC_MODUL_IIP_H_ */
