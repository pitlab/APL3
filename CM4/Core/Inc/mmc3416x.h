/*
 * mmc3416x.h
 *
 *  Created on: Feb 1, 2025
 *      Author: PitLab
 */

#ifndef INC_MMC3416X_H_
#define INC_MMC3416X_H_
#include "sys_def_CM4.h"


#define CZULOSC_MMC34160	4.88e-7	//0,488 mGauss na bit a 1 mGauss to 100nT

#define MMC34160_I2C_ADR	0x60
#define READ				0x01

//definicje rejestrów układu
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


//definicje Sekwencji Pomiarowej
#define SPMMC3416_REFIL_SET		0	//wyślij polecenie rozpoczęcia ładowania kondensatora do polecenia SET
#define SPMMC3416_CZEKAJ_REFSET	1	//czekaj 50ms na naładowanie
#define SPMMC3416_SET			2	//wyślij polecenie SET
#define SPMMC3416_START_POM_HP	3	//wyślij polecenie wykonania pomiaru H+
#define SPMMC3416_START_STAT_P	4	//wyślij polecenie odczytania statusu
#define SPMMC3416_START_CZYT_HP	5	//wyślij polecenie odczytu pomiaru H+
#define SPMMC3416_REFIL_RESET	6	//wyślij polecenie rozpoczęcia ładowania kondensatora do polecenia RESET
#define SPMMC3416_CZEKAJ_REFRES	7	//czekaj 50ms na naładowanie
#define SPMMC3416_RESET			8	//wyślij polecenie RESET
#define SPMMC3416_START_POM_HM	9	//wyślij polecenie wykonania pomiaru H-
#define SPMMC3416_START_STAT_M	10	//wyślij polecenie odczytania statusu
#define SPMMC3416_START_CZYT_HM	11	//wyślij polecenie odczytu pomiaru H-
#define SPMMC3416_LICZ_OPERACJI	12	//liczba operacji



//definicje poleceń magnetometru MMC3416x
#define POL_TM			(1 << 0)	//TM Take measurement, set ‘1’ will initiate measurement
#define POL_SET			(1 << 5) 	//SET
#define POL_RESET		(1 << 6) 	//RESET
#define POL_REFILL		(1 << 7)	//Refill Cap Writing “1” will recharge the capacitor at CAP pin, it is requested to be issued before SET/RESET command.


uint8_t InicjujMMC3416x(void);
uint8_t ObslugaMMC3416x(void);
uint8_t StartujPomiarMMC3416x(void);
uint8_t StartujOdczytMMC3416x(void);
uint8_t PolecenieMMC3416x(uint8_t chPolecenie);
uint8_t MagMMC_CzytajStatus(void);
uint8_t MagMMC_CzytajDane(void);

#endif /* INC_MMC3416X_H_ */
