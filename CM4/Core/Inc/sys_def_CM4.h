/*
 * sys_def_CM4.h
 *
 *  Created on: Dec 3, 2024
 *      Author: PitLab
 */

#ifndef INC_SYS_DEF_CM4_H_
#define INC_SYS_DEF_CM4_H_

#include "stm32h755xx.h"
#include "errcode.h"
#include "stm32h7xx_hal.h"

//definicje adresów modułów i układów na maistrali SPI
#define ADR_MOD1	0
#define ADR_MOD2	1
#define ADR_MOD3	2
#define ADR_MOD4	3
#define ADR_NIC		4		//nic nie jest wybrane, pusta linia
#define ADR_EXPIO	5
#define ADR_FRAM	6


#define LICZBA_ODCINKOW_CZASU	20			//liczba odcinków czasu na jakie jest dzielony czas pełnego obiegu pętli głównej
#define CZESTOTLIWOSC_PETLI		200			//częstotliwość pętli głównj [Hz]
#define CZAS_ODCINKA			1000000/(CZESTOTLIWOSC_PETLI * LICZBA_ODCINKOW_CZASU)	//czas na jeden odcinek [us]


#define OKRES_PWM		2500	//okres kanału PWM w mikrosekundach
#define PRZERWA_PPM		3000	//przerwa między paczkami impulsów PPM odbiornika RC
#define KANALY_SERW		16		//liczba sterowanych kanałów serw
#define KANALY_ODB		16		//liczba odbieranych kanałów na każdym z dwu wejść odbiorników RC


//flagi inicjalizacji sprzętu
#define INIT_WYKR_MTK		0x00000001
#define INIT_WYKR_UBLOX		0x00000002
#define INIT_GNSS_GOTOWY	0x00000004
#define INIT_MAG3			0x00000008
#define INIT_MS5611			0x00000010


//definije kanałów IO
#define MIO11	(0 << 0) 	//MOD_IO11
#define MIO12	(0 << 1) 	//MOD_IO12
#define MIO21	(0 << 2)	//MOD_IO21
#define MIO22	(0 << 3) 	//MOD_IO22
#define MIO31	(0 << 4) 	//MOD_IO31
#define MIO32	(0 << 5) 	//MOD_IO32
#define MIO41   (0 << 6) 	//MOD_IO41
#define MIO42	(0 << 7)	//MOD_IO42

#define MASKA_INIT_GNSS		0x07



#endif /* INC_SYS_DEF_CM4_H_ */
