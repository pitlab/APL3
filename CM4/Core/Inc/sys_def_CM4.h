/*
 * sys_def_CM4.h
 *
 *  Created on: Dec 3, 2024
 *      Author: PitLab
 */

#ifndef INC_SYS_DEF_CM4_H_
#define INC_SYS_DEF_CM4_H_
#include "stm32h755xx.h"
#include "stm32h7xx_hal.h"
#include "sys_def_wspolny.h"
#include <math.h>
#include "errcode.h"
#include "stdio.h"


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

#define I2C_TIMOUT		2	//czas w ms timoutu operacji I2C


//definicje układów na magistralach I2C3 i I2C4 kodowane na osobnych bitach. Oznaczają że dane z tych czujników są dostępne do obróbki
#define MAG_MMC				0x01	//magnetometr na IMU
#define MAG_IIS				0x02	//magnetometr na IMU - odczyt danych
#define MAG_HMC				0x04	//magnetometr na GNSS
#define CISN_ROZN_MS2545	0x08	//ciśnienie różnicowe zewnętrzne
#define CISN_TEMP_MS2545	0x10	//ciśnienie różnicowe i temepratura
#define MAG_IIS_STATUS		0x20	//magnetometr na IMU - odczyt statusu
#define MAG_MMC_STATUS		0x40	//magnetometr na IMU - odczyt statusu


//timeouty w milisekundach dla magistrali I2C zależące od ilości przesyłanych danych. Dla 100kHz czas przesłania 1 bajtu to 0,1ms, dla 25kHz to 0,4ms
#define TOUT_I2C4_2B		1
#define TOUT_I2C4_7B		2

//definicje kanałów IO
#define MIO10	(1 << 0) 	//MOD_IO10
#define MIO11	(1 << 1) 	//MOD_IO11
#define MIO20	(1 << 2)	//MOD_IO20
#define MIO21	(1 << 3) 	//MOD_IO21
#define MIO30	(1 << 4) 	//MOD_IO30
#define MIO31	(1 << 5) 	//MOD_IO31
#define MIO40   (1 << 6) 	//MOD_IO40
#define MIO41	(1 << 7)	//MOD_IO41

#define MASKA_INIT_GNSS		0x07

//znaczenie bitów zmiennej chNoweDaneI2C
#define DANE_I2C_HMC	0x01	//Dane z czujnika HMC

#define MAX_PROB_INICJALIZACJI		5	//po tylu błędnych próbach inicjalizacji uznajemy czujnik za nieobecny i nie próbujemy więcej.

#endif /* INC_SYS_DEF_CM4_H_ */
