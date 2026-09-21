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
#include "SysDefWspolny.h"
#include <math.h>
#include "KodyBledow.h"
#include "stdio.h"


//definicje adresów modułów i układów na maistrali SPI
#define ADR_MOD1	0
#define ADR_MOD2	1
#define ADR_MOD3	2
#define ADR_MOD4	3
#define ADR_NIC		4		//nic nie jest wybrane, pusta linia
#define ADR_EXPIO	5
#define ADR_FRAM	6


#define I2C_TIMOUT		2	//czas w ms timoutu operacji I2C


//definicje układów na magistrali I2C4 kodowane na osobnych bitach. Oznaczają że dane z tych czujników są dostępne do obróbki



//definicje układów na magistralach I2C3 i I2C4 kodowane na osobnych bitach. Oznaczają że dane z tych czujników są dostępne do obróbki
#define CISN_ROZN_MS2545	0x0001	//ciśnienie różnicowe zewnętrzne
#define CISN_TEMP_MS2545	0x0002	//ciśnienie różnicowe i temperatura
#define MAG_HMC5883			0x0004	//magnetometr na GNSS
#define INA219_PRAD			0x0008
#define INA219_NAPIECIE		0x0010
#define INA226_PRAD			0x0020
#define INA226_NAPIECIE		0x0040
#define TOF_VL53L1			0x0080

#define MAG_MMC				0x0100	//magnetometr na IMU - odczyt danych
#define MAG_IIS				0x0200	//magnetometr na IMU - odczyt danych
#define MAG_IIS_STATUS		0x0400	//magnetometr na IMU - odczyt statusu
#define MAG_MMC_STATUS		0x0800	//magnetometr na IMU - odczyt statusu

//timeouty w milisekundach dla magistrali I2C zależące od ilości przesyłanych danych. Dla 100kHz czas przesłania 1 bajtu to 0,1ms, dla 25kHz to 0,4ms
#define TOUT_I2C4_2B		2	//testowo
#define TOUT_I2C4_7B		3
#define TOUT_SPI			3		//czas oczekiwania na operację na szynie w ms
#define I2C_READ			0x01
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

//#define KALMAN_WYSOKOSCI_4D
//definicje pól wektora stanu filtra Kalmana wysokości
#define KAL_WYS_ESTYMATA_WYSOKOSCI		0
#define KAL_WYS_ESTYMATA_PREDKOSCI		1
#define KAL_WYS_ESTYMATA_PRZYSPIESZENIA	2
#define KAL_WYS_BLAD_PRZYSPIESZ_AKCEL1	3
#define KAL_WYS_BLAD_PRZYSPIESZ_AKCEL2	4
#define KAL_WYS_BLAD_WYSOKOSCI_BARO1	5
#define KAL_WYS_BLAD_WYSOKOSCI_BARO2	6
#define KAL_WYS_BLAD_WYSOKOSCI_GNSS1	7
#define KAL_WYS_BLAD_WYSOKOSCI_GNSS2	8
#define KAL_WYS_ESTYMATA_WYS_GRUNTU		9

#endif /* INC_SYS_DEF_CM4_H_ */
