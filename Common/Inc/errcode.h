/*
 * errcode.h
 *
 *  Created on: 29 sty 2019
 *      Author: PitLab
 */

#ifndef ERRCODE_H_
#define ERRCODE_H_

#define ERR_OK					0	//wszystko w porządku
#define ERR_HAL					1	//błąd HAL
#define ERR_NOT_READY			2	// to samo co HAL BUSY
#define ERR_TIMEOUT				3
#define ERR_ZLA_ILOSC_DANYCH	4
#define ERR_DONE				5	//zadanie wykonane
#define ERR_PARITY				6
#define ERR_CRC					7	//błędne CRC z danych
#define ERR_DIV0				9
#define ERR_BUF_OVERRUN			10
#define ERR_ZAPIS_KONFIG		11	//błąd zapisu konfiguracji
#define ERR_BRAK_KONFIG			12	//brak konfiguracji
#define ERR_RAMKA_GOTOWA		13	//ramka telekomunikacyjna całkowicie zdekodowana
#define ERR_ZLY_NAGL			14
#define ERR_ZLY_STAN_PROT		15
#define ERR_ZLE_POLECENIE		16
#define ERR_ZLY_INTERFEJS		17	//nieobsługiwany interfejs komunikacyjny
#define ERR_ZLE_DANE			18
#define ERR_ZLY_ADRES			19

#define ERR_ZWARCIE_NIZ			20
#define ERR_ZWARCIE_WYZ			21
#define ERR_ZWARCIE_GND			22
#define ERR_ZWARCIE_VCC			23

#define ERR_SRAM_TEST			30 //błąd pamięci SRAM
#define ERR_BRAK_KAMERY			31	//nie wykryto obecności kamery
#define ERR_BRAK_MAGN			32	//nie wykryto obecności magnetometru
#define ERR_BRAK_FLASH_NOR		33

#define ERR_I2C_BERR			40
#define ERR_I2C_ARB_LOST		41
#define ERR_I2C_ACK_FAIL		42
#define ERR_I2C_SDA_LOCK		43
#define ERR_I2C_SCL_LOCK		44
#define ERR_I2C_TIMEOUT			45

#define ERR_BRAK_POZW_ZAPISU	50	//brak pozwolenia zapisu dla obszaru pamieci Flash
#define ERR_SEMAFOR_ZAJETY		51

#define ERR_NIE_ZAINICJOWANY	0xFF




#endif /* ERRCODE_H_ */
