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
#define ERR_HAL_BUSY			2	//
#define ERR_DONE				3	//zadanie wykonane
#define ERR_TIMEOUT				4
#define ERR_ZLA_ILOSC_DANYCH	5
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
#define ERR_ZAJETY_SEMAFOR		24

#define ERR_SRAM_TEST			30 //błąd pamięci SRAM
#define ERR_BRAK_KAMERY			31	//nie wykryto obecności kamery
#define ERR_BRAK_MAG_ZEW		32	//nie wykryto obecności magnetometru zewnętrznego
#define ERR_BRAK_FLASH_NOR		33
#define ERR_BRAK_BMP581			34
#define ERR_BRAK_ICM42688		35
#define ERR_BRAK_LSM6DSV		36
#define ERR_BRAK_MMC34160		37
#define ERR_BRAK_IIS2MDS		38

#define ERR_I2C_BERR			40
#define ERR_I2C_ARB_LOST		41
#define ERR_I2C_ACK_FAIL		42
#define ERR_I2C_SDA_LOCK		43
#define ERR_I2C_SCL_LOCK		44
#define ERR_I2C_TIMEOUT			45


#define ERR_BRAK_POZW_ZAPISU	50	//brak pozwolenia zapisu dla obszaru pamieci Flash
#define ERR_SEMAFOR_ZAJETY		51
#define ERR_BRAK_PROBKI_AUDIO	52
#define ERR_PELEN_BUF_KOM		53	//bufor próbek głosowych do wypowiedzenie jest pełen

#define ERR_BRAK_DANYCH			60
#define ERR_DLUGOSC_LONLAT		61	//niewłaściwa długość znaków do zdekodowania długosci i szerokości geogragicznej w sygnale NMEA
#define ERR_ZLY_STAN_NMEA		62	//zły stan dekodera protokołu NMEA

#define ERR_ZA_ZIMNO			63
#define ERR_ZA_CIEPLO			64
#define ERR_ZLA_KONFIG			65	//zła konfiguracja, została nadpisana dwartoscią domyślną
#define ERR_ZLE_OBLICZENIA		66	//zła wartość kalibracji, zbytnio obdiega od tego co ma zostać wyliczone

#define ERR_NIE_ZAINICJOWANY	0xFF




#endif /* ERRCODE_H_ */
