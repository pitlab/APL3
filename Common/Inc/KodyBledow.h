/*
 * KodyBledow.h
 *
 *  Created on: 29 sty 2019
 *      Author: PitLab
 */

#ifndef ERRCODE_H_
#define ERRCODE_H_

#define BLAD_OK					0	//wszystko w porządku
#define BLAD_HAL				1	//błąd HAL
#define BLAD_HAL_BUSY			2	//
#define BLAD_GOTOWE				3	//zadanie wykonane
#define BLAD_TIMEOUT			4
#define BLAD_ZLA_ILOSC_DANYCH	5
#define BLAD_CRC				7	//błędne CRC z danych
#define BLAD_ODMOWA_WYKONANIA	8	//odmowa wykonania polecenia ze wzgledu na bezpieczeństwo lotu
#define BLAD_PROCES_TRWA		9
#define BLAD_BUF_OVERRUN		10
#define BLAD_BRAK_KONFIG		12	//brak konfiguracji
#define BLAD_BRAK_DANYCH		13
#define BLAD_ZLY_NAGL			14
#define BLAD_ZLY_STAN_PROT		15
#define BLAD_ZLE_POLECENIE		16
#define BLAD_ZLY_INTERFEJS		17	//nieobsługiwany interfejs komunikacyjny
#define BLAD_ZLE_DANE			18
#define BLAD_ZLY_ADRES			19

#define BLAD_NIC_DO_ROBOTY		26	//zadanie nie znalazło nic do wykonania

//#define BLAD_SRAM_TEST			30 //błąd pamięci SRAM
#define BLAD_BRAK_KAMERY			31	//nie wykryto obecności kamery
#define BLAD_BRAK_CZUJNIKA		32	//ogólny błąd dotyczący czujników które normalnie zawsze powinny być sprawne
#define BLAD_BRAK_FLASH_NOR		33
#define BLAD_BRAK_WYSWIETLACZA	41

//#define BLAD_BRAK_POZW_ZAPISU	50	//brak pozwolenia zapisu dla obszaru pamieci Flash
#define BLAD_SEMAFOR_ZAJETY		51
#define BLAD_BRAK_PROBKI_AUDIO	52
#define BLAD_PELEN_BUF_KOM		53	//bufor próbek głosowych do wypowiedzenie jest pełen

#define BLAD_DLUGOSC_LONLAT		61	//niewłaściwa długość znaków do zdekodowania długosci i szerokości geogragicznej w sygnale NMEA
#define BLAD_ZLY_STAN_NMEA		62	//zły stan dekodera protokołu NMEA

#define BLAD_ZA_ZIMNO			63
#define BLAD_ZA_CIEPLO			64
#define BLAD_ZLA_KONFIG			65	//zła konfiguracja, została nadpisana dwartoscią domyślną
#define BLAD_ZLE_OBLICZENIA		66	//zła wartość kalibracji, zbytnio obdiega od tego co ma zostać wyliczone
//#define BLAD_ZA_DLUGO			67	//czas potrzebny do obróbki danych np. całkowania zbytnio odbiega od normy

//#define ERR_NASTAWA_PID			68	//niewłaściwa wartość nastawy regulatora PID
//#define BLAD_NASTAWA_FRAM		68	//niewłaściwa wartość nastawy odczytanaj z FRAM
#define BLAD_OTWARCIA_GNIAZDA	69	//bład otwarcia gniazda TCP
#define BLAD_ZAJETY_JPEG		70	//kompresor, który powinien czekać na dane nie jest dostępny
//#define BLAD_BRAK_ZNACZN_JPEG	71	//nie znalezino danego znacznika w skompresowanych danych

//#define BLAD_NIE_ZAINICJOWANY	0xFF




#endif /* ERRCODE_H_ */
