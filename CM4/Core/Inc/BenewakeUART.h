/*
 * Benewake.h
 *
 *  Created on: 14 lis 2026
 *      Author: PitLab
 */

#ifndef INC_BENEWAKEUART_H_
#define INC_BENEWAKEUART_H_
#include "SysDefCM4.h"


#define ROZMIAR_BUF_ANA_LIDARU	64
#define ROZMIAR_BUF_LIDARU		10	//ramka danych ma 9 bajtów a najdłuższe polecenie ma 8 bajtów
#define ROZMIAR_BUF_POLECEN		6
#define TIMEOUT_POLECENIA		100	//czas w milisekundach na odbiór odpowiedzi na polecenie - dokumentacja sugeruje 100ms

//definicje poleceń lidaru Benewake
#define LB_WERSJA		0x01
#define LB_RESET		0x02
#define LB_FRAME_RATE	0x03
#define LB_TRIGGER		0x04
#define LB_FORMAT_WY	0x05
#define LB_BAUD_RATE	0x06
#define LB_WLACZ_WY		0x07
#define LB_INTERFEJS	0x0A
#define LB_ADRES_I2C	0x0B
#define LB_ODCZYT_I2C	0x00
#define LB_TRYB_IO		0x3B
#define LB_PROG			0x22
#define LB_NISKA_MOC	0x35
#define LB_USTAW_FABR	0x10
#define LB_ZAPISZ_UST	0x11

//definicje stanów protokołu Benewake
#define PB_NAGLOWEK		0
#define PB_ROZMIAR		1
#define PB_POLECENIE	2
#define PB_DANE			3
#define PB_SUMA			4
#define NAGLOWEK_BENEWAKE	0x5A
#define ADRES_I2C_BENEWAKE	0x20

//sekwencja odczytu I2C
#define SB_WYSLIJ_POLECENIE	0
#define SB_CZYTAJ_DANE		3

uint8_t ObsługaLidaruBenewake(void);
uint8_t InicjujLidarUart(void);
uint8_t WyślijPolecenieLidarowiUART(uint8_t *cBufor, uint16_t sRozmiar);
uint8_t OdbierzPolecenieLidaraUART(uint8_t *cBufor, uint16_t sRozmiar);
uint8_t InicjujUart4RxJakoLidar(void);
uint8_t AnalizujProtokolBenewake(uint8_t cZnak);
uint8_t InicjujOdczytLidaruBenwakeI2C();
uint8_t CzytajDaneLidaru();

#endif /* INC_BENEWAKEUART_H_ */
