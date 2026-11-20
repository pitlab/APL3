/*
 * Benewake.h
 *
 *  Created on: 14 lis 2026
 *      Author: PitLab
 */

#ifndef INC_BENEWAKE_H_
#define INC_BENEWAKE_H_
#include "SysDefCM4.h"


#define ROZMIAR_BUF_LIDARU		10	//ramka danych ma 9 bajtów a najdłuższe polecenie ma 8 bajtów
#define TIMEOUT_POLECENIA		100	//czas w milisekundach na odbiór odpowiedzi na polecenie - dokumentacja sugeruje 100ms
#define NAGLOWEK_BENEWAKE	0x5A
#define ADRES_I2C_BENEWAKE	0x20

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

//sekwencja odczytu I2C
#define SB_WYSLIJ_POLECENIE	0
#define SB_CZYTAJ_DANE		1

uint8_t ObsługaI2C_LidaruBenewake(void);
uint8_t InicjujLidarI2C(void);


#endif /* INC_BENEWAKE_H_ */
