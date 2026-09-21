/*
 * INA226.h
 *
 *  Created on: 21 wrz 2026
 *      Author: PitLab
 */
#ifndef INC_INA226_H_
#define INC_INA226_H_
#include "SysDefCM4.h"


#define ADRES_I2C_INA226	0x88
#define INA226_ID_PRODUCENTA_H	0x54	//T
#define INA226_ID_PRODUCENTA_L	0x49	//I
#define TIMEOUT_INA226		10	//czas w milisekundach na odczyt danych

//wartości rejestrów
#define R226_KONFIGURACJA	0x00
#define R226_NAP_BOCZNIKA	0x01
#define R226_NAP_OBWODU		0x02
#define R226_MOC			0x03
#define R226_PRAD			0x04
#define R226_KALIBRACJA		0x05
#define R226_WL_MASEK		0x06
#define R226_ALERT_LIMIT	0x07
#define R226_ID_PRODUCENTA	0xFE
#define R226_ID_UKLADU		0xFF

#define INA226_LSB_NAPIECIA	1.25e-3	//LSB pomiaru napięcia to 1,25mV
#define ZAKRES_POMIARU_PRADU	10.0f	//amperów
#define INA226_LSB_PRADU		(ZAKRES_POMIARU_PRADU / 32768)
#define REZ_POMIAROWY_INA226	0.1f		//rezystancja opornika pomiarowego w omach
#define WARTOSC_KALIB_INA226	(0.00512f / (INA226_LSB_PRADU * REZ_POMIAROWY_INA226))

uint8_t ObsługaNA226(void);
uint8_t InicjujINA226(void);
uint8_t ZmierzNapięcieINA226(float *fNapiecie);
uint8_t ZmierzPrądINA226(float *fPrad);
uint8_t INA226_CzytajNapięcie(void);
uint8_t INA226_CzytajPrąd(void);

#endif /* INC_INA226_H_ */
