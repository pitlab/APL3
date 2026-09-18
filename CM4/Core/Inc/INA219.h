/*
 * INA219.h
 *
 *  Created on: 17 wrz 2026
 *      Author: PitLab
 */
#ifndef INC_INA219_H_
#define INC_INA219_H_
#include "SysDefCM4.h"


#define ADRES_I2C_INA219	0x80
#define TIMEOUT_INA219		10	//czas w milisekundach na odczyt danych

//wartości rejestrów
#define R219_KONFIGURACJA	0x00
#define R219_NAP_BOCZNIKA	0x01
#define R219_NAP_OBWODU		0x02
#define R219_MOC			0x03
#define R219_PRAD			0x04
#define R219_KALIBRACJA		0x05

#define INA219_LSB_NAPIECIA	4e-3	//LCB pomiaru naoiecia to 4mV
#define INA219_PRZESUN_NAP	3		//wynik pomiaru napiecia jest przesuniety o 3 bity w lewo

#define ZAKRES_POMIARU_PRADU	10.0f	//amperów
#define INA219_LSB_PRADU		(ZAKRES_POMIARU_PRADU / 32768)
#define REZYSTOR_POMIAROWY		0.005f		//rezystancja opornika pomiarowego w omach
#define WARTOSC_KALIBRACJI		(0.04096f / (INA219_LSB_PRADU * REZYSTOR_POMIAROWY))

uint8_t ObsługaNA219(void);
uint8_t InicjujINA219(void);
uint8_t ZmierzNapięcieINA219(float *fNapiecie);
uint8_t ZmierzPrądINA219(float *fPrad);

#endif /* INC_INA219_H_ */
