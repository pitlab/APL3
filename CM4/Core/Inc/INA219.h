/*
 * INA219.h
 *
 *  Created on: 17 wrz 2026
 *      Author: PitLab
 */
#ifndef INC_INA219_H_
#define INC_INA219_H_
#include "SysDefCM4.h"


#define I219_ADRES_I2C		0x80
#define I219_TIMEOUT		10	//czas w milisekundach na odczyt danych

//wartości rejestrów
#define I219_KONFIGURACJA	0x00
#define I219_NAP_BOCZNIKA	0x01
#define I219_NAP_OBWODU		0x02
#define I219_MOC			0x03
#define I219_PRAD			0x04
#define I219_KALIBRACJA		0x05


uint8_t ObsługaNA219(void);
uint8_t InicjujINA219(void);
uint8_t ZmierzINA219(float *fPrad, float *fNapiecie);


#endif /* INC_INA219_H_ */
