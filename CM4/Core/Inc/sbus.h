/*
 * sbus.h
 *
 *  Created on: 14 lut 2026
 *      Author: PitLab
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#include "SysDefCM4.h"
#include "Wymiana.h"
#include "WeWyRC.h"

#define ROZMIAR_BUF_ANA_SBUS	64
#define MASKA_ROZM_BUF_ANA_SBUS	0x3F

#define OKRES_RAMKI_PPM_RC	30000	//Czas między kolejnymi ramkami PPM odbiornika RC [us]
#define ROZM_BUF_ODB_SBUS	25
#define ROZMIAR_RAMKI_SBUS	25
#define OKRES_RAMKI_SBUS	1300	//Czas między kolejnymi wychodzącymi ramkami SBus czas jest odmierzany w kwantach 5ms. Niepełne 3 kwanty są po to aby nie porzucał ramek jeżeli pętla wykona się odrobinę szybciej
#define SBUS_NAGLOWEK		0x0F	//bajt nagłówkowy ramki SBus
#define SBUS_STOPKA			0x00	//bajt stopki protokołu BSBus
#define MAX_PRZESUN_NAGL	2		//nagłówek ramki S-Bus może być przesunięty maksymalnie o tyle bajtów do tyłu
#define SBUS_MIN			380		//minimalna wartość jaką przenosi protokół SBus
#define SBUS_NEU			1030	//wartość SBus w pozycji neutralnej
#define SBUS_MAX			1680	//maksymalna wartość jaką przenosi protokół SBus



//deklaracje funkcji
uint8_t InicjujUart2RxJakoSbus(GPIO_InitTypeDef *InitGPIO);
uint8_t InicjujUart4RxJakoSbus(GPIO_InitTypeDef *InitGPIO);
uint8_t InicjujUart4TxJakoSbus(GPIO_InitTypeDef *InitGPIO);
uint8_t OdbiórRamkiSBus(uint8_t *chRamkaSBus, uint8_t *chWskNapRamki, uint8_t *chBuforAnalizy, uint8_t chWskNapBuf, uint8_t *chWskOprBuf);
uint8_t DekodowanieRamkiBSBus(uint8_t* chRamkaWe, stRC_t *stRC);
uint8_t ObsługaRamkiSBus(void);

#endif /* INC_SBUS_H_ */
