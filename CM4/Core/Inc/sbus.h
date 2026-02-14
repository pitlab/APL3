/*
 * sbus.h
 *
 *  Created on: 14 lut 2026
 *      Author: PitLab
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#include "sys_def_CM4.h"
#include "wymiana.h"

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

//#define ZEGAR_PWM			2000000	//[Hz] aby osiągnać krok 500ns
//#define IMPULS_PWM			1000
//#define LICZBA_WEJSC_RC		2
//#define LICZBA_WYJSC_RC		9
//#define LICZBA_KONFIG_WYJSC_RC	5	//pierwsze 8 wyjść zajmują pół bajtu, ostatnie wyjście definiujące grupę 8..16 cały bajt


//deklaracje funkcji
uint8_t InicjujUart2RxJakoSbus(GPIO_InitTypeDef *InitGPIO);
uint8_t InicjujUart4RxJakoSbus(GPIO_InitTypeDef *InitGPIO);
uint8_t InicjujUart4TxJakoSbus(GPIO_InitTypeDef *InitGPIO);
uint8_t DekodowanieRamkiBSBus(uint8_t* chRamkaWe, int16_t *sKanaly);
uint8_t FormowanieRamkiSBus(uint8_t *chRamkaSBus, uint8_t *chWskNapRamki, uint8_t *chBuforAnalizy, uint8_t chWskNapBuf, uint8_t *chWskOprBuf);
uint8_t ObslugaRamkiSBus(void);

#endif /* INC_SBUS_H_ */
