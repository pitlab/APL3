/*
 * komunikacja.h
 *
 *  Created on: May 26, 2024
 *      Author: PitLab
 */

#ifndef INC_PROTOKOLKOM_H_
#define INC_PROTOKOLKOM_H_
#include "sys_def_CM7.h"
#include "polecenia_komunikacyjne.h"


#define ROZMIAR_BUF_ODB_DMA		256
#define ROZMIAR_BUF_ANALIZY_ODB	2*ROZMIAR_BUF_ODB_DMA


typedef struct		//dobrze aby cała struktura zmieściła się na stronie flasch 30 bajtów
{
	uint8_t chAdres;						//własny adres sieciowy
	uint8_t chNazwa[DLUGOSC_NAZWY];			//nazwa BSP
	uint8_t chAdrIP[4];						//numer IP
} stBSP_ID_t;


//definicje funkcji
uint8_t InicjujProtokol(void);
uint8_t InicjalizacjaWatkuOdbiorczegoLPUART1(void);
uint8_t ObslugaWatkuOdbiorczegoLPUART1(void);
uint8_t AnalizujDaneKom(uint8_t chWe, uint8_t chInterfejs);
uint8_t DekodujRamke(uint8_t chWe, uint8_t *chAdrZdalny, uint8_t *chZnakCzasu, uint8_t *chPolecenie, uint8_t *chRozmDanych, uint8_t *chDane, uint8_t chInterfejs);
void InicjujCRC16(uint16_t sInit, uint16_t sWielomian);
uint16_t LiczCRC16(uint8_t chDane);
uint8_t PrzygotujRamke(uint8_t chAdrZdalny, uint8_t chAdrLokalny,  uint8_t chZnakCzasu, uint8_t chPolecenie, uint8_t chRozmDanych, uint8_t *chDane, uint8_t *chRamka);
uint8_t WyslijRamke(uint8_t chAdrZdalny, uint8_t chPolecenie, uint8_t chRozmDanych, uint8_t *chDane, uint8_t chInterfejs);
uint8_t Wyslij_KodBledu(uint8_t chKodBledu, uint8_t chParametr, uint8_t chInterfejs);
void StartKomUart(void const * argument);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t WyslijDebugUART7(uint8_t chZnak);

uint8_t TestKomunikacji(void);
uint8_t TestKomunikacjiSTD(void);
uint8_t TestKomunikacjiDMA(void);



#endif /* INC_PROTOKOLKOM_H_ */

