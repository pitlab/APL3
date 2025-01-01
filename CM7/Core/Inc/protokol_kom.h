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


#define ROZM_BUF_KOL	32	//rozmiar musi być potęgą 2 aby zapętlić bufor operacją AND
#define ROZMIAR_BUF_NAD_DMA		64
#define ROZMIAR_BUF_ODB_DMA		16





//definicje funkcji lokalnych
uint8_t InitProtokol(void);
void InicjalizacjaWatkuOdbiorczegoLPUART1(void);
void ObslugaWatkuOdbiorczegoLPUART1(void);
uint8_t AnalizujDaneKom(uint8_t chWe, uint8_t chInterfejs);
uint8_t DekodujRamke(uint8_t chWe, uint8_t *chAdrZdalny, uint8_t *chZnakCzasu, uint8_t *chPolecenie, uint8_t *chRozmDanych, uint8_t *chDane, uint8_t chInterfejs);
void InicjujCRC16(uint16_t sInit, uint16_t sWielomian);
uint16_t LiczCRC16(uint8_t chDane);
uint8_t PrzygotujRamke(uint8_t chAdrZdalny, uint8_t chAdrLokalny,  uint8_t chZnakCzasu, uint8_t chPolecenie, uint8_t chRozmDanych, uint8_t *chDane, uint8_t *chRamka);
uint8_t WyslijRamke(uint8_t chAdrZdalny, uint8_t chPolecenie, uint8_t chRozmDanych, uint8_t *chDane, uint8_t chInterfejs);
uint8_t Wyslij_OK(uint8_t chParametr1, uint8_t chParametr2, uint8_t chInterfejs);
uint8_t Wyslij_ERR(uint8_t chKodBledu, uint8_t chParametr, uint8_t chInterfejs);
void StartKomUart(void const * argument);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

uint8_t TestKomunikacji(void);

//deklaracje funkcji zdalnych


#endif /* INC_PROTOKOLKOM_H_ */

