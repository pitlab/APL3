/*
 * komunikacja.h
 *
 *  Created on: May 26, 2024
 *      Author: PitLab
 */

#ifndef INC_PROTOKOLKOM_H_
#define INC_PROTOKOLKOM_H_
#include "SysDefCM7.h"
#include "PoleceniaKomunikacyjne.h"


#define ROZMIAR_BUF_ODB_DMA		256
#define ROZMIAR_BUF_ANALIZY_ODB	2*ROZMIAR_BUF_ODB_DMA


typedef struct		//dobrze aby cała struktura zmieściła się na stronie flasch 30 bajtów
{
	uint8_t cAdres;						//własny adres sieciowy
	uint8_t cNazwa[DLUGOSC_NAZWY];			//nazwa BSP
	uint8_t cAdrIP[4];						//numer IP
} stBSP_ID_t;


//definicje funkcji
uint8_t InicjujProtokol(void);
uint8_t InicjalizacjaWatkuOdbiorczegoLPUART1(void);
uint8_t ObslugaWatkuOdbiorczegoLPUART1(void);
void WatekOdbiorczyLPUART1(void *argument);
uint8_t AnalizujDaneKom(uint8_t cWe, uint8_t chInterfejs);
uint8_t DekodujRamke(uint8_t cWe, uint8_t *cAdrZdalny, uint8_t *cZnakCzasu, uint8_t *cPolecenie, uint8_t *cRozmDanych, uint8_t *cDane, uint8_t cInterfejs);
void InicjujCRC16(uint16_t sInit, uint16_t sWielomian);
uint16_t LiczCRC16(uint8_t cDane);
uint8_t PrzygotujRamke(uint8_t cAdrZdalny, uint8_t cAdrLokalny,  uint8_t cZnakCzasu, uint8_t cPolecenie, uint8_t cRozmDanych, uint8_t *cDane, uint8_t *cRamka);
uint8_t WyslijRamke(uint8_t cAdrZdalny, uint8_t cPolecenie, uint8_t cRozmDanych, uint8_t *cDane, uint8_t cInterfejs);
uint8_t Wyslij_KodBledu(uint8_t cKodBledu, uint8_t cParametr, uint8_t cInterfejs);
void StartKomUart(void const * argument);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t WyslijDebugUART7(uint8_t cZnak);

uint8_t TestKomunikacji(void);
uint8_t TestKomunikacjiSTD(void);
uint8_t TestKomunikacjiDMA(void);



#endif /* INC_PROTOKOLKOM_H_ */

