/*
 * GNSS.h
 *
 *  Created on: Dec 15, 2024
 *      Author: PitLab
 */

#ifndef SRC_GNSS_H_
#define SRC_GNSS_H_

#include "sys_def_CM4.h"

#define ROZMIAR_BUF_NAD_GNSS	55	//najdłuższy komunikat ma 55 bajtów
#define ROZMIAR_BUF_ODB_GNSS	15	//ma być nieparzysty aby mozna było migać LEDem
#define ROZMIAR_BUF_ANA_GNSS	32
#define MASKA_ROZM_BUF_ANA_GNSS		0x1F

#define CYKL_STARTU_ZMIAN_PREDK	400		//od tego czasu wykonaj serię zapisów zmian prędkości transmitowanych na 9600, 19200, 38400, 75600 i 115200
#define CYKL_STARTU_INI_MTK		500		//po tym czasie rozpocznij inicjalizację GPS
#define CYKL_STARTU_INI_UBLOX	600		//po tym czasie rozpocznia inicjalizację u-Blox

#define TIMEOUT_GNSS			600		//timeout braku odbierania danych z GNSS

uint8_t InicjujGNSS(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void SumaKontrolnaUBX(uint8_t *chRamka);

#endif /* SRC_GNSS_H_ */
