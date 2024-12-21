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

#define CYKL_STARTU_INI_MTK		400
#define CYKL_STARTU_INI_UBLOX	600

uint8_t InicjujGNSS(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* SRC_GNSS_H_ */
