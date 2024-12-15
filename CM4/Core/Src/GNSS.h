/*
 * GNSS.h
 *
 *  Created on: Dec 15, 2024
 *      Author: PitLab
 */

#ifndef SRC_GNSS_H_
#define SRC_GNSS_H_

#include "sys_def_CM4.h"

#define ROZMIAR_BUF_ODBIORU_GNSS	16
#define ROZMIAR_BUF_ANALIZY_GNSS	4 * ROZMIAR_BUF_ODBIORU_GNSS
#define MASKA_ROZM_BUF_ODB_GNSS		0x0F


uint8_t InicjujGNSS(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* SRC_GNSS_H_ */
