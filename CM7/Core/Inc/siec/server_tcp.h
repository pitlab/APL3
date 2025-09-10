/*
 * server_tcp.h
 *
 *  Created on: Sep 8, 2025
 *      Author: PitLab
 */

#ifndef INC_SIEC_SERVER_TCP_H_
#define INC_SIEC_SERVER_TCP_H_
#include "sys_def_CM7.h"
#include "display.h"

#define DLUGOSC_WIADOMOSCI_GG	(DISP_X_SIZE / FONT_SL)	//tyle znaków mieści się na ekranie


uint8_t AnalizaKomunikatuTCP(uint8_t* dane, uint16_t sDlugosc);

#endif /* INC_SIEC_SERVER_TCP_H_ */
