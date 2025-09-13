/*
 * serwer_tcp.h
 *
 *  Created on: Sep 8, 2025
 *      Author: PitLab
 */

#ifndef INC_SIEC_SERWER_TCP_H_
#define INC_SIEC_SERWER_TCP_H_
#include "sys_def_CM7.h"
#include <lwip/altcp.h>
#include <lwip/tcp.h>


uint8_t OtworzPortSertweraTCP(void);
uint8_t ObslugaSerweraTCP(void);


#endif /* INC_SIEC_SERWER_TCP_H_ */
