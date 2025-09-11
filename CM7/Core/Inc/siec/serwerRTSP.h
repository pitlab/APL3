/*
 * serwerRTSP.h
 *
 *  Created on: Sep 10, 2025
 *      Author: PitLab
 */

#ifndef INC_SIEC_SERWERRTSP_H_
#define INC_SIEC_SERWERRTSP_H_
#include "sys_def_CM7.h"
#include <lwip/altcp.h>
#include <lwip/tcp.h>



#define PORT_RTSP   554
#define PORT_RTP    5004


err_t OtworzPolaczenieSerweraRTSP(void);
void ObslugaSerweraRTSP(void);
//err_t AnalizaKomunikatuRTSP(uint8_t* dane, uint16_t sDlugosc);
void WatekStreamujacyRTP(void *arg);


#endif /* INC_SIEC_SERWERRTSP_H_ */
