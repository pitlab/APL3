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

#define ROZMIAR_PAKIETU_RTP	1400

#define RTP_PT_JPEG         26
#define RTP_JPEG_RESTART	0x40

err_t OtworzPolaczenieSerweraRTSP(int *nGniazdoPolaczenia);
void ObslugaSerweraRTSP(int nGniazdoPolaczenia);
//err_t AnalizaKomunikatuRTSP(uint8_t* dane, uint16_t sDlugosc);
void WatekStreamujacyRTP(void *arg);


#endif /* INC_SIEC_SERWERRTSP_H_ */
