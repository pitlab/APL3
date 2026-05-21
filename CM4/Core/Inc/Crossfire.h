/*
 * Crossfire.h
 *
 *  Created on: 21 maj 2026
 *      Author: PitLab
 */

#ifndef INC_CROSSFIRE_H_
#define INC_CROSSFIRE_H_

#include "SysDefCM4.h"
#include "wymiana.h"
#include "SBus.h"


#define ROZMIAR_BUF_ANA_CRSF		ROZMIAR_BUF_ANA_SBUS
#define MASKA_ROZM_BUF_ANA_CRSF		ROZMIAR_BUF_ANA_SBUS
#define ROZMIAR_RAMKI_CRSF		64	//żadna ramka nie może być dłuższa niż tyle
#define ROZMIAR_BUF_ODB_CRSF	25


//Etapy Odbioru Ramki Crossfire
#define EORC_ADRES			0
#define EORC_DLUGOSC		1
#define EORC_TYP			2
#define EORC_DANE			3
#define EORC_CRC			4


//definicje adresów urzadzeń
#define CRSF_ADR_BROADCAST		0x00 	//Broadcast address
#define CRSF_ADR_CLOUD			0x0E 	//Cloud
#define CRSF_ADR_USB			0x10 	//USB Device
#define CRSF_ADR_BLUETOOTH		0x12 	//Bluetooth Module/WiFi
#define CRSF_ADR_WIFI			0x13 	//Wi-Fi receiver (mobile game/simulator)
#define CRSF_ADR_VIDEO			0x14 	//Video Receiver
#define CRSF_ADR_NAT			0x20	//-0x7F Dynamic address space for NAT
#define CRSF_ADR_OSD			0x80 	//OSD / TBS CORE PNP PRO
#define CRSF_ADR_ESC1			0x90 	//ESC 1
#define CRSF_ADR_ESC2			0x91 	//ESC 2
#define CRSF_ADR_ESC3			0x92 	//ESC 3
#define CRSF_ADR_ESC4			0x93 	//ESC 4
#define CRSF_ADR_ESC5			0x94 	//ESC 5
#define CRSF_ADR_ESC6			0x95 	//ESC 6
#define CRSF_ADR_ESC7			0x96 	//ESC 7
#define CRSF_ADR_ESC8			0x97 	//ESC 8
#define CRSF_ADR_RES1			0x8A 	//Reserved
#define CRSF_ADR_RES2			0xB0 	//Crossfire reserved
#define CRSF_ADR_RES3			0xB2 	//Crossfire reserved
#define CRSF_ADR_CURR_SENS		0xC0 	//Voltage/ Current Sensor / PNP PRO digital current sensor
#define CRSF_ADR_GPS			0xC2 	//GPS / PNP PRO GPS
#define CRSF_ADR_TBSBB			0xC4 	//TBS Blackbox
#define CRSF_ADR_FLIGHT_CTRL	0xC8 	//Flight controller
#define CRSF_ADR_RES4			0xCA 	//Reserved
#define CRSF_ADR_TAG			0xCC 	//Race tag
#define CRSF_ADR_VTX			0xCE 	//VTX
#define CRSF_ADR_RC				0xEA 	//Remote Control
#define CRSF_ADR_REPEAT_RX		0xEB 	//Repeater Receiver
#define CRSF_ADR_RC_REC			0xEC 	//R/C Receiver / Crossfire Rx
#define CRSF_ADR_REPEAT_TX		0xED 	//Repeater Transmitter Module
#define CRSF_ADR_RC_TX			0xEE 	//R/C Transmitter Module / Crossfire Tx
#define CRSF_ADR_RES5			0xF0 	//reserved
#define CRSF_ADR_RES6			0xF2 	//reserved


#define CRSF_ADRES_URZADZENIA	CRSF_ADR_FLIGHT_CTRL	//pierwszy bajt ramki zawierający adres urządzenia


uint8_t InicjujCrossfire(void);
uint8_t OdbiórRamkiCrossfire(uint8_t *chRamka, uint8_t *cEtapOdbioru, uint8_t *cLicznikDanych, uint8_t *chBufor, uint8_t chWskNapBuf, uint8_t *chWskOprBuf);
uint8_t AnalizujCrossfire(stWymianyCM4_t *psDaneCM4, stWymianyCM7_t *psDaneCM7);
uint8_t crc8(const uint8_t * ptr, uint8_t len);
uint8_t ObsługaRamkiCrossfire(void);

#endif /* INC_CROSSFIRE_H_ */
