/*
 * telemetria.h
 *
 *  Created on: May 14, 2025
 *      Author: PitLab
 */

#ifndef INC_TELEMETRIA_H_
#define INC_TELEMETRIA_H_

#include "sys_def_CM7.h"
#include "polecenia_komunikacyjne.h"

//definicje zmiennych telemetrycznych
//zmienne IMU
#define TELEID_AKCEL1X		0
#define TELEID_AKCEL1Y		1
#define TELEID_AKCEL1Z		2
#define TELEID_AKCEL2X		3
#define TELEID_AKCEL2Y		4
#define TELEID_AKCEL2Z		5
#define TELEID_ZYRO1P		6
#define TELEID_ZYRO1Q		7
#define TELEID_ZYRO1R		8
#define TELEID_ZYRO2P		9
#define TELEID_ZYRO2Q		10
#define TELEID_ZYRO2R		11
#define TELEID_MAGNE1X		12
#define TELEID_MAGNE1Y		13
#define TELEID_MAGNE1Z		14
#define TELEID_MAGNE2X		15
#define TELEID_MAGNE2Y		16
#define TELEID_MAGNE2Z		17
#define TELEID_MAGNE3X		18
#define TELEID_MAGNE3Y		19
#define TELEID_MAGNE3Z		20
#define TELEID_TEMPIMU1		21
#define TELEID_TEMPIMU2		22

//zmienne AHRS
#define TELEID_KAT_IMU1X	23
#define TELEID_KAT_IMU1Y	24
#define TELEID_KAT_IMU1Z	25
#define TELEID_KAT_IMU2X	26
#define TELEID_KAT_IMU2Y	27
#define TELEID_KAT_IMU2Z	28
#define TELEID_KAT_ZYRO1X	29
#define TELEID_KAT_ZYRO1Y	30
#define TELEID_KAT_ZYRO1Z	31
#define TELEID_KAT_AKCELX	32
#define TELEID_KAT_AKCELY	33
#define TELEID_KAT_AKCELZ	34

//zmienne barametryczne
#define TELEID_CISBEZW1		35
#define TELEID_CISBEZW2		36
#define TELEID_WYSOKOSC1	37
#define TELEID_WYSOKOSC2	38
#define TELEID_CISROZN1		39
#define TELEID_CISROZN2		40
#define TELEID_PREDIAS1		41
#define TELEID_PREDIAS2		42
#define TELEID_TEMPCISB1	43
#define TELEID_TEMPCISB2	44
#define TELEID_TEMPCISR1	45
#define TELEID_TEMPCISR2	46

//odbiorniki RC
#define TELEID_RC_KAN1		47
#define TELEID_RC_KAN2		48
#define TELEID_RC_KAN3		49
#define TELEID_RC_KAN4		50
#define TELEID_RC_KAN5		51
#define TELEID_RC_KAN6		52
#define TELEID_RC_KAN7		53
#define TELEID_RC_KAN8		54
#define TELEID_RC_KAN9		55
#define TELEID_RC_KAN10		56
#define TELEID_RC_KAN11		57
#define TELEID_RC_KAN12		58
#define TELEID_RC_KAN13		59
#define TELEID_RC_KAN14		60
#define TELEID_RC_KAN15		61
#define TELEID_RC_KAN16		62

//serwa
#define TELEID_SERWO1		63
#define TELEID_SERWO2		64
#define TELEID_SERWO3		65
#define TELEID_SERWO4		66
#define TELEID_SERWO5		67
#define TELEID_SERWO6		68
#define TELEID_SERWO7		69
#define TELEID_SERWO8		70
#define TELEID_SERWO9		71
#define TELEID_SERWO10		72
#define TELEID_SERWO11		73
#define TELEID_SERWO12		74
#define TELEID_SERWO13		75
#define TELEID_SERWO14		76
#define TELEID_SERWO15		77
#define TELEID_SERWO16		78




#define LICZBA_ZMIENNYCH_TELEMETRYCZNYCH	95
#define MAX_LICZBA_ZMIENNYCH_TELEMETRYCZNYCH	112
#define LICZBA_BAJTOW_ID_TELEMETRII			(MAX_LICZBA_ZMIENNYCH_TELEMETRYCZNYCH / 8)	//liczba bajtów w ramce telemetrii identyfikujaca przesyłane zmienne

#define TEMETETRIA_WYLACZONA	0xFFFF

void InicjalizacjaTelemetrii(void);
void ObslugaTelemetrii(uint8_t chInterfejs);
float PobierzZmiennaTele(uint8_t chZmienna);
uint8_t WstawDaneDoRamkiTele(uint8_t chIndNapRam, uint8_t chPozycja, uint8_t chIdZmiennej, float fDane);
uint8_t PrzygotujRamkeTele(uint8_t chIndNapRam, uint8_t chAdrZdalny, uint8_t chAdrLokalny, uint8_t chRozmDanych);
void Float2Char16(float fData, uint8_t* chData);
uint8_t ZapiszKonfiguracjeTelemetrii(void);

#endif /* INC_TELEMETRIA_H_ */
