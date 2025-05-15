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
#define TELEM1_AKCEL1X		0x000000001
#define TELEM1_AKCEL1Y		0x000000002
#define TELEM1_AKCEL1Z		0x000000004
#define TELEM1_AKCEL2X		0x000000008
#define TELEM1_AKCEL2Y		0x000000010
#define TELEM1_AKCEL2Z		0x000000020
#define TELEM1_ZYRO1P		0x000000040
#define TELEM1_ZYRO1Q		0x000000080
#define TELEM1_ZYRO1R		0x000000100
#define TELEM1_ZYRO2P		0x000000200
#define TELEM1_ZYRO2Q		0x000000400
#define TELEM1_ZYRO2R		0x000000800
#define TELEM1_MAGNE1X		0x000001000
#define TELEM1_MAGNE1Y		0x000002000
#define TELEM1_MAGNE1Z		0x000004000
#define TELEM1_MAGNE2X		0x000008000
#define TELEM1_MAGNE2Y		0x000010000
#define TELEM1_MAGNE2Z		0x000020000
#define TELEM1_MAGNE3X		0x000040000
#define TELEM1_MAGNE3Y		0x000080000
#define TELEM1_MAGNE3Z		0x000100000
#define TELEM1_KAT_IMU1X	0x000200000
#define TELEM1_KAT_IMU1Y	0x000400000
#define TELEM1_KAT_IMU1Z	0x000800000
#define TELEM1_KAT_IMU2X	0x001000000
#define TELEM1_KAT_IMU2Y	0x002000000
#define TELEM1_KAT_IMU2Z	0x004000000
#define TELEM1_KAT_ZYRO1X	0x008000000
#define TELEM1_KAT_ZYRO1Y	0x010000000
#define TELEM1_KAT_ZYRO1Z	0x020000000


#define TELEM2_CISBEZW1		0x100000001
#define TELEM2_CISBEZW2		0x100000002
#define TELEM2_WYSOKOSC1	0x100000004
#define TELEM2_WYSOKOSC2	0x100000008
#define TELEM2_CISROZN1		0x100000010
#define TELEM2_CISROZN2		0x100000020
#define TELEM2_PREDIAS1		0x100000040
#define TELEM2_PREDIAS2		0x100000080
#define TELEM2_TEMPCIS1		0x100000100
#define TELEM2_TEMPCIS2		0x100000200
#define TELEM2_TEMPIMU1		0x100000400
#define TELEM2_TEMPIMU2		0x100000800
#define TELEM2_TEMPCISR1	0x100001000
#define TELEM2_TEMPCISR2	0x100002000

#define LICZBA_ZMIENNYCH_TELEMETRYCZNYCH	(32+14)
#define LICZBA_BAJTOW_ID_TELEMETRII			(LICZBA_ZMIENNYCH_TELEMETRYCZNYCH / 8)	//liczba bajtów w ramce telemetrii identyfikujaca przesyłane zmienne

void InicjalizacjaTelemetrii(void);
void ObslugaTelemetrii(uint8_t chInterfejs);
float PobierzZmiennaTele(uint64_t lZmienna);
uint8_t WstawDoRamkiTele(uint8_t chIndNapRam, uint8_t chPozycja, float fDane);
uint8_t PrzygotujRamkeTele(uint8_t chIndNapRam, uint8_t chAdrZdalny, uint8_t chAdrLokalny, uint64_t lListaZmiennych, uint8_t chRozmDanych);
void Float2Char16(float fData, uint8_t* chData);
void ZapiszKonfiguracjeTelemetrii(void);

#endif /* INC_TELEMETRIA_H_ */
