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


#define PROB_ODCZYTU_TELEMETRII		3
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

#define TELEID_RC_KAN1		47		//kanał 1 odbiorników RC
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

#define TELEID_SERWO1		63		//serwo 1
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
//max do 127


//zmiene telemetryczne w ramce 2
#define TELEID_PID_PRZE_WZAD	128	//wartość zadana regulatora sterowania przechyleniem
#define TELEID_PID_PRZE_WYJ		129	//wyjście regulatora sterowania przechyleniem
#define TELEID_PID_PRZE_WY_P	130	//wyjście członu P
#define TELEID_PID_PRZE_WY_I	131	//wyjście członu I
#define TELEID_PID_PRZE_WY_D	132	//wyjście członu D

#define TELEID_PID_PK_PRZE_WYJ	133	//wyjście regulatora sterowania prędkością kątową przechylenia
#define TELEID_PID_PK_PRZE_WY_P	134	//wyjście członu P
#define TELEID_PID_PK_PRZE_WY_I	135	//wyjście członu I
#define TELEID_PID_PK_PRZE_WY_D	136	//wyjście członu D

#define TELEID_PID_POCH_WZAD	137	//wartość zadana regulatora sterowania pochyleniem
#define TELEID_PID_POCH_WYJ		138	//wyjście regulatora sterowania pochyleniem
#define TELEID_PID_POCH_WY_P	139	//wyjście członu P
#define TELEID_PID_POCH_WY_I	140	//wyjście członu I
#define TELEID_PID_POCH_WY_D	141	//wyjście członu D

#define TELEID_PID_PK_POCH_WYJ	142	//wyjście regulatora sterowania prędkością kątową pochylenia
#define TELEID_PID_PK_POCH_WY_P	143	//wyjście członu P
#define TELEID_PID_PK_POCH_WY_I	144	//wyjście członu I
#define TELEID_PID_PK_POCH_WY_D	145	//wyjście członu D

#define TELEID_PID_ODCH_WZAD	146	//wartość zadana regulatora sterowania odchyleniem
#define TELEID_PID_ODCH_WYJ		147	//wyjście regulatora sterowania odchyleniem
#define TELEID_PID_ODCH_WY_P	148	//wyjście członu P
#define TELEID_PID_ODCH_WY_I	149	//wyjście członu I
#define TELEID_PID_ODCH_WY_D	150	//wyjście członu D

#define TELEID_PID_PK_ODCH_WYJ	151	//wyjście regulatora sterowania prędkością kątową odchylenia
#define TELEID_PID_PK_ODCH_WY_P	152	//wyjście członu P
#define TELEID_PID_PK_ODCH_WY_I	153	//wyjście członu I
#define TELEID_PID_PK_ODCH_WY_D	154	//wyjście członu D

#define TELEID_PID_WYSO_WZAD	155	//wartość zadana regulatora sterowania wysokością
#define TELEID_PID_WYSO_WYJ		156	//wyjście regulatora sterowania odchyleniem
#define TELEID_PID_WYSO_WY_P	157	//wyjście członu P
#define TELEID_PID_WYSO_WY_I	158	//wyjście członu I
#define TELEID_PID_WYSO_WY_D	159	//wyjście członu D

#define TELEID_PID_PR_WYSO_WYJ	160	//wyjście regulatora sterowania prędkością zmiany wysokości
#define TELEID_PID_PR_WYSO_WY_P	161	//wyjście członu P
#define TELEID_PID_PR_WYSO_WY_I	162	//wyjście członu I
#define TELEID_PID_PR_WYSO_WY_D	163	//wyjście członu D

#define TELEID_PID_NAWN_WZAD	164	//wartość zadana regulatora sterowania nawigacją w kierunku północnym
#define TELEID_PID_NAWN_WYJ		165	//wyjście regulatora sterowania nawigacją w kierunku północnym
#define TELEID_PID_NAWN_WY_P	166	//wyjście członu P
#define TELEID_PID_NAWN_WY_I	167	//wyjście członu I
#define TELEID_PID_NAWN_WY_D	168	//wyjście członu D

#define TELEID_PID_PR_NAWN_WYJ	169	//wyjście regulatora sterowania prędkością w kierunku północnym
#define TELEID_PID_PR_NAWN_WY_P	170	//wyjście członu P
#define TELEID_PID_PR_NAWN_WY_I	171	//wyjście członu I
#define TELEID_PID_PR_NAWN_WY_D	172	//wyjście członu D

#define TELEID_PID_NAWE_WZAD	173	//wartość zadana regulatora sterowania nawigacją w kierunku wschodnim
#define TELEID_PID_NAWE_WYJ		174	//wyjście regulatora sterowania nawigacją w kierunku północnym
#define TELEID_PID_NAWE_WY_P	175	//wyjście członu P
#define TELEID_PID_NAWE_WY_I	176	//wyjście członu I
#define TELEID_PID_NAWE_WY_D	177	//wyjście członu D

#define TELEID_PID_PR_NAWE_WYJ	178	//wyjście regulatora sterowania prędkością w kierunku wschodnim
#define TELEID_PID_PR_NAWE_WY_P	179	//wyjście członu P
#define TELEID_PID_PR_NAWE_WY_I	180	//wyjście członu I
#define TELEID_PID_PR_NAWE_WY_D	181	//wyjście członu D

#define LICZBA_ZMIENNYCH_TELEMETRYCZNYCH	182
//max do 255

#define MAX_ZMIENNYCH_TELEMETR_W_RAMCE	115		//tyle zmiennych może być przesłanych w jednej ramce telemetrycznej (ramek może być kilka)
#define MAX_INDEKSOW_TELEMETR_W_RAMCE	128		//zmienne w ramce można wybrać z takiej puli indeksów
#define LICZBA_BAJTOW_ID_TELEMETRII		16		//liczba bajtów w ramce telemetrii identyfikujaca przesyłane zmienne
#define LICZBA_RAMEK_TELEMETR			2		//obecnie są 2 ramki dla zmiennych 0..127 i 128..256
#define MASKA_LICZBY_RAMEK_TELE			0x01
#define TELEMETRIA_WYLACZONA		0xFFFF
#define OKRESOW_TELEMETRII_W_RAMCE		120		//w ramce przesyłane jest na raz się tyle 16-bitowych okresów telemetrii (liczba podzialna przez 15, bo tyle danych mieści się na stronie flash)

void InicjalizacjaTelemetrii(void);
void ObslugaTelemetrii(uint8_t chInterfejs);
float PobierzZmiennaTele(uint16_t sZmienna);
uint8_t WstawDaneDoRamkiTele(uint8_t chIndNapRam, uint8_t chIndAdresow, uint8_t chPozycja, uint16_t sIdZmiennej, float fDane);
void PrzygotujRamkeTele(uint8_t chIndNapRam, uint8_t chAdrZdalny, uint8_t chAdrLokalny, uint8_t chRozmDanych);
void Float2Char16(float fData, uint8_t* chData);
uint8_t ZapiszKonfiguracjeTelemetrii(uint16_t sPrzesuniecie);

#endif /* INC_TELEMETRIA_H_ */
