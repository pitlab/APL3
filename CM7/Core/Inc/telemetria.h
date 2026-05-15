/*
 * telemetria.h
 *
 *  Created on: May 14, 2025
 *      Author: PitLab
 */

#ifndef INC_TELEMETRIA_H_
#define INC_TELEMETRIA_H_

#include "SysDefCM7.h"
#include "PoleceniaKomunikacyjne.h"

//polecenia ramki PK_WSTRZYMAJ_TELEMETRIE służące do zarządzania strumieniem telemetrii
#define TELEM_SZYBKA	2	//ma być transmitowana szybka ramka telemetryczna z wynikami FFT
#define TELEM_WSTRZYMAJ	1	//zatrzymaj wysyłanie telemetrii
//#define TELEM_WZNOW		0	//wznów wysyłanie telemetrii
#define TELEM_NORMALNA	0	//ma być transmitowana ramka normalna (to samo co wznowienie)

#define PROB_ODCZYTU_TELEMETRII		3
//definicje zmiennych telemetrycznych
//zmienne IMU
#define TID_AKCEL1X		0
#define TID_AKCEL1Y		1
#define TID_AKCEL1Z		2
#define TID_AKCEL2X		3
#define TID_AKCEL2Y		4
#define TID_AKCEL2Z		5
#define TID_ZYRO1P		6
#define TID_ZYRO1Q		7
#define TID_ZYRO1R		8
#define TID_ZYRO2P		9
#define TID_ZYRO2Q		10
#define TID_ZYRO2R		11
#define TID_MAGNE1X		12
#define TID_MAGNE1Y		13
#define TID_MAGNE1Z		14
#define TID_MAGNE2X		15
#define TID_MAGNE2Y		16
#define TID_MAGNE2Z		17
#define TID_MAGNE3X		18
#define TID_MAGNE3Y		19
#define TID_MAGNE3Z		20
#define TID_TEMPIMU1		21
#define TID_TEMPIMU2		22

//zmienne AHRS
#define TID_KAT_IMU1X	23
#define TID_KAT_IMU1Y	24
#define TID_KAT_IMU1Z	25
#define TID_KAT_IMU2X	26
#define TID_KAT_IMU2Y	27
#define TID_KAT_IMU2Z	28
#define TID_KAT_ZYRO1X	29
#define TID_KAT_ZYRO1Y	30
#define TID_KAT_ZYRO1Z	31
#define TID_KAT_AKCELX	32
#define TID_KAT_AKCELY	33
#define TID_KAT_AKCELZ	34

//zmienne barometryczne
#define TID_CISBEZW1		35
#define TID_CISBEZW2		36
#define TID_WYSOKOSC1	37
#define TID_WYSOKOSC2	38
#define TID_CISROZN1		39
#define TID_CISROZN2		40
#define TID_PREDIAS1		41
#define TID_PREDIAS2		42
#define TID_TEMPCISB1	43
#define TID_TEMPCISB2	44
#define TID_TEMPCISR1	45
#define TID_TEMPCISR2	46

#define TID_RC_KAN1		47		//kanał 1 odbiorników RC
#define TID_RC_KAN2		48
#define TID_RC_KAN3		49
#define TID_RC_KAN4		50
#define TID_RC_KAN5		51
#define TID_RC_KAN6		52
#define TID_RC_KAN7		53
#define TID_RC_KAN8		54
#define TID_RC_KAN9		55
#define TID_RC_KAN10		56
#define TID_RC_KAN11		57
#define TID_RC_KAN12		58
#define TID_RC_KAN13		59
#define TID_RC_KAN14		60
#define TID_RC_KAN15		61
#define TID_RC_KAN16		62

#define TID_SERWO1		63		//serwo 1
#define TID_SERWO2		64
#define TID_SERWO3		65
#define TID_SERWO4		66
#define TID_SERWO5		67
#define TID_SERWO6		68
#define TID_SERWO7		69
#define TID_SERWO8		70
#define TID_SERWO9		71
#define TID_SERWO10		72
#define TID_SERWO11		73
#define TID_SERWO12		74
#define TID_SERWO13		75
#define TID_SERWO14		76
#define TID_SERWO15		77
#define TID_SERWO16		78

#define TID_DOTYK_ADC0	79
#define TID_DOTYK_ADC1	80
#define TID_DOTYK_ADC2	81
#define TID_CZAS_PETLI	82	//czas trwania ostatniej petli głównej w us
#define TID_ROZNE_F11	83	//zmienna debugująca fRóżne[11]
//max do 127

#define TID_FFT_ZYRO_AKCEL	127	//wyniki transformaty Fouriera przesyłane w specyficznej szybkiej ramce

//zmiene telemetryczne w ramce 2
#define TID_PID_PRZE_WZAD		128	//wartość zadana regulatora sterowania przechyleniem
#define TID_PID_PRZE_WE_D		129	//wartość wejsciowa członu D po filtrze
#define TID_PID_PRZE_CALK		130	//wartość całki członu I
#define TID_PID_PRZE_WYJ		131	//wyjście regulatora sterowania przechyleniem
#define TID_PID_PRZE_WY_P		132	//wyjście członu P
#define TID_PID_PRZE_WY_I		133	//wyjście członu I
#define TID_PID_PRZE_WY_D		134	//wyjście członu D

#define TID_PID_PK_PRZE_WZAD	135	//wartość zadana regulatora sterowania prędkością kątową przechylenia
#define TID_PID_PK_PRZE_WE_D	136	//wartość wejsciowa członu D po filtrze
#define TID_PID_PK_PRZE_CALK	137	//wartość całki członu I
#define TID_PID_PK_PRZE_WYJ		138	//wyjście regulatora sterowania prędkością kątową przechylenia
#define TID_PID_PK_PRZE_WY_P	139	//wyjście członu P
//#define TID_PID_PK_PRZE_WY_I	140	//wyjście członu I
#define TID_PID_PK_PRZE_WY_D	140	//wyjście członu D
#define TID_PID_PK_PRZE_WYPRZ	141	//wartość wyprzedzająca

#define TID_PID_POCH_WZAD		142	//wartość zadana regulatora sterowania pochyleniem
#define TID_PID_POCH_WE_D		143	//wartość wejsciowa członu D po filtrze
#define TID_PID_POCH_CALK		144	//wartość całki członu I
#define TID_PID_POCH_WYJ		145	//wyjście regulatora sterowania pochyleniem
#define TID_PID_POCH_WY_P		146	//wyjście członu P
#define TID_PID_POCH_WY_I		147	//wyjście członu I
#define TID_PID_POCH_WY_D		148	//wyjście członu D

#define TID_PID_PK_POCH_WZAD	149	//wartość zadana
#define TID_PID_PK_POCH_WE_D	150	//wartość wejsciowa członu D po filtrze
#define TID_PID_PK_POCH_CALK	151	//wartość całki członu I
#define TID_PID_PK_POCH_WYJ		152	//wyjście regulatora sterowania prędkością kątową pochylenia
#define TID_PID_PK_POCH_WY_P	153	//wyjście członu P
//#define TID_PID_PK_POCH_WY_I	154	//wyjście członu I
#define TID_PID_PK_POCH_WY_D	154	//wyjście członu D
#define TID_PID_PK_POCH_WYPRZ	155	//wartość wyprzedzająca

#define TID_PID_ODCH_WZAD		156	//wartość zadana regulatora sterowania odchyleniem
#define TID_PID_ODCH_WE_D		157	//wartość wejsciowa członu D po filtrze
#define TID_PID_ODCH_CALK		158	//wartość całki członu I
#define TID_PID_ODCH_WYJ		159	//wyjście regulatora sterowania odchyleniem
#define TID_PID_ODCH_WY_P		160	//wyjście członu P
#define TID_PID_ODCH_WY_I		161	//wyjście członu I
#define TID_PID_ODCH_WY_D		162	//wyjście członu D

#define TID_PID_PK_ODCH_WZAD	163	//wartość zadana
#define TID_PID_PK_ODCH_WE_D	164	//wartość wejsciowa członu D po filtrze
#define TID_PID_PK_ODCH_CALK	165	//wartość całki członu I
#define TID_PID_PK_ODCH_WYJ		166	//wyjście regulatora sterowania prędkością kątową odchylenia
#define TID_PID_PK_ODCH_WY_P	167	//wyjście członu P
//#define TID_PID_PK_ODCH_WY_I	168	//wyjście członu I
#define TID_PID_PK_ODCH_WY_D	168	//wyjście członu D
#define TID_PID_PK_ODCH_WYPRZ	169	//wartość wyprzedzająca

#define TID_PID_WYSO_WZAD		170	//wartość zadana regulatora sterowania wysokością
#define TID_PID_WYSO_WE_D		171	//wartość wejsciowa członu D po filtrze
#define TID_PID_WYSO_CALK		172	//wartość całki członu I
#define TID_PID_WYSO_WYJ		173	//wyjście regulatora sterowania odchyleniem
#define TID_PID_WYSO_WY_P		174	//wyjście członu P
#define TID_PID_WYSO_WY_I		175	//wyjście członu I
#define TID_PID_WYSO_WY_D		176	//wyjście członu D

#define TID_PID_PR_WYSO_WZAD	177	//wartość zadana
#define TID_PID_PR_WYSO_WE_D	178	//wartość wejsciowa członu D po filtrze
#define TID_PID_PR_WYSO_CALK	179	//wartość całki członu I
#define TID_PID_PR_WYSO_WYJ		180	//wyjście regulatora sterowania prędkością zmiany wysokości
#define TID_PID_PR_WYSO_WY_P	181	//wyjście członu P
//#define TID_PID_PR_WYSO_WY_I	182	//wyjście członu I
#define TID_PID_PR_WYSO_WY_D	182	//wyjście członu D
#define TID_PID_PR_WYSO_WYPRZ	183	//wartość wyprzedzająca

#define TID_PID_NAWN_WZAD		184	//wartość zadana regulatora sterowania nawigacją w kierunku północnym
#define TID_PID_NAWN_WE_D		185	//wartość wejsciowa członu D po filtrze
#define TID_PID_NAWN_CALK		186	//wartość całki członu I
#define TID_PID_NAWN_WYJ		187	//wyjście regulatora sterowania nawigacją w kierunku północnym
#define TID_PID_NAWN_WY_P		188	//wyjście członu P
#define TID_PID_NAWN_WY_I		189	//wyjście członu I
#define TID_PID_NAWN_WY_D		190	//wyjście członu D

#define TID_PID_PR_NAWN_WZAD	191	//wartość zadana
#define TID_PID_PR_NAWN_WE_D	192	//wartość wejsciowa członu D po filtrze
#define TID_PID_PR_NAWN_CALK	193	//wartość całki członu I
#define TID_PID_PR_NAWN_WYJ		194	//wyjście regulatora sterowania prędkością w kierunku północnym
#define TID_PID_PR_NAWN_WY_P	195	//wyjście członu P
#define TID_PID_PR_NAWN_WY_I	196	//wyjście członu I
#define TID_PID_PR_NAWN_WY_D	197	//wyjście członu D


#define TID_PID_NAWE_WZAD		198	//wartość zadana regulatora sterowania nawigacją w kierunku wschodnim
#define TID_PID_NAWE_WE_D		199	//wartość wejsciowa członu D po filtrze
#define TID_PID_NAWE_CALK		200	//wartość całki członu I
#define TID_PID_NAWE_WYJ		201	//wyjście regulatora sterowania nawigacją w kierunku północnym
#define TID_PID_NAWE_WY_P		202	//wyjście członu P
#define TID_PID_NAWE_WY_I		203	//wyjście członu I
#define TID_PID_NAWE_WY_D		204	//wyjście członu D

#define TID_PID_PR_NAWE_WZAD	205	//wartość zadana
#define TID_PID_PR_NAWE_WE_D	206	//wartość wejsciowa członu D po filtrze
#define TID_PID_PR_NAWE_CALK	207	//wartość całki członu I
#define TID_PID_PR_NAWE_WYJ		208	//wyjście regulatora sterowania prędkością w kierunku wschodnim
#define TID_PID_PR_NAWE_WY_P	209	//wyjście członu P
#define TID_PID_PR_NAWE_WY_I	210	//wyjście członu I
#define TID_PID_PR_NAWE_WY_D	211	//wyjście członu D


#define TID_PID_STROJENIE1		212	//wartość parametru strojącego 1
#define TID_PID_STROJENIE2		213	//wartość parametru strojącego 2
//max do 255

#define LICZBA_ZMIENNYCH_TELEMETRYCZNYCH	214

#define MAX_ZMIENNYCH_TELEMETR_W_RAMCE	115		//tyle zmiennych może być przesłanych w jednej ramce telemetrycznej (ramek może być kilka)
#define MAX_INDEKSOW_TELEMETR_W_RAMCE	128		//zmienne w ramce można wybrać z takiej puli indeksów
#define LICZBA_BAJTOW_ID_TELEMETRII		16		//liczba bajtów w ramce telemetrii identyfikujaca przesyłane zmienne
#define LICZBA_RAMEK_TELEMETR			2		//obecnie są 2 ramki dla zmiennych 0..127 i 128..256
#define MASKA_LICZBY_RAMEK_TELE			0x01
#define TELEMETRIA_WYLACZONA		0xFFFF
#define OKRESOW_TELEMETRII_W_RAMCE		120		//w ramce przesyłane jest na raz się tyle 16-bitowych okresów telemetrii (liczba podzialna przez 15, bo tyle danych mieści się na stronie flash)

void InicjalizacjaTelemetrii(void);
uint8_t ObslugaTelemetrii(uint8_t chInterfejs);
float PobierzZmiennaTele(uint16_t sZmienna);
uint8_t WstawDaneDoRamkiTele(uint8_t chIndNapRam, uint8_t chIndAdresow, uint8_t chPozycja, uint16_t sIdZmiennej, float fDane);
uint8_t WstawDaneDoSzybkiejRamkiTele(uint8_t chIndNapRam, uint8_t chIndeksTestu, uint16_t *sIndeksFFT);
void PrzygotujRamkeTele(uint8_t chIndeksRamki, uint8_t chTypRamki, uint8_t chAdrZdalny, uint8_t chAdrLokalny, uint8_t chRozmDanych);
void Float2Char16(float fData, uint8_t* chData);
uint8_t ZapiszKonfiguracjeTelemetrii(uint16_t sPrzesuniecie);
void WłączTelemetrię(uint8_t chOperacja);
#endif /* INC_TELEMETRIA_H_ */
