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
#define TID_AKCEL1X				0
#define TID_AKCEL1Y				1
#define TID_AKCEL1Z				2
#define TID_AKCEL2X				3
#define TID_AKCEL2Y				4
#define TID_AKCEL2Z				5
#define TID_ZYRO1P				6
#define TID_ZYRO1Q				7
#define TID_ZYRO1R				8
#define TID_ZYRO2P				9
#define TID_ZYRO2Q				10
#define TID_ZYRO2R				11
#define TID_MAGNE1X				12
#define TID_MAGNE1Y				13
#define TID_MAGNE1Z				14
#define TID_MAGNE2X				15
#define TID_MAGNE2Y				16
#define TID_MAGNE2Z				17
#define TID_MAGNE3X				18
#define TID_MAGNE3Y				19
#define TID_MAGNE3Z				20
#define TID_TEMPIMU1			21
#define TID_TEMPIMU2			22

//zmienne AHRS
#define TID_KAT_IMU1X			23
#define TID_KAT_IMU1Y			24
#define TID_KAT_IMU1Z			25
#define TID_KAT_IMU2X			26
#define TID_KAT_IMU2Y			27
#define TID_KAT_IMU2Z			28
#define TID_KAT_ZYRO1X			29
#define TID_KAT_ZYRO1Y			30
#define TID_KAT_ZYRO1Z			31
#define TID_KAT_AKCELX			32
#define TID_KAT_AKCELY			33
#define TID_KAT_AKCELZ			34

//zmienne barometryczne
#define TID_CISBEZW1			35
#define TID_CISBEZW2			36
#define TID_WYSOKOSC_AGL1		37
#define TID_WYSOKOSC_AGL2		38
#define TID_WYSOKOSC_MSL1		39
#define TID_WYSOKOSC_MSL2		40
#define TID_CISROZN1			41
#define TID_CISROZN2			42
#define TID_PREDIAS1			43
#define TID_PREDIAS2			44
#define TID_WARIO1				45
#define TID_WARIO2				46
#define TID_TEMPCISB1			47
#define TID_TEMPCISB2			48
#define TID_TEMPCISR1			49
#define TID_TEMPCISR2			50

#define TID_RC_KAN1				51		//kanał 1 odbiorników RC
#define TID_RC_KAN2				52
#define TID_RC_KAN3				53
#define TID_RC_KAN4				54
#define TID_RC_KAN5				55
#define TID_RC_KAN6				56
#define TID_RC_KAN7				57
#define TID_RC_KAN8				58
#define TID_RC_KAN9				59
#define TID_RC_KAN10			60
#define TID_RC_KAN11			61
#define TID_RC_KAN12			62
#define TID_RC_KAN13			63
#define TID_RC_KAN14			64
#define TID_RC_KAN15			65
#define TID_RC_KAN16			66

#define TID_SERWO1				67		//serwo 1
#define TID_SERWO2				68
#define TID_SERWO3				69
#define TID_SERWO4				70
#define TID_SERWO5				71
#define TID_SERWO6				72
#define TID_SERWO7				73
#define TID_SERWO8				74
#define TID_SERWO9				75
#define TID_SERWO10				76
#define TID_SERWO11				77
#define TID_SERWO12				78
#define TID_SERWO13				79
#define TID_SERWO14				80
#define TID_SERWO15				81
#define TID_SERWO16				82

#define TID_BSP_AKCELX			83
#define TID_BSP_AKCELY			84
#define TID_BSP_AKCELZ			85
#define TID_BSP_ZYROP			86
#define TID_BSP_ZYROQ			87
#define TID_BSP_ZYROR			88
#define TID_BSP_MAGNEX			89
#define TID_BSP_MAGNEY			90
#define TID_BSP_MAGNEZ			91

#define TID_BSP_IMUX			92
#define TID_BSP_IMUY			93
#define TID_BSP_IMUZ			94
#define TID_BSP_AGL				95
#define TID_BSP_AMSL			96
#define TID_BSP_IAS				97
#define TID_BSP_KURS			98

#define TID_BSP_PRED_POLN		99
#define TID_BSP_PRED_WSCH		100
#define TID_BSP_PRED_WDOL		101
#define TID_BSP_SZER_GEO		102
#define TID_BSP_DLUG_GEO		103

#define TID_DOTYK_ADC0			104
#define TID_DOTYK_ADC1			105
#define TID_DOTYK_ADC2			106
#define TID_CZAS_PETLI			107	//czas trwania ostatniej petli głównej w us
#define TID_JAKOSC_UP_RC1		108
#define TID_JAKOSC_UP_RC2		109
#define TID_JAKOSC_DOWN_RC		110

#define TID_ADC1_1				111
#define TID_ADC1_2				112
#define TID_ADC2_1				113
#define TID_ADC2_2				114
#define TID_BAT1_NAPIECIE		115
#define TID_BAT1_PRAD			116
#define TID_BAT1_ENERGIA		117
#define TID_BAT2_NAPIECIE		118
#define TID_BAT2_PRAD			119
#define TID_BAT2_ENERGIA		120
#define TID_BAT_RTC_NAPIECIE	121
#define TID_TEMPERATURA_CPU		122
#define TID_NAPIECIE_WE1		123
#define TID_NAPIECIE_WE2		124
#define TID_NAPIECIE_SERW		125
#define TID_NAPIECIE_USB		126
#define TID_FFT_ZYRO_AKCEL		127	//wyniki transformaty Fouriera przesyłane w specyficznej szybkiej ramce

//--- zmienne telemetryczne w ramce 2 -----------------------------------------------
#define TID_PID_PRZE_WZAD		128	//wartość zadana regulatora sterowania przechyleniem
#define TID_PID_PRZE_FZAD		129	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
#define TID_PID_PRZE_FWEJ		130	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define TID_PID_PRZE_FROZ		131	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_PRZE_WY_P		132	//wyjście członu P
#define TID_PID_PRZE_WY_I		133	//wyjście członu I
#define TID_PID_PRZE_WY_D		134	//wyjście członu D
#define TID_PID_PRZE_WYPRZ		135	//wyjście członu wyprzedzającego
#define TID_PID_PRZE_WYJ		136	//wyjście regulatora sterowania przechyleniem
#define TID_PID_PK_PRZE_WZAD	137	//wartość zadana regulatora sterowania prędkością kątową przechylenia
#define TID_PID_PK_PRZE_FZAD	138	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
#define TID_PID_PK_PRZE_FWEJ	139	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define TID_PID_PK_PRZE_FROZ	140	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_PK_PRZE_WY_P	141	//wyjście członu P
#define TID_PID_PK_PRZE_WY_D	142	//wyjście członu D
#define TID_PID_PK_PRZE_WYPRZ	143	//wyjście członu wyprzedzającego
#define TID_PID_PK_PRZE_WYJ		144	//wyjście regulatora sterowania prędkością kątową przechylenia

#define TID_PID_POCH_WZAD		145	//wartość zadana regulatora sterowania pochyleniem
#define TID_PID_POCH_FZAD		146	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
#define TID_PID_POCH_FWEJ		147	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define TID_PID_POCH_FROZ		148	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_POCH_WY_P		149	//wyjście członu P
#define TID_PID_POCH_WY_I		150	//wyjście członu I
#define TID_PID_POCH_WY_D		151	//wyjście członu D
#define TID_PID_POCH_WYPRZ		152	//wyjście członu wyprzedzającego
#define TID_PID_POCH_WYJ		153	//wyjście regulatora sterowania pochyleniem
#define TID_PID_PK_POCH_WZAD	154	//wartość zadana regulatora sterowania prędkością kątową pochylenia
#define TID_PID_PK_POCH_FZAD	155	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
#define TID_PID_PK_POCH_FWEJ	156	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define TID_PID_PK_POCH_FROZ	157	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_PK_POCH_WY_P	158	//wyjście członu P
#define TID_PID_PK_POCH_WY_D	159	//wyjście członu D
#define TID_PID_PK_POCH_WYPRZ	160	//wyjście członu wyprzedzającego
#define TID_PID_PK_POCH_WYJ		161	//wyjście regulatora sterowania prędkością kątową pochylenia

#define TID_PID_ODCH_WZAD		162	//wartość zadana regulatora sterowania odchyleniem
#define TID_PID_ODCH_FZAD		163	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
#define TID_PID_ODCH_FWEJ		164	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define TID_PID_ODCH_FROZ		165	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_ODCH_WY_P		166	//wyjście członu P
#define TID_PID_ODCH_WY_I		167	//wyjście członu I
#define TID_PID_ODCH_WY_D		168	//wyjście członu D
#define TID_PID_ODCH_WYPRZ		169	//wyjście członu wyprzedzającego
#define TID_PID_ODCH_WYJ		170	//wyjście regulatora sterowania odchyleniem
#define TID_PID_PK_ODCH_WZAD	171	//wartość zadana regulatora sterowania prędkością kątową odchylenia
#define TID_PID_PK_ODCH_FZAD	172	//przefiltrowana wartość zadana do liczenia wartosci wyprzedzającej
#define TID_PID_PK_ODCH_FWEJ	173	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define TID_PID_PK_ODCH_FROZ	174	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_PK_ODCH_WY_P	175	//wyjście członu P
#define TID_PID_PK_ODCH_WY_D	176	//wyjście członu D
#define TID_PID_PK_ODCH_WYPRZ	177	//wyjście członu wyprzedzającego
#define TID_PID_PK_ODCH_WYJ		178	//wyjście regulatora sterowania prędkością kątową odchylenia

#define TID_PID_WYSO_WZAD		179	//wartość zadana regulatora sterowania wysokością
#define TID_PID_WYSO_FZAD		180	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
#define TID_PID_WYSO_FWEJ		181	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define TID_PID_WYSO_FROZ		182	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_WYSO_WY_P		183	//wyjście członu P
#define TID_PID_WYSO_WY_I		184	//wyjście członu I
#define TID_PID_WYSO_WY_D		185	//wyjście członu D
#define TID_PID_WYSO_WYPRZ		186	//wyjście członu wyprzedzającego
#define TID_PID_WYSO_WYJ		187	//wyjście regulatora sterowania odchyleniem
#define TID_PID_PR_WYSO_WZAD	188	//wartość zadana regulatora prędkości zmiany wysokości
#define TID_PID_PR_WYSO_FWEJ	189	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_PR_WYSO_FZAD	190	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
#define TID_PID_PK_WYSO_FROZ	191	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_PR_WYSO_WY_P	192	//wyjście członu P
#define TID_PID_PR_WYSO_WY_D	193	//wyjście członu D
#define TID_PID_PR_WYSO_WYPRZ	194	//wyjście członu wyprzedzającego
#define TID_PID_PR_WYSO_WYJ		195	//wyjście regulatora sterowania prędkością zmiany wysokości

#define TID_PID_NAWN_WZAD		196	//wartość zadana regulatora sterowania nawigacją w kierunku północnym
#define TID_PID_NAWN_FWEJ		197	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define TID_PID_NAWN_FROZ		198	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_NAWN_WY_P		199	//wyjście członu P
#define TID_PID_NAWN_WY_I		200	//wyjście członu I
#define TID_PID_NAWN_WY_D		201	//wyjście członu D
#define TID_PID_NAWN_WYJ		202	//wyjście regulatora sterowania nawigacją w kierunku północnym

#define TID_PID_PR_NAWN_WZAD	203	//wartość zadana
#define TID_PID_PR_NAWN_FWEJ	204	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define TID_PID_PR_NAWN_FROZ	205	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_PR_NAWN_WY_P	206	//wyjście członu P
#define TID_PID_PR_NAWN_WY_I	207	//wyjście członu I
#define TID_PID_PR_NAWN_WY_D	208	//wyjście członu D
#define TID_PID_PR_NAWN_WYJ		209	//wyjście regulatora sterowania prędkością w kierunku północnym

#define TID_PID_NAWE_WZAD		210	//wartość zadana regulatora sterowania nawigacją w kierunku wschodnim
#define TID_PID_NAWE_FWEJ		211	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define TID_PID_NAWE_FROZ		212	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_NAWE_WY_P		213	//wyjście członu P
#define TID_PID_NAWE_WY_I		214	//wyjście członu I
#define TID_PID_NAWE_WY_D		215	//wyjście członu D
#define TID_PID_NAWE_WYJ		216	//wyjście regulatora sterowania nawigacją w kierunku północnym

#define TID_PID_PR_NAWE_WZAD	217	//wartość zadana
#define TID_PID_PR_NAWE_FWEJ	218	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define TID_PID_PR_NAWE_FROZ	219	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define TID_PID_PR_NAWE_WY_P	220	//wyjście członu P
#define TID_PID_PR_NAWE_WY_I	221	//wyjście członu I
#define TID_PID_PR_NAWE_WY_D	222	//wyjście członu D
#define TID_PID_PR_NAWE_WYJ		223	//wyjście regulatora sterowania prędkością w kierunku wschodnim

#define TID_PID_STROJENIE1		224	//wartość parametru strojącego 1
#define TID_PID_STROJENIE2		225	//wartość parametru strojącego 2
//max do 255

#define LICZBA_ZMIENNYCH_TELEMETRYCZNYCH	218

#define MAX_ZMIENNYCH_TELEMETR_W_RAMCE	115		//tyle zmiennych może być przesłanych w jednej ramce telemetrycznej (ramek może być kilka)
#define MAX_INDEKSOW_TELEMETR_W_RAMCE	128		//zmienne w ramce można wybrać z takiej puli indeksów
#define LICZBA_BAJTOW_ID_TELEMETRII		16		//liczba bajtów w ramce telemetrii identyfikujaca przesyłane zmienne
#define LICZBA_RAMEK_TELEMETR			2		//obecnie są 2 ramki dla zmiennych 0..127 i 128..256
#define MASKA_LICZBY_RAMEK_TELE			0x01
#define TELEMETRIA_WYLACZONA		0xFFFF
#define OKRESOW_TELEMETRII_W_RAMCE		120		//w ramce przesyłane jest na raz się tyle 16-bitowych okresów telemetrii (liczba podzialna przez 15, bo tyle danych mieści się na stronie flash)

void InicjalizacjaTelemetrii(void);
uint8_t ObslugaTelemetrii(uint8_t chInterfejs);
float PobierzZmiennaTele(uint16_t sZmienna, stWymianyCM4_t *stDane);
uint8_t WstawDaneDoRamkiTele(uint8_t chIndNapRam, uint8_t chIndAdresow, uint8_t chPozycja, uint16_t sIdZmiennej, float fDane);
uint8_t WstawDaneDoSzybkiejRamkiTele(uint8_t chIndNapRam, uint8_t chIndeksTestu, uint16_t *sIndeksFFT);
void PrzygotujRamkeTele(uint8_t chIndeksRamki, uint8_t chTypRamki, uint8_t chAdrZdalny, uint8_t chAdrLokalny, uint8_t chRozmDanych);
void Float2Char16(float fData, uint8_t* chData);
uint8_t ZapiszKonfiguracjeTelemetrii(uint16_t sPrzesuniecie);
void WłączTelemetrię(uint8_t chOperacja);
#endif /* INC_TELEMETRIA_H_ */
