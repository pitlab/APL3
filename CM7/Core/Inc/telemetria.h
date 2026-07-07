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
#define TID_WYSOKOSC1			37
#define TID_WYSOKOSC2			38
#define TID_CISROZN1			39
#define TID_CISROZN2			40
#define TID_PREDIAS1			41
#define TID_PREDIAS2			42
#define TID_WARIO1				43
#define TID_WARIO2				44
#define TID_TEMPCISB1			45
#define TID_TEMPCISB2			46
#define TID_TEMPCISR1			47
#define TID_TEMPCISR2			48

#define TID_RC_KAN1				49		//kanał 1 odbiorników RC
#define TID_RC_KAN2				50
#define TID_RC_KAN3				51
#define TID_RC_KAN4				52
#define TID_RC_KAN5				53
#define TID_RC_KAN6				54
#define TID_RC_KAN7				55
#define TID_RC_KAN8				56
#define TID_RC_KAN9				57
#define TID_RC_KAN10			58
#define TID_RC_KAN11			59
#define TID_RC_KAN12			60
#define TID_RC_KAN13			61
#define TID_RC_KAN14			62
#define TID_RC_KAN15			63
#define TID_RC_KAN16			64

#define TID_SERWO1				65		//serwo 1
#define TID_SERWO2				66
#define TID_SERWO3				67
#define TID_SERWO4				68
#define TID_SERWO5				69
#define TID_SERWO6				70
#define TID_SERWO7				71
#define TID_SERWO8				72
#define TID_SERWO9				73
#define TID_SERWO10				74
#define TID_SERWO11				75
#define TID_SERWO12				76
#define TID_SERWO13				77
#define TID_SERWO14				78
#define TID_SERWO15				79
#define TID_SERWO16				80

#define TID_BSP_AKCELX			81
#define TID_BSP_AKCELY			82
#define TID_BSP_AKCELZ			83
#define TID_BSP_ZYROP			84
#define TID_BSP_ZYROQ			85
#define TID_BSP_ZYROR			86
#define TID_BSP_MAGNEX			87
#define TID_BSP_MAGNEY			88
#define TID_BSP_MAGNEZ			89

#define TID_BSP_IMUX			90
#define TID_BSP_IMUY			91
#define TID_BSP_IMUZ			92
#define TID_BSP_AGL				93
#define TID_BSP_AMSL			94
#define TID_BSP_IAS				95
#define TID_BSP_KURS			96

#define TID_BSP_PRED_PÓŁN		97
#define TID_BSP_PRED_WSCH		98
#define TID_BSP_PRED_WDÓŁ		99
#define TID_BSP_SZER_GEO		100
#define TID_BSP_DŁUG_GEO		101

#define TID_DOTYK_ADC0			102
#define TID_DOTYK_ADC1			103
#define TID_DOTYK_ADC2			104
#define TID_CZAS_PETLI			105	//czas trwania ostatniej petli głównej w us
#define TID_JAKOSC_UP_RC1		106
#define TID_JAKOSC_UP_RC2		107
#define TID_JAKOSC_DOWN_RC		108

#define TID_ADC1_1				109
#define TID_ADC1_2				110
#define TID_ADC2_1				111
#define TID_ADC2_2				112
#define TID_BAT1_NAPIECIE		113
#define TID_BAT1_PRAD			114
#define TID_BAT1_ENERGIA		115
#define TID_BAT2_NAPIECIE		116
#define TID_BAT2_PRAD			117
#define TID_BAT2_ENERGIA		118
#define TID_BAT_RTC_NAPIECIE	119
#define TID_TEMPERATURA_CPU		120
#define TID_VREF_ADC			121
#define TID_NAPIECIE_WE1		122
#define TID_NAPIECIE_WE2		123
#define TID_NAPIECIE_SERW		124
#define TID_NAPIECIE_USB		125
//max do 127

#define TID_FFT_ZYRO_AKCEL	127	//wyniki transformaty Fouriera przesyłane w specyficznej szybkiej ramce

//--- zmienne telemetryczne w ramce 2 -----------------------------------------------
#define TID_PID_PRZE_WZAD		128	//wartość zadana regulatora sterowania przechyleniem
#define TID_PID_PRZE_FWEJ		129	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_PRZE_CALK		130	//wartość całki członu I
#define TID_PID_PRZE_WYJ		131	//wyjście regulatora sterowania przechyleniem
#define TID_PID_PRZE_WY_P		132	//wyjście członu P
#define TID_PID_PRZE_WY_I		133	//wyjście członu I
#define TID_PID_PRZE_WY_D		134	//wyjście członu D

#define TID_PID_PK_PRZE_WZAD	135	//wartość zadana regulatora sterowania prędkością kątową przechylenia
#define TID_PID_PK_PRZE_FWEJ	136	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_PK_PRZE_FZAD	137	//przefiltrowana wartość zadana do liczenia wartosci wyprzedzającej
#define TID_PID_PK_PRZE_WYJ		138	//wyjście regulatora sterowania prędkością kątową przechylenia
#define TID_PID_PK_PRZE_WY_P	139	//wyjście członu P
#define TID_PID_PK_PRZE_WY_D	140	//wyjście członu D
#define TID_PID_PK_PRZE_WYPRZ	141	//wyjście akcji wyprzedzającej

#define TID_PID_POCH_WZAD		142	//wartość zadana regulatora sterowania pochyleniem
#define TID_PID_POCH_FWEJ		143 //przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_POCH_CALK		144	//wartość całki członu I
#define TID_PID_POCH_WYJ		145	//wyjście regulatora sterowania pochyleniem
#define TID_PID_POCH_WY_P		146	//wyjście członu P
#define TID_PID_POCH_WY_I		147	//wyjście członu I
#define TID_PID_POCH_WY_D		148	//wyjście członu D

#define TID_PID_PK_POCH_WZAD	149	//wartość zadana
#define TID_PID_PK_POCH_FWEJ	150	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_PK_POCH_FZAD	151	//przefiltrowana wartość zadana do liczenia wartosci wyprzedzającej
#define TID_PID_PK_POCH_WYJ		152	//wyjście regulatora sterowania prędkością kątową pochylenia
#define TID_PID_PK_POCH_WY_P	153	//wyjście członu P
#define TID_PID_PK_POCH_WY_D	154	//wyjście członu D
#define TID_PID_PK_POCH_WYPRZ	155	//wyjście akcji wyprzedzającej

#define TID_PID_ODCH_WZAD		156	//wartość zadana regulatora sterowania odchyleniem
#define TID_PID_ODCH_FWEJ		157	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_ODCH_CALK		158	//wartość całki członu I
#define TID_PID_ODCH_WYJ		159	//wyjście regulatora sterowania odchyleniem
#define TID_PID_ODCH_WY_P		160	//wyjście członu P
#define TID_PID_ODCH_WY_I		161	//wyjście członu I
#define TID_PID_ODCH_WY_D		162	//wyjście członu D

#define TID_PID_PK_ODCH_WZAD	163	//wartość zadana
#define TID_PID_PK_ODCH_FWEJ	164	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_PK_ODCH_FZAD	165	//przefiltrowana wartość zadana do liczenia wartosci wyprzedzającej
#define TID_PID_PK_ODCH_WYJ		166	//wyjście regulatora sterowania prędkością kątową odchylenia
#define TID_PID_PK_ODCH_WY_P	167	//wyjście członu P
#define TID_PID_PK_ODCH_WY_D	168	//wyjście członu D
#define TID_PID_PK_ODCH_WYPRZ	169	//wyjście akcji wyprzedzającej

#define TID_PID_WYSO_WZAD		170	//wartość zadana regulatora sterowania wysokością
#define TID_PID_WYSO_FWEJ		171	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_WYSO_CALK		172	//wartość całki członu I
#define TID_PID_WYSO_WYJ		173	//wyjście regulatora sterowania odchyleniem
#define TID_PID_WYSO_WY_P		174	//wyjście członu P
#define TID_PID_WYSO_WY_I		175	//wyjście członu I
#define TID_PID_WYSO_WY_D		176	//wyjście członu D

#define TID_PID_PR_WYSO_WZAD	177	//wartość zadana
#define TID_PID_PR_WYSO_FWEJ	178	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_PR_WYSO_FZAD	179	//przefiltrowana wartość zadana do liczenia wartosci wyprzedzającej
#define TID_PID_PR_WYSO_WYJ		180	//wyjście regulatora sterowania prędkością zmiany wysokości
#define TID_PID_PR_WYSO_WY_P	181	//wyjście członu P
#define TID_PID_PR_WYSO_WY_D	182	//wyjście członu D
#define TID_PID_PR_WYSO_WYPRZ	183	//wyjście akcji wyprzedzającej

#define TID_PID_NAWN_WZAD		184	//wartość zadana regulatora sterowania nawigacją w kierunku północnym
#define TID_PID_NAWN_FWEJ		185	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_NAWN_CALK		186	//wartość całki członu I
#define TID_PID_NAWN_WYJ		187	//wyjście regulatora sterowania nawigacją w kierunku północnym
#define TID_PID_NAWN_WY_P		188	//wyjście członu P
#define TID_PID_NAWN_WY_I		189	//wyjście członu I
#define TID_PID_NAWN_WY_D		190	//wyjście członu D

#define TID_PID_PR_NAWN_WZAD	191	//wartość zadana
#define TID_PID_PR_NAWN_FWEJ	192	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_PR_NAWN_WYJ		193	//wyjście regulatora sterowania prędkością w kierunku północnym
#define TID_PID_PR_NAWN_CALK	194	//wartość całki członu I
#define TID_PID_PR_NAWN_WY_P	195	//wyjście członu P
#define TID_PID_PR_NAWN_WY_I	196	//wyjście członu I
#define TID_PID_PR_NAWN_WY_D	197	//wyjście członu D

#define TID_PID_NAWE_WZAD		198	//wartość zadana regulatora sterowania nawigacją w kierunku wschodnim
#define TID_PID_NAWE_FWEJ		199	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define TID_PID_NAWE_CALK		200	//wartość całki członu I
#define TID_PID_NAWE_WYJ		201	//wyjście regulatora sterowania nawigacją w kierunku północnym
#define TID_PID_NAWE_WY_P		202	//wyjście członu P
#define TID_PID_NAWE_WY_I		203	//wyjście członu I
#define TID_PID_NAWE_WY_D		204	//wyjście członu D

#define TID_PID_PR_NAWE_WZAD	205	//wartość zadana
#define TID_PID_PR_NAWE_FWEJ	206	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
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
