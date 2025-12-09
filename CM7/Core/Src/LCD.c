//////////////////////////////////////////////////////////////////////////////
//
// Moduł rysowania na ekranie
// Abstrahuje od sprzętu, może przynajmniej teoretycznie pracować z różnymi wyświetlaczami
//
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "LCD.h"
#include <stdio.h>
#include <math.h>
#include <RPi35B_480x320.h>
#include <ili9488.h>
#include <string.h>
#include <W25Q128JV.h>
#include "dotyk.h"
#include "napisy.h"
#include "flash_nor.h"
#include "wymiana.h"
#include "audio.h"
#include "protokol_kom.h"
#include "rejestrator.h"
#include "wspolne.h"
#include <stdlib.h>
#include "CAN.h"
#include "czas.h"
#include "fraktale.h"
#include "konfig_fram.h"
#include "pid_kanaly.h"
#include "kamera.h"
#include "pamiec.h"
#include "analiza_obrazu.h"
#include "ip4_addr.h"
#include "ff.h"
#include "lwip/stats.h"
#include "jpeg.h"
#include "LCD_mem.h"
#include "osd.h"
#include "bmp.h"
#include "jpeg_utils_conf.h"
#include "jpeg_utils.h"

//deklaracje zmiennych
extern uint8_t MidFont[];
extern uint8_t BigFont[];
const char *build_date = __DATE__;
const char *build_time = __TIME__;
extern const unsigned short obr_multimetr[];
extern const unsigned short obr_multitool[];
extern const unsigned short obr_mmedia[];
extern const unsigned short obr_fraktal[];
extern const unsigned short obr_RAM[];
extern const unsigned short obr_NOR[];
extern const unsigned short obr_QSPI[];
extern const unsigned short obr_fraktak[];
extern const unsigned short obr_dotyk[];
extern const unsigned short obr_fft[];
extern const unsigned short obr_glosnik1[];
extern const unsigned short obr_glosnik2[];
extern const unsigned short obr_glosnik_neg[];
extern const unsigned short obr_foto[];
extern const unsigned short obr_Mikołaj_Rey[];
extern const unsigned short obr_Wydajnosc[];
extern const unsigned short obr_mtest[];
extern const unsigned short obr_volume[];
extern const unsigned short obr_touch[0xFFC];
extern const unsigned short obr_dotyk[0xFFC];
extern const unsigned short obr_dotyk_zolty[0xFFC];
extern const unsigned short obr_kartaSD[0xFFC];
extern const unsigned short obr_kartaSDneg[0xFFC];
//extern const unsigned short obr_baczek[0xF80];
extern const unsigned short obr_kal_mag_n1[0xFFC];
extern const unsigned short obr_kontrolny[0xFFC];
extern const unsigned short obr_kostka3D[0xFFC];
extern const unsigned short obr_cisnienie[0xFFC];
extern const unsigned short obr_okregi[0xFFC];
extern const unsigned short obr_narzedzia[0xFFC];
extern const unsigned short obr_aparaturaRC[0xFFC];
extern const unsigned short obr_bmp24[0xFFC];
extern const unsigned short obr_bmp8[0xFFC];


//wygenerowane przez chata GPT
extern const unsigned short obr_Polaczenie[0xFFC];
extern const unsigned short obr_konfiguracja[0xFFC];
extern const unsigned short obr_KonfKartySD[0xFFC];
extern const unsigned short obr_aparat[0xFFC];
extern const unsigned short obr_KonfigDotyk[0xFFC];
extern const unsigned short obr_kamera[0xFFC];
extern const unsigned short obr_papuga1[0xFFC];
extern const unsigned short obr_powrot1[0xFFC];
extern const unsigned short obr_glosnik1[0xFFC];
extern const unsigned short obr_zyroskop[0xFFC];

extern const char *chNapisLcd[MAX_NAPISOW];
extern const char *chOpisBledow[MAX_KOMUNIKATOW];
extern const char *chNazwyMies3Lit[13];
extern int16_t __attribute__ ((aligned (16))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforPapuga[ROZMIAR_BUFORA_PAPUGI];
//definicje zmiennych
uint8_t chTrybPracy;
uint8_t chNowyTrybPracy;
uint8_t chWrocDoTrybu;
uint8_t chRysujRaz;
uint8_t chMenuSelPos, chStarySelPos;	//wybrana pozycja menu i poprzednia pozycja
static uint8_t chErr;	//zmienna obowiązuje lokalnie
char chNapis[100], chNapisPodreczny[30];
float fTemperaturaKalibracji;
uint8_t chLiczIter;		//licznik iteracji wyświetlania
extern struct _statusDotyku statusDotyku;
extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
extern uint8_t chPort_exp_wysylany[];
extern uint8_t chPort_exp_odbierany[];
extern uint8_t chGlosnosc;		//regulacja głośności odtwarzania komunikatów w zakresie 0..SKALA_GLOSNOSCI
extern SD_HandleTypeDef hsd1;
extern uint8_t chStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
extern unia_wymianyCM7_t uDaneCM7;
extern volatile unia_wymianyCM4_t uDaneCM4;
float fKostka[8][3] = {		//załóżmy wstępnie że kostka będzie miała rozmiar połowy wyświetlacza i umieszczona centralnie na wyswietlaczu. Dla kątów zerowych będzie widzana z góry
			{-DISP_X_SIZE/4, -DISP_Y_SIZE/4, DISP_Y_SIZE/4},
			{ DISP_X_SIZE/4, -DISP_Y_SIZE/4, DISP_Y_SIZE/4},
			{ DISP_X_SIZE/4,  DISP_Y_SIZE/4, DISP_Y_SIZE/4},
			{-DISP_X_SIZE/4,  DISP_Y_SIZE/4, DISP_Y_SIZE/4},
			{-DISP_X_SIZE/4, -DISP_Y_SIZE/4, -DISP_Y_SIZE/4},
			{ DISP_X_SIZE/4, -DISP_Y_SIZE/4, -DISP_Y_SIZE/4},
			{ DISP_X_SIZE/4,  DISP_Y_SIZE/4, -DISP_Y_SIZE/4},
			{-DISP_X_SIZE/4,  DISP_Y_SIZE/4, -DISP_Y_SIZE/4}};

float fSymKatKostki[3];	//zmienna do przechowywania wyników symulacji ruchu kostki
int16_t sKostkaPoprzednia[8][2];	//poprzednia pozycja kostki 3D: [wierzchołki][x,y]
uint8_t chSekwencerKalibracji;		//wskazuje na daną oś jako kolejny etap kalibracji
prostokat_t stPrzycisk;
uint8_t chStanPrzycisku;
uint8_t chEtapKalibracji;
prostokat_t stWykr;	//wykres biegunowy magnetometru
uint8_t chHistR[ROZMIAR_HIST_KOLOR], chHistG[ROZMIAR_HIST_KOLOR], chHistB[ROZMIAR_HIST_KOLOR];
uint8_t chHistCB8[ROZMIAR_HIST_CB8];

extern uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforKamerySRAM[ROZM_BUF_YUV420];
extern uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) sBuforKameryDRAM[ROZM_BUF16_KAM];
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforLCD[DISP_X_SIZE * DISP_Y_SIZE * 3];
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforOSD[DISP_X_SIZE * DISP_Y_SIZE * 3];	//pamięć obrazu OSD w formacie RGB888
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaAxiSRAM"))) chBuforJpeg[ILOSC_BUF_JPEG][ROZM_BUF_WY_JPEG];
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) chBuforTestowyDRAM[100];
extern stKonfKam_t stKonfKam;
extern uint32_t nRozmiarObrazuKamery;
extern stDiagKam_t stDiagKam;	//diagnostyka stanu kamery
extern JPEG_HandleTypeDef hjpeg;
extern uint32_t nRozmiarObrazuJPEG;	//w bajtach
extern const uint8_t chNaglJpegSOI[20];
extern const uint8_t chNaglJpegEOI[2];
extern FIL SDJpegFile;       //struktura pliku z obrazem
extern uint8_t chNazwaPlikuObrazu[DLG_NAZWY_PLIKU_OBR];	//początek nazwy pliku z obrazem, po tym jest data i czas

extern uint8_t chWskNapBufKam;	//wskaźnik napełnaniania bufora kamery
extern volatile uint8_t chObrazKameryGotowy;	//flaga gotowości obrazu, ustawiana w callbacku
uint8_t chTimeout;
extern stKonfOsd_t stKonfOSD;
extern volatile uint8_t chTrybPracyKamery;	//steruje co dalej robić z obrazem pozyskanym przez DCMI
uint32_t nCzas, nCzasHist;
extern uint32_t nCzasBlend, nCzasLCD;

//Definicje ekranów menu
struct tmenu stMenuGlowne[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Pomiary",		"Wyniki pomiarow czujnikow",				TP_POMIARY, 		obr_multimetr},
	{"Kalibracje", 	"Kalibracja czujnikow pokladowych",			TP_KALIBRACJE,		obr_kontrolny},
	{"Nastawy",  	"Ustawienia podsystemow",					TP_NASTAWY, 		obr_konfiguracja},
	{"Wydajnosc",	"Pomiary wydajnosci systemow",				TP_WYDAJNOSC,		obr_Wydajnosc},
	{"Karta SD",	"Rejestrator i parametry karty SD",			TP_KARTA_SD,		obr_kartaSD},
	{"Ethernet",	"Obsluga ethernetu",						TP_ETHERNET,		obr_Polaczenie},
	{"Testy",		"Test",										TP_TESTY,			obr_narzedzia},
	{"OSD",			"Obsluga OSD",								TP_MENU_OSD,		obr_aparat},
	{"Kamera",		"Obsluga kamery",							TP_MEDIA_KAMERA,	obr_kamera},
	{"Audio",  		"Obsluga multimediow: dzwiek i obraz",		TP_MEDIA_AUDIO,		obr_glosnik1}};



struct tmenu stMenuKalibracje[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Kal IMU", 	"Kalibracja czujników IMU",					TP_KAL_IMU,			obr_zyroskop},
	{"Kal Magn", 	"Kalibracja magnetometrow",					TP_KAL_MAG,			obr_kal_mag_n1},
	{"Kal Baro", 	"Kalibracja cisnienia wg wzorca 10 pieter",	TP_KAL_BARO,		obr_cisnienie},
	{"Kal Dotyk", 	"Kalibracja panelu dotykowego na LCD",		TP_KAL_DOTYK,		obr_KonfigDotyk},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"HardFault",	"Genruje wystapiene HardFault",				TP_KAL_HARD_FAULT,	obr_narzedzia},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

struct tmenu stMenuPomiary[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Dane IMU",	"Wyniki pomiarow czujnikow IMU",			TP_POMIARY_IMU, 	obr_multimetr},
	{"Dane cisn",	"Wyniki pomiarow czujnikow cisnienia",		TP_POMIARY_CISN, 	obr_multimetr},
	{"Dane RC",		"Dane z odbiornika RC",						TP_POMIARY_RC,		obr_aparaturaRC},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"TestDotyk",	"Testy panelu dotykowego",					TP_POMIARY_DOTYKU,			obr_dotyk_zolty},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

struct tmenu stMenuNastawy[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"PID Pochyl",	"Nastawy PID pochylenia",					TP_NAST_PID_POCH, 	obr_narzedzia},
	{"PID Przech",	"Nastawy PID przechylenia",					TP_NAST_PID_PRZECH, obr_narzedzia},
	{"PID Odchyl",	"Nastawy PID odchylenia",					TP_NAST_PID_ODCH,	obr_narzedzia},
	{"PID Wysoko",	"Nastawy PID wysokosci",					TP_NAST_PID_WYSOK,	obr_narzedzia},
	{"PID NawigN",	"Nastawy PID nawigacji północnej",			TP_NAST_PID_NAW_N,	obr_narzedzia},
	{"PID NawigE",	"Nastawy PID nawigacji wschodniej",			TP_NAST_PID_NAW_E,	obr_narzedzia},
	{"Mikser",		"Nastawy miksera silnikow",					TP_NAST_MIKSERA,	obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};


struct tmenu stMenuWydajnosc[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Fraktale",	"Benchmark fraktalowy"	,					TP_FRAKTALE,		obr_fraktal},
	{"Zapis NOR", 	"Test zapisu do flash NOR",					TP_POM_ZAPISU_NOR,	obr_NOR},
	{"Trans NOR", 	"Pomiar predkosci flasha NOR 16-bit",		TP_POMIAR_FNOR,		obr_NOR},
	{"Trans QSPI",	"Pomiar predkosci flasha QSPI 4-bit",		TP_POMIAR_FQSPI,	obr_QSPI},
	{"Trans RAM",	"Pomiar predkosci SRAM i DRAM 16-bit",		TP_POMIAR_SRAM,		obr_RAM},
	{"EmuMagCAN",	"Emulator magnetometru na magistrali CAN",	TP_EMU_MAG_CAN,		obr_kal_mag_n1},
	{"Kostka 3D", 	"Rysuje kostke 3D w funkcji katow IMU",		TP_IMU_KOSTKA,		obr_kostka3D},
	{"Magistrala",	"Test magistrali danych i adresowej",		TP_W3,				obr_Wydajnosc},
	{"Pamiec",		"Status pamieci",							TP_STAN_PAMIECI,	obr_Wydajnosc},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

struct tmenu stMenuAudio[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Miki Rej",  	"Deklaracja Mikolaja Reja o APL",			TP_MREJ,	 		obr_Mikołaj_Rey},
	{"Papuga",		"Rejestrator i odtwarzacz dzwieku",			TP_PAPUGA,			obr_papuga1},
	{"Miki DRAM",	"Test wymowy z DRAM",						TP_MM2,				obr_glosnik2},
	{"Test Tonu",	"Test tonu wario",							TP_TEST_TONU,		obr_glosnik2},
	{"FFT Audio",	"FFT sygnału z mikrofonu",					TP_AUDIO_FFT,		obr_fft},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"Test kom.",	"Test komunikatow audio",					TP_MM_KOM,			obr_glosnik2},
	{"Startowy",	"Ekran startowy",							TP_WITAJ,			obr_kontrolny},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

struct tmenu stMenuKamera[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Kam. SRAM",	"Kamera w trybie ciaglym z pam. SRAM",		TP_KAMERA,			obr_kamera},
	{"Kam. DRAM",	"Kamera w trybie ciaglym z pam. DRAM",		TP_KAM_DRAM,		obr_kamera},
	{"Jpeg Y8",		"Kompresja obrazu czarno-bialego Y8",		TP_KAM_Y8,			obr_kamera},
	{"Jpeg Y420",	"Kompresja obrazu kolorowego YUV420",		TP_KAM_YUV420,		obr_kamera},
	{"RTSP Y422",	"Kompresja YUV42 dla serwera RTSP",			TP_RTSP_YUV422,		obr_kamera},
	{"Zdj Y8",		"Wykonaj zdjecie jpg Y8",					TP_KAM_ZDJ_Y8,		obr_aparat},
	{"Zdj YUV420",	"Wykonaj zdjecie jpg YUV420",				TP_KAM_ZDJ_YUV420,	obr_aparat},
	{"Zdj YUV422",	"Wykonaj zdjecie jpg YUV422",				TP_KAM_ZDJ_YUV422,	obr_aparat},
	{"Status kam",	"Wykonuje diagnostyke kamery",				TP_KAM_DIAG,		obr_narzedzia},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

struct tmenu stMenuOsd[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Obraz 480", 	"Pokazuje obraz OSD 480x320",				TPO_TEST_OSD480,	obr_kamera},
	{"Jpeg 444",	"Zapisz plik JPEG w formacie 444",			TPO_JPEG444,		obr_kartaSD},
	{"Jpeg 422",	"Zapisz plik JPEG w formacie 422",			TPO_JPEG422,		obr_kartaSD},
	{"Jpeg 420", 	"Zapisz plik JPEG w formacie 420",			TPO_JPEG420,		obr_kartaSD},
	{"Jpeg Y8",		"Zapisz plik JPEG w formacie Y8",			TPO_JPEG_Y8,		obr_kartaSD},
	{"OSD 480",		"OSD 480x320",								TPO_OSD480,			obr_kamera},
	{"OSD 320",		"OSD 320x240",								TPO_OSD320,			obr_kamera},
	{"OSD 240",		"OSD 240x160",								TPO_OSD240,			obr_kamera},
	{"BMP Luma",	"Zapisz luminancji do pliku BMP",			TPO_ZAPIS_BMP,		obr_kartaSD},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

struct tmenu stMenuKartaSD[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Wlacz Rej",   "Wlacza rejestrator",   					TPKS_WLACZ_REJ,		obr_kartaSD},
	{"Wylacz Rej",  "Wylacza rejestrator",   					TPKS_WYLACZ_REJ,	obr_kartaSD},
	{"Parametry",	"Parametry karty SD",						TPKS_PARAMETRY,		obr_kartaSD},
	{"ZapisBin24",	"Zapis bufora LCD 480x320 na karte  ",		TPKS_ZAPISZ_BIN,	obr_bmp8},
	{"ZapisBMP8",	"Zapis bufora LCD 480x320 na karte  ",		TPKS_ZAPISZ_BMP8,	obr_bmp8},
	{"Test trans",	"Wyniki pomiaru predkosci transferu",		TPKS_POMIAR, 		obr_multimetr},
	{"Bmp24 240",	"Zapis bufora LCD 480x320 na karte  ",		TPKS_ZAP_BMP24_240,	obr_bmp24},
	{"Bmp24 320",	"Zapis bufora LCD 320x240 na karte  ",		TPKS_ZAP_BMP24_320,	obr_bmp24},
	{"BMP24 480",	"Zapis bufora LCD 240x180 na karte  ",		TPKS_ZAP_BMP24_480,	obr_bmp24},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

struct tmenu stMenuEthernet[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Info",		"Informacje o laczu sieciowym",				TP_ETH_INFO,		obr_Polaczenie},
	{"Gadu-Gadu",	"Test serwera TCP jako Gadu-Gadu",			TP_ETH_GADU_GADU,	obr_Polaczenie},
	{"nic",			"nic",										ET_ETH1,			obr_Polaczenie},
	{"nic",			"nic",										ET_ETH1,			obr_Polaczenie},
	{"nic",			"nic",										ET_ETH1,			obr_Polaczenie},
	{"nic",			"nic",										ET_ETH1,			obr_Polaczenie},
	{"nic",			"nic",										ET_ETH1,			obr_Polaczenie},
	{"nic",			"nic",										ET_ETH1,			obr_Polaczenie},
	{"nic",			"nic",										ET_ETH1,			obr_Polaczenie},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

struct tmenu stMenuTestowe[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"nic",			"nic",										TP_TEST1,			obr_narzedzia},
	{"nic",			"nic",										TP_TEST2,			obr_narzedzia},
	{"nic",			"nic",										TP_TEST3,			obr_narzedzia},
	{"nic",			"nic",										TP_TEST4,			obr_narzedzia},
	{"Zap. DRAM",	"nic",										TP_TEST5,			obr_narzedzia},
	{"nic",			"nic",										TP_TEST6,			obr_narzedzia},
	{"nic",			"nic",										TP_TEST7,			obr_narzedzia},
	{"nic",			"nic",										TP_TEST8,			obr_narzedzia},
	{"nic",			"nic",										TP_TEST9,			obr_narzedzia},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

struct tmenu stMenuIMU[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Zyro Zimno", 	"Kalibracja zera zyroskopow w 10C",			TP_KAL_ZYRO_ZIM,	obr_zyroskop},
	{"Zyro Pokoj", 	"Kalibracja zera zyroskopow w 25C",			TP_KAL_ZYRO_POK,	obr_zyroskop},
	{"Zyro Gorac", 	"Kalibracja zera zyroskopow w 40C",			TP_KAL_ZYRO_GOR,	obr_zyroskop},
	{"Akcel 2D",	"Kalibracja akcelerometrow 2D",				TP_KAL_AKCEL_2D,	obr_kontrolny},
	{"Akcel 3D",	"Kalibracja akcelerometrow 3D",				TP_KAL_AKCEL_3D,	obr_kontrolny},
	{"Wzm ZyroP", 	"Kalibracja wzmocnienia zyroskopow P",		TP_KAL_WZM_ZYROP,	obr_zyroskop},
	{"Wzm ZyroQ", 	"Kalibracja wzmocnienia zyroskopow Q",		TP_KAL_WZM_ZYROQ,	obr_zyroskop},
	{"Wzm ZyroR", 	"Kalibracja wzmocnienia zyroskopow R",		TP_KAL_WZM_ZYROR,	obr_zyroskop},
	{"ZeroZyro", 	"Kasuj dryft scalkowanego kata z zyro.",	TP_KASUJ_DRYFT_ZYRO,obr_kostka3D},
	//{"Kostka 3D", 	"Rysuje kostke 3D w funkcji katow IMU",		TP_IMU_KOSTKA,	obr_kostka3D},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

struct tmenu stMenuMagnetometr[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Kal Magn1", 	"Kalibracja magnetometru 1",				TP_MAG_KAL1,		obr_kal_mag_n1},
	{"Kal Magn2", 	"Kalibracja magnetometru 2",				TP_MAG_KAL2,		obr_kal_mag_n1},
	{"Kal Magn3", 	"Kalibracja magnetometru 3",				TP_MAG_KAL3,		obr_kal_mag_n1},
	{"MAG1",		"nic  ",									TP_MAG1,			obr_dotyk_zolty},
	{"MAG2",		"nic  ",									TP_MAG2,			obr_dotyk_zolty},
	{"Spr Magn1",	"Sprawdz kalibracje magnetometru 1",		TP_SPR_MAG1,		obr_kal_mag_n1},
	{"Spr Magn2",	"Sprawdz kalibracje magnetometru 2",		TP_SPR_MAG2,		obr_kal_mag_n1},
	{"Spr Magn3",	"Sprawdz kalibracje magnetometru 3",		TP_SPR_MAG3,		obr_kal_mag_n1},
	{"Spr Mag 2D",	"Sprawdz płaski obrotu dla 3 magnetom.",	TP_SPR_PLASKI,		obr_okregi},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran główny odświeżany w głównej pętli programu
// Parametry:
// Zwraca: nic
// Czas rysowania pełnego ekranu: 174ms
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujEkran(void)
{
	if ((statusDotyku.chFlagi & DOTYK_SKALIBROWANY) != DOTYK_SKALIBROWANY)		//sprawdź czy ekran dotykowy jest skalibrowany
		chTrybPracy = TP_KAL_DOTYK;

	if (statusDotyku.chFlagi & DOTYK_ODCZYTAC)	//przy najbliższej okazji trzeba odczytać dotyk
		chErr = CzytajDotyk();

	switch (chTrybPracy)
	{
	case TP_MENU_GLOWNE:	// wyświetla menu główne
		//Menu((char*)chNapisLcd[STR_MENU_MAIN], stMenuGlowne, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_MENU_MAIN]);
		Menu(chNapisPodreczny, stMenuGlowne, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_KAL_BARO:	//kalibracja ciśnienia według wzorca
		if (KalibrujBaro(&chSekwencerKalibracji) == ERR_GOTOWE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MENU;
		}
		break;


	case TP_POMIARY_DOTYKU:
		if (TestDotyku() == ERR_GOTOWE)
			chNowyTrybPracy = TP_WROC_DO_MENU;
		break;

	//*** Menu Testy ************************************************
	case TP_TESTY:
		//Menu(strcat((char*)chNapisLcd[STR_TESTY], (char*)chNapisLcd[STR_MENU]), stMenuTestowe, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_TESTY]);
		Menu(chNapisPodreczny, stMenuTestowe, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;



	case TP_TEST1:
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_TESTY;
		}
		break;

	case TP_TEST2:
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_TESTY;
		}
		break;

	case TP_TEST3:
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_TESTY;
		}
		break;

	case TP_TEST4:
	case TP_TEST5:
		for (uint8_t n=0; n<100; n++)
			chBuforTestowyDRAM[n] = n;
		chTrybPracy = chWrocDoTrybu;
		chNowyTrybPracy = TP_WROC_DO_TESTY;
		break;

	case TP_TEST6:
	case TP_TEST7:
	case TP_TEST8:
	case TP_TEST9:





	//*** Audio ************************************************
	case TP_MEDIA_AUDIO:
		//Menu(strcat((char*)chNapisLcd[STR_MENU_MEDIA_AUDIO], (char*)chNapisLcd[STR_MENU]), stMenuAudio, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_AUDIO]);
		Menu(chNapisPodreczny, stMenuAudio, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_MREJ:
		InicjujOdtwarzanieDzwieku();
		OdtworzProbkeAudioZeSpisu(PRGA_NIECHAJ_NARODO);
		chNowyTrybPracy = TP_WROC_DO_AUDIO;
		break;


	case TP_PAPUGA:
		extern int16_t sBuforAudioWe[2][2*ROZMIAR_BUFORA_AUDIO_WE];	//bufor komunikatów przychodzących
		uint16_t sLiczbaBuforowNagrania;
		uint16_t y1, y2, sMin = 0x4FFF;
		InicjujRejestracjeDzwieku();

		//czekaj na stabilizcję napięcia na mikrofonie
		for (uint8_t n=0; n<5; n++)
		{
			RysujProstokatWypelniony(n*DISP_Y_SIZE/20, DISP_Y_SIZE - WYS_PASKA_POSTEPU, (n+1)*DISP_Y_SIZE/5, WYS_PASKA_POSTEPU, NIEBIESKI);
			NapelnijBuforDzwieku(sBuforPapuga, DISP_X_SIZE);
			sprintf(chNapis, "Inicjalizacja: %d/%d", n, 5);
			RysujNapis(chNapis, 10, 5);
			RysujPrzebieg(0, sBuforPapuga, SZARY80);
		}
		//znajdź minimum z ostatniej próbki rozbiegowej
		for (int16_t x=0; x<DISP_X_SIZE; x++)
		{
			if (sBuforPapuga[x] < sMin)
				sMin = sBuforPapuga[x];
		}
		WypelnijEkran(CZARNY);

		setColor(ZOLTY);
		for (uint32_t n=0; n<ROZMIAR_BUFORA_PAPUGI; n++)
			sBuforPapuga[n] = 0;	//czyść bufor przed nagraniem

		//narysuj przebieg na ekranie
		y1 = DISP_Y_SIZE / 2;
		sLiczbaBuforowNagrania = ROZMIAR_BUFORA_PAPUGI / 64;
		for (uint16_t n=0; n<sLiczbaBuforowNagrania; n++)
		{
			NapelnijBuforDzwieku((sBuforPapuga + n*64), 64);
			sprintf(chNapis, "Rejestracja: %d/%d", n, sLiczbaBuforowNagrania);
			RysujNapis(chNapis, 10, 20);
			RysujProstokatWypelniony(n*DISP_Y_SIZE/sLiczbaBuforowNagrania, DISP_Y_SIZE - WYS_PASKA_POSTEPU, (n+1)*DISP_Y_SIZE/sLiczbaBuforowNagrania, WYS_PASKA_POSTEPU, NIEBIESKI);

			y2 = sBuforPapuga[n*64] - sMin;
			if (y2 > DISP_Y_SIZE / 2)	//ogranicz duże wartosci aby rysując nie mazało po pamieci ekranu
				y2 =  DISP_Y_SIZE / 2;
			if (y2 < -DISP_Y_SIZE/2)
				y2 = -DISP_Y_SIZE / 2;

			y2 += DISP_Y_SIZE / 2;	//przesuń na środek
			RysujLinie(2*n, y1, 2*n+1, y2);
			y1 = y2;
		}
		RysujProstokatWypelniony(0, DISP_Y_SIZE - WYS_PASKA_POSTEPU, DISP_X_SIZE, WYS_PASKA_POSTEPU, CZARNY);//pasek postępu

		NormalizujDzwiek(sBuforPapuga, ROZMIAR_BUFORA_PAPUGI, 100);	//normalizuj dźwięk do ustalonej gośności
		sprintf(chNapis, "Odtwarzanie      ");
		RysujNapis(chNapis, 10, 20);
		InicjujOdtwarzanieDzwieku();
		OdtworzProbkeAudio((uint32_t)sBuforPapuga, ROZMIAR_BUFORA_PAPUGI * 2);	//*2 bo rozmiar komunikatu jest w bajtach
		chNowyTrybPracy = TP_WROC_DO_AUDIO;
		break;


	case TP_MM2:
		InicjujOdtwarzanieDzwieku();
		PrzepiszProbkeDoDRAM(PRGA_NIECHAJ_NARODO);
		chNowyTrybPracy = TP_WROC_DO_AUDIO;
		break;


	case TP_TEST_TONU:
		InicjujOdtwarzanieDzwieku();
		TestTonuAudio();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			ZatrzymajTon();
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_AUDIO;
		}
		break;


	case TP_AUDIO_FFT:			//FFT sygnału z mikrofonu
		//extern int32_t nBuforAudioWe[ROZMIAR_BUFORA_AUDIO_WE];	//bufor komunikatów przychodzących
		extern int16_t sBuforAudioWe[2][2*ROZMIAR_BUFORA_AUDIO_WE];	//bufor komunikatów przychodzących
		extern uint8_t chWskaznikBuforaAudio;
		uint8_t chWskKasowania;
		InicjujRejestracjeDzwieku();
		do
		{
			chWskKasowania = chWskaznikBuforaAudio;
			chWskaznikBuforaAudio ^= 0x01;
			chWskaznikBuforaAudio &= 0x01;
			NapelnijBuforDzwieku(&sBuforAudioWe[chWskaznikBuforaAudio][0], 2*ROZMIAR_BUFORA_AUDIO_WE);
			RysujPrzebieg(sBuforAudioWe[chWskKasowania], sBuforAudioWe[chWskaznikBuforaAudio], BIALY);
		}
		while ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		chTrybPracy = chWrocDoTrybu;
		chNowyTrybPracy = TP_WROC_DO_AUDIO;
		break;



	case TP_MM_KOM:
		InicjujOdtwarzanieDzwieku();
		TestKomunikatow();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_AUDIO;
		}
		break;


	case TP_WITAJ:
		if (!chLiczIter)
			chLiczIter = 15;			//ustaw czas wyświetlania x 200ms
		Ekran_Powitalny(nZainicjowanoCM7);	//przywitaj użytkownika i prezentuj wykryty sprzęt
		if (!chLiczIter)				//jeżeli koniec odliczania to wyjdź
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_AUDIO;
		}
		break;


	//*** Kamera ************************************************
	case TP_MEDIA_KAMERA:			//menu kamera
		//Menu((char*)chNapisLcd[STR_MENU_KAMERA], stMenuKamera, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_KAMERA]);
		Menu(chNapisPodreczny, stMenuKamera, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;

	/*case TP_ZDJECIE:	//wykonaj pojedyncze zdjęcie i zapisz je w pliku binarnym txt
		extern uint16_t sLicznikLiniiKamery;
		stKonfOSD.chOSDWlaczone = 0;	//nie właczaj OSD
		sLicznikLiniiKamery = 0;
		chErr = ZrobZdjecie(sBuforKamerySRAM, DISP_X_SIZE * DISP_Y_SIZE / 2);
		if (chErr)
		{
			setColor(MAGENTA);
			sprintf(chNapis, "Blad: %d  ", chErr);
		}
		else
		{
			setColor(ZIELONY);
			sprintf(chNapis, "Linii: %d  ", sLicznikLiniiKamery);
			WyswietlZdjecie(480, 320, sBuforKamerySRAM);
		}
		RysujNapis(chNapis, KOL12, 300);
		FRESULT fres = 0;
		extern RTC_HandleTypeDef hrtc;
		extern RTC_TimeTypeDef sTime;
		extern RTC_DateTypeDef sDate;

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		sprintf(chNapis, "Zdjecie%04d%02d%02d_%02d%02d%02d.txt",sDate.Year+2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

		fres = f_open(&SDJpegFile, chNapis, FA_OPEN_ALWAYS | FA_WRITE);
		if (fres == FR_OK)
		{
			f_puts((char*)sBuforKamerySRAM, &SDJpegFile);	//zapis do pliku
			f_close(&SDJpegFile);
		}
		osDelay(600);
		chNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;*/

	case TP_KAMERA:	//ciagła praca kamery z pamięcią SRAM
		stKonfOSD.chOSDWlaczone = 1;	//włacz OSD z histogramem
		stKonfOSD.sSzerokosc = DISP_X_SIZE;
		stKonfOSD.sWysokosc = DISP_Y_SIZE;
		WypelnijEkranwBuforze(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, chBuforOSD, PRZEZR_100);	//czyść poprzednią zawartość
		chErr = UstawObrazKamery(DISP_X_SIZE, DISP_Y_SIZE, OBR_RGB565, KAM_FILM);
		chErr = PolaczBuforOSDzObrazem(chBuforOSD, (uint8_t*)sBuforKamerySRAM, chBuforLCD, DISP_X_SIZE, DISP_Y_SIZE);
		RozpocznijPraceDCMI(&stKonfKam, sBuforKamerySRAM, DISP_X_SIZE * DISP_Y_SIZE / 2);
		do
		{
			nCzas = PobierzCzasT6();
			LiczHistogramRGB565(sBuforKamerySRAM, STD_OBRAZU_DVGA, chHistR, chHistG, chHistB);	//licz histogram
			nCzas = MinalCzas(nCzas);
			nCzasHist = PobierzCzasT6();
			RysujHistogramOSD_RGB32(chBuforOSD, chHistR, chHistG, chHistB);
			nCzasHist = MinalCzas(nCzasHist);
			WyswietlZdjecieRGB666(DISP_X_SIZE, DISP_Y_SIZE, chBuforLCD);
			nCzasLCD = MinalCzas(nCzasLCD);
			sprintf(chNapis, "Tobl: %ld ms, Trys: %ld ms, %ld fps", nCzas / 1000, nCzasHist / 1000, 1000000 / nCzasLCD);
			RysujNapiswBuforze(chNapis, 0, DISP_Y_SIZE - FONT_BH, DISP_X_SIZE, chBuforOSD, (uint8_t*)(KOLOSD_ZOLTY0 + PRZEZR_20), (uint8_t*)PRZEZR_80, ROZMIAR_KOLORU_OSD);
			nCzasLCD = PobierzCzasT6();
		}
		while ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		chNowyTrybPracy = TP_WROC_DO_KAMERA;
		stKonfOSD.chOSDWlaczone = 0;	//wyłącz OSD
		break;

	case TP_KAM_DRAM:	//ciagła praca kamery z pamięcią DRAM
		stKonfOSD.chOSDWlaczone = 0;	//wyłacz OSD z histogramem
		//stKonfOSD.chOSDWlaczone = 1;	//włacz OSD z histogramem
		stKonfOSD.sSzerokosc = DISP_X_SIZE;
		stKonfOSD.sWysokosc = DISP_Y_SIZE;
		chErr = UstawObrazKamery(DISP_X_SIZE, DISP_Y_SIZE, OBR_RGB565, KAM_FILM);
		//chErr = PolaczBuforOSDzObrazem(chBuforOSD, (uint8_t*)sBuforKameryDRAM, chBuforLCD, DISP_X_SIZE, DISP_Y_SIZE);
		RozpocznijPraceDCMI(&stKonfKam, sBuforKameryDRAM, DISP_X_SIZE * DISP_Y_SIZE / 2);
		do
		{
			//LiczHistogramRGB565(sBuforKameryDRAM, STD_OBRAZU_DVGA, chHistR, chHistG, chHistB);
			RysujHistogramOSD_RGB32(chBuforOSD, chHistR, chHistG, chHistB);
			KonwersjaRGB565doRGB666(sBuforKameryDRAM, chBuforLCD, DISP_X_SIZE * DISP_Y_SIZE);
			WyswietlZdjecieRGB666(DISP_X_SIZE, DISP_Y_SIZE, chBuforLCD);

			//RysujHistogramRGB32(chHistR, chHistG, chHistB);
		}
		while ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		chNowyTrybPracy = TP_WROC_DO_KAMERA;
		stKonfOSD.chOSDWlaczone = 0;	//wyłącz OSD
		break;


	case TP_KAM_Y8:	//praca z obrazem czarno-białym
		stKonfOSD.chOSDWlaczone = 0;	//nie właczaj OSD
		//ponieważ bufor obrazu 1280x960 jest 8 razy większy niż 480x320 wiec dzielę go na 8 części i w nich zapisuję kolejne klatki
		chErr = UstawObrazKamery(DISP_X_SIZE, DISP_Y_SIZE, OBR_Y8, KAM_FILM);
		if (chErr)
			break;
		chErr = RozpocznijPraceDCMI(&stKonfKam, sBuforKamerySRAM, DISP_X_SIZE * DISP_Y_SIZE / 4);
		if (chErr)
			break;
		do
		{
			while (!chObrazKameryGotowy)	//synchronizuj się do początku nowej klatki obrazu
				osDelay(1);
			chObrazKameryGotowy = 0;
			chWskNapBufKam++;
			chWskNapBufKam &= MASKA_BUFORA_KAMERY;
			//utwórz wskaźnik na konkretny bufor w obrębie zmiennej sBuforKamerySRAM wskazujący na kolejną 1/8 zmiennej
			//uint16_t* sPodBufor = sBuforKamerySRAM + chWskNapBufKam * (DISP_X_SIZE * DISP_Y_SIZE / (4 * 8));
			//chErr = RozpocznijPraceDCMI(stKonfKam, sPodBufor, DISP_X_SIZE * DISP_Y_SIZE / 4);
			if (chWskNapBufKam & 0x01)		//nieparzyste obrazy kompresuj a parzyste wyświetlaj
			{
				nCzas = PobierzCzasT6();
				//chErr = KompresujY8((uint8_t*)sPodBufor, DISP_X_SIZE, DISP_Y_SIZE);
				chErr = KompresujY8((uint8_t*)sBuforKamerySRAM, DISP_X_SIZE, DISP_Y_SIZE);
				nCzas = MinalCzas(nCzas);
			}
			else
			{
				//KonwersjaCB8doRGB666((uint8_t*)sPodBufor, chBuforLCD, DISP_X_SIZE * DISP_Y_SIZE);
				KonwersjaCB8doRGB666((uint8_t*)sBuforKamerySRAM, chBuforLCD, DISP_X_SIZE * DISP_Y_SIZE);
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
#ifdef 	LCD_ILI9488
				WyswietlZdjecieRGB666(DISP_X_SIZE, DISP_Y_SIZE, chBuforLCD);
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
#endif
				sprintf(chNapis, "Tkompr: %ld us, buf: %d", nCzas, chWskNapBufKam);
				setColor(ZOLTY);
				RysujNapis(chNapis, 0, DISP_Y_SIZE - 2*FONT_BH);
			}
		}
		while ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		chNowyTrybPracy = TP_WROC_DO_KAMERA;
			break;


	case TP_KAM_YUV420:	//kompresja obrazu kolorowego
		chErr = UstawObrazKamery(SZER_ZDJECIA, WYS_ZDJECIA, OBR_YUV420, KAM_ZDJECIE);
		if (chErr)
			break;
		do
		{
			chErr = RozpocznijPraceDCMI(&stKonfKam, sBuforKamerySRAM, DISP_X_SIZE * DISP_Y_SIZE / 2 * 3);
			if (chErr)
				break;
			chErr = CzekajNaKoniecPracyDCMI(DISP_Y_SIZE);
			nCzas = PobierzCzasT6();
			chErr = KompresujYUV420((uint8_t*)sBuforKamerySRAM, DISP_X_SIZE, DISP_Y_SIZE);
			if (chErr)
				break;

			chErr = CzekajNaKoniecPracyJPEG();
			nCzas = MinalCzas(nCzas);
			sprintf(chNapis, "%.2f fps, kompr: %.1f", (float)1000000.0/nCzas, (float)nRozmiarObrazuKamery / nRozmiarObrazuJPEG);	//Sprawdzić: hard fault
			setColor(ZOLTY);
			RysujNapis(chNapis, 0, DISP_Y_SIZE - FONT_BH);
		}
		while ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		chNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;

	case TP_RTSP_YUV422:	//kamera generuje standarowy obraz na ekran, DMA2D nakłada OSD i zamienia na format RGB888, obraz jest kompresowany bez nagłówka JPEG
		stKonfOSD.sSzerokosc = 480;
		stKonfOSD.sWysokosc = 320;
		stKonfOSD.chOSDWlaczone = 1;
		chStatusRejestratora &= ~STATREJ_ZAPISZ_JPG;	//wyłącz rejestrację na karcie
		hjpeg.Instance->CONFR1 &= ~JPEG_CONFR1_HDR;		//wyłącz generowanie nagłówka JPEG
		WypelnijEkranwBuforze(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, chBuforOSD, PRZEZR_100);

		chErr = UstawObrazKamery(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, OBR_RGB565, KAM_FILM);		//kolor
		if (chErr)
			break;

		chErr = RozpocznijPraceDCMI(&stKonfKam, sBuforKamerySRAM, stKonfOSD.sSzerokosc * stKonfOSD.sWysokosc / 2);	//kolor
		if (chErr)
			break;
		do
		{
			RysujOSD(&stKonfOSD, &uDaneCM4.dane);
			RysujBitmape888(0, 0, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, chBuforLCD);	//wyświetla połączone obrazy na LCD
			chErr = KompresujRGB888doYUV422(chBuforLCD, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, 30);
		}
		while ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		chNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;


	case TP_KAM_ZDJ_Y8:	//wykonuje zdjecie Y8 jpg
		stKonfOSD.chOSDWlaczone = 0;	//nie właczaj OSD
		sprintf((char*)chNazwaPlikuObrazu, "ZdjY8");	//początek nazwy pliku ze zdjeciem
		chStatusRejestratora |= STATREJ_ZAPISZ_JPG;	//zapisuj do pliku jpeg

		chErr = UstawObrazKamery(SZER_ZDJECIA, WYS_ZDJECIA, OBR_Y8, KAM_ZDJECIE);
		nCzas = PobierzCzasT6();
		chErr = ZrobZdjecie(sBuforKamerySRAM, SZER_ZDJECIA * WYS_ZDJECIA / 4);
		if (chErr)
		{
			chNowyTrybPracy = TP_WROC_DO_KAMERA;
			break;
		}
		RysujNapis((char*)chOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 300);	//"Wdus ekran i trzymaj aby zakonczyc"
		CzekajNaKoniecPracyDCMI(WYS_ZDJECIA);
		nCzas = MinalCzas(nCzas);
		printf("Tdcmi=%ldus\r\n", nCzas);

		nCzas = PobierzCzasT6();
		chErr = KompresujY8((uint8_t*)sBuforKamerySRAM, SZER_ZDJECIA, WYS_ZDJECIA);	//, chBuforJpeg, ROZMIAR_BUF_JPEG);
		nCzas = MinalCzas(nCzas);
		setColor(ZOLTY);
		if (!chErr)
			printf("Tjpeg=%ldus\r\n", nCzas);
		else
		{
			sprintf(chNapis, "Blad: %d ", chErr);
			RysujNapis(chNapis, 10, 30);
			if ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) == DOTYK_DOTKNIETO)
				chNowyTrybPracy = TP_WROC_DO_KAMERA;
			break;
		}

		sprintf(chNapis, "Czas kompr: %ld us, rozm_obr: %ld, kompr: %.2f ", nCzas, nRozmiarObrazuJPEG, (float)(SZER_ZDJECIA*WYS_ZDJECIA) / nRozmiarObrazuJPEG);
		RysujNapis(chNapis, 0, 30);
		chStatusRejestratora |= STATREJ_ZAPISZ_BMP;	//ustaw flagę zapisu obrazu do pliku bmp
		//jest ustawiony większy rozmiar, więc nie wyswietlaj obrazu
		//KonwersjaCB8doRGB666((uint8_t*)sBuforKamerySRAM, chBufLCD, SZER_ZDJECIA * WYS_ZDJECIA);
		//WyswietlZdjecieRGB666(DISP_X_SIZE, DISP_Y_SIZE, chBuforLCD);
		osDelay(3000);
		chNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;


	case TP_KAM_ZDJ_YUV420:	//wykonuje zdjecie YUV420 jpg
		//cał pamięć SRAM wypełnij wzorcem 0x1111 a następnie bufor wzorcem 0x4444
		uint16_t* sWskSram = (uint16_t*)0x60000000;
		for (uint32_t n=0; n<0x400000; n++)
			*(sWskSram + n) = 0x1111;
		for (uint32_t m=0; m<SZER_ZDJECIA/4; m++)
		{
			for (uint32_t n=0; n<WYS_ZDJECIA; n++)
				sBuforKamerySRAM[n+m*WYS_ZDJECIA] = (m & 0x0FFF) | 0x4000;
		}
		sprintf((char*)chNazwaPlikuObrazu, "ZdjYUV420");	//początek nazwy pliku ze zdjeciem
		chStatusRejestratora |= STATREJ_ZAPISZ_JPG;	//zapisuj do pliku jpeg
		//chErr = UstawObrazKamery(DISP_X_SIZE, DISP_Y_SIZE, OBR_YUV420, KAM_ZDJECIE);
		//chErr = ZrobZdjecie(sBuforKamerySRAM, DISP_X_SIZE * DISP_Y_SIZE / 2);	//wynik w sBuforKamerySRAM
		chErr = UstawObrazKamery(60, 40, OBR_YUV420, KAM_ZDJECIE);
		chErr = ZrobZdjecie(sBuforKamerySRAM, 60 * 40 / 2);	//wynik w sBuforKamerySRAM
		nCzas = PobierzCzasT6();
		//chErr = KompresujYUV420((uint8_t*)sBuforKamerySRAM, DISP_X_SIZE, DISP_Y_SIZE);
		chErr = KompresujYUV420((uint8_t*)sBuforKamerySRAM, 60, 40);
		//KonwersjaCB8doRGB666((uint8_t*)sBuforKamerySRAM, chBufLCD, DISP_X_SIZE * DISP_Y_SIZE);
		nCzas = MinalCzas(nCzas);
		setColor(ZOLTY);
		if (chErr)
		{
			setColor(ZOLTY);
			sprintf(chNapis, "Blad: %d", chErr);
			RysujNapis(chNapis, 10, 30);
			if ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) == DOTYK_DOTKNIETO)
				chNowyTrybPracy = TP_WROC_DO_KAMERA;
			break;
		}

		sprintf(chNapis, "Czas kompr: %ld us, rozm_obr: %ld, kompr: %.2f", nCzas, nRozmiarObrazuJPEG, (float)(DISP_X_SIZE*DISP_Y_SIZE) / nRozmiarObrazuJPEG);
		RysujNapis(chNapis, 0, 30);
		osDelay(3000);
		chNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;


	case TP_KAM_ZDJ_YUV422:	//Analiza obrazu pokazuje że coś jest nie tak z obrazem YUV444. Dla obrazu o szerokości 480 pix powtarza się biała linia 4x16 bajtów co 2*480 pikseli
		sprintf((char*)chNazwaPlikuObrazu, "ZdjYUV444");	//początek nazwy pliku ze zdjeciem
		chErr = UstawObrazKamery(DISP_X_SIZE, DISP_Y_SIZE, OBR_YUV444, KAM_ZDJECIE);
		chErr = ZrobZdjecie(sBuforKamerySRAM, DISP_X_SIZE * DISP_X_SIZE * 2 / 3);	//rozmiar obrazu to 3 bajty na piksel
		nCzas = PobierzCzasT6();
		chErr = KompresujYUV444((uint8_t*)sBuforKamerySRAM, DISP_X_SIZE, DISP_Y_SIZE);
		nCzas = MinalCzas(nCzas);
		if (chErr)
		{
			setColor(ZOLTY);
			sprintf(chNapis, "Blad: %d", chErr);
			RysujNapis(chNapis, 10, 30);
			if ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) == DOTYK_DOTKNIETO)
				chNowyTrybPracy = TP_WROC_DO_KAMERA;
			break;
		}
		chNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;


	case TP_KAM_DIAG:
		if (chRysujRaz)
		{
			BelkaTytulu("Diagnostyka kamery");
			chRysujRaz = 0;
		}

		setColor(BIALY);
		UstawCzcionke(MidFont);
		WykonajDiagnostykeKamery(&stDiagKam);

		sprintf(chNapis, "AVG: 0x%X, Prog AEC: 0x%X..0x%X, stab: 0x%X..0x%X", stDiagKam.chSredniaJasnoscAVG, stDiagKam.chProgAEC_H, stDiagKam.chProgAEC_L, stDiagKam.chProgStabAEC_H, stDiagKam.chProgStabAEC_L);
		RysujNapis(chNapis, 0, 30);

		sprintf(chNapis, "AEC/AGC: 0x%X, VTS: %d", stDiagKam.chTrybEAC_EAG, stDiagKam.sMaxCzasEkspoVTS);
		RysujNapis(chNapis, 0, 50);

		sprintf(chNapis, "okno AVG X: %d..%d, Y: %d..%d", stDiagKam.sPoczatOknaAVG_X, stDiagKam.sKoniecOknaAVG_X, stDiagKam.sPoczatOknaAVG_Y, stDiagKam.sKoniecOknaAVG_Y);
		RysujNapis(chNapis, 0, 70);

		sprintf(chNapis, "Poczatek okna obrazu X: %d, Y: %d", stDiagKam.sPoczatOknaObrazu_X, stDiagKam.sPoczatOknaObrazu_Y);
		RysujNapis(chNapis, 0, 90);

		sprintf(chNapis, "Rozmiar okna obrazu X: %d, Y: %d", stDiagKam.sRozmiarOknaObrazu_X, stDiagKam.sRozmiarOknaObrazu_Y);
		RysujNapis(chNapis, 0, 110);

		sprintf(chNapis, "Rozmiar obrazu X (HTS): %d, Y (VTS): %d", stDiagKam.sRozmiarPoz_HTS, stDiagKam.sRozmiarPio_VTS);
		RysujNapis(chNapis, 0, 130);

		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_KAMERA;
		}
		break;


	//*** OSD ************************************************
	case TP_MENU_OSD:
		//Menu((char*)chNapisLcd[STR_MENU_OSD], stMenuOsd, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_OSD]);
		Menu(chNapisPodreczny, stMenuOsd, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TPO_ZAPIS_BMP:	//zapisz chrominancję lub luminację z bufora LCD do zmiennej: sBuforKamerySRAM a nastepnie do pliku bmp
		ZakonczPraceDCMI();	//wyłącz kamerę aby nie nadpisywała obrazu w sBuforKamerySRAM
		stKonfOSD.chOSDWlaczone = 0;	//nie właczaj OSD
		TestKonwersjiRGB888doYCbCr(chBuforLCD, (uint8_t*)sBuforKamerySRAM, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc);
		sprintf((char*)chNazwaPlikuObrazu, "Luma");	//początek nazwy pliku ze zdjeciem
		chStatusRejestratora = STATREJ_ZAPISZ_BMP;	//ustaw flagę zapisu obrazu do pliku bmp
		stKonfKam.chFormatObrazu = OBR_Y8;			//obraz ma sie zapisać jako monochromatyczny
		chNowyTrybPracy = TP_WROC_DO_OSD;
		break;

	case TPO_OSD240:
	case TPO_OSD320:
	case TPO_OSD480:	//obraz a rzeczywistym tle
		if (chTrybPracy == TPO_OSD240)
		{
			stKonfOSD.sSzerokosc = 240;
			stKonfOSD.sWysokosc = 160;
		}
		else
		if (chTrybPracy == TPO_OSD320)
		{
			stKonfOSD.sSzerokosc = 320;
			stKonfOSD.sWysokosc = 240;
		}
		else
		if (chTrybPracy == TPO_OSD480)
		{
			stKonfOSD.sSzerokosc = 480;
			stKonfOSD.sWysokosc = 320;
		}
		WypelnijEkranwBuforze(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, chBuforOSD, PRZEZR_100);
		chErr = UstawObrazKamery(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, OBR_RGB565, KAM_FILM);		//kolor
		if (chErr)
			break;
		hjpeg.Instance->CONFR1 |= JPEG_CONFR1_HDR;	//włącz generowanie nagłówka JPEG
		stKonfOSD.chOSDWlaczone = 1;
		chErr = RozpocznijPraceDCMI(&stKonfKam, sBuforKamerySRAM, stKonfOSD.sSzerokosc * stKonfOSD.sWysokosc / 2);	//kolor
		if (chErr)
			break;
		do
		{
			/*chTimeout = 60;
			while (!chObrazKameryGotowy && chTimeout)	//synchronizuj się do początku nowej klatki obrazu
			{
				osDelay(1);
				chTimeout--;
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
			}
			//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
			chObrazKameryGotowy = 0;*/
			RysujOSD(&stKonfOSD, &uDaneCM4.dane);

			//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
			nCzasLCD = DWT->CYCCNT;	//start pomiaru
			RysujBitmape888(0, 0, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, chBuforLCD);	//wyświetla połączone obrazy na LCD
			nCzasLCD = DWT->CYCCNT - nCzasLCD; //koniec pomiaru
		}
		while ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		chErr = ZakonczPraceDCMI();
		chNowyTrybPracy = TP_WROC_DO_OSD;
		stKonfOSD.chOSDWlaczone = 0;	//wyłącz OSD
		break;



	case TPO_TEST_OSD480:	//obraz na testowym tle
		stKonfOSD.sSzerokosc = 480;
		stKonfOSD.sWysokosc = 320;
		WypelnijEkranwBuforze(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, (uint8_t*)sBuforKamerySRAM, SZARY20);
		WypelnijEkranwBuforze(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, chBuforOSD, PRZEZR_100);
		do
		{
			RysujOSD(&stKonfOSD, &uDaneCM4.dane);
			nCzasLCD = DWT->CYCCNT;	//start pomiaru
			RysujBitmape888(0, 0, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, chBuforLCD);	//wyświetla połączone obrazy na LCD
			nCzasLCD = DWT->CYCCNT - nCzasLCD; //koniec pomiaru
		}
		while ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
			chNowyTrybPracy = TP_WROC_DO_OSD;
		break;


	case TPO_JPEG420:
		ZakonczPraceDCMI();
		hjpeg.Instance->CONFR1 |= JPEG_CONFR1_HDR;	//włącz generowanie nagłówka JPEG
		for (uint8_t n=1; n<=10; n++)	//wykonaj serię obrazów o różnym stopniu kompresji
		{
			printf("Obraz %d%%: ", n*10);
			chStatusRejestratora |= STATREJ_ZAPISZ_JPG;		//zapisuj do pliku jpeg
			sprintf((char*)chNazwaPlikuObrazu, "YUV420_%d", n*10);	//początek nazwy pliku ze zdjeciem
			chErr = KompresujRGB888doYUV420(chBuforLCD, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, n*10);
			while (chStatusRejestratora & STATREJ_ZAPISZ_JPG)	//czekaj aż się zapisze
			{
				printf("Sync, ");
				osDelay(5);
			}
		}
		chNowyTrybPracy = TP_WROC_DO_OSD;
		break;

	case TPO_JPEG422:		//kompresja jpeg obrazu OSD
		ZakonczPraceDCMI();
		hjpeg.Instance->CONFR1 |= JPEG_CONFR1_HDR;	//włącz generowanie nagłówka JPEG
		for (uint8_t n=1; n<=10; n++)	//wykonaj serię obrazów o różnym stopniu kompresji
		{
			printf("Obraz %d%%: ", n*10);
			chStatusRejestratora |= STATREJ_ZAPISZ_JPG;		//zapisuj do pliku jpeg
			sprintf((char*)chNazwaPlikuObrazu, "YUV422_%d", n*10);	//początek nazwy pliku ze zdjeciem
			chErr = KompresujRGB888doYUV422(chBuforLCD, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, n*10);
			while (chStatusRejestratora & STATREJ_ZAPISZ_JPG)	//czekaj aż się zapisze
			{
				printf("Sync, ");
				osDelay(5);
			}
		}
		chNowyTrybPracy = TP_WROC_DO_OSD;
		break;

	case TPO_JPEG444:
		ZakonczPraceDCMI();
		hjpeg.Instance->CONFR1 |= JPEG_CONFR1_HDR;	//włącz generowanie nagłówka JPEG
		for (uint8_t n=1; n<=10; n++)	//wykonaj serię obrazów o różnym stopniu kompresji
		{
			printf("Obraz %d%%: ", n*10);
			chStatusRejestratora |= STATREJ_ZAPISZ_JPG;		//zapisuj do pliku jpeg
			sprintf((char*)chNazwaPlikuObrazu, "YUV444_%d", n*10);	//początek nazwy pliku ze zdjeciem
			chErr = KompresujRGB888doYUV444(chBuforLCD, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, n*10);
			while (chStatusRejestratora & STATREJ_ZAPISZ_JPG)	//czekaj aż się zapisze
			{
				printf("Sync, ");
				osDelay(5);
			}
		}
		chNowyTrybPracy = TP_WROC_DO_OSD;
		break;

	case TPO_JPEG_Y8:
		ZakonczPraceDCMI();
		hjpeg.Instance->CONFR1 |= JPEG_CONFR1_HDR;	//włącz generowanie nagłówka JPEG
		for (uint8_t n=1; n<=10; n++)	//wykonaj serię obrazów o różnym stopniu kompresji
		{
			printf("Obraz %d%%: ", n*10);
			chStatusRejestratora |= STATREJ_ZAPISZ_JPG;		//zapisuj do pliku jpeg
			sprintf((char*)chNazwaPlikuObrazu, "Y8_%d", n*10);	//początek nazwy pliku ze zdjeciem
			chErr = KompresujRGB888doY8(chBuforLCD, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, n*10);
			while (chStatusRejestratora & STATREJ_ZAPISZ_JPG)	//czekaj aż się zapisze
			{
				printf("Sync, ");
				osDelay(5);
			}
		}
		chNowyTrybPracy = TP_WROC_DO_OSD;
		break;

	//*** Ethernet ************************************************
	case TP_ETHERNET:
		//Menu((char*)chNapisLcd[STR_MENU_ETHERNET], stMenuEthernet, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_ETHERNET]);
		Menu(chNapisPodreczny, stMenuEthernet, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_ETH_INFO:
		extern ip4_addr_t ipaddr;
		extern ip4_addr_t netmask;
		extern ip4_addr_t gw;
		extern struct stats_ lwip_stats;

		if (chRysujRaz)
		{
			BelkaTytulu("Statystyki Internetowe");
			chRysujRaz = 0;
		}

		setColor(BIALY);
		UstawCzcionke(MidFont);
		sprintf(chNapis, "NumerIP: %ld.%ld.%ld.%ld",(ipaddr.addr & 0xFF), (ipaddr.addr & 0xFF00)>>8, (ipaddr.addr & 0xFF0000)>>16, (ipaddr.addr & 0xFF000000)>>24);
		RysujNapis(chNapis, 10, 30);
		sprintf(chNapis, "Maska:   %ld.%ld.%ld.%ld", (netmask.addr & 0xFF), (netmask.addr & 0xFF00)>>8, (netmask.addr & 0xFF0000)>>16, (netmask.addr & 0xFF000000)>>24);
		RysujNapis(chNapis, 10, 50);
		sprintf(chNapis, "GateWay: %ld.%ld.%ld.%ld", (gw.addr & 0xFF), (gw.addr & 0xFF00)>>8, (gw.addr & 0xFF0000)>>16, (gw.addr & 0xFF000000)>>24);
		RysujNapis(chNapis, 10, 70);

		sprintf(chNapis, "LINK:    xmit=%d recv=%d drop=%d err=%d", lwip_stats.link.xmit, lwip_stats.link.recv, lwip_stats.link.drop, lwip_stats.link.err);
		RysujNapis(chNapis, 10, 90);
		sprintf(chNapis, "ARP:     xmit=%d recv=%d drop=%d err=%d", lwip_stats.etharp.xmit, lwip_stats.etharp.recv, lwip_stats.etharp.drop, lwip_stats.etharp.err);
		RysujNapis(chNapis, 10, 110);
		sprintf(chNapis, "UDP:     xmit=%d recv=%d drop=%d err=%d", lwip_stats.udp.xmit, lwip_stats.udp.recv, lwip_stats.udp.drop, lwip_stats.udp.err);
		RysujNapis(chNapis, 10, 130);
		sprintf(chNapis, "TCP:     xmit=%d recv=%d drop=%d err=%d", lwip_stats.tcp.xmit, lwip_stats.tcp.recv, lwip_stats.tcp.drop, lwip_stats.tcp.err);
		RysujNapis(chNapis, 10, 150);
		sprintf(chNapis, "TCP Err: checksum=%d length=%d options=%d protocol=%d", lwip_stats.tcp.chkerr, lwip_stats.tcp.lenerr, lwip_stats.tcp.opterr, lwip_stats.tcp.proterr);
		RysujNapis(chNapis, 10, 170);
		sprintf(chNapis, "MemHeap: avail=%ld used=%ld err=%d illegal=%d", (uint32_t)lwip_stats.mem.avail, (uint32_t)lwip_stats.mem.used, lwip_stats.mem.err, lwip_stats.mem.illegal);
		RysujNapis(chNapis, 10, 190);
		sprintf(chNapis, "MemPoll: avail=%ld used=%ld err=%d illegal=%d", (uint32_t)lwip_stats.memp[1]->avail, (uint32_t)lwip_stats.memp[1]->used, lwip_stats.memp[1]->err, lwip_stats.memp[1]->illegal);
		RysujNapis(chNapis, 10, 210);
		sprintf(chNapis, "SYS Err: semafor=%d mutex=%d mbox=%d", lwip_stats.sys.sem.err, lwip_stats.sys.mutex.err, lwip_stats.sys.mbox.err);
		RysujNapis(chNapis, 10, 230);
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_ETH;
		}
		osDelay(100);	//pozwól pracować innym wątkom
		break;


	case TP_ETH_GADU_GADU:
		if (chRysujRaz)
		{
			BelkaTytulu("Gadu-Gadu port:4000");
			chRysujRaz = 0;
		}

		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_ETH;
		}
		break;


	//*** Wydajność ************************************************
	case TP_WYDAJNOSC:			///menu pomiarów wydajności
		//Menu((char*)chNapisLcd[STR_MENU_WYDAJNOSC], stMenuWydajnosc, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_WYDAJNOSC]);
		Menu(chNapisPodreczny, stMenuWydajnosc, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_FRAKTALE:		FraktalDemo();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_POMIAR_SRAM:	TestPredkosciOdczytuRAM();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_POM_ZAPISU_NOR:		TestPredkosciZapisuNOR();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_POMIAR_FNOR:	TestPredkosciOdczytuNOR();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_POMIAR_FQSPI:	//W25_TestTransferu();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_IMU_KOSTKA:	//rysuj kostkę 3D
		float fKat[3];
		for (uint8_t n=0; n<3; n++)
			fKat[n] = -1 *uDaneCM4.dane.fKatZyro2[n];	//do rysowania przyjmij kąty z przeciwnym znakiem - jest OK
		RysujKostkeObrotu(fKat);
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_IMU_KOSTKA_SYM:
		fSymKatKostki[0] += 5 * DEG2RAD;
		fSymKatKostki[1] += 3 * DEG2RAD;
		//fSymKatKostki[2] += 1 * DEG2RAD;
		for (uint8_t n=0; n<3; n++)
		{
			if (fSymKatKostki[n] > M_PI)
				fSymKatKostki[n] = -M_PI;
		}
		RysujKostkeObrotu(fSymKatKostki);
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	///LOG_SD1_VSEL - H=3,3V L=1,8V
	//case TP_W3:		chPort_exp_wysylany[0] |=  EXP02_LOG_VSELECT;	chTrybPracy = TP_WYDAJNOSC;	break;	//LOG_SD1_VSEL - H=3,3V
	//case TP_W4:		chPort_exp_wysylany[0] &= ~EXP02_LOG_VSELECT;	chTrybPracy = TP_WYDAJNOSC;	break;	//LOG_SD1_VSEL - L=1,8V
	case TP_W3: SprawdzMagistrale(0xC0000000);
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_STAN_PAMIECI:
		setColor(ZOLTY);
		sprintf(chNapis, "Wolna sterta: %d", xPortGetFreeHeapSize());
		RysujNapis(chNapis, KOL12, 40);

		sprintf(chNapis, "Minimalna wolna pamiec: %d", xPortGetMinimumEverFreeHeapSize());
		RysujNapis(chNapis, KOL12, 70);

		/*struct mallinfo mi;
		memset(&mi,0,sizeof(struct mallinfo));
		mi = mallinfo();

		sprintf(chNapis, "mallinfo: Arena %ld, Free %ld, Used %ls", mi.arena, mi.fordblks, mi.uordblks);
		RysujNapis(chNapis, KOL12, 100);*/

		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_EMU_MAG_CAN:
		uDaneCM7.dane.chWykonajPolecenie = POL_KAL_ZERO_MAGN2;	//włącz tryb jak dla kalibracji aby nie uwzględniać w wyniku danych kalibracyjnych
		EmulujMagnetometrWizjerCan((float*)uDaneCM4.dane.fMagne2);
		UstawCzcionke(BigFont);
		setColor(KOLOR_X);
		sprintf(chNapis, "Mag X: %.3f uT ", uDaneCM4.dane.fMagne2[0] * 1e6);
		RysujNapis(chNapis, KOL12, 40);
		setColor(KOLOR_Y);
		sprintf(chNapis, "Mag Y: %.3f uT ", uDaneCM4.dane.fMagne2[1] * 1e6);
		RysujNapis(chNapis, KOL12, 70);
		setColor(KOLOR_Z);
		sprintf(chNapis, "Mag Z: %.3f uT ", uDaneCM4.dane.fMagne2[2] * 1e6);
		RysujNapis(chNapis, KOL12, 100);

		if (chRysujRaz)
		{
			BelkaTytulu("Emulacja magnetometru CAN");
			chRysujRaz = 0;
			setColor(SZARY50);
			RysujNapis((char*)chOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 300);	//"Wdus ekran i trzymaj aby zakonczyc"
		}

		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
			uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//zakończ tryb kalibracyjny
		}
	break;


	//*** Karta SD ************************************************
	case TP_KARTA_SD:			///menu Karta SD
		//Menu((char*)chNapisLcd[STR_MENU_KARTA_SD], stMenuKartaSD, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_KARTA_SD]);
		Menu(chNapisPodreczny, stMenuKartaSD, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TPKS_WLACZ_REJ:
		chStatusRejestratora|= STATREJ_WLACZONY;
		WyswietlRejestratorKartySD();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_KARTA;
		}
		break;


	case TPKS_WYLACZ_REJ:	//najpierw zakmnij plik a potem wyłacz rejestrator
		chStatusRejestratora|= STATREJ_ZAMKNIJ_PLIK | STATREJ_BYL_OTWARTY;
		WyswietlRejestratorKartySD();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_KARTA;
		}
		break;


	case TPKS_PARAMETRY:
		WyswietlParametryKartySD();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_KARTA;
		}
		break;


	case TPKS_POMIAR:
		//TestKartySD();
		for (uint32_t y=0; y<DISP_Y_SIZE; y++)
		{
			for (uint32_t x=0; x<DISP_X_SIZE; x++)
			{
				chBuforLCD[y*DISP_X_SIZE*3 + x*3 + 0] = x & 0xFF;
				chBuforLCD[y*DISP_X_SIZE*3 + x*3 + 1] = y & 0xFF;
				chBuforLCD[y*DISP_X_SIZE*3 + x*3 + 2] = (y >> 8) & 0xFF;;
			}
		}
		chErr = ZapiszPlikBmp(chBuforLCD, BMP_KOLOR_24, DISP_X_SIZE, DISP_Y_SIZE);
		chErr = ZapiszPlikBin(chBuforLCD, DISP_X_SIZE * DISP_Y_SIZE * 3);
		chTrybPracy = chWrocDoTrybu;
		chNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


	case TPKS_ZAPISZ_BIN:	chErr = ZapiszPlikBin(chBuforLCD, DISP_X_SIZE * DISP_Y_SIZE * 3);
		chTrybPracy = chWrocDoTrybu;
		chNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


	case TPKS_ZAPISZ_BMP8:	chErr = ZapiszPlikBmp(chBuforLCD, BMP_KOLOR_8, DISP_X_SIZE, DISP_Y_SIZE);
		chTrybPracy = chWrocDoTrybu;
		chNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


	case TPKS_ZAP_BMP24_480:	chErr = ZapiszPlikBmp(chBuforLCD, BMP_KOLOR_24, DISP_X_SIZE, DISP_Y_SIZE);
		chTrybPracy = chWrocDoTrybu;
		chNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


	case TPKS_ZAP_BMP24_320:	chErr = ZapiszPlikBmp(chBuforLCD, BMP_KOLOR_24, 320, 240);
		chTrybPracy = chWrocDoTrybu;
		chNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


	case TPKS_ZAP_BMP24_240:	chErr = ZapiszPlikBmp(chBuforLCD, BMP_KOLOR_24, 240, 160);
		chTrybPracy = chWrocDoTrybu;
		chNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


//*** IMU ************************************************
	case TP_KAL_IMU:			//menu kalibracji IMU
		//Menu((char*)chNapisLcd[STR_MENU_IMU], stMenuIMU, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_KALIBRACJA], chNapisLcd[STR_IMU]);
		Menu(chNapisPodreczny, stMenuIMU, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_KAL_ZYRO_ZIM:
		uDaneCM7.dane.chWykonajPolecenie = POL_KALIBRUJ_ZYRO_ZIM;	//uruchom kalibrację żyroskopów na zimno 10°C
		fTemperaturaKalibracji = TEMP_KAL_ZIMNO;
		if ((uDaneCM4.dane.sPostepProcesu > 0) && (uDaneCM4.dane.sPostepProcesu < CZAS_KALIBRACJI))
			chTrybPracy = TP_PODGLAD_IMU;	//jeżeli proces kalibracji się zaczął to przejdź do trybu podgladu aby nie zaczynać nowego cyklu po zakończniu obecnego

		if ((uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZA_ZIMNO) || (uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZA_CIEPLO))
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WYSWIETL_BLAD;
		}
		break;


	case TP_KAL_ZYRO_POK:
		uDaneCM7.dane.chWykonajPolecenie = POL_KALIBRUJ_ZYRO_POK;	//uruchom kalibrację żyroskopów w temperaturze pokojowej 25°C
		fTemperaturaKalibracji = TEMP_KAL_POKOJ;
		if ((uDaneCM4.dane.sPostepProcesu > 0) && (uDaneCM4.dane.sPostepProcesu < CZAS_KALIBRACJI))
			chTrybPracy = TP_PODGLAD_IMU;	//jeżeli proces kalibracji się zaczął to przejdź do trybu podgladu aby nie zaczynać nowego cyklu po zakończniu obecnego

		if ((uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZA_ZIMNO) || (uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZA_CIEPLO))
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WYSWIETL_BLAD;
		}
		break;


	case TP_KAL_ZYRO_GOR:
		uDaneCM7.dane.chWykonajPolecenie = POL_KALIBRUJ_ZYRO_GOR;	//uruchom kalibrację żyroskopów na gorąco 40°C
		fTemperaturaKalibracji = TEMP_KAL_GORAC;
		if ((uDaneCM4.dane.sPostepProcesu > 0) && (uDaneCM4.dane.sPostepProcesu < CZAS_KALIBRACJI))
			chTrybPracy = TP_PODGLAD_IMU;	//jeżeli proces kalibracji się zaczął to przejdź do trybu podgladu aby nie zaczynać nowego cyklu po zakończniu obecnego

		if ((uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZA_ZIMNO) || (uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZA_CIEPLO))
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WYSWIETL_BLAD;
		}
		break;


	case TP_PODGLAD_IMU:
		uDaneCM7.dane.chWykonajPolecenie = POL_NIC;		//gdy proces się rozpoczął wyłącz dalsze wysyłanie polecenia kalibracji
		if ((uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZA_ZIMNO) || (uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZA_CIEPLO))
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WYSWIETL_BLAD;
		}
		else
			PomiaryIMU();		//wyświetlaj wyniki pomiarów IMU pobrane z CM4

		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_W3;
			uDaneCM7.dane.chWykonajPolecenie = POL_CZYSC_BLEDY;	//po zakończeniu wczyść zwrócony kod błędu
		}
		break;


	case TP_WYSWIETL_BLAD:
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZA_ZIMNO)
			WyswietlKomunikatBledu(KOMUNIKAT_ZA_ZIMNO, (uDaneCM4.dane.fTemper[TEMP_IMU1] + uDaneCM4.dane.fTemper[TEMP_IMU2])/2, fTemperaturaKalibracji, TEMP_KAL_ODCHYLKA);	//Wyświetl komunikat  o tym że jest za zimno i nominalna temperatura kalibracji to TEMP_KAL_POKOJ z odchyłką TEMP_KAL_ODCHYLKA
		else
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZA_CIEPLO)
			WyswietlKomunikatBledu(KOMUNIKAT_ZA_CIEPLO, (uDaneCM4.dane.fTemper[TEMP_IMU1] + uDaneCM4.dane.fTemper[TEMP_IMU2])/2, fTemperaturaKalibracji, TEMP_KAL_ODCHYLKA);	//Wyświetl komunikat  o tym że jest za ciepło i nominalna temperatura kalibracji to TEMP_KAL_POKOJ z odchyłką TEMP_KAL_ODCHYLKA

		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_KAL_IMU;
			uDaneCM7.dane.chWykonajPolecenie = POL_CZYSC_BLEDY;	//po zakończeniu wczyść zwrócony kod błędu
		}
		break;


	case TP_KASUJ_DRYFT_ZYRO:
		uDaneCM7.dane.chWykonajPolecenie = POL_KASUJ_DRYFT_ZYRO;
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == POL_KASUJ_DRYFT_ZYRO)
		{
			uDaneCM7.dane.chWykonajPolecenie = POL_NIC;
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_KAL_IMU;
		}
		break;


	case TP_KAL_WZM_ZYROR:
		chSekwencerKalibracji = SEKW_KAL_WZM_ZYRO_R;
		if (KalibracjaWzmocnieniaZyroskopow(&chSekwencerKalibracji) == ERR_GOTOWE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_KAL_IMU;
		}
		break;


	case TP_KAL_WZM_ZYROQ:
		chSekwencerKalibracji = SEKW_KAL_WZM_ZYRO_Q;
		if (KalibracjaWzmocnieniaZyroskopow(&chSekwencerKalibracji) == ERR_GOTOWE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_KAL_IMU;
		}
		break;


	case TP_KAL_WZM_ZYROP:
		chSekwencerKalibracji = SEKW_KAL_WZM_ZYRO_P;
		if (KalibracjaWzmocnieniaZyroskopow(&chSekwencerKalibracji) == ERR_GOTOWE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_KAL_IMU;
		}
		break;


	case TP_KAL_AKCEL_2D:
	case TP_KAL_AKCEL_3D:

	case TP_KAL_HARD_FAULT:
			printf("Start\n");
			//ITM_Sendchar('T');
			void (*fp)(void) = (void (*)(void))(0x00000000);	//Generuje HardFault
			fp();
		break;


//*** Magnetometr ************************************************
	case TP_MAGNETOMETR:	//menu obsługi magnetometru
		//Menu((char*)chNapisLcd[STR_MENU_MAGNETOMETR], stMenuMagnetometr, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_MAGNETOMETR]);
		Menu(chNapisPodreczny, stMenuMagnetometr, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		chSekwencerKalibracji = 0;
		break;


	case TP_MAG_KAL1:
		chSekwencerKalibracji |= MAG1 + KALIBRUJ;
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_GOTOWE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_MAG_KAL2:
		chSekwencerKalibracji |= MAG2 + KALIBRUJ;
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_GOTOWE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_MAG_KAL3:
		chSekwencerKalibracji |= MAG3 + KALIBRUJ;
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_GOTOWE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_MAG1:	break;
	case TP_MAG2:	break;
	case TP_SPR_PLASKI:	PlaskiObrotMagnetometrow();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_SPR_MAG1:
		chSekwencerKalibracji |= MAG1;	//sprawdzenie, bez kalibracji
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_GOTOWE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_SPR_MAG2:
		chSekwencerKalibracji |= MAG2;	//sprawdzenie, bez kalibracji
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_GOTOWE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_SPR_MAG3:
		chSekwencerKalibracji |= MAG3;	//sprawdzenie, bez kalibracji
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_GOTOWE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


//*** Kalibracje ************************************************
	case TP_KALIBRACJE:		//menu skupiające różne kalibracje
		//Menu((char*)chNapisLcd[STR_MENU_KALIBRACJE], stMenuKalibracje, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_KALIBRACJE]);
		Menu(chNapisPodreczny, stMenuKalibracje, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_KAL_DOTYK:
		chNowyTrybPracy = 0;			//nowy tryb jest ustawiany po starcie dla normalnej pracy ze skalibrowanym panelem. Jednak w przypadku potrzeby kalobracji,
										//powoduje to wyczyszczenie opisu i pierwszego krzyżyka i nie wiadomo o co chodzi, więc kasuję chNowyTrybPracy
		if (KalibrujDotyk() == ERR_GOTOWE)
			chTrybPracy = TP_POMIARY_DOTYKU;
		break;


//*** Pomiary ************************************************
	case TP_POMIARY:		//menu skupiające różne kalibracje
		//Menu((char*)chNapisLcd[STR_MENU_POMIARY], stMenuPomiary, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_POMIARY]);
		Menu(chNapisPodreczny, stMenuPomiary, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_POMIARY_IMU:	PomiaryIMU();		//wyświetlaj wyniki pomiarów IMU pobrane z CM4
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chNowyTrybPracy = TP_WROC_DO_POMIARY;
			ZatrzymajTon();
		}
		break;


	case TP_POMIARY_CISN:	PomiaryCisnieniowe();

		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_POMIARY;
		}
		break;


	case TP_POMIARY_RC:	DaneOdbiornikaRC();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_POMIARY;
		}
		break;


//*** Nastawy ************************************************
	case TP_NASTAWY:		//menu skupiające różne kalibracje
		//Menu((char*)chNapisLcd[STR_MENU_NASTAWY], stMenuNastawy, &chNowyTrybPracy);
		sprintf(chNapisPodreczny, "%s %s", chNapisLcd[STR_MENU], chNapisLcd[STR_NASTAWY]);
		Menu(chNapisPodreczny, stMenuNastawy, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_NAST_PID_PRZECH:		//regulator sterowania przechyleniem (lotkami w samolocie)
		NastawyPID(PID_PRZE);
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_PID_POCH:	//regulator sterowania pochyleniem (sterem wysokości)
		NastawyPID(PID_POCH);
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_PID_ODCH:		//regulator sterowania odchyleniem (sterem kierunku)
		NastawyPID(PID_ODCH);
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_PID_WYSOK:		//regulator sterowania wysokością
		NastawyPID(PID_WYSO);
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_PID_NAW_N:		//regulator sterowania nawigacją w kierunku północnym
		NastawyPID(PID_NAW_N);
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_PID_NAW_E:		//regulator sterowania nawigacją w kierunku wschodnim
		NastawyPID(PID_NAW_E);
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_MIKSERA:		break;
	}


	//rzeczy do wykonania podczas uruchamiania nowego trybu pracy
	if (chNowyTrybPracy)
	{
		chWrocDoTrybu = chTrybPracy;
		chTrybPracy = chNowyTrybPracy;
		chNowyTrybPracy = 0;
		statusDotyku.chFlagi &= ~(DOTYK_DOTKNIETO | DOTYK_ZWOLNONO);	//czyść flagi ekranu dotykowego aby móc reagować na nie w trakcie pracy danego trybu
		chRysujRaz = 1;

		//startuje procesy zwiazane z obsługą nowego trybu pracy
		switch(chTrybPracy)
		{
		case TP_WROC_DO_MENU:		chTrybPracy = TP_MENU_GLOWNE;	break;	//powrót do menu głównego
		case TP_WROC_DO_AUDIO:		chTrybPracy = TP_MEDIA_AUDIO;	break;	//powrót do menu Audio
		case TP_WROC_DO_KAMERA:		chTrybPracy = TP_MEDIA_KAMERA;	break;	//powrót do menu Kamera
		case TP_WROC_DO_OSD:		chTrybPracy = TP_MENU_OSD;		break;	//powrót do menu OSD
		case TP_WROC_DO_WYDAJN:		chTrybPracy = TP_WYDAJNOSC;		break;	//powrót do menu Wydajność
		case TP_WROC_DO_KARTA:		chTrybPracy = TP_KARTA_SD;		break;	//powrót do menu Karta SD
		case TP_WROC_KAL_IMU:		chTrybPracy = TP_KAL_IMU;		break;	//powrót do menu IMU
		case TP_WROC_DO_MAG:		chTrybPracy = TP_MAGNETOMETR;	break;	//powrót do menu Magnetometr
		case TP_WROC_DO_POMIARY:	chTrybPracy = TP_POMIARY;		break;	//powrót do menu Pomiary
		case TP_WROC_DO_NASTAWY:	chTrybPracy = TP_NASTAWY;		break;	//powrót do menu Nastawy
		case TP_FRAKTALE:			InitFraktal(0);		break;
		case TP_WROC_DO_ETH:		chTrybPracy = TP_ETHERNET;		break;	//powrót do menu Ethernet
		case TP_WROC_DO_TESTY:		chTrybPracy = TP_TESTY;			break;	//powrót do menu Testy
		}

		WypelnijEkran(CZARNY);
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran inicjalizacji sprzętu i wyświetla numer wersji
// Parametry: zainicjowano - wskaźnik na tablicę bitów z flagami zainicjowanego sprzętu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void Ekran_Powitalny(uint32_t nZainicjowano)
{
	uint8_t n;
	uint16_t x, y;

	extern const unsigned short plogo165x80[];
	extern uint8_t BSP_SD_IsDetected(void);
	extern stBSP_t stBSP;	//struktura zawierajaca adres i nazwę BSP

	if (chRysujRaz)
	{
		WypelnijEkran(BIALY);
		RysujBitmape((DISP_X_SIZE-165)/2, 5, 165, 80, plogo165x80);

		setColor(SZARY20);
		setBackColor(BIALY);
		UstawCzcionke(BigFont);
		sprintf(chNapis, "%s @%luMHz", (char*)chNapisLcd[STR_WITAJ_TYTUL], HAL_RCC_GetSysClockFreq()/1000000);
		RysujNapis(chNapis, CENTER, 90);

		setColor(SZARY30);
		UstawCzcionke(MidFont);
		//sprintf(chNapis, (char*)chNapisLcd[STR_WITAJ_MOTTO], ó, ć, ó, ó, ż);	//"By móc mieć w rój Wronów na pohybel wrażym hordom""
		sprintf(chNapis, (char*)chNapisLcd[STR_WITAJ_MOTTO2], ó, ó, ż, ó);	//"By móc zmóc wraże hordy rojem Wronów"	//STR_WITAJ_MOTTO2
		RysujNapis(chNapis, CENTER, 115);

		sprintf(chNapis, "Adres: %d, IP: %d.%d.%d.%d, Nazwa: %s", stBSP.chAdres, stBSP.chAdrIP[0], stBSP.chAdrIP[1], stBSP.chAdrIP[2], stBSP.chAdrIP[3],  stBSP.chNazwa);
		RysujNapis(chNapis, CENTER, 135);

		sprintf(chNapis, "(c) PitLab 2025 sv%d.%d.%d @ %s %s", WER_GLOWNA, WER_PODRZ, WER_REPO, build_date, build_time);
		RysujNapis(chNapis, CENTER, 155);
		chRysujRaz = 0;
	}

	//pierwsza kolumna sprzętu wewnętrznego
	x = KOL12;
	y = WYKRYJ_GORA;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_FLASH_NOR]);		//"pamięć Flash NOR"
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, (nZainicjowano & INIT_FLASH_NOR) == INIT_FLASH_NOR);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_FLASH_QSPI]);	//"pamięć Flash QSPI"
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, (nZainicjowano & INIT_FLASH_QSPI) == INIT_FLASH_QSPI);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_KARTA_SD]);		//"Karta SD"
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, BSP_SD_IsDetected());

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_KAMERA_OV5642]);	//"kamera "
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, (nZainicjowano & INIT_KAMERA) == INIT_KAMERA);

	//dane z CM4
	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_GNSS]);		//GNSS
	if (uDaneCM4.dane.nZainicjowano & INIT_WYKR_UBLOX)
		n = sprintf(chNapis, "%s -> %s", (char*)chNapisLcd[STR_SPRAWDZ_GNSS], (char*)chNapisLcd[STR_SPRAWDZ_UBLOX]);	//GNSS -> uBlox
	if (uDaneCM4.dane.nZainicjowano & INIT_WYKR_MTK)
		n = sprintf(chNapis, "%s -> %s", (char*)chNapisLcd[STR_SPRAWDZ_GNSS], (char*)chNapisLcd[STR_SPRAWDZ_MTK]);		//GNSS -> MTK
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n,  (uDaneCM4.dane.nZainicjowano & INIT_GNSS_GOTOWY) == INIT_GNSS_GOTOWY);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, "%s -> %s", (char*)chNapisLcd[STR_SPRAWDZ_GNSS], (char*)chNapisLcd[STR_SPRAWDZ_HMC5883]);	//GNSS -> HMC5883
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_HMC5883) == INIT_HMC5883);

	//druga kolumna sprzętu zewnętrznego
	x = KOL22;
	y = WYKRYJ_GORA;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_MS5611]);		//moduł IMU -> MS5611
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_MS5611) == INIT_MS5611);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_BMP581]);		//moduł IMU -> BMP581
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_BMP581) == INIT_BMP581);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_ICM42688]);		//moduł IMU -> ICM42688
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_ICM42688) == INIT_ICM42688);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_LSM6DSV]);		//moduł IMU -> LSM6DSV
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV) == INIT_LSM6DSV);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_MMC34160]);		//moduł IMU -> MMC34160
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_MMC34160) == INIT_MMC34160);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_IIS2MDC]);		//moduł IMU -> IIS2MDC
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC) == INIT_IIS2MDC);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_ND130]);		//moduł IMU -> ND130
	RysujNapis(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_ND130) == INIT_ND130);

	osDelay(200);
	chLiczIter--;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje animację wykrywania zasobu testera
// Parametry: x,y - współrzędne początku tekstu
// znakow - liczba znaków napisu
// wykryto - zasób wykryty lub nie wykryty
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void Wykrycie(uint16_t x, uint16_t y, uint8_t znakow, uint8_t wykryto)
{
	uint8_t n, kropek, szer_fontu;

	szer_fontu = GetFontX();
	x += znakow * szer_fontu;	//współrzędne końca tekstu i początku kropek
	kropek = MAX_NAPISU_WYKRYCIE - znakow - 2;	//liczba kropek dopełnienia
	for (n=0; n<kropek; n++)
		RysujZnak('.', x+n*szer_fontu, y);
	if (wykryto)
	{
		setColor(TEAL);
		sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_WYKR]);	//"wykryto"
	}
	else
	{
		setColor(KOLOR_X);	//czerwony
		sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_BRAK]);	//"brakuje"
	}
	x += kropek * szer_fontu;
	RysujNapis(chNapis, x , y);
	setColor(SZARY30);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z treścią kodu błędu
// Parametry:
// chKomunikatBledu - treść wyświetlanego komunikato o błędzie
// fParametr1, fParametr2 - opcjonalne parametry komunikatu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WyswietlKomunikatBledu(uint8_t chKomunikatBledu, float fParametr1, float fParametr2, float fParametr3)
{
	if (chRysujRaz)
	{
		WypelnijEkran(CZARNY);
		chRysujRaz = 0;
	}
//		setBackColor(CZARNY);

	//nagłówek komunikatu
	setColor(CZERWONY);
	UstawCzcionke(BigFont);
	sprintf(chNapis, (char*)chOpisBledow[KOMUNIKAT_NAGLOWEK]);	//"Blad wykonania polecenia!",
	RysujNapis(chNapis, CENTER, 70);

	//stopka komunikatu
	UstawCzcionke(MidFont);
	setColor(SZARY50);
	RysujNapis((char*)chOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 250);	//"Wdus ekran i trzymaj aby zakonczyc"

	//treść komunikatu
	setColor(ZOLTY);
	switch(chKomunikatBledu)
	{
	case KOMUNIKAT_ZA_ZIMNO:	//sposób formatowania komunikatu taki sam jak dla BLAD_ZA_CIEPLO
	case KOMUNIKAT_ZA_CIEPLO:	//parametr1 to bieżąca temperatura, parametr 2 to nominalna temperatura kalibracji, parametr 3 to zakres tolerancji odchyłki temperatury
		sprintf(chNapis, (const char*)chOpisBledow[chKomunikatBledu], fParametr1 - KELVIN, ZNAK_STOPIEN, fParametr2 - fParametr3 - KELVIN, ZNAK_STOPIEN, fParametr2 + fParametr3 -  KELVIN, ZNAK_STOPIEN);	break;	//"Zbyt niska temeratura zyroskopow wynoszaca %d%cC. Musi miescic sie w granicach od %d%cC do %d%cC",
	}
	RysujNapiswRamce(chNapis, 20, 90, 440, 200);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z damymi pomiarowymi IMU
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PomiaryIMU(void)
{
	int8_t chTon;
	float fDlugosc;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Dane pomiarowe AHRS");

		setColor(SZARY80);
		sprintf(chNapis, "Akcel1:");
		RysujNapis(chNapis, KOL12, 30);
		sprintf(chNapis, "[m/s^2]");
		RysujNapis(chNapis, KOL12+40*FONT_SL, 30);

		sprintf(chNapis, "Akcel2:");
		RysujNapis(chNapis, KOL12, 50);
		sprintf(chNapis, "[m/s^2]");
		RysujNapis(chNapis, KOL12+40*FONT_SL, 50);

		sprintf(chNapis, "Zyro 1:");
		RysujNapis(chNapis, KOL12, 70);
		sprintf(chNapis, "[rad/s]");
		RysujNapis(chNapis, KOL12+40*FONT_SL, 70);

		sprintf(chNapis, "Zyro 2:");
		RysujNapis(chNapis, KOL12, 90);
		sprintf(chNapis, "[rad/s]");
		RysujNapis(chNapis, KOL12+40*FONT_SL, 90);

		sprintf(chNapis, "Magn 1:");
		RysujNapis(chNapis, KOL12, 110);
		sprintf(chNapis, "[uT]");
		RysujNapis(chNapis, KOL12+40*FONT_SL, 110);

		sprintf(chNapis, "Magn 2:");
		RysujNapis(chNapis, KOL12, 130);
		sprintf(chNapis, "[uT]");
		RysujNapis(chNapis, KOL12+40*FONT_SL, 130);

		sprintf(chNapis, "Magn 3:");
		RysujNapis(chNapis, KOL12, 150);
		sprintf(chNapis, "[uT]");
		RysujNapis(chNapis, KOL12+40*FONT_SL, 150);

		sprintf(chNapis, "K%cty 1:", ą);
		RysujNapis(chNapis, KOL12, 170);
		sprintf(chNapis, "K%cty 2:", ą);
		RysujNapis(chNapis, KOL12, 190);
		sprintf(chNapis, "K%cty Akcel1:", ą);
		RysujNapis(chNapis, KOL12, 210);
		sprintf(chNapis, "K%cty Akcel2:", ą);
		RysujNapis(chNapis, KOL12, 230);
		//sprintf(chNapis, "K%cty %cyro 1:", ą, ż);
		sprintf(chNapis, "Kwaternion Akc:");
		RysujNapis(chNapis, KOL12, 250);
		//sprintf(chNapis, "K%cty %cyro 2:", ą, ż);
		sprintf(chNapis, "Kwaternion Mag:");
		RysujNapis(chNapis, KOL12, 270);

		setColor(SZARY50);
		RysujNapis((char*)chOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 300);	//"Wdus ekran i trzymaj aby zakonczyc"
	}

	//ICM42688
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel1[0]);
	RysujNapis(chNapis, KOL12+8*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel1[1]);
	RysujNapis(chNapis, KOL12+20*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel1[2]);
	RysujNapis(chNapis, KOL12+32*FONT_SL, 30);

	//LSM6DSV
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel2[0]);
	RysujNapis(chNapis, KOL12+8*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel2[1]);
	RysujNapis(chNapis, KOL12+20*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel2[2]);
	RysujNapis(chNapis, KOL12+32*FONT_SL, 50);

	//ICM42688
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal1[0]);
	RysujNapis(chNapis, KOL12+8*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal1[1]);
	RysujNapis(chNapis, KOL12+20*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal1[2]);
	RysujNapis(chNapis, KOL12+32*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_IMU1] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	RysujNapis(chNapis, KOL12+49*FONT_SL, 70);

	//LSM6DSV
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal2[0]);
	RysujNapis(chNapis, KOL12+8*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal2[1]);
	RysujNapis(chNapis, KOL12+20*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal2[2]);
	RysujNapis(chNapis, KOL12+32*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_IMU2] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	RysujNapis(chNapis, KOL12+49*FONT_SL, 90);

	//IIS2MDC
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne1[0]*1e6);
	RysujNapis(chNapis, KOL12+8*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne1[1]*1e6);
	RysujNapis(chNapis, KOL12+20*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne1[2]*1e6);
	RysujNapis(chNapis, KOL12+32*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(POMARANCZ); 	else	setColor(SZARY50);
	fDlugosc = sqrtf(uDaneCM4.dane.fMagne1[0] * uDaneCM4.dane.fMagne1[0] + uDaneCM4.dane.fMagne1[1] * uDaneCM4.dane.fMagne1[1] + uDaneCM4.dane.fMagne1[2] * uDaneCM4.dane.fMagne1[2]);
	sprintf(chNapis, "%.1f %% ", fDlugosc / NOMINALNE_MAGN * 100);	//długość wektora [%]
	RysujNapis(chNapis, KOL12+49*FONT_SL, 110);

	//MMC34160
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne2[0]*1e6);
	RysujNapis(chNapis, KOL12+8*FONT_SL, 130);
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne2[1]*1e6);
	RysujNapis(chNapis, KOL12+20*FONT_SL, 130);
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne2[2]*1e6);
	RysujNapis(chNapis, KOL12+32*FONT_SL, 130);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(POMARANCZ); 	else	setColor(SZARY50);
	fDlugosc = sqrtf(uDaneCM4.dane.fMagne2[0] * uDaneCM4.dane.fMagne2[0] + uDaneCM4.dane.fMagne2[1] * uDaneCM4.dane.fMagne2[1] + uDaneCM4.dane.fMagne2[2] * uDaneCM4.dane.fMagne2[2]);
	sprintf(chNapis, "%.1f %% ", fDlugosc / NOMINALNE_MAGN * 100);	//długość wektora [%]
	RysujNapis(chNapis, KOL12+49*FONT_SL, 130);

	//HMC5883
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne3[0]*1e6);
	RysujNapis(chNapis, KOL12+8*FONT_SL, 150);
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne3[1]*1e6);
	RysujNapis(chNapis, KOL12+20*FONT_SL, 150);
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne3[2]*1e6);
	RysujNapis(chNapis, KOL12+32*FONT_SL, 150);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(POMARANCZ); 	else	setColor(SZARY50);
	fDlugosc = sqrtf(uDaneCM4.dane.fMagne3[0] * uDaneCM4.dane.fMagne3[0] + uDaneCM4.dane.fMagne3[1] * uDaneCM4.dane.fMagne3[1] + uDaneCM4.dane.fMagne3[2] * uDaneCM4.dane.fMagne3[2]);
	sprintf(chNapis, "%.1f %% ", fDlugosc / NOMINALNE_MAGN * 100);	//długość wektora [%]
	RysujNapis(chNapis, KOL12+49*FONT_SL, 150);

	//sygnalizacja tonem wartości osi Z magnetometru
	chTon = LICZBA_TONOW_WARIO/2 - (uDaneCM4.dane.fMagne3[2] / (NOMINALNE_MAGN / (LICZBA_TONOW_WARIO/2)));
	if (chTon > LICZBA_TONOW_WARIO)
		chTon = LICZBA_TONOW_WARIO;
	if (chTon < 0)
		chTon = 0;
	//UstawTon(chTon, 80);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU1[0], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+8*FONT_SL, 170);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU1[1], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+20*FONT_SL, 170);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU1[2], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+32*FONT_SL, 170);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[0], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+8*FONT_SL, 190);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[1], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+20*FONT_SL, 190);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[2], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+32*FONT_SL, 190);

	//kąty z akcelrometru 1
	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel1[0], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+13*FONT_SL, 210);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel1[1], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+25*FONT_SL, 210);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel1[2], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+37*FONT_SL, 210);

	//kąty z akcelrometru 2
	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel2[0], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+13*FONT_SL, 230);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel2[1], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+25*FONT_SL, 230);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel2[2], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+37*FONT_SL, 230);

	/*/kąty z żyroskopu 1
	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[0], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+13*FONT_SL, 250);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[1], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+25*FONT_SL, 250);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[2], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+37*FONT_SL, 250);

	//kąty z żyroskopu 2
	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[0], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+13*FONT_SL, 270);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[1], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+25*FONT_SL, 270);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[2], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+37*FONT_SL, 270); */

	//kwaternion wektora przyspieszenia
	setColor(POMARANCZ);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[0]);
	RysujNapis(chNapis, KOL12+16*FONT_SL, 250);
	setColor(KOLOR_X);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[1]);
	RysujNapis(chNapis, KOL12+26*FONT_SL, 250);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[2]);
	RysujNapis(chNapis, KOL12+36*FONT_SL, 250);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[3]);
	RysujNapis(chNapis, KOL12+46*FONT_SL, 250);

	//kwaternion wektora magnetycznego
	setColor(POMARANCZ);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaMag[0]);
	RysujNapis(chNapis, KOL12+16*FONT_SL, 270);
	setColor(KOLOR_X);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaMag[1]);
	RysujNapis(chNapis, KOL12+26*FONT_SL, 270);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaMag[2]);
	RysujNapis(chNapis, KOL12+36*FONT_SL, 270);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaMag[3]);
	RysujNapis(chNapis, KOL12+46*FONT_SL, 270);

	//Rysuj pasek postepu jeżeli trwa jakiś proces. Zakładam że czas procesu jest zmniejszany od wartości CZAS_KALIBRACJI do zera
	RysujPasekPostepu(CZAS_KALIBRACJI);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z damymi pomiarowymi czujników ciśnienia i GPS
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PomiaryCisnieniowe(void)
{
	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Dane czujnikow cisnienia");

		setColor(SZARY80);
		sprintf(chNapis, "Ci%cn 1:             AGL1:", ś);
		RysujNapis(chNapis, KOL12, 30);
		sprintf(chNapis, "Ci%cn 2:             AGL2:", ś);
		RysujNapis(chNapis, KOL12, 50);
		sprintf(chNapis, "Ci%cR%c%cn 1:          IAS1:", ś, ó, ż);
		RysujNapis(chNapis, KOL12, 70);
		sprintf(chNapis, "Ci%cR%c%cn 2:          IAS2:", ś, ó, ż);
		RysujNapis(chNapis, KOL12, 90);

		sprintf(chNapis, "GNSS D%cug:             Szer:             HDOP:", ł);
		RysujNapis(chNapis, KOL12, 140);
		sprintf(chNapis, "GNSS WysMSL:           Pred:             Kurs:");
		RysujNapis(chNapis, KOL12, 160);
		sprintf(chNapis, "GNSS Czas:             Data:              Sat:");
		RysujNapis(chNapis, KOL12, 180);

		setColor(SZARY50);
		RysujNapis((char*)chOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 300);	//"Wdus ekran i trzymaj aby zakonczyc"
	}

	//MS5611
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(BIALY); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnieBzw[0]);
	RysujNapis(chNapis, KOL12+8*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(CYJAN); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.2f m ", uDaneCM4.dane.fWysokoMSL[0]);
	RysujNapis(chNapis, KOL12+26*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_BARO1] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	RysujNapis(chNapis, KOL12+40*FONT_SL, 30);

	//BMP581
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(BIALY); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnieBzw[1]);
	RysujNapis(chNapis, KOL12+8*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(CYJAN); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.2f m ", uDaneCM4.dane.fWysokoMSL[1]);
	RysujNapis(chNapis, KOL12+26*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_BARO2] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	RysujNapis(chNapis, KOL12+40*FONT_SL, 50);

	//ND130
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(BIALY); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnRozn[0]);
	RysujNapis(chNapis, KOL12+11*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(MAGENTA); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.2f m/s ", uDaneCM4.dane.fPredkosc[0]);
	RysujNapis(chNapis, KOL12+26*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_CISR1] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	RysujNapis(chNapis, KOL12+40*FONT_SL, 70);

	//MS4525
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(BIALY); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnRozn[1]);
	RysujNapis(chNapis, KOL12+11*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(MAGENTA); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.2f m/s ", uDaneCM4.dane.fPredkosc[1]);
	RysujNapis(chNapis, KOL12+26*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_CISR2] - KELVIN , ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	RysujNapis(chNapis, KOL12+40*FONT_SL, 90);


	if (uDaneCM4.dane.stGnss1.chFix)
		setColor(BIALY);	//jest fix
	else
		setColor(SZARY70);	//nie ma fixa

	sprintf(chNapis, "%.7f ", uDaneCM4.dane.stGnss1.dDlugoscGeo);
	RysujNapis(chNapis, KOL12+11*FONT_SL, 140);
	sprintf(chNapis, "%.7f ", uDaneCM4.dane.stGnss1.dSzerokoscGeo);
	RysujNapis(chNapis, KOL12+29*FONT_SL, 140);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.stGnss1.fHdop);
	RysujNapis(chNapis, KOL12+47*FONT_SL, 140);

	sprintf(chNapis, "%.1fm ", uDaneCM4.dane.stGnss1.fWysokoscMSL);
	RysujNapis(chNapis, KOL12+13*FONT_SL, 160);
	sprintf(chNapis, "%.3fm/s ", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi);
	RysujNapis(chNapis, KOL12+29*FONT_SL, 160);
	sprintf(chNapis, "%3.2f%c ", uDaneCM4.dane.stGnss1.fKurs, ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12+47*FONT_SL, 160);

	sprintf(chNapis, "%02d:%02d:%02d ", uDaneCM4.dane.stGnss1.chGodz, uDaneCM4.dane.stGnss1.chMin, uDaneCM4.dane.stGnss1.chSek);
	RysujNapis(chNapis, KOL12+12*FONT_SL, 180);
	if  (uDaneCM4.dane.stGnss1.chMies > 12)	//ograniczenie aby nie pobierało nazwy miesiaca spoza tablicy chNazwyMies3Lit[]
		uDaneCM4.dane.stGnss1.chMies = 0;	//zerowy indeks jest pustą nazwą "---"
	sprintf(chNapis, "%02d %s %04d ", uDaneCM4.dane.stGnss1.chDzien, chNazwyMies3Lit[uDaneCM4.dane.stGnss1.chMies], uDaneCM4.dane.stGnss1.chRok + 2000);
	RysujNapis(chNapis, KOL12+29*FONT_SL, 180);
	sprintf(chNapis, "%d ", uDaneCM4.dane.stGnss1.chLiczbaSatelit);
	RysujNapis(chNapis, KOL12+47*FONT_SL, 180);

	//Rysuj pasek postepu jeżeli trwa jakiś proces. Zakładam że czas procesu jest zmniejszany od wartości CZAS_KALIBRACJI do zera
	RysujPasekPostepu(CZAS_KALIBRACJI);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z damymi odbiornika RC
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void DaneOdbiornikaRC(void)
{
	uint16_t y, n, sSkorygowaneRC, sDlugoscPaska, sDlugoscTla;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Dane odbiornika RC");
		for (n=0; n<KANALY_ODB_RC; n++)
		{
			y = n * 17;
			setColor(SZARY80);
			sprintf(chNapis, "Kan %2d:", n+1);
			RysujNapis(chNapis, KOL12, y+26);
			setColor(SZARY40);
			RysujProstokat(119, y+29, (121 + (PPM_MAX - PPM_MIN) / ROZDZIECZOSC_PASKA_RC), y+29+8);
		}
		setColor(SZARY50);
		RysujNapis((char*)chOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 300);	//"Wdus ekran i trzymaj aby zakonczyc"
	}

	for (n=0; n<KANALY_ODB_RC; n++)
	{
		y = n * 17;
		setColor(SZARY80);
		sprintf(chNapis, "%4d ", uDaneCM4.dane.sKanalRC[n]);
		RysujNapis(chNapis, KOL12+8*FONT_SL, y+26);

		//czasami długość kanału jest poza zakresem, więc skoryguj aby nie komplikować obliczeń
		if (uDaneCM4.dane.sKanalRC[n] < PPM_MIN)
			sSkorygowaneRC = PPM_MIN;
		else
		if (uDaneCM4.dane.sKanalRC[n] > PPM_MAX)
			sSkorygowaneRC = PPM_MAX;
		else
			sSkorygowaneRC = uDaneCM4.dane.sKanalRC[n];

		sDlugoscPaska = (sSkorygowaneRC - PPM_MIN) / ROZDZIECZOSC_PASKA_RC;
		if (sDlugoscPaska)
			RysujProstokatWypelniony(120, y+30, sDlugoscPaska, 6, NIEBIESKI);
		sDlugoscTla = ((PPM_MAX - PPM_MIN) / ROZDZIECZOSC_PASKA_RC) - sDlugoscPaska;
		if (sDlugoscTla)
			RysujProstokatWypelniony(121 + sDlugoscPaska, y+30, sDlugoscTla, 6, CZARNY);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje pasek postepu procesu rdzenia CM4 na dole ekranu
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujPasekPostepu(uint16_t sPelenZakres)
{
	uint16_t x = (uDaneCM4.dane.sPostepProcesu * DISP_X_SIZE) / sPelenZakres;
	if (x)	//nie rysuj paska jeżeli ma zerową długość
	{
		//setColor(BLUE);
		//fillRect(0, DISP_Y_SIZE - WYS_PASKA_POSTEPU, x , DISP_Y_SIZE);
		RysujProstokatWypelniony(0, DISP_Y_SIZE - WYS_PASKA_POSTEPU, x, WYS_PASKA_POSTEPU, NIEBIESKI);
	}
	//setColor(CZARNY);
	//fillRect(x, DISP_Y_SIZE - WYS_PASKA_POSTEPU, DISP_X_SIZE , DISP_Y_SIZE);
	RysujProstokatWypelniony(x, DISP_Y_SIZE - WYS_PASKA_POSTEPU, DISP_X_SIZE, WYS_PASKA_POSTEPU, NIEBIESKI);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z testem generowania tonu
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void TestTonuAudio(void)
{
	extern uint8_t chNumerTonu;
	static uint16_t sLicznikTonu;
	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Test tonu wario");

		setColor(SZARY80);
		sprintf(chNapis, "Numer tonu:");
		RysujNapis(chNapis, KOL12, 30);
		sprintf(chNapis, "Czest. 1 harm.:");
		RysujNapis(chNapis, KOL12, 50);
		sprintf(chNapis, "Czest. 3 harm.:");
		RysujNapis(chNapis, KOL12, 70);
	}

	sLicznikTonu++;
	if (sLicznikTonu > 900)
	{
		sLicznikTonu = 0;
		chNumerTonu++;
		if (chNumerTonu >= LICZBA_TONOW_WARIO)
			chNumerTonu = 0;

		setColor(BIALY);
		sprintf(chNapis, "%d ", chNumerTonu);
		RysujNapis(chNapis, KOL12+16*FONT_SL, 30);
		sprintf(chNapis, "%.1f Hz ", 1.0f * CZESTOTLIWOSC_AUDIO / (MIN_OKRES_TONU + chNumerTonu * SKOK_TONU));
		RysujNapis(chNapis, KOL12+16*FONT_SL, 50);
		sprintf(chNapis, "%.1f Hz ", 3.0f * CZESTOTLIWOSC_AUDIO / (MIN_OKRES_TONU + chNumerTonu * SKOK_TONU));
		RysujNapis(chNapis, KOL12+16*FONT_SL, 70);
		UstawTon(chNumerTonu, 80);
	}

}


////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z danymi karty SD
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WyswietlParametryKartySD(void)
{
	HAL_SD_CardInfoTypeDef CardInfo;
	extern uint8_t BSP_SD_IsDetected(void);
	extern void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo);
	HAL_SD_CardCIDTypedef pCID;
	HAL_SD_CardCSDTypedef pCSD;
	HAL_SD_CardStatusTypeDef pStatus;
	char chOEM[2];
	extern uint8_t chPort_exp_wysylany[];
	float fNapiecie;
	uint16_t sPozY;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Parametry karty SD");

		//zamaż ewentualną pozostałość napisu o braku karty
		UstawCzcionke(BigFont);
		setColor(CZARNY);
		sprintf(chNapis, "                             ");
		RysujNapis(chNapis, CENTER, 50);
		UstawCzcionke(MidFont);
	}

	if (BSP_SD_IsDetected())
	{
		BSP_SD_GetCardInfo(&CardInfo);
		HAL_SD_GetCardCID(&hsd1, &pCID);
		HAL_SD_GetCardCSD(&hsd1, &pCSD);
		HAL_SD_GetCardStatus(&hsd1, &pStatus);
		sPozY = 30;

		setColor(SZARY70);
		sprintf(chNapis, "Typ: %ld ", CardInfo.CardType);
		RysujNapis(chNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Pojemnosc: ");
		RysujNapis(chNapis, KOL12, sPozY);
		switch(pCSD.CSDStruct)
		{
		case 0:	sprintf(chNapis, "Standard");	break;
		case 1:	sprintf(chNapis, "Wysoka  ");	break;
		default:sprintf(chNapis, "Nieznana");	break;
		}
		RysujNapis(chNapis, KOL12 + 11*FONT_SL, sPozY);

		sPozY += 20;
		//sprintf(chNapis, "Klasy: %ld (0x%lX) ", CardInfo.Class, CardInfo.Class);
		sprintf(chNapis, "CCC: ");
		RysujNapis(chNapis, KOL12, sPozY);
		for (uint16_t n=0; n<12; n++)
		{
			if (CardInfo.Class & (1<<n))
				setColor(SZARY80);
			else
				setColor(SZARY40);
			sprintf(chNapis, "%X", n);
			RysujNapis(chNapis, KOL12 + ((5+n)*FONT_SL), sPozY);
		}
		sPozY += 20;

		setColor(SZARY70);
		sprintf(chNapis, "Klasa predk: %d ", pStatus.SpeedClass);
		RysujNapis(chNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Klasa UHS: %d ", pStatus.UhsSpeedGrade);
		RysujNapis(chNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Klasa Video: %d ", pStatus.VideoSpeedClass);
		RysujNapis(chNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(chNapis, "PerformanceMove: %d MB/s", pStatus.PerformanceMove);
		RysujNapis(chNapis, KOL12, sPozY);
		sPozY += 20;

		sprintf(chNapis, "Wsp pred. O/Z: %d ", pCSD.WrSpeedFact);
		RysujNapis(chNapis, KOL12, sPozY);
		sPozY += 20;

		if (chPort_exp_wysylany[0] & EXP02_LOG_VSELECT)	//LOG_SD1_VSEL: H=3,3V
			fNapiecie = 3.3;
		else
			fNapiecie = 1.8;
		sprintf(chNapis, "Napi%ccie I/O: %.1fV ", ę, fNapiecie);
		RysujNapis(chNapis, KOL12, sPozY);
		sPozY += 20;

		float fPodstawaCzasu, fMnoznik;
		fPodstawaCzasu = 1e-9;	//1ns
		for (uint8_t n=0; n<(pCSD.TAAC & 0x07); n++)
			fPodstawaCzasu *= 10;
		switch ((pCSD.TAAC & 0x78)>>3)
		{
		case 1: fMnoznik = 1.0f;	break;
		case 2: fMnoznik = 1.2f;	break;
		case 3: fMnoznik = 1.3f;	break;
		case 4: fMnoznik = 1.5f;	break;
		case 5: fMnoznik = 2.0f;	break;
		case 6: fMnoznik = 2.5f;	break;
		case 7: fMnoznik = 3.0f;	break;
		case 8: fMnoznik = 3.5f;	break;
		case 9: fMnoznik = 4.0f;	break;
		case 10: fMnoznik = 4.5f;	break;
		case 11: fMnoznik = 5.0f;	break;
		case 12: fMnoznik = 5.5f;	break;
		case 13: fMnoznik = 6.0f;	break;
		case 14: fMnoznik = 7.0f;	break;
		case 15: fMnoznik = 8.0f;	break;
		default: fMnoznik = 0.0f;
		}
		sprintf(chNapis, "Czas dost. async: %.1e s ", fPodstawaCzasu*fMnoznik);
		RysujNapis(chNapis, KOL12, sPozY);
		sPozY += 20;

		sprintf(chNapis, "Czas dost. sync: %d cyk.zeg ", pCSD.NSAC);
		RysujNapis(chNapis, KOL12, sPozY);
		sPozY += 20;

		sprintf(chNapis, "MaxBusClkFrec: %d", pCSD.MaxBusClkFrec);
		RysujNapis(chNapis, KOL12, sPozY);
		sPozY += 20;


		//druga kolumna
		sPozY = 30;
		sprintf(chNapis, "Liczba sektor%cw: %ld ", ó, CardInfo.BlockNbr);		//Specifies the Card Capacity in blocks
		RysujNapis(chNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Rozmiar sektora: %ld B ", CardInfo.BlockSize);		//Specifies one block size in bytes
		RysujNapis(chNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Pojemno%c%c karty: %ld MB ", ś, ć, CardInfo.BlockNbr * (CardInfo.BlockSize / 512) / 2048);		//Specifies one block size in bytes
		RysujNapis(chNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Rozm Jedn Alok: %d kB", (8<<pStatus.AllocationUnitSize));		//Specifies one block size in bytes
		RysujNapis(chNapis, KOL22, sPozY);
		sPozY += 20;
		//sprintf(chNapis, "Liczba blok%cw log.: %ld ", ó, CardInfo.LogBlockNbr);		//Specifies the Card logical Capacity in blocks
		//RysujNapis(chNapis, KOL22, sPozY);
		//sPozY += 20;
		sprintf(chNapis, "RdBlockLen: %d ", (2<<pCSD.RdBlockLen));
		RysujNapis(chNapis, KOL22, sPozY);
		sPozY += 20;
		//sprintf(chNapis, "Rozmiar karty log: %ld MB ", CardInfo.LogBlockNbr * (CardInfo.LogBlockSize / 512) / 2048);		//Specifies one block size in bytes
		//RysujNapis(chNapis, KOL22, sPozY);
		//sPozY += 20;

		setColor(SZARY70);
		sprintf(chNapis, "Format: ");
		RysujNapis(chNapis, KOL22, sPozY);
		switch (pCSD.FileFormat)
		{
		case 0: sprintf(chNapis, "HDD z partycja ");	break;
		case 1: sprintf(chNapis, "DOS FAT ");			break;
		case 2: sprintf(chNapis, "UnivFileFormat ");	break;
		default: sprintf(chNapis, "Nieznany ");			break;
		}
		setColor(SZARY80);
		RysujNapis(chNapis, KOL22+8*FONT_SL, sPozY);
		sPozY += 20;

		setColor(SZARY70);
		sprintf(chNapis, "Manuf ID: ");
		RysujNapis(chNapis, KOL22, sPozY);
		switch (pCID.ManufacturerID)
		{
		case 0x02:	sprintf(chNapis, "Goodram/Toshiba ");	break;
		case 0x03:	sprintf(chNapis, "SandDisk ");	break;
		case 0xdF:
		case 0x05:	sprintf(chNapis, "Lenovo ");	break;
		case 0x09:	sprintf(chNapis, "APT ");	break;
		case 0x1B:	sprintf(chNapis, "Samsung ");	break;
		case 0x1D:	sprintf(chNapis, "ADATA ");	break;
		case 0x1F:
		case 0x41:	sprintf(chNapis, "Kingstone ");	break;
		case 0x6F:	sprintf(chNapis, "Kodak ");	break;
		case 0x74:	sprintf(chNapis, "Trnasced ");	break;
		case 0x82:	sprintf(chNapis, "Sony ");	break;
		default:	sprintf(chNapis, "%X ", pCID.ManufacturerID);
		}
		setColor(SZARY80);
		RysujNapis(chNapis, KOL22+10*FONT_SL, sPozY);
		sPozY += 20;

		setColor(SZARY70);
		chOEM[0] = (pCID.OEM_AppliID & 0xFF00)>>8;
		chOEM[1] = pCID.OEM_AppliID & 0x00FF;
		sprintf(chNapis, "OEM_AppliID: %c%c ", chOEM[0], chOEM[1]);
		RysujNapis(chNapis, KOL22, sPozY);
		sPozY += 20;

		sprintf(chNapis, "Nr seryjny: %ld ", pCID.ProdSN);
		RysujNapis(chNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Data prod: %s %d ", chNazwyMies3Lit[(pCID.ManufactDate & 0xF)], ((pCID.ManufactDate>>4) & 0xFF) + 2000);
		RysujNapis(chNapis, KOL22, sPozY);
		sPozY += 20;

	}
	else
	{
		UstawCzcionke(BigFont);
		setColor(CZERWONY);
		sprintf(chNapis, "Wolne %carty, tu brak karty! ", ż);
		RysujNapis(chNapis, CENTER, 50);
		chRysujRaz = 1;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z parametrami rejestratora na karcie SD
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WyswietlRejestratorKartySD(void)
{
	extern uint8_t chKodBleduFAT;
	extern uint16_t sMaxDlugoscWierszaLogu;
	uint16_t sPozY;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Rejestrator na karcie SD");
	}

	sPozY = 30;
	setColor(SZARY80);

	sprintf(chNapis, "Karta SD: ");
	RysujNapis(chNapis, KOL12, sPozY);
	setColor(ZOLTY);
	if ((chPort_exp_odbierany[0] & EXP04_LOG_CARD_DET)	== 0)	//LOG_SD1_CDETECT - wejście detekcji obecności karty
	{
		setColor(KOLOR_Y);
		sprintf(chNapis, "Obecna");
	}
	else
	{
		setColor(KOLOR_X);
		sprintf(chNapis, "Brak! ");
	}
	RysujNapis(chNapis, KOL12 + 11*FONT_SL, sPozY);
	sPozY += 20;


	setColor(SZARY80);
	sprintf(chNapis, "Stan FATu: ");
	RysujNapis(chNapis, KOL12, sPozY);

	if (chStatusRejestratora & STATREJ_FAT_GOTOWY)
	{
		setColor(KOLOR_Y);
		sprintf(chNapis, "Gotowy                ");	//długością ma przykryć nadłuższy komunikat o błędzie
	}
	else
	{
		setColor(KOLOR_X);
		PobierzKodBleduFAT(chKodBleduFAT, chNapis);

	}
	RysujNapis(chNapis, KOL12 + 11*FONT_SL, sPozY);
	sPozY += 20;

	setColor(SZARY80);
	sprintf(chNapis, "Plik logu: ");
	RysujNapis(chNapis, KOL12, sPozY);

	if (chStatusRejestratora & STATREJ_OTWARTY_PLIK)
	{
		setColor(KOLOR_Y);
		sprintf(chNapis, "Otwarty   ");
	}
	else
	{
		setColor(ZOLTY);
		if (chStatusRejestratora & STATREJ_BYL_OTWARTY)
			sprintf(chNapis, "Zatrzymany");
		else
			sprintf(chNapis, "Brak ");
	}
	RysujNapis(chNapis, KOL12 + 11*FONT_SL, sPozY);
	sPozY += 20;

	setColor(SZARY80);
	sprintf(chNapis, "Rejestrator: ");
	RysujNapis(chNapis, KOL12, sPozY);
	setColor(ZOLTY);
	if (chStatusRejestratora & STATREJ_WLACZONY)
	{
		setColor(KOLOR_Y);
		sprintf(chNapis, "W%c%cczony  ", ł, ą);
	}
	else
	{
		setColor(ZOLTY);
		sprintf(chNapis, "Zatrzymany");
	}
	RysujNapis(chNapis, KOL12 + 13*FONT_SL, sPozY);
	sPozY += 20;

	setColor(SZARY80);
	sprintf(chNapis, "Zapelnienie: ");
	RysujNapis(chNapis, KOL12, sPozY);
	float fZapelnienie = (float)sMaxDlugoscWierszaLogu / ROZMIAR_BUFORA_LOGU;
	if (fZapelnienie < 0.75)
		setColor(KOLOR_Y);	//zielony
	else
	if (fZapelnienie < 0.95)
		setColor(ZOLTY);
	else
		setColor(KOLOR_X);	//czerwony
	sprintf(chNapis, "%d/%d ", sMaxDlugoscWierszaLogu, ROZMIAR_BUFORA_LOGU);
	RysujNapis(chNapis, KOL12 + 13*FONT_SL, sPozY);
	sPozY += 20;
}



////////////////////////////////////////////////////////////////////////////////
// Parametry: chKodBleduFAT - numer kodu błędu typu FRESULT
// Zwraca: wskaźnik na string z kodem błędu FAT
////////////////////////////////////////////////////////////////////////////////
void PobierzKodBleduFAT(uint8_t chKodBledu, char* napis)
{
	//wersja krótsza z nazwą błędu
	switch (chKodBledu)
	{
	case FR_DISK_ERR: 			sprintf(napis, "FR_DISK_ERR");			break;
	case FR_INT_ERR:			sprintf(napis, "FR_INT_ERR");				break;
	case FR_NOT_READY: 			sprintf(napis, "FR_NOT_READY");			break;
	case FR_NO_FILE:			sprintf(napis, "FR_NO_FILE");				break;
	case FR_NO_PATH:			sprintf(napis, "FR_NO_PATH");				break;
	case FR_INVALID_NAME:		sprintf(napis, "FR_INVALID_NAME");		break;
	case FR_DENIED:				sprintf(napis, "FR_DENIED");				break;
	case FR_EXIST:				sprintf(napis, "FR_EXIST");				break;
	case FR_INVALID_OBJECT:		sprintf(napis, "FR_INVALID_OBJECT");		break;
	case FR_WRITE_PROTECTED:	sprintf(napis, "FR_WRITE_PROTECTED");		break;
	case FR_INVALID_DRIVE:		sprintf(napis, "FR_INVALID_DRIVE");		break;
	case FR_NOT_ENABLED:		sprintf(napis, "FR_NOT_ENABLED");			break;
	case FR_NO_FILESYSTEM:		sprintf(napis, "FR_NO_FILESYSTEM");		break;
	case FR_MKFS_ABORTED:		sprintf(napis, "FR_MKFS_ABORTED");		break;
	case FR_TIMEOUT:			sprintf(napis, "FR_TIMEOUT");				break;
	case FR_LOCKED:				sprintf(napis, "FR_LOCKED");				break;
	case FR_NOT_ENOUGH_CORE:	sprintf(napis, "FR_NOT_ENOUGH_CORE");		break;
	case FR_TOO_MANY_OPEN_FILES:sprintf(napis, "FR_TOO_MANY_OPEN_FILES");	break;
	case FR_INVALID_PARAMETER:	sprintf(napis, "FR_INVALID_PARAMETER");	break;
	default: sprintf(napis, "Blad nieznany");
	}

	//wersja dłuższa  opisem błędu
	/*switch (chKodBleduFAT)
	{
	case FR_DISK_ERR: 			sprintf(chNapis, "A hard error in low level disk I/O layer");	break;
	case FR_INT_ERR:			sprintf(chNapis, "Assertion failed");							break;
	case FR_NOT_READY: 			sprintf(chNapis, "The physical drive cannot work");				break;
	case FR_NO_FILE:			sprintf(chNapis, "Could not find the file");					break;
	case FR_NO_PATH:			sprintf(chNapis, "Could not find the path");					break;
	case FR_INVALID_NAME:		sprintf(chNapis, "The path name format is invalid");			break;
	case FR_DENIED:				sprintf(chNapis, "Access denied or directory full");			break;
	case FR_EXIST:				sprintf(chNapis, "Access denied due to prohibited access");		break;
	case FR_INVALID_OBJECT:		sprintf(chNapis, "The file/directory object is invalid");		break;
	case FR_WRITE_PROTECTED:	sprintf(chNapis, "The physical drive is write protected");		break;
	case FR_INVALID_DRIVE:		sprintf(chNapis, "The logical drive number is invalid");		break;
	case FR_NOT_ENABLED:		sprintf(chNapis, "The volume has no work area");				break;
	case FR_NO_FILESYSTEM:		sprintf(chNapis, "There is no valid FAT volume");				break;
	case FR_MKFS_ABORTED:		sprintf(chNapis, "The f_mkfs() aborted due to any problem");	break;
	case FR_TIMEOUT:			sprintf(chNapis, "Could not get a grant to access the volume");	break;
	case FR_LOCKED:				sprintf(chNapis, "Operation rejected due file sharing policy");	break;
	case FR_NOT_ENOUGH_CORE:	sprintf(chNapis, "LFN working buffer could not be allocated");	break;
	case FR_TOO_MANY_OPEN_FILES:sprintf(chNapis, "Number of open files > _FS_LOCK");			break;
	case FR_INVALID_PARAMETER:	sprintf(chNapis, "Given parameter is invalid");					break;
	}*/

}



////////////////////////////////////////////////////////////////////////////////
// Rysuje kostkę obracajacą się o podany kąt
// Parametry: *fKat - wskaźnik na tablicę float[3] katów obrotu
// Zwraca: czas obliczeń (bez rysowania) w us
////////////////////////////////////////////////////////////////////////////////
uint32_t RysujKostkeObrotu(float *fKat)
{
	float fKostkaRob[8][3];
	int16_t sKostka[8][3];
	uint32_t nCzas = PobierzCzasT6();
	stPlas_t stPla[6];
	int16_t sWektA[3], sWektB[3];
	//int16_t sWektor[6][3];

	//int16_t sPunktRzutowania[3] = {0, 0, -500};
	//int16_t sWektorRzutowania[6][3];
	//int32_t nIloczynSkalarny[6];	//iloczyn skalarny wektora normalnego płaszczyzny i wektora rzutowania

	for (uint16_t n=0; n<8; n++)
	{
		ObrocWektor(&fKostka[n][0], &fKostkaRob[n][0], fKat);
		for (uint16_t m=0; m<3; m++)
			sKostka[n][m] = (int16_t)fKostkaRob[n][m];	//konwersja z float na int16_t
	}
	nCzas = MinalCzas(nCzas);
	//zamaż poprzednią kostkę kolorem tła
	setColor(CZARNY);
	RysujLinie(sKostkaPoprzednia[0][0] + DISP_X_SIZE/2, sKostkaPoprzednia[0][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[1][0] + DISP_X_SIZE/2, sKostkaPoprzednia[1][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostkaPoprzednia[1][0] + DISP_X_SIZE/2, sKostkaPoprzednia[1][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[2][0] + DISP_X_SIZE/2, sKostkaPoprzednia[2][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostkaPoprzednia[2][0] + DISP_X_SIZE/2, sKostkaPoprzednia[2][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[3][0] + DISP_X_SIZE/2, sKostkaPoprzednia[3][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostkaPoprzednia[3][0] + DISP_X_SIZE/2, sKostkaPoprzednia[3][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[0][0] + DISP_X_SIZE/2, sKostkaPoprzednia[0][1] + DISP_Y_SIZE/2);

	//rysuj obrys kostki z dołu
	RysujLinie(sKostkaPoprzednia[4][0] + DISP_X_SIZE/2, sKostkaPoprzednia[4][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[5][0] + DISP_X_SIZE/2, sKostkaPoprzednia[5][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostkaPoprzednia[5][0] + DISP_X_SIZE/2, sKostkaPoprzednia[5][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[6][0] + DISP_X_SIZE/2, sKostkaPoprzednia[6][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostkaPoprzednia[6][0] + DISP_X_SIZE/2, sKostkaPoprzednia[6][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[7][0] + DISP_X_SIZE/2, sKostkaPoprzednia[7][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostkaPoprzednia[7][0] + DISP_X_SIZE/2, sKostkaPoprzednia[7][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[4][0] + DISP_X_SIZE/2, sKostkaPoprzednia[4][1] + DISP_Y_SIZE/2);

	//rysuj linie pionowych ścianek
	RysujLinie(sKostkaPoprzednia[0][0] + DISP_X_SIZE/2, sKostkaPoprzednia[0][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[4][0] + DISP_X_SIZE/2, sKostkaPoprzednia[4][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostkaPoprzednia[1][0] + DISP_X_SIZE/2, sKostkaPoprzednia[1][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[5][0] + DISP_X_SIZE/2, sKostkaPoprzednia[5][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostkaPoprzednia[2][0] + DISP_X_SIZE/2, sKostkaPoprzednia[2][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[6][0] + DISP_X_SIZE/2, sKostkaPoprzednia[6][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostkaPoprzednia[3][0] + DISP_X_SIZE/2, sKostkaPoprzednia[3][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[7][0] + DISP_X_SIZE/2, sKostkaPoprzednia[7][1] + DISP_Y_SIZE/2);


	//Aby obliczyć równanie płaszczyzny przechodzącej przez 3 punkty P1, P2 i P4 liczę współrzędne wektorów sWektA i sWektB
	//ścianka górna, czerwona
	for (uint16_t n=0; n<3; n++)
	{
		sWektA[n] = sKostka[1][n] - sKostka[0][n];		//wektor dłuższego boku
		sWektB[n] = sKostka[3][n] - sKostka[0][n];		//wektor krótszego boku
	}

	//liczę iloczyn wektorowy P1P2 x P1P3. Wzór: a x b = (a2b3-a3b2, a3b1-a1b3, a1b2-a2b1)
	stPla[0].nA = sWektA[1]*sWektB[2] - sWektA[2]*sWektB[1];
	stPla[0].nB = sWektA[2]*sWektB[0] - sWektA[0]*sWektB[2];
	stPla[0].nC = sWektA[0]*sWektB[1] - sWektA[1]*sWektB[0];

	//aby obliczyć D podstawiam punkt C pod równanie płaszczyzny Ax+By+Cz+D = 0 => D = -(Ax+By+Cz)
	stPla[0].nD = -1*(stPla[0].nA * sKostka[3][0] + stPla[0].nB * sKostka[3][1] + stPla[0].nC * sKostka[3][2]);
	stPla[0].nDlug = sqrtf(stPla[0].nA * stPla[0].nA + stPla[0].nB * stPla[0].nB + stPla[0].nC * stPla[0].nC);

	//ścianka dolna, zielona
	for (uint16_t n=0; n<3; n++)	//dla xyz
	{
		sWektA[n] = sKostka[5][n] - sKostka[4][n];		//wektor dłuższego boku
		sWektB[n] = sKostka[7][n] - sKostka[4][n];		//wektor krótszego boku
	}
	stPla[1].nA = sWektA[1]*sWektB[2] - sWektA[2]*sWektB[1];
	stPla[1].nB = sWektA[2]*sWektB[0] - sWektA[0]*sWektB[2];
	stPla[1].nC = sWektA[0]*sWektB[1] - sWektA[1]*sWektB[0];
	stPla[1].nD = -1*(stPla[1].nA * sKostka[7][0] + stPla[1].nB * sKostka[7][1] + stPla[1].nC * sKostka[7][2]);
	stPla[1].nDlug = sqrtf(stPla[1].nA * stPla[1].nA + stPla[1].nB * stPla[1].nB + stPla[1].nC * stPla[1].nC);


	//We współrzędnych oka tylny wielokąt może być zidentyfikowany przez nieujemny iloczyn skalarny normalnej powierzchni i wektora od punktu zbieżności do dowolnego punktu wielokąta.
	//Iloczyn skalarny jest: dodatni dla tylnego wielokąta, zerowy dla wielokąta widzianego jako krawędź

	//za wektor normalny do ściany przyjmuję krawędź prostopadłą wychodzącą z narożnika w stronę środka
	/*for (uint16_t m=0; m<3; m++)	//dla xyz
	{
		sWektor[0][m] = sKostka[0][m] - sKostka[4][m];
		sWektorRzutowania[0][m] = sKostka[0][m] - sPunktRzutowania[m];
		sWektor[1][m] = sKostka[4][m] - sKostka[0][m];
		sWektorRzutowania[1][m] = sKostka[4][m] - sPunktRzutowania[m];
	}

	//liczę iloczyn skalarny między wektorem normalnym płaszczyzny a [DISP_X_SIZE/2, DISP_Y_SIZE/2, 1000]
	for (uint16_t n=0; n<2; n++)	//dla każdej ściany
		nIloczynSkalarny[n] = sWektor[n][0] * sWektorRzutowania[n][0] + sWektor[n][1] * sWektorRzutowania[n][1] + sWektor[n][2] * sWektorRzutowania[n][2]; */

	//rysuj obrys kostki z góry
	setColor(CZERWONY);
	//if (nIloczynSkalarny[0] <= 0)	//jeżeli odległość górnej płaszczyzny od ekrany jest większa niż dolnej płaszcyzny
	{
		RysujLinie(sKostka[0][0] + DISP_X_SIZE/2, sKostka[0][1] + DISP_Y_SIZE/2, sKostka[1][0] + DISP_X_SIZE/2, sKostka[1][1] + DISP_Y_SIZE/2);
		RysujLinie(sKostka[1][0] + DISP_X_SIZE/2, sKostka[1][1] + DISP_Y_SIZE/2, sKostka[2][0] + DISP_X_SIZE/2, sKostka[2][1] + DISP_Y_SIZE/2);
		RysujLinie(sKostka[2][0] + DISP_X_SIZE/2, sKostka[2][1] + DISP_Y_SIZE/2, sKostka[3][0] + DISP_X_SIZE/2, sKostka[3][1] + DISP_Y_SIZE/2);
		RysujLinie(sKostka[3][0] + DISP_X_SIZE/2, sKostka[3][1] + DISP_Y_SIZE/2, sKostka[0][0] + DISP_X_SIZE/2, sKostka[0][1] + DISP_Y_SIZE/2);
	}

	//rysuj obrys kostki z dołu
	setColor(ZIELONY);
	//if (nIloczynSkalarny[1] <= 0)	//jeżeli odległość dolnej płaszczyzny od ekranu jest większa niż górnej płaszczyzny
	{
		RysujLinie(sKostka[4][0] + DISP_X_SIZE/2, sKostka[4][1] + DISP_Y_SIZE/2, sKostka[5][0] + DISP_X_SIZE/2, sKostka[5][1] + DISP_Y_SIZE/2);
		RysujLinie(sKostka[5][0] + DISP_X_SIZE/2, sKostka[5][1] + DISP_Y_SIZE/2, sKostka[6][0] + DISP_X_SIZE/2, sKostka[6][1] + DISP_Y_SIZE/2);
		RysujLinie(sKostka[6][0] + DISP_X_SIZE/2, sKostka[6][1] + DISP_Y_SIZE/2, sKostka[7][0] + DISP_X_SIZE/2, sKostka[7][1] + DISP_Y_SIZE/2);
		RysujLinie(sKostka[7][0] + DISP_X_SIZE/2, sKostka[7][1] + DISP_Y_SIZE/2, sKostka[4][0] + DISP_X_SIZE/2, sKostka[4][1] + DISP_Y_SIZE/2);
	}

	//rysuj linie pionowych ścianek
	setColor(ZOLTY);
	RysujLinie(sKostka[0][0] + DISP_X_SIZE/2, sKostka[0][1] + DISP_Y_SIZE/2, sKostka[4][0] + DISP_X_SIZE/2, sKostka[4][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostka[1][0] + DISP_X_SIZE/2, sKostka[1][1] + DISP_Y_SIZE/2, sKostka[5][0] + DISP_X_SIZE/2, sKostka[5][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostka[2][0] + DISP_X_SIZE/2, sKostka[2][1] + DISP_Y_SIZE/2, sKostka[6][0] + DISP_X_SIZE/2, sKostka[6][1] + DISP_Y_SIZE/2);
	RysujLinie(sKostka[3][0] + DISP_X_SIZE/2, sKostka[3][1] + DISP_Y_SIZE/2, sKostka[7][0] + DISP_X_SIZE/2, sKostka[7][1] + DISP_Y_SIZE/2);


	//zapamietaj współrzędne kostki aby w nastepnym cyklu ją wymazać
	for (uint16_t n=0; n<8; n++)
	{
		for (uint16_t m=0; m<2; m++)
			sKostkaPoprzednia[n][m] = sKostka[n][m];
	}
	setColor(SZARY50);
	sprintf(chNapis, "t=%ldus, phi=%.1f, theta=%.1f, psi=%.1f ", nCzas, fKat[0]*RAD2DEG, fKat[1]*RAD2DEG, fKat[2]*RAD2DEG);
	RysujNapis(chNapis, 0, 0);
	return nCzas;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran dla kalibracji wzmocnienia żyroskopów
// Parametry: *chSekwencer	- zmiennaWskazująca na kalibracje konktretnej osi
// Zwraca: ERR_GOTOWE / BLAD_OK - informację o tym czy wyjść z trybu kalibracji czy nie
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibracjaWzmocnieniaZyroskopow(uint8_t *chSekwencer)
{
	char chNazwaOsi;
	float fPochylenie, fPrzechylenie;
	uint8_t chErr = BLAD_OK;
	//uint8_t chRozmiar;

	if (chRysujRaz)
	{
		//chRysujRaz = 0;	będzie wyzerowane w funkcji rysowania poziomicy
		BelkaTytulu("Kalibr. wzmocnienia zyro");
		setColor(SZARY80);

		sprintf(chNapis, "Ca%cka %cyro 1:", ł, ż);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK, 100);
		sprintf(chNapis, "Ca%cka %cyro 2:", ł, ż);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK, 120);
		sprintf(chNapis, "K%cty w p%caszczy%cnie kalibracji", ą, ł, ź);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK, 140);
		sprintf(chNapis, "Pochylenie:");
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK, 160);
		sprintf(chNapis, "Przechylenie:");
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK, 180);
		sprintf(chNapis, "WspKal %cyro1:", ż);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK, 200);
		sprintf(chNapis, "WspKal %cyro2:", ż);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK, 220);

		setColor(SZARY40);
		stPrzycisk.sX1 = 10 + LIBELLA_BOK;
		stPrzycisk.sY1 = 240;
		stPrzycisk.sX2 = stPrzycisk.sX1 + 210;
		stPrzycisk.sY2 = stPrzycisk.sY1 + 75;
		RysujPrzycisk(stPrzycisk, "Odczyt", RYSUJ);

		chEtapKalibracji = 0;
		chStanPrzycisku = 0;
		setColor(ZOLTY);
		sprintf(chNapis, "B%cbelek powinien by%c w %crodku poziomicy", ą, ć, ś);
		RysujNapis(chNapis, CENTER, 50);
		setColor(SZARY60);
		sprintf(chNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		RysujNapis(chNapis, CENTER, 70);
		statusDotyku.chFlagi &= ~(DOTYK_ZWOLNONO | DOTYK_DOTKNIETO);	//czyść flagi ekranu dotykowego
	}

	//sekwencer kalibracji
	switch (*chSekwencer)
	{
	case SEKW_KAL_WZM_ZYRO_R:
		chNazwaOsi = 'Z';
		//fPochylenie = uDaneCM4.dane.fKatIMU2[1];
		//fPrzechylenie = uDaneCM4.dane.fKatIMU2[0];
		fPochylenie = uDaneCM4.dane.fKatAkcel1[1];
		fPrzechylenie = uDaneCM4.dane.fKatAkcel1[0];
		Poziomica(-fPrzechylenie, fPochylenie);	//przechylenie, pochylenie
		setColor(KOLOR_Z);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[2], ZNAK_STOPIEN);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[2], ZNAK_STOPIEN);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;

	case SEKW_KAL_WZM_ZYRO_Q:
		chNazwaOsi = 'Y';
		fPochylenie = atan2f(uDaneCM4.dane.fAkcel1[1], uDaneCM4.dane.fAkcel1[0]) + 90 * DEG2RAD;	//atan(y/x)
		fPrzechylenie = atan2f(uDaneCM4.dane.fAkcel1[1], uDaneCM4.dane.fAkcel1[2]) + 90 * DEG2RAD;	//atan(y/z)
		Poziomica(fPrzechylenie, -fPochylenie);	//przechylenie, pochylenie
		setColor(KOLOR_Y);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[1], ZNAK_STOPIEN);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[1], ZNAK_STOPIEN);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);	//przechylenie
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;

	case SEKW_KAL_WZM_ZYRO_P:
		chNazwaOsi = 'X';
		fPochylenie = atan2f(uDaneCM4.dane.fAkcel1[2], uDaneCM4.dane.fAkcel1[0]);	//atan(z/x)
		fPrzechylenie = atan2f(uDaneCM4.dane.fAkcel1[0], uDaneCM4.dane.fAkcel1[1]) - 90 * DEG2RAD;	//atan(x/y)
		Poziomica(-fPrzechylenie, fPochylenie);	//przechylenie, pochylenie
		setColor(KOLOR_X);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[0], ZNAK_STOPIEN);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[0], ZNAK_STOPIEN);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);	//przechylenie:
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;
	}

	setColor(ZOLTY);
	sprintf(chNapis, "Skieruj osia %c do dolu i wykonaj %d obroty w dowolna strone", chNazwaOsi, OBR_KAL_WZM);
	RysujNapis(chNapis, CENTER, 30);

	//wyświetl współczynnik kalibracji
	if ((uDaneCM4.dane.nZainicjowano & INIT_WYK_KAL_WZM_ZYRO) && (chEtapKalibracji >= 2))
	{
		setColor(BIALY);										//nowy współczynnik
		sprintf(chNapis, "%.3f", uDaneCM4.dane.uRozne.f32[0]);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 200);
		sprintf(chNapis, "%.3f", uDaneCM4.dane.uRozne.f32[1]);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 220);
	}
	else
	{
		setColor(SZARY80);										//stary współczynnik
		sprintf(chNapis, "%.3f", uDaneCM4.dane.uRozne.f32[2]);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 200);
		sprintf(chNapis, "%.3f", uDaneCM4.dane.uRozne.f32[3]);
		RysujNapis(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 220);
	}

	//napis dla przycisku
	setColor(ZOLTY);
	switch (chEtapKalibracji)
	{
	case 0:	//odczyt wartości bieżącej wzmocnienia
		uDaneCM7.dane.chWykonajPolecenie = POL_CZYTAJ_WZM_ZYROP + *chSekwencer;
		sprintf(chNapis, "Odczyt");
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == POL_CZYTAJ_WZM_ZYROP + *chSekwencer)	//jeżeli nastapił odczyt to przejdź do nastepnego etapu
			chEtapKalibracji = 1;
		break;

	case 1:	//zerowanie całki
		uDaneCM7.dane.chWykonajPolecenie = POL_ZERUJ_CALKE_ZYRO;	//zeruje całkę żyroskopów przed kalibracją
		sprintf(chNapis, "Start");
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == POL_ZERUJ_CALKE_ZYRO)	//jeżeli nastapiło wyzerowanie to nie czekaj na przycisk tylko od razu zacznij całkować
		{
			chEtapKalibracji = 2;
			uDaneCM7.dane.chWykonajPolecenie = POL_CALKUJ_PRED_KAT;		//wykonaj całkowanie kąta podczas obrotu
		}
		break;

	case 2:	//całkowanie
		uDaneCM7.dane.chWykonajPolecenie = POL_CALKUJ_PRED_KAT;		//wykonaj całkowanie kąta podczas obrotu
		sprintf(chNapis, "Oblicz");
		break;

	case 3:	//obliczenie kalibracji
		uDaneCM7.dane.chWykonajPolecenie = POL_KALIBRUJ_ZYRO_WZMR + *chSekwencer;	//uruchom kalibrację dla scałkowanego kąta R, Q lub P
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZLE_OBLICZENIA)
			sprintf(chNapis, "Blad! ");
		else
			sprintf(chNapis, "Gotowe");
		break;

	default:	//wyjście lub przejscie do kalibracji kolejnej osi
		sprintf(chNapis, "Wyjdz ");
		chErr = ERR_GOTOWE;	//zakończ kalibrację
		break;
	}

	/*setBackColor(SZARY40);	//kolor tła napisu kolorem przycisku
	UstawCzcionke(BigFont);
	RysujNapis(chNapis, stPrzycisk.sX1 + (stPrzycisk.sX2 - stPrzycisk.sX1)/2 - chRozmiar*FONT_BL/2 , stPrzycisk.sY1 + (stPrzycisk.sY2 - stPrzycisk.sY1)/2 - FONT_BH/2);
	setBackColor(CZARNY);
	UstawCzcionke(MidFont);*/


	//sprawdź czy jest naciskany przycisk
	if (statusDotyku.chFlagi & DOTYK_DOTKNIETO)
	{
		//czy naciśnięto na przycisk?
		if ((statusDotyku.sY > stPrzycisk.sY1) && (statusDotyku.sY < stPrzycisk.sY2) && (statusDotyku.sX > stPrzycisk.sX1) && (statusDotyku.sX < stPrzycisk.sX2))
		{
			chStanPrzycisku = 1;
			RysujPrzycisk(stPrzycisk, chNapis, ODSWIEZ);
		}
		else
			chErr = ERR_GOTOWE;	//zakończ kabrację gdy nacięnięto poza przyciskiem

		statusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;
	}
	else	//DOTYK_DOTKNIETO
	{
		if (chStanPrzycisku)
		{
			chEtapKalibracji++;
			chEtapKalibracji &= 0x07;
			chStanPrzycisku = 0;
		}
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran poziomicy
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void Poziomica(float fKatAkcelX, float fKatAkcelY)
{
	static uint16_t x2, y2;
	uint16_t x, y;

	if (chRysujRaz)
	{
		//setColor(LIBELLA1);
		//fillRect(0, DISP_Y_SIZE - LIBELLA_BOK, LIBELLA_BOK, DISP_Y_SIZE);	//wypełnienie płynem
		RysujProstokatWypelniony(0, DISP_Y_SIZE - LIBELLA_BOK, LIBELLA_BOK, LIBELLA_BOK, LIBELLA1);
		chRysujRaz = 0;
		x2 = (DISP_Y_SIZE - MENU_NAG_WYS)/2;					//współrzędne "starego" bąbelka do zamazania
		y2 = MENU_NAG_WYS + (DISP_Y_SIZE - MENU_NAG_WYS)/2;
	}

	//oblicz współrzędne bąbelka
	x = LIBELLA_BOK / 2 - 5 * RAD2DEG * fKatAkcelY;
	y = (DISP_Y_SIZE - LIBELLA_BOK) + LIBELLA_BOK / 2 - 5 * RAD2DEG * fKatAkcelX;

	//ogranicz ruch poza skalę
	if (x > LIBELLA_BOK - BABEL)
		x = LIBELLA_BOK - BABEL;
	if (y > DISP_Y_SIZE - BABEL)
		y = DISP_Y_SIZE - BABEL;
	if (x < BABEL)
		x = BABEL;
	if (y < DISP_Y_SIZE - LIBELLA_BOK + BABEL)
		y = DISP_Y_SIZE - LIBELLA_BOK + BABEL;

	if ((x != x2) || (y != y2))
	{
		//zamaż stary bąbelek kolorem tła
		setColor(LIBELLA1);
		RysujKolo(x2, y2, BABEL);

		//rysuj bąbelek
		setColor(LIBELLA3);
		RysujKolo(x, y, BABEL-2);
		setColor(LIBELLA2);
		RysujOkrag(x, y, BABEL-1);	//obwódka bąbelka

		x2 = x;
		y2 = y;

		//skala 5°
		setColor(CZARNY);
		RysujOkrag(LIBELLA_BOK / 2, DISP_Y_SIZE - LIBELLA_BOK / 2, 50);

		//skala 10°
		setColor(CZARNY);
		RysujOkrag(LIBELLA_BOK / 2, DISP_Y_SIZE - LIBELLA_BOK / 2, 75);

		//skala 15°
		setColor(CZARNY);
		RysujOkrag(LIBELLA_BOK / 2, DISP_Y_SIZE - LIBELLA_BOK / 2, 100);

		RysujLiniePozioma(0, DISP_Y_SIZE - LIBELLA_BOK/2, LIBELLA_BOK);
		RysujLiniePionowa(LIBELLA_BOK / 2, DISP_Y_SIZE - LIBELLA_BOK, LIBELLA_BOK);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje przycisk ekranowy z napisem
// Parametry: prost - współrzędne przycisku
// chNapis - tekst do wymisania na środku
// chCzynnosc - definiuje operacje: RYSUJ - pełne narysowanie przycisku, lub ODSWIEZ - podmianę samego napisu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujPrzycisk(prostokat_t prost, char *chNapis, uint8_t chCzynnosc)
{
	uint8_t chRozmiar;

	chRozmiar = strlen(chNapis);
	if (chCzynnosc == RYSUJ)
	{
		//setColor(SZARY40);
		//fillRect(prost.sX1, prost.sY1 ,prost.sX2, prost.sY2);	//rysuj przycisk
		RysujProstokatWypelniony(prost.sX1, prost.sY1, prost.sX2-prost.sX1, prost.sY2-prost.sY1, SZARY40);
	}

	setColor(ZOLTY);
	setBackColor(SZARY40);	//kolor tła napisu kolorem przycisku
	UstawCzcionke(BigFont);
	RysujNapis(chNapis, prost.sX1 + (prost.sX2 - prost.sX1)/2 - chRozmiar*FONT_BL/2 , prost.sY1 + (prost.sY2 - prost.sY1)/2 - FONT_BH/2);
	setBackColor(CZARNY);
	UstawCzcionke(MidFont);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran dla kalibracji zera (offsetu) magnetometru. Dla obrotu wokół X rysuje wykres YZ, dla obrotu wokół Y rysuje XZ dla Z rysuje XY
// Analizuje dane przekazane w zmiennej uDaneCM4.dane.uRozne.f32[6] przez CM4. Funkcja znajjduje i sygnalizuje znalezienia ich maksimów poprzez komunikaty głosowe: 1-minX, 2-maxX, 3-minY, 4-maxY, 5-minZ, 6-maxZ
// Wykres jest skalowany do maksimum wartosci bezwzględnej obu zmiennych
// Parametry: *chEtap	- wskaźnik na zmienną wskazującą na kalibracje konktretnej osi w konkretnym magnetometrze. Starszy półbajt koduje numer magnetometru, najmłodsze 2 bity oś obracaną a bit 4 procedurę kalibracji
// uDaneCM4.dane.uRozne.f32[6] - minima i maksima magnetometrów zbierana przez CM4
// Zwraca: ERR_GOTOWE / BLAD_OK - informację o tym czy wyjść z trybu kalibracji czy nie
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibracjaZeraMagnetometru(uint8_t *chEtap)
{
	char chNazwaOsi;
	uint8_t chErr = BLAD_OK;
	float fWspSkal;		//współczynnik skalowania wykresu
	float fMag[3];		//dane bieżącego magnetometru
	uint16_t sX, sY;	//bieżace współrzędne na wykresie
	static uint16_t sPoprzedX, sPoprzedY;	//poprzednie współrzędne na wykresie
	static float fMin[3], fMax[3];	//minimum i  maksimum wskazań
	float fAbsMax;		//maksimum z wartości bezwzględnej minimum i maksimum
	uint8_t chCzujnik;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		if (*chEtap & KALIBRUJ)
			sprintf(chNapis, "%s %s %d", chNapisLcd[STR_KALIBRACJA], chNapisLcd[STR_MAGNETOMETRU], (*chEtap & MASKA_CZUJNIKA)>>4);
		else
			sprintf(chNapis, "%s %s %d", chNapisLcd[STR_WERYFIKACJA], chNapisLcd[STR_MAGNETOMETRU], (*chEtap & MASKA_CZUJNIKA)>>4);
		BelkaTytulu(chNapis);

		setColor(SZARY80);
		sprintf(chNapis, "%s X:", chNapisLcd[STR_MAGN]);
		RysujNapis(chNapis, KOL12, 80);
		sprintf(chNapis, "%s Y:", chNapisLcd[STR_MAGN]);
		RysujNapis(chNapis, KOL12, 100);
		sprintf(chNapis, "%s Z:", chNapisLcd[STR_MAGN]);
		RysujNapis(chNapis, KOL12, 120);
		sprintf(chNapis, "Pochylenie:");
		RysujNapis(chNapis, KOL12, 140);
		sprintf(chNapis, "Przechylenie:");
		RysujNapis(chNapis, KOL12, 160);
		if (*chEtap & KALIBRUJ)
		{
			sprintf(chNapis, "%s X:", chNapisLcd[STR_EKSTREMA]);
			RysujNapis(chNapis, KOL12, 180);
			sprintf(chNapis, "%s Y:", chNapisLcd[STR_EKSTREMA]);
			RysujNapis(chNapis, KOL12, 200);
			sprintf(chNapis, "%s Z:", chNapisLcd[STR_EKSTREMA]);
			RysujNapis(chNapis, KOL12, 220);
		}
		else
		{
			sprintf(chNapis, "%s X:", chNapisLcd[STR_KAL]);
			RysujNapis(chNapis, KOL12, 180);
			sprintf(chNapis, "%s Y:", chNapisLcd[STR_KAL]);
			RysujNapis(chNapis, KOL12, 200);
			sprintf(chNapis, "%s Z:", chNapisLcd[STR_KAL]);
			RysujNapis(chNapis, KOL12, 220);
		}


		setColor(SZARY60);
		sprintf(chNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		RysujNapis(chNapis, CENTER, 45);

		setColor(SZARY40);
		stPrzycisk.sX1 = 10;
		stPrzycisk.sY1 = 250;
		stPrzycisk.sX2 = stPrzycisk.sX1 + 210;
		stPrzycisk.sY2 = DISP_Y_SIZE;
		RysujPrzycisk(stPrzycisk, "Nastepna Os", RYSUJ);

		setColor(SZARY40);
		stWykr.sX1 = DISP_X_SIZE - SZER_WYKR_MAG;
		stWykr.sY1 = DISP_Y_SIZE - SZER_WYKR_MAG;
		stWykr.sX2 = DISP_X_SIZE;
		stWykr.sY2 = DISP_Y_SIZE;
		RysujProstokat(stWykr.sX1, stWykr.sY1, stWykr.sX2, stWykr.sY2);	//rysuj ramkę wykresu
		sPoprzedX = stWykr.sX1 + SZER_WYKR_MAG/2;	//środek wykresu
		sPoprzedY = stWykr.sY1 + SZER_WYKR_MAG/2;
		chStanPrzycisku = 0;
		for (uint16_t n=0; n<3; n++)
		{
			if (*chEtap & KALIBRUJ)
				fMin[n] = fMax[n] = 0;		//podczas kalibracji zbieraj wartosci minimów i maksimów
			else
			{
				fMin[n] = -NOMINALNE_MAGN;
				fMax[n] =  NOMINALNE_MAGN;		//podczas sprawdzenia pracuj na stałym współczynniku
			}
		}
		*chEtap |= ZERUJ;	//przed pomiarem wystaw potrzebę wyzerowania znalezionych ekstemów magnetometru
		statusDotyku.chFlagi &= ~(DOTYK_ZWOLNONO | DOTYK_DOTKNIETO);	//czyść flagi ekranu dotykowego
	}

	if (*chEtap & ZERUJ)
	{
		uDaneCM7.dane.chWykonajPolecenie = POL_ZERUJ_EKSTREMA;
		DodajProbkeDoMalejKolejki(PRGA_00, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//komunikat "zero"
		if ( uDaneCM4.dane.chOdpowiedzNaPolecenie == POL_ZERUJ_EKSTREMA)
		{
			*chEtap &= ~ZERUJ;
			for (uint16_t n=0; n<3; n++)
			{
				fMin[n] = 0.0f;
				fMax[n] = 0.0f;
			}
		}
		else
			return BLAD_OK;	//nie idź dalej dopóki nie dostanie potwierdzenia że ekstrema zostały wyzerowane.
	}

	//pobierz dane z konkretnego magnetometru
	if ((*chEtap & MASKA_OSI) < 3)	//pobieraj dane tylko dla konkretnych osi. Gdy obsłużone wszystkie to zakończ
	{
		switch (*chEtap & MASKA_CZUJNIKA)		//rodzaj magnetometru
		{
		case MAG1:
			if (*chEtap & KALIBRUJ)
				uDaneCM7.dane.chWykonajPolecenie = POL_KAL_ZERO_MAGN1;
			else
				uDaneCM7.dane.chWykonajPolecenie = POL_POBIERZ_KONF_MAGN1;

			for (uint16_t n=0; n<3; n++)
				fMag[n] = uDaneCM4.dane.fMagne1[n];
			break;

		case MAG2:
			if (*chEtap & KALIBRUJ)
				uDaneCM7.dane.chWykonajPolecenie = POL_KAL_ZERO_MAGN2;
			else
				uDaneCM7.dane.chWykonajPolecenie = POL_POBIERZ_KONF_MAGN2;
			for (uint16_t n=0; n<3; n++)
				fMag[n] = uDaneCM4.dane.fMagne2[n];
			break;

		case MAG3:
			if (*chEtap & KALIBRUJ)
				uDaneCM7.dane.chWykonajPolecenie = POL_KAL_ZERO_MAGN3;
			else
				uDaneCM7.dane.chWykonajPolecenie = POL_POBIERZ_KONF_MAGN3;
			for (uint16_t n=0; n<3; n++)
				fMag[n] = uDaneCM4.dane.fMagne3[n];
			break;
		}
	}

	//znajdowanie ekstremów, rysowanie wykresu i generowanie komunikatów głosowych
	switch (*chEtap & MASKA_OSI)		//rodzaj osi
	{
	case 0:	chNazwaOsi = 'X';
		if (*chEtap & KALIBRUJ)
		{
			if (uDaneCM4.dane.uRozne.f32[2] < fMin[1])	//minimum Y
			{
				fMin[1] = uDaneCM4.dane.uRozne.f32[2];
				DodajProbkeDoMalejKolejki(PRGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
				DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[3] > fMax[1])	//maksimum Y
			{
				fMax[1] = uDaneCM4.dane.uRozne.f32[3];
				DodajProbkeDoMalejKolejki(PRGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
				DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
			}
			else
			if (uDaneCM4.dane.uRozne.f32[4] < fMin[2])	//minimum Z
			{
				fMin[2] = uDaneCM4.dane.uRozne.f32[4];
				DodajProbkeDoMalejKolejki(PRGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
				DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[5] > fMax[2])	//maksimum Z
			{
				fMax[2] = uDaneCM4.dane.uRozne.f32[5];
				DodajProbkeDoMalejKolejki(PRGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
				DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
			}

			//obsłuż ekstrema w osi X ale bez komunkatów o ekstremach
			if (uDaneCM4.dane.uRozne.f32[0] < fMin[0])	//minimum X
				fMin[0] = uDaneCM4.dane.uRozne.f32[0];
			if (uDaneCM4.dane.uRozne.f32[1] > fMax[0])	//maksimum X
				fMax[0] = uDaneCM4.dane.uRozne.f32[1];

			fAbsMax = MaximumGlobalne(fMin, fMax);	//znajdź maksimum globalne wszystkich osi
		}
		else
			fAbsMax = NOMINALNE_MAGN;

		//rysuj wykres biegunowy Y-Z
		if (fAbsMax > MIN_MAG_WYKR)
		{
			fWspSkal = (float)(SZER_WYKR_MAG / 2.0f) / fAbsMax;
			sX = (int16_t)(fMag[1] * fWspSkal) + stWykr.sX1 + SZER_WYKR_MAG/2;
			sY = (int16_t)(fMag[2] * fWspSkal) + stWykr.sY1 + SZER_WYKR_MAG/2;
			if ((sX > stWykr.sX1) && (sX < stWykr.sX2) && (sY > stWykr.sY1) && (sY < stWykr.sY2) && ((sX != sPoprzedX) && (sY != sPoprzedY)))
			{
				setColor(KOLOR_X);
				RysujLinie(sX, sY, sPoprzedX, sPoprzedY);
				sPoprzedX = sX;
				sPoprzedY = sY;
			}
		}
		break;

	case 1: chNazwaOsi = 'Y';
		if (*chEtap & KALIBRUJ)
		{
			if (uDaneCM4.dane.uRozne.f32[0] < fMin[0])	//minimum X
			{
				fMin[0] = uDaneCM4.dane.uRozne.f32[0];
				DodajProbkeDoMalejKolejki(PRGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
				DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[1] > fMax[0])	//maksimum X
			{
				fMax[0] = uDaneCM4.dane.uRozne.f32[1];
				DodajProbkeDoMalejKolejki(PRGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
				DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
			}
			else
			if (uDaneCM4.dane.uRozne.f32[4] < fMin[2])	//minimum Z
			{
				fMin[2] = uDaneCM4.dane.uRozne.f32[4];
				DodajProbkeDoMalejKolejki(PRGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
				DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[5] > fMax[2])	//maksimum Z
			{
				fMax[2] = uDaneCM4.dane.uRozne.f32[5];
				DodajProbkeDoMalejKolejki(PRGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
				DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
			}
			fAbsMax = MaximumGlobalne(fMin, fMax);	//znajdź maksimum globalne wszystkich osi
		}
		else
			fAbsMax = NOMINALNE_MAGN;

		//rysuj wykres biegunowy X-Z
		if (fAbsMax > MIN_MAG_WYKR)
		{
			fWspSkal = (float)(SZER_WYKR_MAG / 2.0f) / fAbsMax;
			sX = (int16_t)(fMag[0] * fWspSkal) + stWykr.sX1 + SZER_WYKR_MAG/2;
			sY = (int16_t)(fMag[2] * fWspSkal) + stWykr.sY1 + SZER_WYKR_MAG/2;
			if ((sX > stWykr.sX1) && (sX < stWykr.sX2) && (sY > stWykr.sY1) && (sY < stWykr.sY2) && ((sX != sPoprzedX) && (sY != sPoprzedY)))
			{
				setColor(KOLOR_Y);
				RysujLinie(sX, sY, sPoprzedX, sPoprzedY);
				sPoprzedX = sX;
				sPoprzedY = sY;
			}
		}
		break;

	case 2: chNazwaOsi = 'Z';
		if (*chEtap & KALIBRUJ)
		{
			if (uDaneCM4.dane.uRozne.f32[0] < fMin[0])	//minimum X
			{
				fMin[0] = uDaneCM4.dane.uRozne.f32[0];
				DodajProbkeDoMalejKolejki(PRGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
				DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[1] > fMax[0])	//maksimum X
			{
				fMax[0] = uDaneCM4.dane.uRozne.f32[1];
				DodajProbkeDoMalejKolejki(PRGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
				DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
			}
			else
			if (uDaneCM4.dane.uRozne.f32[2] < fMin[1])	//minimum Y
			{
				fMin[1] = uDaneCM4.dane.uRozne.f32[2];
				DodajProbkeDoMalejKolejki(PRGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
				DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[3] > fMax[1])	//maksimum Y
			{
				fMax[1] = uDaneCM4.dane.uRozne.f32[3];
				DodajProbkeDoMalejKolejki(PRGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
				DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
			}
			fAbsMax = MaximumGlobalne(fMin, fMax);	//znajdź maksimum globalne wszystkich osi
		}
		else
			fAbsMax = NOMINALNE_MAGN;

		//rysuj wykres biegunowy X-Y
		if (fAbsMax > MIN_MAG_WYKR)
		{
			fWspSkal = (float)(SZER_WYKR_MAG / 2.0f) / fAbsMax;
			sX = (int16_t)(fMag[0] * fWspSkal) + stWykr.sX1 + SZER_WYKR_MAG/2;
			sY = (int16_t)(fMag[1] * fWspSkal) + stWykr.sY1 + SZER_WYKR_MAG/2;
			if ((sX > stWykr.sX1) && (sX < stWykr.sX2) && (sY > stWykr.sY1) && (sY < stWykr.sY2) && ((sX != sPoprzedX) && (sY != sPoprzedY)))
			{
				setColor(KOLOR_Z);
				RysujLinie(sX, sY, sPoprzedX, sPoprzedY);
				sPoprzedX = sX;
				sPoprzedY = sY;
			}
		}
		break;
	}

	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f [uT] ", fMag[0]*1e6);
	RysujNapis(chNapis, KOL12 + 8*FONT_SL, 80);
	sprintf(chNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[0], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12 + 12*FONT_SL, 140);
	if (*chEtap & KALIBRUJ)
	{
		sprintf(chNapis, "%.2f, %.2f ", uDaneCM4.dane.uRozne.f32[0]*1e6, uDaneCM4.dane.uRozne.f32[1]*1e6);	//ekstrema X
		RysujNapis(chNapis, KOL12 + 12*FONT_SL, 180);
	}
	else
	{
		sprintf(chNapis, "%.3e, %.4f ", uDaneCM4.dane.uRozne.f32[0], uDaneCM4.dane.uRozne.f32[1]);	//przesunięcie i skalowanie X
		RysujNapis(chNapis, KOL12 + 7*FONT_SL, 180);
	}

	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f [uT] ", fMag[1]*1e6);
	RysujNapis(chNapis, 10 + 8*FONT_SL, 100);
	sprintf(chNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[1], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12 + 14*FONT_SL, 160);
	if (*chEtap & KALIBRUJ)
	{
		sprintf(chNapis, "%.2f, %.2f ", uDaneCM4.dane.uRozne.f32[2]*1e6, uDaneCM4.dane.uRozne.f32[3]*1e6);	//ekstrema Y
		RysujNapis(chNapis, KOL12 + 12*FONT_SL, 200);
	}
	else
	{
		sprintf(chNapis, "%.3e, %.4f ", uDaneCM4.dane.uRozne.f32[2], uDaneCM4.dane.uRozne.f32[3]);		//przesunięcie i skalowanie Y
		RysujNapis(chNapis, KOL12 + 7*FONT_SL, 200);
	}

	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f [uT] ", fMag[2]*1e6);
	RysujNapis(chNapis, 10 + 8*FONT_SL, 120);
	if (*chEtap & KALIBRUJ)
	{
		sprintf(chNapis, "%.2f, %.2f ", uDaneCM4.dane.uRozne.f32[4]*1e6, uDaneCM4.dane.uRozne.f32[5]*1e6);	//ekstrema Z
		RysujNapis(chNapis, KOL12 + 12*FONT_SL, 220);
	}
	else
	{
		sprintf(chNapis, "%.3e, %.4f ", uDaneCM4.dane.uRozne.f32[4], uDaneCM4.dane.uRozne.f32[5]);		//przesunięcie i skalowanie Z
		RysujNapis(chNapis, KOL12 + 7*FONT_SL, 220);
	}

	//sprawdź czy jest naciskany przycisk
	if (statusDotyku.chFlagi & DOTYK_DOTKNIETO)
	{
		//czy naciśnięto na przycisk?
		if ((statusDotyku.sY > stPrzycisk.sY1) && (statusDotyku.sY < stPrzycisk.sY2) && (statusDotyku.sX > stPrzycisk.sX1) && (statusDotyku.sX < stPrzycisk.sX2))
			chStanPrzycisku = 1;
		else
		{
			chErr = ERR_GOTOWE;	//zakończ kabrację gdy nacięnięto poza przyciskiem
			uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//neutralne polecenie kończy szukanie ekstremów w CM4
		}
		statusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;
	}
	else	//DOTYK_DOTKNIETO
	{
		if (chStanPrzycisku)	//został naciśniety
		{
			(*chEtap)++;
			chStanPrzycisku = 0;	//usuń ststus naciśnięcia
			sPoprzedX = stWykr.sX1 + SZER_WYKR_MAG/2;	//środek wykresu
			sPoprzedY = stWykr.sY1 + SZER_WYKR_MAG/2;
			DodajProbkeDoMalejKolejki(PRGA_PRZYCISK, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//komunikat audio naciśnięcia przycisku
		}
	}

	chCzujnik = ((*chEtap & MASKA_CZUJNIKA) >> 4) - 1;	//oblicz indeks czujnika
	if ((*chEtap & MASKA_OSI) == 3)	//czy obsłużone już wszystkie 3 osie i jesteśmy w trybie kalibracji?
	{
		if (*chEtap & KALIBRUJ)
			uDaneCM7.dane.chWykonajPolecenie = POL_ZAPISZ_KONF_MAGN1 + chCzujnik;	//zapisz konfigurację bieżącego magnetometru
		else
			chErr = ERR_GOTOWE;		//koniec wertyfikacji
	}

	//wyjdź dopiero gdy dostanie potwierdzenie zapisu
	if (((*chEtap & MASKA_OSI) == 3) && (uDaneCM4.dane.chOdpowiedzNaPolecenie == POL_ZAPISZ_KONF_MAGN1 + chCzujnik))
	{
		uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//neutralne polecenie kończy szukanie ekstremów w CM4
		chErr = ERR_GOTOWE;		//koniec kalibracji
		DodajProbkeDoMalejKolejki(PRGA_GOTOWE, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//komunikat kończący: Gotowe
	}

	setColor(ZOLTY);
	sprintf(chNapis, "Ustaw UAV osi%c %c w kier. Wsch%cd-Zach%cd i obracaj wok%c%c %c", ą, chNazwaOsi, ó, ó, ó, ł, chNazwaOsi);
	RysujNapis(chNapis, CENTER, 25);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Znajduje maksimum globalne wszystkich osi
// Parametry: *sMin, *sMax - wskaźniki na minima i maksima wszystkich 3 osi
// Zwraca: globalne maksimum będące wartoscią bezwzgledną wszyskich danych wejściowych
////////////////////////////////////////////////////////////////////////////////
float MaximumGlobalne(float* fMin, float* fMax)
{
	float fMaxGlob = 0;

	for (uint16_t n=0; n<3; n++)
	{
		if (fMaxGlob < *(fMax + n))
			fMaxGlob = *(fMax + n);
		if (fMaxGlob < fabsf(*(fMin + n)))
			fMaxGlob = fabsf(*(fMin + n));
	}
	return fMaxGlob;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran kalibracji barometru
// Parametry: *chEtap - wskaźnik na zmienną zawierającą bieżący etap procesu kalibracji
// uDaneCM4.dane.uRozne.fRozne[0] - pierwsze uśrednione ciśnienie czujnika 1
// uDaneCM4.dane.uRozne.fRozne[1] - pierwsze uśrednione ciśnienie czujnika 2
// uDaneCM4.dane.uRozne.fRozne[2] - drugie uśrednione ciśnienie czujnika 1
// uDaneCM4.dane.uRozne.fRozne[3] - drugie uśrednione ciśnienie czujnika 2
// uDaneCM4.dane.uRozne.fRozne[4] - współczynnik skalowania czujnika 1
// uDaneCM4.dane.uRozne.fRozne[5] - współczynnik skalowania czujnika 2
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibrujBaro(uint8_t *chEtap)
{
	uint8_t chErr = BLAD_OK;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Kalibr. pomiaru cisnienia");

		setColor(ZOLTY);
		sprintf(chNapis, "U%crednij ci%cn. pocz. Pokonaj wys=27m i ponownie u%crednij", ś, ś, ś);
		RysujNapis(chNapis, CENTER, 30);

		setColor(SZARY60);
		sprintf(chNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		RysujNapis(chNapis, CENTER, 50);

		setColor(SZARY80);
		sprintf(chNapis, "Czujnik 1");
		RysujNapis(chNapis, KOL12 + 15*FONT_SL, 80);
		sprintf(chNapis, "Czujnik 2");
		RysujNapis(chNapis, KOL12 + 30*FONT_SL, 80);

		sprintf(chNapis, "Biezace cisn:");
		RysujNapis(chNapis, KOL12, 100);
		sprintf(chNapis, "Sredn cisn 1:");
		RysujNapis(chNapis, KOL12, 120);
		sprintf(chNapis, "Sredn.cisn 2:");
		RysujNapis(chNapis, KOL12, 140);
		sprintf(chNapis, "Skalowanie:");
		RysujNapis(chNapis, KOL12, 160);


		setColor(SZARY40);
		stPrzycisk.sX1 = 10;
		stPrzycisk.sY1 = 210;
		stPrzycisk.sX2 = stPrzycisk.sX1 + 250;
		stPrzycisk.sY2 = DISP_Y_SIZE - WYS_PASKA_POSTEPU - 1;
		RysujPrzycisk(stPrzycisk, "Start", RYSUJ);
		chStanPrzycisku = 0;
		*chEtap = 0;
	}

	// Zwraca: kod błędu
	setColor(BIALY);
	sprintf(chNapis, "%.0f Pa", uDaneCM4.dane.fCisnieBzw[0]);
	RysujNapis(chNapis, KOL12 + 14*FONT_SL, 100);
	sprintf(chNapis, "%.0f Pa", uDaneCM4.dane.fCisnieBzw[1]);
	RysujNapis(chNapis, KOL12 + 30*FONT_SL, 100);
	sprintf(chNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[0]);	//pierwsze uśrednione ciśnienie czujnika 1
	RysujNapis(chNapis, KOL12 + 14*FONT_SL, 120);
	sprintf(chNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[1]);	//pierwsze uśrednione ciśnienie czujnika 2
	RysujNapis(chNapis, KOL12 + 30*FONT_SL, 120);
	sprintf(chNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[2]);	//drugie uśrednione ciśnienie czujnika 1
	RysujNapis(chNapis, KOL12 + 14*FONT_SL, 140);
	sprintf(chNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[3]);	//drugie uśrednione ciśnienie czujnika 2
	RysujNapis(chNapis, KOL12 + 30*FONT_SL, 140);
	sprintf(chNapis, "%.6f ", uDaneCM4.dane.uRozne.f32[4]);	//współczynnik skalowania czujnika 1
	RysujNapis(chNapis, KOL12 + 14*FONT_SL, 160);
	sprintf(chNapis, "%.6f ", uDaneCM4.dane.uRozne.f32[5]);	//współczynnik skalowania czujnika 2
	RysujNapis(chNapis, KOL12 + 30*FONT_SL, 160);

	switch (*chEtap)
	{
	case 0:	//czyść zmienne robocze, załaduj bieżacy współczynnik
		uDaneCM7.dane.chWykonajPolecenie = POL_INICJUJ_USREDN;	//zeruj licznik uśredniania
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == POL_INICJUJ_USREDN)
			(*chEtap)++;
		break;

	case 1:
	case 3:
		uDaneCM7.dane.chWykonajPolecenie = POL_ZERUJ_LICZNIK;	//zeruj licznik uśredniania
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == POL_ZERUJ_LICZNIK)
		{
			sprintf(chNapis, "Usrednij %d", (*chEtap) / 2 + 1);
			RysujPrzycisk(stPrzycisk, chNapis, ODSWIEZ);
			if (chStanPrzycisku == 1)
				(*chEtap)++;	//wyzerowało się wiec przejdź do nastepnego etapu
		}
		break;

	case 2:
		uDaneCM7.dane.chWykonajPolecenie = POL_USREDNIJ_CISN1;	//trwa uśrednianie ciśnienia 1
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_GOTOWE)
			(*chEtap)++;
		break;

	case 4:
		uDaneCM7.dane.chWykonajPolecenie = POL_USREDNIJ_CISN2;	//trwa uśrednianie ciśnienia 2
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_GOTOWE)
			(*chEtap)++;
		break;

	case 5:
		uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//Zakończ wykonywanie poleceń kalibracyjnych
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZLE_OBLICZENIA)
			RysujPrzycisk(stPrzycisk, "Blad!", ODSWIEZ);
		else
			RysujPrzycisk(stPrzycisk, "Gotowe", ODSWIEZ);
		sprintf(chNapis, "dP1 = %.2f Pa", fabs(uDaneCM4.dane.uRozne.f32[0] - uDaneCM4.dane.uRozne.f32[2]));	//Rożnica ciśnień czujnika 1
		RysujNapis(chNapis, 10 + stPrzycisk.sX2, 240);
		sprintf(chNapis, "dP2 = %.2f Pa", fabs(uDaneCM4.dane.uRozne.f32[1] - uDaneCM4.dane.uRozne.f32[3]));	//Rożnica ciśnień czujnika 2
		RysujNapis(chNapis, 10 + stPrzycisk.sX2, 260);
		if (chStanPrzycisku == 1)
			chErr = ERR_GOTOWE;	//zakończ
		break;
	}

	//sprawdź czy jest naciskany przycisk
	if (statusDotyku.chFlagi & DOTYK_DOTKNIETO)
	{
		//czy naciśnięto na przycisk?
		if ((statusDotyku.sY > stPrzycisk.sY1) && (statusDotyku.sY < stPrzycisk.sY2) && (statusDotyku.sX > stPrzycisk.sX1) && (statusDotyku.sX < stPrzycisk.sX2))
		{
			chStanPrzycisku = 1;
			DodajProbkeDoMalejKolejki(PRGA_PRZYCISK, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//komunikat audio naciśnięcia przycisku
		}
		else
		{
			chErr = ERR_GOTOWE;	//zakończ kabrację gdy nacięnięto poza przyciskiem
			uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//neutralne polecenie
		}
		statusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;
	}
	else	//DOTYK_DOTKNIETO
	{
		if (chStanPrzycisku)
			chStanPrzycisku = 0;
	}

	RysujPasekPostepu(CZAS_KALIBRACJI);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran płaskiego obrotu wszystkich dostępnych magnetometrów
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PlaskiObrotMagnetometrow(void)
{
	static uint16_t sPoprzed1X, sPoprzed1Y;	//poprzednie współrzędne magnetometru 1
	static uint16_t sPoprzed2X, sPoprzed2Y;	//poprzednie współrzędne magnetometru 2
	static uint16_t sPoprzed3X, sPoprzed3Y;	//poprzednie współrzędne magnetometru 3
	uint16_t sX, sY;
	float fWspSkal;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		sprintf(chNapis, "%s 2D %s", chNapisLcd[STR_WERYFIKACJA], chNapisLcd[STR_MAGNETOMETRU]);
		BelkaTytulu(chNapis);

		setColor(SZARY80);
		sprintf(chNapis, "Przechylenie:");
		RysujNapis(chNapis, KOL12, 80);
		sprintf(chNapis, "Pochylenie:");
		RysujNapis(chNapis, KOL12, 100);

		sprintf(chNapis, "%s 1:", chNapisLcd[STR_MAGN]);
		RysujNapis(chNapis, KOL12, 120);
		sprintf(chNapis, "%s 2:", chNapisLcd[STR_MAGN]);
		RysujNapis(chNapis, KOL12, 140);
		sprintf(chNapis, "%s 3:", chNapisLcd[STR_MAGN]);
		RysujNapis(chNapis, KOL12, 160);

		sprintf(chNapis, "Inklinacja: %.2f %c", INKLINACJA_MAG * RAD2DEG, ZNAK_STOPIEN);
		RysujNapis(chNapis, KOL12, 180);
		sprintf(chNapis, "Deklinacja: %.2f %c", DEKLINACJA_MAG * RAD2DEG, ZNAK_STOPIEN);
		RysujNapis(chNapis, KOL12, 200);

		setColor(SZARY60);
		sprintf(chNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		RysujNapis(chNapis, CENTER, 30);

		setColor(SZARY40);
		stWykr.sX1 = DISP_X_SIZE - SZER_WYKR_MAG;
		stWykr.sY1 = DISP_Y_SIZE - SZER_WYKR_MAG;
		stWykr.sX2 = DISP_X_SIZE;
		stWykr.sY2 = DISP_Y_SIZE;
		RysujProstokat(stWykr.sX1, stWykr.sY1, stWykr.sX2, stWykr.sY2);	//rysuj ramkę wykresu
		sPoprzed1X = sPoprzed2X = sPoprzed3X = stWykr.sX1 + SZER_WYKR_MAG/2;	//środek wykresu
		sPoprzed1Y = sPoprzed2Y = sPoprzed3Y = stWykr.sY1 + SZER_WYKR_MAG/2;
		RysujOkrag(stWykr.sX1 + SZER_WYKR_MAG/2, stWykr.sY1 + SZER_WYKR_MAG/2, PROMIEN_RZUTU_MAGN);

		//legenda kierunków  magnetycznych
		setColor(SZARY80);
		RysujZnak('N', stWykr.sX1 + SZER_WYKR_MAG - 20, stWykr.sY1 + SZER_WYKR_MAG/2);
		RysujZnak('E', stWykr.sX1 + SZER_WYKR_MAG/2, stWykr.sY1 + SZER_WYKR_MAG - 20);
		RysujZnak('S', stWykr.sX1 + 20, stWykr.sY1 + SZER_WYKR_MAG/2);
		RysujZnak('W', stWykr.sX1 + SZER_WYKR_MAG/2, stWykr.sY1 + 20 - FONT_SH);
	}

	//Promień okregu czyli SZER_WYKR_MAG/2 * cosf(INKLINACJA_MAG) ma odpowiadać płaskiemu rzutowi długosci wektora czyli NOMINALNE_MAGN * cosf(INKLINACJA_MAG)
	//Ponieważ to jest dosyć małe, więc powiększam dwukrotnie i po skróceniu wyglada tak:
	//fWspSkal = (float)(2*SZER_WYKR_MAG * cosf(INKLINACJA_MAG)) / (NOMINALNE_MAGN * cosf(INKLINACJA_MAG));	//za duże
	//fWspSkal = (float)SZER_WYKR_MAG / NOMINALNE_MAGN;	//za małe
	fWspSkal = (float)PROMIEN_RZUTU_MAGN / (NOMINALNE_MAGN * cosf(INKLINACJA_MAG));

	//rysuj wykres X-Y magnetometru 1
	sX = (int16_t)(uDaneCM4.dane.fMagne1[0] * fWspSkal) + stWykr.sX1 + SZER_WYKR_MAG/2;
	sY = (int16_t)(uDaneCM4.dane.fMagne1[1] * fWspSkal) + stWykr.sY1 + SZER_WYKR_MAG/2;
	if ((sX > stWykr.sX1) && (sX < stWykr.sX2) && (sY > stWykr.sY1) && (sY < stWykr.sY2) && ((sX != sPoprzed1X) && (sY != sPoprzed1Y)))
	{
		setColor(CYJAN);
		RysujLinie(sX, sY, sPoprzed1X, sPoprzed1Y);
		sPoprzed1X = sX;
		sPoprzed1Y = sY;
	}

	//rysuj wykres X-Y magnetometru 2
	sX = (int16_t)(uDaneCM4.dane.fMagne2[0] * fWspSkal) + stWykr.sX1 + SZER_WYKR_MAG/2;
	sY = (int16_t)(uDaneCM4.dane.fMagne2[1] * fWspSkal) + stWykr.sY1 + SZER_WYKR_MAG/2;
	if ((sX > stWykr.sX1) && (sX < stWykr.sX2) && (sY > stWykr.sY1) && (sY < stWykr.sY2) && ((sX != sPoprzed2X) && (sY != sPoprzed2Y)))
	{
		setColor(MAGENTA);
		RysujLinie(sX, sY, sPoprzed2X, sPoprzed2Y);
		sPoprzed2X = sX;
		sPoprzed2Y = sY;
	}

	//rysuj wykres X-Y magnetometru 3
	sX = (int16_t)(uDaneCM4.dane.fMagne3[0] * fWspSkal) + stWykr.sX1 + SZER_WYKR_MAG/2;
	sY = (int16_t)(uDaneCM4.dane.fMagne3[1] * fWspSkal) + stWykr.sY1 + SZER_WYKR_MAG/2;
	if ((sX > stWykr.sX1) && (sX < stWykr.sX2) && (sY > stWykr.sY1) && (sY < stWykr.sY2) && ((sX != sPoprzed3X) && (sY != sPoprzed3Y)))
	{
		setColor(ZOLTY);
		RysujLinie(sX, sY, sPoprzed3X, sPoprzed3Y);
		sPoprzed3X = sX;
		sPoprzed3Y = sY;
	}

	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[0], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12 + 14*FONT_SL, 80);

	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[1], ZNAK_STOPIEN);
	RysujNapis(chNapis, KOL12 + 12*FONT_SL, 100);


	setColor(CYJAN);
	sprintf(chNapis, "%.2f, %.2f [uT] ", uDaneCM4.dane.fMagne1[0]*1000, uDaneCM4.dane.fMagne1[1]*1e6f);
	RysujNapis(chNapis, KOL12 + 8*FONT_SL, 120);

	setColor(MAGENTA);
	sprintf(chNapis, "%.2f, %.2f [uT] ", uDaneCM4.dane.fMagne2[0]*1000, uDaneCM4.dane.fMagne2[1]*1e6f);
	RysujNapis(chNapis, KOL12 + 8*FONT_SL, 140);

	setColor(ZOLTY);
	sprintf(chNapis, "%.2f, %.2f [uT] ", uDaneCM4.dane.fMagne3[0]*1000, uDaneCM4.dane.fMagne3[1]*1e6f);
	RysujNapis(chNapis, KOL12 + 8*FONT_SL, 160);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran Nastaw regulatora PID
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void NastawyPID(uint8_t chKanal)
{
	float fNastawy[ROZMIAR_REG_PID/4];
	uint8_t chErr;
	un8_32_t un8_32;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		sprintf(chNapis, "%s %s", chNapisLcd[STR_NASTAWY_PID], chNapisLcd[STR_PRZECHYLENIA + chKanal/2]);
		BelkaTytulu(chNapis);

		setColor(SZARY80);
		sprintf(chNapis, "Regulator g%c%cwny", ł, ó);
		RysujNapis(chNapis, KOL12, 60);
		sprintf(chNapis, "Kp:");
		RysujNapis(chNapis, KOL12, 80);
		sprintf(chNapis, "Ti:");
		RysujNapis(chNapis, KOL12, 100);
		sprintf(chNapis, "Td:");
		RysujNapis(chNapis, KOL12, 120);
		sprintf(chNapis, "Max I:");
		RysujNapis(chNapis, KOL12, 140);
		sprintf(chNapis, "Min Wy:");
		RysujNapis(chNapis, KOL12, 160);
		sprintf(chNapis, "Max Wy:");
		RysujNapis(chNapis, KOL12, 180);
		sprintf(chNapis, "Filtr D:");
		RysujNapis(chNapis, KOL12, 200);
		sprintf(chNapis, "k%ctowy:", ą);
		RysujNapis(chNapis, KOL12, 220);

		sprintf(chNapis, "Regulator pochodnej");
		RysujNapis(chNapis, KOL22, 60);
		sprintf(chNapis, "Kp:");
		RysujNapis(chNapis, KOL22, 80);
		sprintf(chNapis, "Ti:");
		RysujNapis(chNapis, KOL22, 100);
		sprintf(chNapis, "Td:");
		RysujNapis(chNapis, KOL22, 120);
		sprintf(chNapis, "Max I:");
		RysujNapis(chNapis, KOL22, 140);
		sprintf(chNapis, "Min Wy:");
		RysujNapis(chNapis, KOL22, 160);
		sprintf(chNapis, "Max Wy:");
		RysujNapis(chNapis, KOL22, 180);
		sprintf(chNapis, "Filtr D:");
		RysujNapis(chNapis, KOL22, 200);
		setColor(SZARY60);
		sprintf(chNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		RysujNapis(chNapis, CENTER, 30);

		//odczytaj nastawy regulatorów
		chErr = CzytajFram(FAU_PID_P0 + (chKanal + 0) * ROZMIAR_REG_PID, ROZMIAR_REG_PID/4, fNastawy);
		if (chErr == BLAD_OK)
		{
			un8_32.daneFloat = fNastawy[6];
			if (un8_32.dane8[0] & PID_WLACZONY)
				setColor(SZARY60);	//regulator wyłączony
			else
				setColor(BIALY);	//regulator pracuje

			sprintf(chNapis, "%.3f ", fNastawy[0]);	//Kp
			RysujNapis(chNapis, KOL12 + 4*FONT_SL, 80);
			sprintf(chNapis, "%.3f ", fNastawy[1]);	//Ti
			RysujNapis(chNapis, KOL12 + 4*FONT_SL, 100);
			sprintf(chNapis, "%.3f ", fNastawy[2]);	//Td
			RysujNapis(chNapis, KOL12 + 4*FONT_SL, 120);
			sprintf(chNapis, "%.3f ", fNastawy[3]);	//max całki
			RysujNapis(chNapis, KOL12 + 7*FONT_SL, 140);
			sprintf(chNapis, "%.3f ", fNastawy[4]);	//min wyjścia
			RysujNapis(chNapis, KOL12 + 8*FONT_SL, 160);
			sprintf(chNapis, "%.3f ", fNastawy[5]);	//max wyjścia
			RysujNapis(chNapis, KOL12 + 8*FONT_SL, 180);
			sprintf(chNapis, "%d", un8_32.dane8[0] & PID_MASKA_FILTRA_D);
			RysujNapis(chNapis, KOL12 + 9*FONT_SL, 200);	//filtr D
			if (un8_32.dane8[0] & PID_KATOWY)
				sprintf(chNapis, "Tak");
			else
				sprintf(chNapis, "Nie");
			RysujNapis(chNapis, KOL12 + 8*FONT_SL, 220);	//kątowy
		}
		else
			chRysujRaz = 1;	//jeżeli się nie odczytało to wyświetl jeszcze raz

		chErr = CzytajFram(FAU_PID_P0 + (chKanal + 1) * ROZMIAR_REG_PID, ROZMIAR_REG_PID/4, fNastawy);
		if (chErr == BLAD_OK)
		{
			un8_32.daneFloat = fNastawy[6];
			if (un8_32.dane8[0] & PID_WLACZONY)
				setColor(SZARY60);	//regulator wyłączony
			else
				setColor(BIALY);	//regulator pracuje

			sprintf(chNapis, "%.3f ", fNastawy[0]);	//Kp
			RysujNapis(chNapis, KOL22 + 4*FONT_SL, 80);
			sprintf(chNapis, "%.3f ", fNastawy[1]);	//Ti
			RysujNapis(chNapis, KOL22 + 4*FONT_SL, 100);
			sprintf(chNapis, "%.3f ", fNastawy[2]);	//Td
			RysujNapis(chNapis, KOL22 + 4*FONT_SL, 120);
			sprintf(chNapis, "%.3f ", fNastawy[3]);	//max całki
			RysujNapis(chNapis, KOL22 + 7*FONT_SL, 140);
			sprintf(chNapis, "%.3f ", fNastawy[4]);	//min wyjścia
			RysujNapis(chNapis, KOL22 + 8*FONT_SL, 160);
			sprintf(chNapis, "%.3f ", fNastawy[5]);	//max wyjścia
			RysujNapis(chNapis, KOL22 + 8*FONT_SL, 180);
			sprintf(chNapis, "%d", un8_32.dane8[0] & PID_MASKA_FILTRA_D);
			RysujNapis(chNapis, KOL22 + 9*FONT_SL, 200);	//filtr D
		}
		else
			chRysujRaz = 1;

		uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//nie odczytuj więcej danych
	}
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje zawartość pamieci FRAM z CM4
// Parametry:
// [we] sAdres - adres komórki pamieci FRAM
// [we] chRozmiar - ilość liczb float do odczytu
// [wy] *fDane - wskaźnik na odczytywaną strukturę danych float
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajFram(uint16_t sAdres, uint8_t chRozmiar, float* fDane)
{
	chTimeout = 10;
	uDaneCM7.dane.chRozmiar = chRozmiar;
	uDaneCM7.dane.sAdres = sAdres;
	uDaneCM7.dane.chWykonajPolecenie = POL_CZYTAJ_FRAM_FLOAT;
	do
	{
		osDelay(5);
		chTimeout--;
	}
	while ((uDaneCM4.dane.sAdres != uDaneCM7.dane.sAdres) && (chTimeout));
	for (uint8_t n=0; n<uDaneCM4.dane.chRozmiar; n++)
		*(fDane + n) = uDaneCM4.dane.uRozne.f32[n];

	if (chTimeout)
		return BLAD_OK;
	else
		return BLAD_TIMEOUT;
}



////////////////////////////////////////////////////////////////////////////////
// Wyświetla dane na wykresie 480 punktów. Bierze co drugą próbkę aby rozciagnać podstawę czasu
// Parametry:
// [we] *nDane - wskaźnik na dane do wyświetlenia
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void RysujPrzebieg(int16_t *sDaneKasowania, int16_t *sDaneRysowania, uint16_t sKolor)
{
	int16_t y1 = DISP_Y_SIZE/2, y2;
	int16_t nMin = 0x4FFF;//, nMax = -0x4FFF;

	if (sDaneKasowania)		//zamazuj poprzednie dane jeżeli jest podany wskaźnik na te dane
	{
		//znajdź ekstrema sygnału
		for (int16_t x=0; x<DISP_X_SIZE; x++)
		{
			if (*(sDaneKasowania + 2*x) < nMin)
				nMin = *(sDaneKasowania + 2*x);
		}
		setColor(CZARNY);
		for (int16_t x=0; x<480; x++)
		{
			y2 = (uint16_t)(*(sDaneKasowania + 2*x) - nMin);
			if (y2 > DISP_Y_SIZE / 2)	//ogranicz duże wartosci aby rysując nie mazało po pamieci ekranu
				y2 =  DISP_Y_SIZE / 2;
			if (y2 < -DISP_Y_SIZE/2)
				y2 = -DISP_Y_SIZE / 2;

			y2 += DISP_Y_SIZE / 2;
			RysujLinie(x, y1, x+1, y2);
			y1 = y2;
		}
	}

	setColor(sKolor);
	nMin = 0x4FFF;
	for (int16_t x=0; x<DISP_X_SIZE; x++)
	{
		if (*(sDaneRysowania + 2*x) < nMin)
			nMin = *(sDaneRysowania + 2*x);
	}

	for (int16_t x=0; x<480; x++)
	{
		y2 = (uint16_t)(*(sDaneRysowania + 2*x) - nMin);
		if (y2 > DISP_Y_SIZE / 2)	//ogranicz duże wartosci aby rysując nie mazało po pamieci ekranu
			y2 =  DISP_Y_SIZE / 2;
		if (y2 < -DISP_Y_SIZE/2)
			y2 = -DISP_Y_SIZE / 2;

		y2 += DISP_Y_SIZE / 2;
		RysujLinie(x, y1, x+1, y2);
		y1 = y2;
	}
}
