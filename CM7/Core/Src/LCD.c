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
#include <AnalizaDrgan.h>
#include <Audio.h>
#include <Bmp.h>
#include <Czas.h>
#include <Dotyk.h>
#include <Fraktale.h>
#include <Jpeg.h>
#include <Kamera.h>
#include <stdio.h>
#include <math.h>
#include <LCD/RPi35B_480x320.h>
#include <KonfigFram.h>
#include <LCD/ILI9488.h>
#include <Napisy.h>
#include <Pamiec.h>
#include <Rejestrator.h>
#include <string.h>
#include <W25Q128JV.h>
#include "FlashNOR.h"
#include "Wymiana.h"
#include "SampleAudio.h"
#include "ProtokolKomunikacyjny.h"
#include "Wspolne.h"
#include <stdlib.h>
#include <Telemetria.h>
#include "CAN.h"
#include "KanalyPID.h"
#include "AnalizaObrazu.h"
#include "ip4_addr.h"
#include "ff.h"
#include "lwip/stats.h"
#include "LCD/LCD_mem.h"
#include "OSD.h"

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
extern const unsigned short spectrum[0xFFC];

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
extern const unsigned short obr_pid[0xFFC];
extern const unsigned short obr_pid4[0xFFC];
extern const unsigned short obr_silnik[0xFFC];
extern const unsigned short obr_mikser[0xFFC];


extern const char *cNapisLcd[MAX_NAPISOW];
extern const char *cOpisBledow[MAX_KOMUNIKATOW];
extern const char *cNazwyMies3Lit[13];
extern int16_t sBuforPapuga[ROZMIAR_BUFORA_PAPUGI];
extern volatile uint8_t cCzasSwieceniaLED[LICZBA_LED];	//czas świecenia liczony w kwantach 0,1s jest zmniejszany w przerwaniu TIM17_IRQHandler

uint8_t cTrybPracy;
uint8_t cNowyTrybPracy;
uint8_t cWrocDoTrybu;
uint8_t cRysujRaz;
char cNapis[100], cNapisPodreczny[30];
float fTemperaturaKalibracji;
uint8_t chLiczIter;		//licznik iteracji wyświetlania
extern stStatusDotyku_t stStatusDotyku;
extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
extern uint8_t cPort_exp_wysylany[];
extern uint8_t cPort_exp_odbierany[];
//extern uint8_t cGlosnosc;		//regulacja głośności odtwarzania komunikatów w zakresie 0..SKALA_GLOSNOSCI
extern SD_HandleTypeDef hsd1;
extern uint8_t cStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
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
uint8_t cSekwencerKalibracji;		//wskazuje na daną oś jako kolejny etap kalibracji
prostokat_t stPrzycisk;
uint8_t cStanPrzycisku;
uint8_t cEtapKalibracji;
prostokat_t stWykr;	//wykres biegunowy magnetometru
uint8_t cHistR[ROZMIAR_HIST_KOLOR], cHistG[ROZMIAR_HIST_KOLOR], cHistB[ROZMIAR_HIST_KOLOR];
//uint8_t chHistCB8[ROZMIAR_HIST_CB8];

extern uint16_t sBuforKamery[ROZM_BUF_YUV420];
extern uint8_t cBuforLCD[DISP_X_SIZE * DISP_Y_SIZE * 3];
extern uint8_t cBuforOSD[DISP_X_SIZE * DISP_Y_SIZE * 3];	//pamięć obrazu OSD w formacie RGB888
//extern uint8_t cBuforJpeg[ILOSC_BUF_JPEG][ROZM_BUF_WY_JPEG];
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) cBuforTestowyDRAM[100];
extern stKonfKam_t stKonfKam;
extern uint32_t nRozmiarObrazuKamery;
extern stDiagKam_t stDiagKam;	//diagnostyka stanu kamery
extern JPEG_HandleTypeDef hjpeg;
extern uint32_t nRozmiarObrazuJPEG;	//w bajtach
extern const uint8_t chNaglJpegSOI[20];
extern const uint8_t cNaglJpegEOI[2];
extern FIL SDJpegFile;       //struktura pliku z obrazem
extern uint8_t cNazwaPlikuObrazu[DLG_NAZWY_PLIKU_OBR];	//początek nazwy pliku z obrazem, po tym jest data i czas

extern uint8_t cWskNapBufKam;	//wskaźnik napełnaniania bufora kamery
extern volatile uint8_t cObrazKameryGotowy;	//flaga gotowości obrazu, ustawiana w callbacku
uint8_t cTimeout;
extern stKonfOsd_t stKonfOSD;
extern volatile uint8_t chTrybPracyKamery;	//steruje co dalej robić z obrazem pozyskanym przez DCMI
uint32_t nCzas, nCzasHist;
extern uint32_t nCzasBlend, nCzasLCD;
extern stFFT_t stKonfigFFT;
extern float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fBuforPomiarow[LICZBA_WYKRESOW_FFT][FFT_MAX_ROZMIAR];	//bufor do zbierania danych wejściowych
extern float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fWynikFFT[LICZBA_TESTOW_FFT][LICZBA_ZMIENNYCH_FFT][FFT_MAX_ROZMIAR / 2];	//wartość sygnału wyjściowego
extern stZesp_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) stWejscie[FFT_MAX_ROZMIAR];		//zmiennna wejściowa
extern stZesp_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) xXomega[FFT_MAX_ROZMIAR];		//wynik transformaty
extern stIdentyfikacjaSilnikow_t stIdentSiln;

//Definicje ekranów menu
menu_t stMenuGlowne[MENU_WIERSZE * MENU_KOLUMNY] = {
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



menu_t stMenuKalibracje[MENU_WIERSZE * MENU_KOLUMNY] = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Kal IMU", 	"Kalibracja czujników IMU",					TP_KAL_IMU,			obr_zyroskop},
	{"Kal Magn", 	"Kalibracja magnetometrow",					TP_KAL_MAG,			obr_kal_mag_n1},
	{"Kal Baro", 	"Kalibracja cisnienia wg wzorca 10 pieter",	TP_KAL_BARO,		obr_cisnienie},
	{"Kal Dotyk", 	"Kalibracja panelu dotykowego na LCD",		TP_KAL_DOTYK,		obr_KonfigDotyk},
	{"nic",			"nic",										TP_KAL1,			obr_narzedzia},
	{"nic",			"nic",										TP_KAL1,			obr_narzedzia},
	{"nic",			"nic",										TP_KAL1,			obr_narzedzia},
	{"nic",			"nic",										TP_KAL1,			obr_narzedzia},
	{"HardFault",	"Genruje wystapiene HardFault",				TP_KAL_HARD_FAULT,	obr_narzedzia},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

menu_t stMenuPomiary[MENU_WIERSZE * MENU_KOLUMNY] = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Dane AHRS",	"Wyniki pomiarow IMU i obliczeń AHRS",		TP_POMIARY_AHRS, 	obr_multimetr},
	{"Czujniki",	"Wyniki pomiarow pozostalych czujnikow ",	TP_POMIARY_CZUJN, 	obr_multimetr},
	{"Odb RC",		"Dane z odbiornika RC",						TP_POMIARY_RC,		obr_aparaturaRC},
	{"Wyj RC",		"Dane na wyjściach RC: serwa, ESC",			TP_POMIARY_SERWA,	obr_aparaturaRC},
	{"FFT akcel",	"FFT akceleometrów",						TP_POMIARY_FFT_ACC,	spectrum},
	{"FFT zyro",	"FFT akceleometrów",						TP_POMIARY_FFT_ZYR,	spectrum},
	{"Drgania",		"Test rezonansu ramy",						TP_POMIARY_ANALIZA_DRGAN, spectrum},
	{"Startowy",	"Ekran startowy",							TP_WITAJ,			obr_kontrolny},
	{"TestDotyk",	"Testy panelu dotykowego",					TP_POMIARY_DOTYKU,	obr_dotyk_zolty},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

menu_t stMenuNastawy[MENU_WIERSZE * MENU_KOLUMNY] = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"PID Przech",	"Nastawy PID przechylenia",					TP_NAST_PID_PRZECH, obr_pid},
	{"PID Pochyl",	"Nastawy PID pochylenia",					TP_NAST_PID_POCH, 	obr_pid},
	{"PID Odchyl",	"Nastawy PID odchylenia",					TP_NAST_PID_ODCH,	obr_pid},
	{"PID Wysoko",	"Nastawy PID wysokosci",					TP_NAST_PID_WYSOK,	obr_pid},
	{"PID NawigN",	"Nastawy PID nawigacji polnocnej",			TP_NAST_PID_NAWIG_PÓŁN,	obr_pid4},
	{"PID NawigE",	"Nastawy PID nawigacji wschodniej",			TP_NAST_PID_NAWIG_WSCH,	obr_pid4},
	{"Mikser",		"Nastawy miksera silnikow",					TP_NAST_MIKSERA,	obr_mikser},
	{"Silniki",		"Identyfikacja silnikow",					TP_NAST_IDENT_SILN,	obr_silnik},
	{"nic",			"nic",										TP_NAST8,			obr_narzedzia},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};


menu_t stMenuWydajnosc[MENU_WIERSZE * MENU_KOLUMNY] = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Fraktale",	"Benchmark fraktalowy"	,					TP_FRAKTALE,		obr_fraktal},
	{"Petla AP",	"Wydajnosc petli glownej autopilota",		TP_POMIAR_PGAP,		obr_Wydajnosc},
	{"Zapis NOR", 	"Test zapisu do flash NOR",					TP_POM_ZAPISU_NOR,	obr_NOR},
	{"Trans NOR", 	"Pomiar predkosci flasha NOR 16-bit",		TP_POMIAR_FNOR,		obr_NOR},
	{"Trans QSPI",	"Pomiar predkosci flasha QSPI 4-bit",		TP_POMIAR_FQSPI,	obr_QSPI},
	{"Trans RAM",	"Pomiar predkosci SRAM i DRAM 16-bit",		TP_POMIAR_SRAM,		obr_RAM},
	{"EmuMagCAN",	"Emulator magnetometru na magistrali CAN",	TP_EMU_MAG_CAN,		obr_kal_mag_n1},
	{"Kostka 3D", 	"Rysuje kostke 3D w funkcji katow IMU",		TP_IMU_KOSTKA,		obr_kostka3D},
	{"Pamiec",		"Status pamieci",							TP_STAN_PAMIECI,	obr_Wydajnosc},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

menu_t stMenuAudio[MENU_WIERSZE * MENU_KOLUMNY] = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Miki Rej",  	"Deklaracja Mikolaja Reja o APL",			TP_MREJ,	 		obr_Mikołaj_Rey},
	{"Papuga",		"Rejestrator i odtwarzacz dzwieku",			TP_PAPUGA,			obr_papuga1},
	{"Miki DRAM",	"Test wymowy z DRAM",						TP_MM2,				obr_glosnik2},
	{"Test Tonu",	"Test tonu wario",							TP_TEST_TONU,		obr_glosnik2},
	{"FFT Audio",	"FFT sygnału z mikrofonu",					TP_AUDIO_FFT,		obr_fft},
	{"Gotowy by",	"Wymowa komunikatu",						TP_MM_KOM1,			obr_narzedzia},
	{"Alleluja",	"Wymowa komunikatu",						TP_MM_KOM2,			obr_narzedzia},
	{"Test kom.",	"Test komunikatow audio",					TP_MM_KOM,			obr_glosnik2},
	{"nic",			"nic",										TP_MM8,				obr_narzedzia},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

menu_t stMenuKamera[MENU_WIERSZE * MENU_KOLUMNY] = {
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

 menu_t stMenuOsd[MENU_WIERSZE * MENU_KOLUMNY] = {
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

menu_t stMenuKartaSD[MENU_WIERSZE * MENU_KOLUMNY] = {
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

menu_t stMenuEthernet[MENU_WIERSZE * MENU_KOLUMNY] = {
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

menu_t stMenuTestowe[MENU_WIERSZE * MENU_KOLUMNY] = {
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

menu_t stMenuIMU[MENU_WIERSZE * MENU_KOLUMNY] = {
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
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_powrot1}};

menu_t stMenuMagnetometr[MENU_WIERSZE * MENU_KOLUMNY] = {
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
// Wątek wyświetlacza
// Parametry: argument* ?
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WatekWyswietlacza(void *argument)
{
	uint8_t cBłąd;
	for(;;)
	{
		if (nZainicjowanoCM7 & INIT_LCD480x320)		//obsłuż wyświetlacz tylko wtedy jest zainicjowany
		{
			cBłąd = RysujEkran();
			if (cBłąd)
				cCzasSwieceniaLED[LED_ZIEL] = 3;	//x0,1s - sygnalizacja błędów obsługi poleceń
		}
		else
			osDelay(1000);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran główny odświeżany w głównej pętli programu
// Parametry:
// Zwraca: nic
// Czas rysowania pełnego ekranu: 174ms
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujEkran(void)
{
	uint8_t cBłąd = BLAD_OK;

	if ((stStatusDotyku.chFlagi & DOTYK_SKALIBROWANY) != DOTYK_SKALIBROWANY)		//sprawdź czy ekran dotykowy jest skalibrowany
		cTrybPracy = TP_KAL_DOTYK;

	switch (cTrybPracy)
	{
	case TP_MENU_GLOWNE:	// wyświetla menu główne
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_MENU_MAIN]);
		cBłąd |= Menu(cNapisPodreczny, stMenuGlowne, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;

	case TP_KAL_BARO:	//kalibracja ciśnienia według wzorca

		if (KalibrujBaro(&cSekwencerKalibracji) == BLAD_GOTOWE)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_MENU;
		}
		break;

	case TP_POMIARY_DOTYKU:
		if (TestDotyku() == BLAD_GOTOWE)
			cNowyTrybPracy = TP_WROC_DO_MENU;
		break;

	//*** Menu Testy ************************************************
	case TP_TESTY:
		//Menu(strcat((char*)cNapisLcd[STR_TESTY], (char*)cNapisLcd[STR_MENU]), stMenuTestowe, &cNowyTrybPracy);
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_TESTY]);
		cBłąd = Menu(cNapisPodreczny, stMenuTestowe, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;

	case TP_TEST1:
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_TESTY;
		}
		break;

	case TP_TEST2:
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_TESTY;
		}
		break;

	case TP_TEST3:
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_TESTY;
		}
		break;

	case TP_TEST4:
	case TP_TEST5:
		for (uint8_t n=0; n<100; n++)
			cBuforTestowyDRAM[n] = n;
		cTrybPracy = cWrocDoTrybu;
		cNowyTrybPracy = TP_WROC_DO_TESTY;
		break;

	case TP_TEST6:
	case TP_TEST7:
	case TP_TEST8:
	case TP_TEST9:





	//*** Audio ************************************************
	case TP_MEDIA_AUDIO:
		//Menu(strcat((char*)cNapisLcd[STR_MENU_MEDIA_AUDIO], (char*)cNapisLcd[STR_MENU]), stMenuAudio, &chNowyTrybPracy);
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_AUDIO]);
		cBłąd = Menu(cNapisPodreczny, stMenuAudio, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_MREJ:
		InicjujOdtwarzanieDzwieku();
		OdtworzProbkeAudioZeSpisu(PGA_NIECHAJ_NARODO);
		cNowyTrybPracy = TP_WROC_DO_AUDIO;
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
			sprintf(cNapis, "Inicjalizacja: %d/%d", n, 5);
			RysujNapis(cNapis, 10, 5);
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
			sprintf(cNapis, "Rejestracja: %d/%d", n, sLiczbaBuforowNagrania);
			RysujNapis(cNapis, 10, 20);
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
		sprintf(cNapis, "Odtwarzanie      ");
		RysujNapis(cNapis, 10, 20);
		InicjujOdtwarzanieDzwieku();
		OdtworzProbkeAudio((uint32_t)sBuforPapuga, ROZMIAR_BUFORA_PAPUGI * 2);	//*2 bo rozmiar komunikatu jest w bajtach
		cNowyTrybPracy = TP_WROC_DO_AUDIO;
		break;


	case TP_MM2:
		InicjujOdtwarzanieDzwieku();
		PrzepiszProbkeDoDRAM(PGA_NIECHAJ_NARODO);
		cNowyTrybPracy = TP_WROC_DO_AUDIO;
		break;

	case TP_MM_KOM1:
		InicjujOdtwarzanieDzwieku();
		OdtworzProbkeAudioZeSpisu(PGA_GOTOWY_SLUZYC);
		cNowyTrybPracy = TP_WROC_DO_AUDIO;
		break;

	case TP_MM_KOM2:
		InicjujOdtwarzanieDzwieku();
		OdtworzProbkeAudioZeSpisu(PGA_ALLELUJA);
		cNowyTrybPracy = TP_WROC_DO_AUDIO;
		break;

	case TP_TEST_TONU:
		InicjujOdtwarzanieDzwieku();
		TestTonuAudio();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			ZatrzymajTon();
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_AUDIO;
		}
		break;


	case TP_AUDIO_FFT:			//FFT sygnału z mikrofonu
		//extern int32_t nBuforAudioWe[ROZMIAR_BUFORA_AUDIO_WE];	//bufor komunikatów przychodzących
		extern int16_t sBuforAudioWe[2][2*ROZMIAR_BUFORA_AUDIO_WE];	//bufor komunikatów przychodzących
		extern uint8_t cWskaznikBuforaAudio;
		uint8_t cWskKasowania;
		InicjujRejestracjeDzwieku();
		do
		{
			cWskKasowania = cWskaznikBuforaAudio;
			cWskaznikBuforaAudio ^= 0x01;
			cWskaznikBuforaAudio &= 0x01;
			NapelnijBuforDzwieku(&sBuforAudioWe[cWskaznikBuforaAudio][0], 2*ROZMIAR_BUFORA_AUDIO_WE);
			RysujPrzebieg(sBuforAudioWe[cWskKasowania], sBuforAudioWe[cWskaznikBuforaAudio], BIALY);
		}
		while ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		cTrybPracy = cWrocDoTrybu;
		cNowyTrybPracy = TP_WROC_DO_AUDIO;
		break;


	case TP_MM_KOM:
		InicjujOdtwarzanieDzwieku();
		TestKomunikatow();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_AUDIO;
		}
		break;


	//*** Kamera ************************************************
	case TP_MEDIA_KAMERA:			//menu kamera
		//Menu((char*)cNapisLcd[STR_MENU_KAMERA], stMenuKamera, &cNowyTrybPracy);
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_KAMERA]);
		cBłąd = Menu(cNapisPodreczny, stMenuKamera, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;

	/*case TP_ZDJECIE:	//wykonaj pojedyncze zdjęcie i zapisz je w pliku binarnym txt
		extern uint16_t sLicznikLiniiKamery;
		stKonfOSD.chOSDWlaczone = 0;	//nie właczaj OSD
		sLicznikLiniiKamery = 0;
		cBłąd = ZrobZdjecie(sBuforKamery, DISP_X_SIZE * DISP_Y_SIZE / 2);
		if (cBłąd)
		{
			setColor(MAGENTA);
			sprintf(cNapis, "Blad: %d  ", cBłąd);
		}
		else
		{
			setColor(ZIELONY);
			sprintf(cNapis, "Linii: %d  ", sLicznikLiniiKamery);
			WyswietlZdjecie(480, 320, sBuforKamery);
		}
		RysujNapis(cNapis, KOL12, 300);
		FRESULT fres = 0;
		extern RTC_HandleTypeDef hrtc;
		extern RTC_TimeTypeDef sTime;
		extern RTC_DateTypeDef sDate;

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		sprintf(cNapis, "Zdjecie%04d%02d%02d_%02d%02d%02d.txt",sDate.Year+2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

		fres = f_open(&SDJpegFile, cNapis, FA_OPEN_ALWAYS | FA_WRITE);
		if (fres == FR_OK)
		{
			f_puts((char*)sBuforKamery, &SDJpegFile);	//zapis do pliku
			f_close(&SDJpegFile);
		}
		osDelay(600);
		cNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;*/

	case TP_KAMERA:	//ciagła praca kamery z pamięcią SRAM
		stKonfOSD.chOSDWlaczone = 1;	//włacz OSD z histogramem
		stKonfOSD.sSzerokosc = DISP_X_SIZE;
		stKonfOSD.sWysokosc = DISP_Y_SIZE;
		WypelnijEkranwBuforze(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, cBuforOSD, PRZEZR_100);	//czyść poprzednią zawartość
		cBłąd = UstawObrazKamery(DISP_X_SIZE, DISP_Y_SIZE, OBR_RGB565, KAM_FILM);
		cBłąd = PolaczBuforOSDzObrazem(cBuforOSD, (uint8_t*)sBuforKamery, cBuforLCD, DISP_X_SIZE, DISP_Y_SIZE);
		RozpocznijPraceDCMI(&stKonfKam, sBuforKamery, DISP_X_SIZE * DISP_Y_SIZE / 2);
		do
		{
			nCzas = PobierzCzasT6();
			LiczHistogramRGB565(sBuforKamery, STD_OBRAZU_DVGA, cHistR, cHistG, cHistB);	//licz histogram
			nCzas = MinalCzas(nCzas);
			nCzasHist = PobierzCzasT6();
			RysujHistogramOSD_RGB32(cBuforOSD, cHistR, cHistG, cHistB);
			nCzasHist = MinalCzas(nCzasHist);
			WyswietlZdjecieRGB666(DISP_X_SIZE, DISP_Y_SIZE, cBuforLCD);
			nCzasLCD = MinalCzas(nCzasLCD);
			sprintf(cNapis, "Tobl: %ld ms, Trys: %ld ms, %ld fps", nCzas / 1000, nCzasHist / 1000, 1000000 / nCzasLCD);
			RysujNapiswBuforze(cNapis, 0, DISP_Y_SIZE - FONT_BH, DISP_X_SIZE, cBuforOSD, (uint8_t*)(KOLOSD_ZOLTY0 + PRZEZR_20), (uint8_t*)PRZEZR_80, ROZMIAR_KOLORU_OSD);
			nCzasLCD = PobierzCzasT6();
		}
		while ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		cNowyTrybPracy = TP_WROC_DO_KAMERA;
		stKonfOSD.chOSDWlaczone = 0;	//wyłącz OSD
		break;

	case TP_KAM_DRAM:	//ciagła praca kamery z pamięcią DRAM
		stKonfOSD.chOSDWlaczone = 0;	//wyłacz OSD z histogramem
		//stKonfOSD.chOSDWlaczone = 1;	//włacz OSD z histogramem
		stKonfOSD.sSzerokosc = DISP_X_SIZE;
		stKonfOSD.sWysokosc = DISP_Y_SIZE;
		cBłąd = UstawObrazKamery(DISP_X_SIZE, DISP_Y_SIZE, OBR_RGB565, KAM_FILM);
		//cBłąd = PolaczBuforOSDzObrazem(cBuforOSD, (uint8_t*)sBuforKameryDRAM, cBuforLCD, DISP_X_SIZE, DISP_Y_SIZE);
		RozpocznijPraceDCMI(&stKonfKam, sBuforKamery, DISP_X_SIZE * DISP_Y_SIZE / 2);
		do
		{
			//LiczHistogramRGB565(sBuforKameryDRAM, STD_OBRAZU_DVGA, chHistR, cHistG, cHistB);
			RysujHistogramOSD_RGB32(cBuforOSD, cHistR, cHistG, cHistB);
			KonwersjaRGB565doRGB666(sBuforKamery, cBuforLCD, DISP_X_SIZE * DISP_Y_SIZE);
			WyswietlZdjecieRGB666(DISP_X_SIZE, DISP_Y_SIZE, cBuforLCD);

			//RysujHistogramRGB32(cHistR, cHistG, cHistB);
		}
		while ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		cNowyTrybPracy = TP_WROC_DO_KAMERA;
		stKonfOSD.chOSDWlaczone = 0;	//wyłącz OSD
		break;


	case TP_KAM_Y8:	//praca z obrazem czarno-białym
		stKonfOSD.chOSDWlaczone = 0;	//nie właczaj OSD
		//ponieważ bufor obrazu 1280x960 jest 8 razy większy niż 480x320 wiec dzielę go na 8 części i w nich zapisuję kolejne klatki
		cBłąd = UstawObrazKamery(DISP_X_SIZE, DISP_Y_SIZE, OBR_Y8, KAM_FILM);
		if (cBłąd)
			break;
		cBłąd = RozpocznijPraceDCMI(&stKonfKam, sBuforKamery, DISP_X_SIZE * DISP_Y_SIZE / 4);
		if (cBłąd)
			break;
		do
		{
			while (!cObrazKameryGotowy)	//synchronizuj się do początku nowej klatki obrazu
				osDelay(1);
			cObrazKameryGotowy = 0;
			cWskNapBufKam++;
			cWskNapBufKam &= MASKA_BUFORA_KAMERY;
			//utwórz wskaźnik na konkretny bufor w obrębie zmiennej sBuforKamery wskazujący na kolejną 1/8 zmiennej
			//uint16_t* sPodBufor = sBuforKamery + chWskNapBufKam * (DISP_X_SIZE * DISP_Y_SIZE / (4 * 8));
			//cBłąd = RozpocznijPraceDCMI(stKonfKam, sPodBufor, DISP_X_SIZE * DISP_Y_SIZE / 4);
			if (cWskNapBufKam & 0x01)		//nieparzyste obrazy kompresuj a parzyste wyświetlaj
			{
				nCzas = PobierzCzasT6();
				//cBłąd = KompresujY8((uint8_t*)sPodBufor, DISP_X_SIZE, DISP_Y_SIZE);
				cBłąd = KompresujY8((uint8_t*)sBuforKamery, DISP_X_SIZE, DISP_Y_SIZE);
				nCzas = MinalCzas(nCzas);
			}
			else
			{
				//KonwersjaCB8doRGB666((uint8_t*)sPodBufor, cBuforLCD, DISP_X_SIZE * DISP_Y_SIZE);
				KonwersjaCB8doRGB666((uint8_t*)sBuforKamery, cBuforLCD, DISP_X_SIZE * DISP_Y_SIZE);
#ifdef 	LCD_ILI9488
				WyswietlZdjecieRGB666(DISP_X_SIZE, DISP_Y_SIZE, cBuforLCD);
#endif
				sprintf(cNapis, "Tkompr: %ld us, buf: %d", nCzas, cWskNapBufKam);
				setColor(ZOLTY);
				RysujNapis(cNapis, 0, DISP_Y_SIZE - 2*FONT_BH);
			}
		}
		while ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		cNowyTrybPracy = TP_WROC_DO_KAMERA;
			break;


	case TP_KAM_YUV420:	//kompresja obrazu kolorowego
		cBłąd = UstawObrazKamery(SZER_ZDJECIA, WYS_ZDJECIA, OBR_YUV420, KAM_ZDJECIE);
		if (cBłąd)
			break;
		do
		{
			cBłąd = RozpocznijPraceDCMI(&stKonfKam, sBuforKamery, DISP_X_SIZE * DISP_Y_SIZE / 2 * 3);
			if (cBłąd)
				break;
			cBłąd = CzekajNaKoniecPracyDCMI(DISP_Y_SIZE);
			nCzas = PobierzCzasT6();
			cBłąd = KompresujYUV420((uint8_t*)sBuforKamery, DISP_X_SIZE, DISP_Y_SIZE);
			if (cBłąd)
				break;

			cBłąd = CzekajNaKoniecPracyJPEG();
			nCzas = MinalCzas(nCzas);
			sprintf(cNapis, "%.2f fps, kompr: %.1f", (float)1000000.0/nCzas, (float)nRozmiarObrazuKamery / nRozmiarObrazuJPEG);	//Sprawdzić: hard fault
			setColor(ZOLTY);
			RysujNapis(cNapis, 0, DISP_Y_SIZE - FONT_BH);
		}
		while ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		cNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;

	case TP_RTSP_YUV422:	//kamera generuje standarowy obraz na ekran, DMA2D nakłada OSD i zamienia na format RGB888, obraz jest kompresowany bez nagłówka JPEG
		stKonfOSD.sSzerokosc = 480;
		stKonfOSD.sWysokosc = 320;
		stKonfOSD.chOSDWlaczone = 1;
		cStatusRejestratora &= ~STATREJ_ZAPISZ_JPG;	//wyłącz rejestrację na karcie
		hjpeg.Instance->CONFR1 &= ~JPEG_CONFR1_HDR;		//wyłącz generowanie nagłówka JPEG
		WypelnijEkranwBuforze(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, cBuforOSD, PRZEZR_100);

		cBłąd = UstawObrazKamery(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, OBR_RGB565, KAM_FILM);		//kolor
		if (cBłąd)
			break;

		cBłąd = RozpocznijPraceDCMI(&stKonfKam, sBuforKamery, stKonfOSD.sSzerokosc * stKonfOSD.sWysokosc / 2);	//kolor
		if (cBłąd)
			break;
		do
		{
			RysujOSD(&stKonfOSD, &uDaneCM4.dane);
			RysujBitmape888(0, 0, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, cBuforLCD);	//wyświetla połączone obrazy na LCD
			cBłąd = KompresujRGB888doYUV422(cBuforLCD, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, 30);
		}
		while ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		cNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;


	case TP_KAM_ZDJ_Y8:	//wykonuje zdjecie Y8 jpg
		stKonfOSD.chOSDWlaczone = 0;	//nie właczaj OSD
		sprintf((char*)cNazwaPlikuObrazu, "ZdjY8");	//początek nazwy pliku ze zdjeciem
		cStatusRejestratora |= STATREJ_ZAPISZ_JPG;	//zapisuj do pliku jpeg

		cBłąd = UstawObrazKamery(SZER_ZDJECIA, WYS_ZDJECIA, OBR_Y8, KAM_ZDJECIE);
		nCzas = PobierzCzasT6();
		cBłąd = ZrobZdjecie(sBuforKamery, SZER_ZDJECIA * WYS_ZDJECIA / 4);
		if (cBłąd)
		{
			cNowyTrybPracy = TP_WROC_DO_KAMERA;
			break;
		}
		RysujNapis((char*)cOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 300);	//"Wdus ekran i trzymaj aby zakonczyc"
		CzekajNaKoniecPracyDCMI(WYS_ZDJECIA);
		nCzas = MinalCzas(nCzas);
		printf("Tdcmi=%ldus\r\n", nCzas);

		nCzas = PobierzCzasT6();
		cBłąd = KompresujY8((uint8_t*)sBuforKamery, SZER_ZDJECIA, WYS_ZDJECIA);	//, cBuforJpeg, ROZMIAR_BUF_JPEG);
		nCzas = MinalCzas(nCzas);
		setColor(ZOLTY);
		if (!cBłąd)
			printf("Tjpeg=%ldus\r\n", nCzas);
		else
		{
			sprintf(cNapis, "Blad: %d ", cBłąd);
			RysujNapis(cNapis, 10, 30);
			if ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) == DOTYK_DOTKNIETO)
				cNowyTrybPracy = TP_WROC_DO_KAMERA;
			break;
		}

		sprintf(cNapis, "Czas kompr: %ld us, rozm_obr: %ld, kompr: %.2f ", nCzas, nRozmiarObrazuJPEG, (float)(SZER_ZDJECIA*WYS_ZDJECIA) / nRozmiarObrazuJPEG);
		RysujNapis(cNapis, 0, 30);
		cStatusRejestratora |= STATREJ_ZAPISZ_BMP;	//ustaw flagę zapisu obrazu do pliku bmp
		//jest ustawiony większy rozmiar, więc nie wyswietlaj obrazu
		//KonwersjaCB8doRGB666((uint8_t*)sBuforKamery, chBufLCD, SZER_ZDJECIA * WYS_ZDJECIA);
		//WyswietlZdjecieRGB666(DISP_X_SIZE, DISP_Y_SIZE, cBuforLCD);
		osDelay(3000);
		cNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;


	case TP_KAM_ZDJ_YUV420:	//wykonuje zdjecie YUV420 jpg
		//cał pamięć SRAM wypełnij wzorcem 0x1111 a następnie bufor wzorcem 0x4444
		uint16_t* sWskSram = (uint16_t*)0x60000000;
		for (uint32_t n=0; n<0x400000; n++)
			*(sWskSram + n) = 0x1111;
		for (uint32_t m=0; m<SZER_ZDJECIA/4; m++)
		{
			for (uint32_t n=0; n<WYS_ZDJECIA; n++)
				sBuforKamery[n+m*WYS_ZDJECIA] = (m & 0x0FFF) | 0x4000;
		}
		sprintf((char*)cNazwaPlikuObrazu, "ZdjYUV420");	//początek nazwy pliku ze zdjeciem
		cStatusRejestratora |= STATREJ_ZAPISZ_JPG;	//zapisuj do pliku jpeg
		//cBłąd = UstawObrazKamery(DISP_X_SIZE, DISP_Y_SIZE, OBR_YUV420, KAM_ZDJECIE);
		//cBłąd = ZrobZdjecie(sBuforKamery, DISP_X_SIZE * DISP_Y_SIZE / 2);	//wynik w sBuforKamery
		cBłąd = UstawObrazKamery(60, 40, OBR_YUV420, KAM_ZDJECIE);
		cBłąd = ZrobZdjecie(sBuforKamery, 60 * 40 / 2);	//wynik w sBuforKamery
		nCzas = PobierzCzasT6();
		//cBłąd = KompresujYUV420((uint8_t*)sBuforKamery, DISP_X_SIZE, DISP_Y_SIZE);
		cBłąd = KompresujYUV420((uint8_t*)sBuforKamery, 60, 40);
		//KonwersjaCB8doRGB666((uint8_t*)sBuforKamery, chBufLCD, DISP_X_SIZE * DISP_Y_SIZE);
		nCzas = MinalCzas(nCzas);
		setColor(ZOLTY);
		if (cBłąd)
		{
			setColor(ZOLTY);
			sprintf(cNapis, "Blad: %d", cBłąd);
			RysujNapis(cNapis, 10, 30);
			if ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) == DOTYK_DOTKNIETO)
				cNowyTrybPracy = TP_WROC_DO_KAMERA;
			break;
		}

		sprintf(cNapis, "Czas kompr: %ld us, rozm_obr: %ld, kompr: %.2f", nCzas, nRozmiarObrazuJPEG, (float)(DISP_X_SIZE*DISP_Y_SIZE) / nRozmiarObrazuJPEG);
		RysujNapis(cNapis, 0, 30);
		osDelay(3000);
		cNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;


	case TP_KAM_ZDJ_YUV422:	//Analiza obrazu pokazuje że coś jest nie tak z obrazem YUV444. Dla obrazu o szerokości 480 pix powtarza się biała linia 4x16 bajtów co 2*480 pikseli
		sprintf((char*)cNazwaPlikuObrazu, "ZdjYUV444");	//początek nazwy pliku ze zdjeciem
		cBłąd = UstawObrazKamery(DISP_X_SIZE, DISP_Y_SIZE, OBR_YUV444, KAM_ZDJECIE);
		cBłąd = ZrobZdjecie(sBuforKamery, DISP_X_SIZE * DISP_X_SIZE * 2 / 3);	//rozmiar obrazu to 3 bajty na piksel
		nCzas = PobierzCzasT6();
		cBłąd = KompresujYUV444((uint8_t*)sBuforKamery, DISP_X_SIZE, DISP_Y_SIZE);
		nCzas = MinalCzas(nCzas);
		if (cBłąd)
		{
			setColor(ZOLTY);
			sprintf(cNapis, "Blad: %d", cBłąd);
			RysujNapis(cNapis, 10, 30);
			if ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) == DOTYK_DOTKNIETO)
				cNowyTrybPracy = TP_WROC_DO_KAMERA;
			break;
		}
		cNowyTrybPracy = TP_WROC_DO_KAMERA;
		break;


	case TP_KAM_DIAG:
		if (cRysujRaz)
		{
			BelkaTytulu("Diagnostyka kamery");
			cRysujRaz = 0;
		}

		setColor(BIALY);
		UstawCzcionke(MidFont);
		WykonajDiagnostykeKamery(&stDiagKam);

		sprintf(cNapis, "AVG: 0x%X, Prog AEC: 0x%X..0x%X, stab: 0x%X..0x%X", stDiagKam.chSredniaJasnoscAVG, stDiagKam.chProgAEC_H, stDiagKam.chProgAEC_L, stDiagKam.chProgStabAEC_H, stDiagKam.chProgStabAEC_L);
		RysujNapis(cNapis, 0, 30);

		sprintf(cNapis, "AEC/AGC: 0x%X, VTS: %d", stDiagKam.chTrybEAC_EAG, stDiagKam.sMaxCzasEkspoVTS);
		RysujNapis(cNapis, 0, 50);

		sprintf(cNapis, "okno AVG X: %d..%d, Y: %d..%d", stDiagKam.sPoczatOknaAVG_X, stDiagKam.sKoniecOknaAVG_X, stDiagKam.sPoczatOknaAVG_Y, stDiagKam.sKoniecOknaAVG_Y);
		RysujNapis(cNapis, 0, 70);

		sprintf(cNapis, "Poczatek okna obrazu X: %d, Y: %d", stDiagKam.sPoczatOknaObrazu_X, stDiagKam.sPoczatOknaObrazu_Y);
		RysujNapis(cNapis, 0, 90);

		sprintf(cNapis, "Rozmiar okna obrazu X: %d, Y: %d", stDiagKam.sRozmiarOknaObrazu_X, stDiagKam.sRozmiarOknaObrazu_Y);
		RysujNapis(cNapis, 0, 110);

		sprintf(cNapis, "Rozmiar obrazu X (HTS): %d, Y (VTS): %d", stDiagKam.sRozmiarPoz_HTS, stDiagKam.sRozmiarPio_VTS);
		RysujNapis(cNapis, 0, 130);

		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_KAMERA;
		}
		break;


	//*** OSD ************************************************
	case TP_MENU_OSD:
		//Menu((char*)cNapisLcd[STR_MENU_OSD], stMenuOsd, &cNowyTrybPracy);
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_OSD]);
		cBłąd = Menu(cNapisPodreczny, stMenuOsd, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TPO_ZAPIS_BMP:	//zapisz chrominancję lub luminację z bufora LCD do zmiennej: sBuforKamery a nastepnie do pliku bmp
		ZakonczPraceDCMI();	//wyłącz kamerę aby nie nadpisywała obrazu w sBuforKamery
		stKonfOSD.chOSDWlaczone = 0;	//nie właczaj OSD
		TestKonwersjiRGB888doYCbCr(cBuforLCD, (uint8_t*)sBuforKamery, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc);
		sprintf((char*)cNazwaPlikuObrazu, "Luma");	//początek nazwy pliku ze zdjeciem
		cStatusRejestratora = STATREJ_ZAPISZ_BMP;	//ustaw flagę zapisu obrazu do pliku bmp
		stKonfKam.chFormatObrazu = OBR_Y8;			//obraz ma sie zapisać jako monochromatyczny
		cNowyTrybPracy = TP_WROC_DO_OSD;
		break;

	case TPO_OSD240:
	case TPO_OSD320:
	case TPO_OSD480:	//obraz a rzeczywistym tle
		if (cTrybPracy == TPO_OSD240)
		{
			stKonfOSD.sSzerokosc = 240;
			stKonfOSD.sWysokosc = 160;
		}
		else
		if (cTrybPracy == TPO_OSD320)
		{
			stKonfOSD.sSzerokosc = 320;
			stKonfOSD.sWysokosc = 240;
		}
		else
		if (cTrybPracy == TPO_OSD480)
		{
			stKonfOSD.sSzerokosc = 480;
			stKonfOSD.sWysokosc = 320;
		}
		WypelnijEkranwBuforze(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, cBuforOSD, PRZEZR_100);
		cBłąd = UstawObrazKamery(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, OBR_RGB565, KAM_FILM);		//kolor
		if (cBłąd)
			break;
		hjpeg.Instance->CONFR1 |= JPEG_CONFR1_HDR;	//włącz generowanie nagłówka JPEG
		stKonfOSD.chOSDWlaczone = 1;
		cBłąd = RozpocznijPraceDCMI(&stKonfKam, sBuforKamery, stKonfOSD.sSzerokosc * stKonfOSD.sWysokosc / 2);	//kolor
		if (cBłąd)
			break;
		do
		{
			/*cTimeout = 60;
			while (!cObrazKameryGotowy && cTimeout)	//synchronizuj się do początku nowej klatki obrazu
			{
				osDelay(1);
				cTimeout--;
			}
			cObrazKameryGotowy = 0;*/
			RysujOSD(&stKonfOSD, &uDaneCM4.dane);

			nCzasLCD = DWT->CYCCNT;	//start pomiaru
			RysujBitmape888(0, 0, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, cBuforLCD);	//wyświetla połączone obrazy na LCD
			nCzasLCD = DWT->CYCCNT - nCzasLCD; //koniec pomiaru
		}
		while ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
		cBłąd = ZakonczPraceDCMI();
		cNowyTrybPracy = TP_WROC_DO_OSD;
		stKonfOSD.chOSDWlaczone = 0;	//wyłącz OSD
		break;



	case TPO_TEST_OSD480:	//obraz na testowym tle
		stKonfOSD.sSzerokosc = 480;
		stKonfOSD.sWysokosc = 320;
		WypelnijEkranwBuforze(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, (uint8_t*)sBuforKamery, SZARY20);
		WypelnijEkranwBuforze(stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, cBuforOSD, PRZEZR_100);
		do
		{
			RysujOSD(&stKonfOSD, &uDaneCM4.dane);
			nCzasLCD = DWT->CYCCNT;	//start pomiaru
			RysujBitmape888(0, 0, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, cBuforLCD);	//wyświetla połączone obrazy na LCD
			nCzasLCD = DWT->CYCCNT - nCzasLCD; //koniec pomiaru
		}
		while ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) != DOTYK_DOTKNIETO);
			cNowyTrybPracy = TP_WROC_DO_OSD;
		break;


	case TPO_JPEG420:
		ZakonczPraceDCMI();
		hjpeg.Instance->CONFR1 |= JPEG_CONFR1_HDR;	//włącz generowanie nagłówka JPEG
		for (uint8_t n=1; n<=10; n++)	//wykonaj serię obrazów o różnym stopniu kompresji
		{
			printf("Obraz %d%%: ", n*10);
			cStatusRejestratora |= STATREJ_ZAPISZ_JPG;		//zapisuj do pliku jpeg
			sprintf((char*)cNazwaPlikuObrazu, "YUV420_%d", n*10);	//początek nazwy pliku ze zdjeciem
			cBłąd = KompresujRGB888doYUV420(cBuforLCD, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, n*10);
			while (cStatusRejestratora & STATREJ_ZAPISZ_JPG)	//czekaj aż się zapisze
			{
				printf("Sync, ");
				osDelay(5);
			}
		}
		cNowyTrybPracy = TP_WROC_DO_OSD;
		break;

	case TPO_JPEG422:		//kompresja jpeg obrazu OSD
		ZakonczPraceDCMI();
		hjpeg.Instance->CONFR1 |= JPEG_CONFR1_HDR;	//włącz generowanie nagłówka JPEG
		for (uint8_t n=1; n<=10; n++)	//wykonaj serię obrazów o różnym stopniu kompresji
		{
			printf("Obraz %d%%: ", n*10);
			cStatusRejestratora |= STATREJ_ZAPISZ_JPG;		//zapisuj do pliku jpeg
			sprintf((char*)cNazwaPlikuObrazu, "YUV422_%d", n*10);	//początek nazwy pliku ze zdjeciem
			cBłąd = KompresujRGB888doYUV422(cBuforLCD, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, n*10);
			while (cStatusRejestratora & STATREJ_ZAPISZ_JPG)	//czekaj aż się zapisze
			{
				printf("Sync, ");
				osDelay(5);
			}
		}
		cNowyTrybPracy = TP_WROC_DO_OSD;
		break;

	case TPO_JPEG444:
		ZakonczPraceDCMI();
		hjpeg.Instance->CONFR1 |= JPEG_CONFR1_HDR;	//włącz generowanie nagłówka JPEG
		for (uint8_t n=1; n<=10; n++)	//wykonaj serię obrazów o różnym stopniu kompresji
		{
			printf("Obraz %d%%: ", n*10);
			cStatusRejestratora |= STATREJ_ZAPISZ_JPG;		//zapisuj do pliku jpeg
			sprintf((char*)cNazwaPlikuObrazu, "YUV444_%d", n*10);	//początek nazwy pliku ze zdjeciem
			cBłąd = KompresujRGB888doYUV444(cBuforLCD, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, n*10);
			while (cStatusRejestratora & STATREJ_ZAPISZ_JPG)	//czekaj aż się zapisze
			{
				printf("Sync, ");
				osDelay(5);
			}
		}
		cNowyTrybPracy = TP_WROC_DO_OSD;
		break;

	case TPO_JPEG_Y8:
		ZakonczPraceDCMI();
		hjpeg.Instance->CONFR1 |= JPEG_CONFR1_HDR;	//włącz generowanie nagłówka JPEG
		for (uint8_t n=1; n<=10; n++)	//wykonaj serię obrazów o różnym stopniu kompresji
		{
			printf("Obraz %d%%: ", n*10);
			cStatusRejestratora |= STATREJ_ZAPISZ_JPG;		//zapisuj do pliku jpeg
			sprintf((char*)cNazwaPlikuObrazu, "Y8_%d", n*10);	//początek nazwy pliku ze zdjeciem
			cBłąd = KompresujRGB888doY8(cBuforLCD, stKonfOSD.sSzerokosc, stKonfOSD.sWysokosc, n*10);
			while (cStatusRejestratora & STATREJ_ZAPISZ_JPG)	//czekaj aż się zapisze
			{
				printf("Sync, ");
				osDelay(5);
			}
		}
		cNowyTrybPracy = TP_WROC_DO_OSD;
		break;

	//*** Ethernet ************************************************
	case TP_ETHERNET:
		//Menu((char*)cNapisLcd[STR_MENU_ETHERNET], stMenuEthernet, &cNowyTrybPracy);
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_ETHERNET]);
		cBłąd = Menu(cNapisPodreczny, stMenuEthernet, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_ETH_INFO:
		extern ip4_addr_t ipaddr;
		extern ip4_addr_t netmask;
		extern ip4_addr_t gw;
#ifdef ETH_WLACZONY
		extern struct stats_ lwip_stats;
#endif

		if (cRysujRaz)
		{
			BelkaTytulu("Statystyki Internetowe");
			cRysujRaz = 0;
		}

		setColor(BIALY);
		UstawCzcionke(MidFont);
		sprintf(cNapis, "NumerIP: %ld.%ld.%ld.%ld",(ipaddr.addr & 0xFF), (ipaddr.addr & 0xFF00)>>8, (ipaddr.addr & 0xFF0000)>>16, (ipaddr.addr & 0xFF000000)>>24);
		RysujNapis(cNapis, 10, 30);
		sprintf(cNapis, "Maska:   %ld.%ld.%ld.%ld", (netmask.addr & 0xFF), (netmask.addr & 0xFF00)>>8, (netmask.addr & 0xFF0000)>>16, (netmask.addr & 0xFF000000)>>24);
		RysujNapis(cNapis, 10, 50);
		sprintf(cNapis, "GateWay: %ld.%ld.%ld.%ld", (gw.addr & 0xFF), (gw.addr & 0xFF00)>>8, (gw.addr & 0xFF0000)>>16, (gw.addr & 0xFF000000)>>24);
		RysujNapis(cNapis, 10, 70);
#ifdef ETH_WLACZONY
		sprintf(cNapis, "LINK:    xmit=%d recv=%d drop=%d err=%d", lwip_stats.link.xmit, lwip_stats.link.recv, lwip_stats.link.drop, lwip_stats.link.err);
		RysujNapis(cNapis, 10, 90);
		sprintf(cNapis, "ARP:     xmit=%d recv=%d drop=%d err=%d", lwip_stats.etharp.xmit, lwip_stats.etharp.recv, lwip_stats.etharp.drop, lwip_stats.etharp.err);
		RysujNapis(cNapis, 10, 110);
		sprintf(cNapis, "UDP:     xmit=%d recv=%d drop=%d err=%d", lwip_stats.udp.xmit, lwip_stats.udp.recv, lwip_stats.udp.drop, lwip_stats.udp.err);
		RysujNapis(cNapis, 10, 130);
		sprintf(cNapis, "TCP:     xmit=%d recv=%d drop=%d err=%d", lwip_stats.tcp.xmit, lwip_stats.tcp.recv, lwip_stats.tcp.drop, lwip_stats.tcp.err);
		RysujNapis(cNapis, 10, 150);
		sprintf(cNapis, "TCP Err: checksum=%d length=%d options=%d protocol=%d", lwip_stats.tcp.chkerr, lwip_stats.tcp.lenerr, lwip_stats.tcp.opterr, lwip_stats.tcp.proterr);
		RysujNapis(cNapis, 10, 170);
		sprintf(cNapis, "MemHeap: avail=%ld used=%ld err=%d illegal=%d", (uint32_t)lwip_stats.mem.avail, (uint32_t)lwip_stats.mem.used, lwip_stats.mem.err, lwip_stats.mem.illegal);
		RysujNapis(cNapis, 10, 190);
		sprintf(cNapis, "MemPoll: avail=%ld used=%ld err=%d illegal=%d", (uint32_t)lwip_stats.memp[1]->avail, (uint32_t)lwip_stats.memp[1]->used, lwip_stats.memp[1]->err, lwip_stats.memp[1]->illegal);
		RysujNapis(cNapis, 10, 210);
		sprintf(cNapis, "SYS Err: semafor=%d mutex=%d mbox=%d", lwip_stats.sys.sem.err, lwip_stats.sys.mutex.err, lwip_stats.sys.mbox.err);
#else
		sprintf(cNapis, "Uzywanie ethernetu wylaczone");
#endif
		RysujNapis(cNapis, 10, 230);
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_ETH;
		}
		osDelay(100);	//pozwól pracować innym wątkom
		break;


	case TP_ETH_GADU_GADU:
		if (cRysujRaz)
		{
			BelkaTytulu("Gadu-Gadu port:4000");
			cRysujRaz = 0;
		}

		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_ETH;
		}
		break;


	//*** Wydajność ************************************************
	case TP_WYDAJNOSC:			///menu pomiarów wydajności
		//Menu((char*)cNapisLcd[STR_MENU_WYDAJNOSC], stMenuWydajnosc, &cNowyTrybPracy);
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_WYDAJNOSC]);
		cBłąd = Menu(cNapisPodreczny, stMenuWydajnosc, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_FRAKTALE:		FraktalDemo();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_POMIAR_SRAM:	TestPredkosciOdczytuRAM();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_POM_ZAPISU_NOR:		TestPredkosciZapisuNOR();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_POMIAR_FNOR:	TestPredkosciOdczytuNOR();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_POMIAR_FQSPI:	//W25_TestTransferu();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;


	case TP_IMU_KOSTKA:	//rysuj kostkę 3D
		float fKat[3];
		for (uint8_t n=0; n<3; n++)
			fKat[n] = -1 *uDaneCM4.dane.fKatZyro2[n];	//do rysowania przyjmij kąty z przeciwnym znakiem - jest OK
		RysujKostkeObrotu(fKat);
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_WYDAJN;
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
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;

	case TP_STAN_PAMIECI:
		setColor(ZOLTY);
		sprintf(cNapis, "Wolna sterta: %d", xPortGetFreeHeapSize());
		RysujNapis(cNapis, KOL12, 40);

		sprintf(cNapis, "Minimalna wolna pamiec: %d", xPortGetMinimumEverFreeHeapSize());
		RysujNapis(cNapis, KOL12, 70);

		/*struct mallinfo mi;
		memset(&mi,0,sizeof(struct mallinfo));
		mi = mallinfo();

		sprintf(cNapis, "mallinfo: Arena %ld, Free %ld, Used %ls", mi.arena, mi.fordblks, mi.uordblks);
		RysujNapis(cNapis, KOL12, 100);*/

		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;

	case TP_EMU_MAG_CAN:
		uDaneCM7.dane.cWykonajPolecenie = POL7_KAL_ZERO_MAGN2;	//włącz tryb jak dla kalibracji aby nie uwzględniać w wyniku danych kalibracyjnych
		EmulujMagnetometrWizjerCan((float*)uDaneCM4.dane.fMagne2);
		UstawCzcionke(BigFont);
		setColor(KOLOR_X);
		sprintf(cNapis, "Mag X: %.3f uT ", uDaneCM4.dane.fMagne2[0] * 1e6);
		RysujNapis(cNapis, KOL12, 40);
		setColor(KOLOR_Y);
		sprintf(cNapis, "Mag Y: %.3f uT ", uDaneCM4.dane.fMagne2[1] * 1e6);
		RysujNapis(cNapis, KOL12, 70);
		setColor(KOLOR_Z);
		sprintf(cNapis, "Mag Z: %.3f uT ", uDaneCM4.dane.fMagne2[2] * 1e6);
		RysujNapis(cNapis, KOL12, 100);

		if (cRysujRaz)
		{
			BelkaTytulu("Emulacja magnetometru CAN");
			cRysujRaz = 0;
			setColor(SZARY50);
			RysujNapis((char*)cOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 300);	//"Wdus ekran i trzymaj aby zakonczyc"
		}

		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_WYDAJN;
			uDaneCM7.dane.cWykonajPolecenie = POL7_NIC;	//zakończ tryb kalibracyjny
		}
		break;

	case TP_POMIAR_PGAP:
		uDaneCM7.dane.cWykonajPolecenie = POL7_CZYTAJ_CZAS_PETLI_GL;
		if ((uDaneCM4.dane.cPotwierdzenieWykonania == POL7_CZYTAJ_CZAS_PETLI_GL) && (uDaneCM4.dane.sAdres == uDaneCM7.dane.sAdres))
		{
			PokazCzasOdcinkowPGAP((uint16_t*)&uDaneCM4.dane.uRozne.U16[0]);
			uDaneCM7.dane.sAdres++;	//inkrementacja adresu zapewnia unikalność polecenia. CM4 nie wykonuje powtórzonych poleceń
		}
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_WYDAJN;
			uDaneCM7.dane.cWykonajPolecenie = POL7_NIC;
		}
		break;


	//*** Karta SD ************************************************
	case TP_KARTA_SD:			///menu Karta SD
		//Menu((char*)cNapisLcd[STR_MENU_KARTA_SD], stMenuKartaSD, &cNowyTrybPracy);
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_KARTA_SD]);
		cBłąd = Menu(cNapisPodreczny, stMenuKartaSD, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TPKS_WLACZ_REJ:
		cStatusRejestratora|= STATREJ_WLACZONY;
		WyswietlRejestratorKartySD();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_KARTA;
		}
		break;


	case TPKS_WYLACZ_REJ:	//najpierw zakmnij plik a potem wyłacz rejestrator
		cStatusRejestratora|= STATREJ_ZAMKNIJ_PLIK | STATREJ_BYL_OTWARTY;
		WyswietlRejestratorKartySD();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_KARTA;
		}
		break;


	case TPKS_PARAMETRY:
		WyswietlParametryKartySD();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_KARTA;
		}
		break;


	case TPKS_POMIAR:
		//TestKartySD();
		for (uint32_t y=0; y<DISP_Y_SIZE; y++)
		{
			for (uint32_t x=0; x<DISP_X_SIZE; x++)
			{
				cBuforLCD[y*DISP_X_SIZE*3 + x*3 + 0] = x & 0xFF;
				cBuforLCD[y*DISP_X_SIZE*3 + x*3 + 1] = y & 0xFF;
				cBuforLCD[y*DISP_X_SIZE*3 + x*3 + 2] = (y >> 8) & 0xFF;;
			}
		}
		cBłąd = ZapiszPlikBmp(cBuforLCD, BMP_KOLOR_24, DISP_X_SIZE, DISP_Y_SIZE);
		cBłąd = ZapiszPlikBin(cBuforLCD, DISP_X_SIZE * DISP_Y_SIZE * 3);
		cTrybPracy = cWrocDoTrybu;
		cNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


	case TPKS_ZAPISZ_BIN:	cBłąd = ZapiszPlikBin(cBuforLCD, DISP_X_SIZE * DISP_Y_SIZE * 3);
		cTrybPracy = cWrocDoTrybu;
		cNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


	case TPKS_ZAPISZ_BMP8:	cBłąd = ZapiszPlikBmp(cBuforLCD, BMP_KOLOR_8, DISP_X_SIZE, DISP_Y_SIZE);
		cTrybPracy = cWrocDoTrybu;
		cNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


	case TPKS_ZAP_BMP24_480:	cBłąd = ZapiszPlikBmp(cBuforLCD, BMP_KOLOR_24, DISP_X_SIZE, DISP_Y_SIZE);
		cTrybPracy = cWrocDoTrybu;
		cNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


	case TPKS_ZAP_BMP24_320:	cBłąd = ZapiszPlikBmp(cBuforLCD, BMP_KOLOR_24, 320, 240);
		cTrybPracy = cWrocDoTrybu;
		cNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


	case TPKS_ZAP_BMP24_240:	cBłąd = ZapiszPlikBmp(cBuforLCD, BMP_KOLOR_24, 240, 160);
		cTrybPracy = cWrocDoTrybu;
		cNowyTrybPracy = TP_WROC_DO_KARTA;
		break;


//*** IMU ************************************************
	case TP_KAL_IMU:			//menu kalibracji IMU
		//Menu((char*)cNapisLcd[STR_MENU_IMU], stMenuIMU, &cNowyTrybPracy);
		sprintf(cNapisPodreczny, "%s %s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_KALIBRACJA], cNapisLcd[STR_IMU]);
		cBłąd = Menu(cNapisPodreczny, stMenuIMU, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_KAL_ZYRO_ZIM:
		uDaneCM7.dane.cWykonajPolecenie = POL7_KALIBRUJ_ZYRO_ZIM;	//uruchom kalibrację żyroskopów na zimno 10°C
		fTemperaturaKalibracji = TEMP_KAL_ZIMNO;
		if ((uDaneCM4.dane.sPostepProcesu > 0) && (uDaneCM4.dane.sPostepProcesu < CZAS_KALIBRACJI))
			cTrybPracy = TP_PODGLAD_IMU;	//jeżeli proces kalibracji się zaczął to przejdź do trybu podgladu aby nie zaczynać nowego cyklu po zakończniu obecnego

		if ((uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZA_ZIMNO) || (uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZA_CIEPLO))
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WYSWIETL_BLAD;
		}
		break;


	case TP_KAL_ZYRO_POK:
		uDaneCM7.dane.cWykonajPolecenie = POL7_KALIBRUJ_ZYRO_POK;	//uruchom kalibrację żyroskopów w temperaturze pokojowej 25°C
		fTemperaturaKalibracji = TEMP_KAL_POKOJ;
		if ((uDaneCM4.dane.sPostepProcesu > 0) && (uDaneCM4.dane.sPostepProcesu < CZAS_KALIBRACJI))
			cTrybPracy = TP_PODGLAD_IMU;	//jeżeli proces kalibracji się zaczął to przejdź do trybu podgladu aby nie zaczynać nowego cyklu po zakończniu obecnego

		if ((uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZA_ZIMNO) || (uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZA_CIEPLO))
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WYSWIETL_BLAD;
		}
		break;


	case TP_KAL_ZYRO_GOR:
		uDaneCM7.dane.cWykonajPolecenie = POL7_KALIBRUJ_ZYRO_GOR;	//uruchom kalibrację żyroskopów na gorąco 40°C
		fTemperaturaKalibracji = TEMP_KAL_GORAC;
		if ((uDaneCM4.dane.sPostepProcesu > 0) && (uDaneCM4.dane.sPostepProcesu < CZAS_KALIBRACJI))
			cTrybPracy = TP_PODGLAD_IMU;	//jeżeli proces kalibracji się zaczął to przejdź do trybu podgladu aby nie zaczynać nowego cyklu po zakończniu obecnego

		if ((uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZA_ZIMNO) || (uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZA_CIEPLO))
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WYSWIETL_BLAD;
		}
		break;


	case TP_PODGLAD_IMU:
		uDaneCM7.dane.cWykonajPolecenie = POL7_NIC;		//gdy proces się rozpoczął wyłącz dalsze wysyłanie polecenia kalibracji
		if ((uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZA_ZIMNO) || (uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZA_CIEPLO))
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WYSWIETL_BLAD;
		}
		else
		{
			PomiaryAHRS();		//wyświetlaj wyniki pomiarów AHRS pobrane z CM4
			if (uDaneCM4.dane.sPostepProcesu)
				cCzasSwieceniaLED[LED_NIEB] = 5;	//świeć niebieskim LED w trakcie kalibracji
		}

		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_KAL_IMU;
			uDaneCM7.dane.cWykonajPolecenie = POL7_CZYSC_BLEDY;	//po zakończeniu wczyść zwrócony kod błędu
		}
		break;


	case TP_WYSWIETL_BLAD:
		if (uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZA_ZIMNO)
			WyswietlKomunikatBledu(KOMUNIKAT_ZA_ZIMNO, (uDaneCM4.dane.fTemper[TEMP_IMU1] + uDaneCM4.dane.fTemper[TEMP_IMU2])/2, fTemperaturaKalibracji, TEMP_KAL_ODCHYLKA);	//Wyświetl komunikat  o tym że jest za zimno i nominalna temperatura kalibracji to TEMP_KAL_POKOJ z odchyłką TEMP_KAL_ODCHYLKA
		else
		if (uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZA_CIEPLO)
			WyswietlKomunikatBledu(KOMUNIKAT_ZA_CIEPLO, (uDaneCM4.dane.fTemper[TEMP_IMU1] + uDaneCM4.dane.fTemper[TEMP_IMU2])/2, fTemperaturaKalibracji, TEMP_KAL_ODCHYLKA);	//Wyświetl komunikat  o tym że jest za ciepło i nominalna temperatura kalibracji to TEMP_KAL_POKOJ z odchyłką TEMP_KAL_ODCHYLKA

		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_KAL_IMU;
			uDaneCM7.dane.cWykonajPolecenie = POL7_CZYSC_BLEDY;	//po zakończeniu wczyść zwrócony kod błędu
		}
		break;


	case TP_KASUJ_DRYFT_ZYRO:
		uDaneCM7.dane.cWykonajPolecenie = POL7_KASUJ_DRYFT_ZYRO;
		if (uDaneCM4.dane.cPotwierdzenieWykonania == POL7_KASUJ_DRYFT_ZYRO)
		{
			uDaneCM7.dane.cWykonajPolecenie = POL7_NIC;
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_KAL_IMU;
		}
		break;


	case TP_KAL_WZM_ZYROR:
		cSekwencerKalibracji = SEKW_KAL_WZM_ZYRO_R;
		if (KalibracjaWzmocnieniaZyroskopow(&cSekwencerKalibracji) == BLAD_GOTOWE)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_KAL_IMU;
		}
		break;


	case TP_KAL_WZM_ZYROQ:
		cSekwencerKalibracji = SEKW_KAL_WZM_ZYRO_Q;
		if (KalibracjaWzmocnieniaZyroskopow(&cSekwencerKalibracji) == BLAD_GOTOWE)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_KAL_IMU;
		}
		break;


	case TP_KAL_WZM_ZYROP:
		cSekwencerKalibracji = SEKW_KAL_WZM_ZYRO_P;
		if (KalibracjaWzmocnieniaZyroskopow(&cSekwencerKalibracji) == BLAD_GOTOWE)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_KAL_IMU;
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
		//Menu((char*)cNapisLcd[STR_MENU_MAGNETOMETR], stMenuMagnetometr, &cNowyTrybPracy);
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_MAGNETOMETR]);
		cBłąd = Menu(cNapisPodreczny, stMenuMagnetometr, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		cSekwencerKalibracji = 0;
		break;


	case TP_MAG_KAL1:
		cSekwencerKalibracji |= MAG1 + KALIBRUJ;
		if (KalibracjaZeraMagnetometru(&cSekwencerKalibracji) == BLAD_GOTOWE)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_MAG_KAL2:
		cSekwencerKalibracji |= MAG2 + KALIBRUJ;
		if (KalibracjaZeraMagnetometru(&cSekwencerKalibracji) == BLAD_GOTOWE)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_MAG_KAL3:
		cSekwencerKalibracji |= MAG3 + KALIBRUJ;
		if (KalibracjaZeraMagnetometru(&cSekwencerKalibracji) == BLAD_GOTOWE)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_MAG1:	break;
	case TP_MAG2:	break;
	case TP_SPR_PLASKI:	PlaskiObrotMagnetometrow();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_SPR_MAG1:
		cSekwencerKalibracji |= MAG1;	//sprawdzenie, bez kalibracji
		if (KalibracjaZeraMagnetometru(&cSekwencerKalibracji) == BLAD_GOTOWE)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_SPR_MAG2:
		cSekwencerKalibracji |= MAG2;	//sprawdzenie, bez kalibracji
		if (KalibracjaZeraMagnetometru(&cSekwencerKalibracji) == BLAD_GOTOWE)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


	case TP_SPR_MAG3:
		cSekwencerKalibracji |= MAG3;	//sprawdzenie, bez kalibracji
		if (KalibracjaZeraMagnetometru(&cSekwencerKalibracji) == BLAD_GOTOWE)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_MAG;
		}
		break;


//*** Kalibracje ************************************************
	case TP_KALIBRACJE:		//menu skupiające różne kalibracje
		//Menu((char*)cNapisLcd[STR_MENU_KALIBRACJE], stMenuKalibracje, &cNowyTrybPracy);
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_KALIBRACJE]);
		cBłąd = Menu(cNapisPodreczny, stMenuKalibracje, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_KAL_DOTYK:
		cNowyTrybPracy = 0;			//nowy tryb jest ustawiany po starcie dla normalnej pracy ze skalibrowanym panelem. Jednak w przypadku potrzeby kalobracji,
										//powoduje to wyczyszczenie opisu i pierwszego krzyżyka i nie wiadomo o co chodzi, więc kasuję chNowyTrybPracy
		if (KalibrujDotyk() == BLAD_GOTOWE)
			cTrybPracy = TP_POMIARY_DOTYKU;
		break;


//*** Pomiary ************************************************
	case TP_POMIARY:		//menu skupiające różne kalibracje
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_POMIARY]);
		cBłąd = Menu(cNapisPodreczny, stMenuPomiary, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_POMIARY_AHRS:	PomiaryAHRS();		//wyświetlaj wyniki pomiarów IMU pobrane z CM4
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cNowyTrybPracy = TP_WROC_DO_POMIARY;
			ZatrzymajTon();
		}
		break;


	case TP_POMIARY_CZUJN:	PomiaryCzujnikow();
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_POMIARY;
		}
		break;


	case TP_POMIARY_RC:	//DaneOdbiornikaRC();
		RysujPaskiKanalowRC(STR_DANE_ODBIORNIKA_RC, (uint16_t *)uDaneCM4.dane.sKanalRC);
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_POMIARY;
		}
		break;


	case TP_POMIARY_SERWA:	RysujPaskiKanalowRC(STR_DANE_WYJSC_RC, (uint16_t *)uDaneCM4.dane.sWyjscieRC);
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_POMIARY;
		}
		break;

	case TP_POMIARY_ANALIZA_DRGAN:
		if (cRysujRaz)
			RozpocznijAnalizęDrgań(&stKonfigFFT, &cTrybPracy);
		if (stKonfigFFT.chStatus & FFT_NOWE_DANE)
			KrokAnalizyDrgań(&stKonfigFFT, &cTrybPracy);
		//intencjonalnie brakuje break; aby po zakończeniu wszedł w obsługę rysowania FFT

	case TP_POMIARY_FFT_ACC:	RysujFFT(&fWynikFFT[0][0][0], &stKonfigFFT, FFT_ACC);	//FFT akcelerometrów
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_POMIARY;
			uDaneCM7.dane.cWykonajPolecenie = POL7_NIC;	//kasuj ewentualne polecenie wysterowania silników analizy drgań
		}
		break;

	case TP_POMIARY_FFT_ZYR:	RysujFFT(&fWynikFFT[0][0][0], &stKonfigFFT, FFT_ZYR);	//FFT żyroskopów
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_POMIARY;
		}
		break;

	case TP_WITAJ:
		if (!chLiczIter)
			chLiczIter = 15;			//ustaw czas wyświetlania x 200ms
		Ekran_Powitalny(nZainicjowanoCM7);	//przywitaj użytkownika i prezentuj wykryty sprzęt
		if (!chLiczIter)				//jeżeli koniec odliczania to wyjdź
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_POMIARY;
		}
		break;


//*** Nastawy ************************************************
	case TP_NASTAWY:		//menu skupiające różne kalibracje
		sprintf(cNapisPodreczny, "%s %s", cNapisLcd[STR_MENU], cNapisLcd[STR_NASTAWY]);
		cBłąd = Menu(cNapisPodreczny, stMenuNastawy, &cNowyTrybPracy);
		cWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_NAST_PID_PRZECH:		//regulator sterowania przechyleniem (lotkami w samolocie)
		NastawyPID(PID_KĄTA_PRZE);
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_PID_POCH:	//regulator sterowania pochyleniem (sterem wysokości)
		NastawyPID(PID_KĄTA_POCH);
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_PID_ODCH:		//regulator sterowania odchyleniem (sterem kierunku)
		NastawyPID(PID_KĄTA_ODCH);
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_PID_WYSOK:		//regulator sterowania wysokością
		NastawyPID(PID_WYSOKOSCI);
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_PID_NAWIG_PÓŁN:		//regulator sterowania nawigacją w kierunku północnym
		NastawyPID(PID_NAWIG_PÓŁN);
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_PID_NAWIG_WSCH:		//regulator sterowania nawigacją w kierunku wschodnim
		NastawyPID(PID_NAWIG_WSCH);
		if(stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			cTrybPracy = cWrocDoTrybu;
			cNowyTrybPracy = TP_WROC_DO_NASTAWY;
		}
		break;


	case TP_NAST_MIKSERA:		break;
	case TP_NAST_IDENT_SILN:	RozpocznijIdentyfikacjęSilników(&stIdentSiln, &cTrybPracy); break;

	case TP_PROCES_IDENT_SILN:
		if (cRysujRaz)
		{
			cRysujRaz = 0;
			RysujProstokatWypelniony(0, 0, DISP_X_SIZE, DISP_Y_SIZE, CZARNY);	//czyści ekran
			BelkaTytulu("Identyfikacja silnikow");
		}
		nCzas = MinalCzas(stIdentSiln.nCzasPoprzedniegoEtapu);
		if ((uint16_t)(nCzas / 1000) >= stIdentSiln.sCzasIdent)
		{
			stIdentSiln.nCzasPoprzedniegoEtapu = PobierzCzasT6();
			stIdentSiln.cNumerEtapu++;		//etap jest inicjowany zerem, więc w pętli zaczyna się od 1

			if (stIdentSiln.cNumerEtapu > stIdentSiln.cLiczbaSilnikow)
			{
				cTrybPracy = cWrocDoTrybu;
				cNowyTrybPracy = TP_WROC_DO_NASTAWY;
				uDaneCM7.dane.sAdres = 0;	//dowolna liczba inna niż poprzednie
				uDaneCM7.dane.cWykonajPolecenie = POL7_PRZYWROC_NAPED;
			}
			else
			{
				//aby CM4 przyjął kolejne takie samo polecenie, musi zmienić się zmienna sAdres, więc pomimo różnicy rozmiaru wykorzystuję ją do przesyłania nr silnika
				uDaneCM7.dane.sAdres = (1 << (stIdentSiln.cNumerEtapu - 1));	//bit numeru silnika
				uDaneCM7.dane.uRozne.U16[0] = stIdentSiln.sWysterowanie;
				uDaneCM7.dane.cWykonajPolecenie = POL7_URUCHOM_INDENT_SILN;

				sprintf(cNapis, "Silnik: %d", stIdentSiln.cNumerEtapu);
				RysujNapis(cNapis, 1, 30);
				sprintf(cNapis, "Wysterowanie: %d",stIdentSiln.sWysterowanie);
				RysujNapis(cNapis, 1, 50);

				cBłąd = DodajProbkeDoKolejki(PGA_SILNIK);
				cBłąd = DodajProbkeDoKolejki(PGA_00 + stIdentSiln.cNumerEtapu);	//cyfra 1..8

				if (stIdentSiln.fSkładowaPochylenia[stIdentSiln.cNumerEtapu - 1] > 1.0)
					cBłąd = DodajProbkeDoKolejki(PGA_PRZEDNI);
				else
				if (stIdentSiln.fSkładowaPochylenia[stIdentSiln.cNumerEtapu - 1] < -1.0)
					cBłąd = DodajProbkeDoKolejki(PGA_TYLNY);

				if (stIdentSiln.fSkładowaPrzechylenia[stIdentSiln.cNumerEtapu - 1] > 1.0)
					cBłąd = DodajProbkeDoKolejki(PGA_PRAWY);
				else
				if (stIdentSiln.fSkładowaPrzechylenia[stIdentSiln.cNumerEtapu - 1] < -1.0)
					cBłąd = DodajProbkeDoKolejki(PGA_LEWY);
			}
		}
		break;

	default:
		printf("zly tryb pracy\n\r");
	}


	//rzeczy do wykonania podczas uruchamiania nowego trybu pracy
	if (cNowyTrybPracy)
	{
		switch(cNowyTrybPracy)
		{
		case TP_WROC_DO_MENU:		cTrybPracy = TP_MENU_GLOWNE;	break;	//powrót do menu głównego
		case TP_WROC_DO_AUDIO:		cTrybPracy = TP_MEDIA_AUDIO;	break;	//powrót do menu Audio
		case TP_WROC_DO_KAMERA:		cTrybPracy = TP_MEDIA_KAMERA;	break;	//powrót do menu Kamera
		case TP_WROC_DO_OSD:		cTrybPracy = TP_MENU_OSD;		break;	//powrót do menu OSD
		case TP_WROC_DO_WYDAJN:		cTrybPracy = TP_WYDAJNOSC;		break;	//powrót do menu Wydajność
		case TP_WROC_DO_KARTA:		cTrybPracy = TP_KARTA_SD;		break;	//powrót do menu Karta SD
		case TP_WROC_KAL_IMU:		cTrybPracy = TP_KAL_IMU;		break;	//powrót do menu IMU
		case TP_WROC_DO_MAG:		cTrybPracy = TP_MAGNETOMETR;	break;	//powrót do menu Magnetometr
		case TP_WROC_DO_POMIARY:	cTrybPracy = TP_POMIARY;		break;	//powrót do menu Pomiary
		case TP_WROC_DO_NASTAWY:	cTrybPracy = TP_NASTAWY;		break;	//powrót do menu Nastawy
		case TP_FRAKTALE:			InitFraktal(START_FRAKTAL);		cTrybPracy = TP_FRAKTALE;	break;
		case TP_WROC_DO_ETH:		cTrybPracy = TP_ETHERNET;		break;	//powrót do menu Ethernet
		case TP_WROC_DO_TESTY:		cTrybPracy = TP_TESTY;			break;	//powrót do menu Testy
		default:
			RysujProstokatWypelniony(0, 0, DISP_X_SIZE, DISP_Y_SIZE, CZARNY);	//czyści ekran
			cTrybPracy = cNowyTrybPracy;	break;	//typowe wywołanie pozycji z menu
		}
		cNowyTrybPracy = 0;
		stStatusDotyku.chFlagi &= ~(DOTYK_DOTKNIETO | DOTYK_ZWOLNONO);	//czyść flagi ekranu dotykowego aby móc reagować na nie w trakcie pracy danego trybu
		cRysujRaz = 1;		//jednorazowo rysuj statyczne elementy nowego ekranu
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran inicjalizacji sprzętu i wyświetla numer wersji
// Parametry: zainicjowano - wskaźnik na tablicę bitów z flagami zainicjowanego sprzętu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t Ekran_Powitalny(uint32_t nZainicjowano)
{
	uint8_t n, cBłąd;
	uint16_t x, y;

	extern const unsigned short plogo165x80[];
	extern uint8_t BSP_SD_IsDetected(void);
	extern stBSP_ID_t stBSP_ID;	//struktura zawierajaca adres i nazwę BSP

	if (cRysujRaz)
	{
		cBłąd = WypelnijEkran(BIALY);
		RysujBitmape((DISP_X_SIZE-165)/2, 5, 165, 80, plogo165x80);

		setColor(SZARY20);
		setBackColor(BIALY);
		UstawCzcionke(BigFont);
		sprintf(cNapis, "%s @%luMHz", (char*)cNapisLcd[STR_WITAJ_TYTUL], HAL_RCC_GetSysClockFreq()/1000000);
		RysujNapis(cNapis, CENTER, 90);

		setColor(SZARY30);
		UstawCzcionke(MidFont);
		//sprintf(cNapis, (char*)cNapisLcd[STR_WITAJ_MOTTO], ó, ć, ó, ó, ż);	//"By móc mieć w rój Wronów na pohybel wrażym hordom""
		sprintf(cNapis, (char*)cNapisLcd[STR_WITAJ_MOTTO2], ó, ó, ż, ó);	//"By móc zmóc wraże hordy rojem Wronów"	//STR_WITAJ_MOTTO2
		RysujNapis(cNapis, CENTER, 115);

		sprintf(cNapis, "Adres: %d, IP: %d.%d.%d.%d, Nazwa: %s", stBSP_ID.chAdres, stBSP_ID.chAdrIP[0], stBSP_ID.chAdrIP[1], stBSP_ID.chAdrIP[2], stBSP_ID.chAdrIP[3],  stBSP_ID.chNazwa);
		RysujNapis(cNapis, CENTER, 135);

		sprintf(cNapis, "(c) PitLab 2026 sv%d.%d.%d @ %s %s", WER_GLOWNA, WER_PODRZ, WER_REPO, build_date, build_time);
		RysujNapis(cNapis, CENTER, 155);
		cRysujRaz = 0;
	}

	//pierwsza kolumna sprzętu wewnętrznego
	x = KOL12;
	y = WYKRYJ_GORA;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_FLASH_NOR]);		//"pamięć Flash NOR"
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, (nZainicjowano & INIT_FLASH_NOR) == INIT_FLASH_NOR);

	y += WYKRYJ_WIERSZ;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_FLASH_QSPI]);	//"pamięć Flash QSPI"
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, (nZainicjowano & INIT_FLASH_QSPI) == INIT_FLASH_QSPI);

	y += WYKRYJ_WIERSZ;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_KARTA_SD]);		//"Karta SD"
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, BSP_SD_IsDetected());

	y += WYKRYJ_WIERSZ;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_KAMERA_OV5642]);	//"kamera "
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, (nZainicjowano & INIT_KAMERA) == INIT_KAMERA);

	//dane z CM4
	y += WYKRYJ_WIERSZ;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_GNSS]);		//GNSS
	if (uDaneCM4.dane.nZainicjowano & INIT_WYKR_UBLOX)
		n = sprintf(cNapis, "%s -> %s", (char*)cNapisLcd[STR_SPRAWDZ_GNSS], (char*)cNapisLcd[STR_SPRAWDZ_UBLOX]);	//GNSS -> uBlox
	if (uDaneCM4.dane.nZainicjowano & INIT_WYKR_MTK)
		n = sprintf(cNapis, "%s -> %s", (char*)cNapisLcd[STR_SPRAWDZ_GNSS], (char*)cNapisLcd[STR_SPRAWDZ_MTK]);		//GNSS -> MTK
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n,  (uDaneCM4.dane.nZainicjowano & INIT_GNSS_GOTOWY) == INIT_GNSS_GOTOWY);

	y += WYKRYJ_WIERSZ;
	n = sprintf(cNapis, "%s -> %s", (char*)cNapisLcd[STR_SPRAWDZ_GNSS], (char*)cNapisLcd[STR_SPRAWDZ_HMC5883]);	//GNSS -> HMC5883
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_HMC5883) == INIT_HMC5883);

	//druga kolumna sprzętu zewnętrznego
	x = KOL22;
	y = WYKRYJ_GORA;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_IMU1_MS5611]);		//moduł IMU -> MS5611
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_MS5611) == INIT_MS5611);

	y += WYKRYJ_WIERSZ;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_IMU1_BMP581]);		//moduł IMU -> BMP581
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_BMP581) == INIT_BMP581);

	y += WYKRYJ_WIERSZ;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_IMU1_ICM42688]);		//moduł IMU -> ICM42688
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_ICM42688) == INIT_ICM42688);

	y += WYKRYJ_WIERSZ;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_IMU1_LSM6DSV]);		//moduł IMU -> LSM6DSV
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV) == INIT_LSM6DSV);

	y += WYKRYJ_WIERSZ;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_IMU1_MMC34160]);		//moduł IMU -> MMC34160
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_MMC34160) == INIT_MMC34160);

	y += WYKRYJ_WIERSZ;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_IMU1_IIS2MDC]);		//moduł IMU -> IIS2MDC
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC) == INIT_IIS2MDC);

	y += WYKRYJ_WIERSZ;
	n = sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_IMU1_ND130]);		//moduł IMU -> ND130
	RysujNapis(cNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_ND130) == INIT_ND130);

	osDelay(200);
	chLiczIter--;
	return cBłąd;
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
		sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_WYKR]);	//"wykryto"
	}
	else
	{
		setColor(KOLOR_X);	//czerwony
		sprintf(cNapis, (char*)cNapisLcd[STR_SPRAWDZ_BRAK]);	//"brakuje"
	}
	x += kropek * szer_fontu;
	RysujNapis(cNapis, x , y);
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
	if (cRysujRaz)
	{
		WypelnijEkran(CZARNY);
		cRysujRaz = 0;
	}
//		setBackColor(CZARNY);

	//nagłówek komunikatu
	setColor(CZERWONY);
	UstawCzcionke(BigFont);
	sprintf(cNapis, (char*)cOpisBledow[KOMUNIKAT_NAGLOWEK]);	//"Blad wykonania polecenia!",
	RysujNapis(cNapis, CENTER, 70);

	//stopka komunikatu
	UstawCzcionke(MidFont);
	setColor(SZARY50);
	RysujNapis((char*)cOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 250);	//"Wdus ekran i trzymaj aby zakonczyc"

	//treść komunikatu
	setColor(ZOLTY);
	switch(chKomunikatBledu)
	{
	case KOMUNIKAT_ZA_ZIMNO:	//sposób formatowania komunikatu taki sam jak dla BLAD_ZA_CIEPLO
	case KOMUNIKAT_ZA_CIEPLO:	//parametr1 to bieżąca temperatura, parametr 2 to nominalna temperatura kalibracji, parametr 3 to zakres tolerancji odchyłki temperatury
		sprintf(cNapis, (const char*)cOpisBledow[chKomunikatBledu], fParametr1 - KELVIN, ZNAK_STOPIEN, fParametr2 - fParametr3 - KELVIN, ZNAK_STOPIEN, fParametr2 + fParametr3 -  KELVIN, ZNAK_STOPIEN);	break;	//"Zbyt niska temeratura zyroskopow wynoszaca %d%cC. Musi miescic sie w granicach od %d%cC do %d%cC",
	}
	RysujNapiswRamce(cNapis, 20, 90, 440, 200);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z damymi pomiarowymi IMU
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PomiaryAHRS(void)
{
	int8_t chTon;
	float fDlugosc;

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		BelkaTytulu("Dane pomiarowe AHRS");

		setColor(SZARY80);
		sprintf(cNapis, "Akcel1:");
		RysujNapis(cNapis, KOL12, 30);
		sprintf(cNapis, "[m/s^2]");
		RysujNapis(cNapis, KOL12+40*FONT_SL, 30);

		sprintf(cNapis, "Akcel2:");
		RysujNapis(cNapis, KOL12, 50);
		sprintf(cNapis, "[m/s^2]");
		RysujNapis(cNapis, KOL12+40*FONT_SL, 50);

		sprintf(cNapis, "Zyro 1:");
		RysujNapis(cNapis, KOL12, 70);
		sprintf(cNapis, "[rad/s]");
		RysujNapis(cNapis, KOL12+40*FONT_SL, 70);

		sprintf(cNapis, "Zyro 2:");
		RysujNapis(cNapis, KOL12, 90);
		sprintf(cNapis, "[rad/s]");
		RysujNapis(cNapis, KOL12+40*FONT_SL, 90);

		sprintf(cNapis, "Magn 1:");
		RysujNapis(cNapis, KOL12, 110);
		sprintf(cNapis, "[uT]");
		RysujNapis(cNapis, KOL12+40*FONT_SL, 110);

		sprintf(cNapis, "Magn 2:");
		RysujNapis(cNapis, KOL12, 130);
		sprintf(cNapis, "[uT]");
		RysujNapis(cNapis, KOL12+40*FONT_SL, 130);

		sprintf(cNapis, "Magn 3:");
		RysujNapis(cNapis, KOL12, 150);
		sprintf(cNapis, "[uT]");
		RysujNapis(cNapis, KOL12+40*FONT_SL, 150);

		sprintf(cNapis, "K%cty 1:", ą);
		RysujNapis(cNapis, KOL12, 170);
		sprintf(cNapis, "[%c]", ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12+40*FONT_SL, 170);

		sprintf(cNapis, "K%cty 2:", ą);
		RysujNapis(cNapis, KOL12, 190);
		sprintf(cNapis, "[%c]", ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12+40*FONT_SL, 190);

		sprintf(cNapis, "K%cty Akc1:", ą);
		RysujNapis(cNapis, KOL12, 210);
		sprintf(cNapis, "[%c]", ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12+46*FONT_SL, 210);

		sprintf(cNapis, "K%cty Akc2:", ą);
		RysujNapis(cNapis, KOL12, 230);
		sprintf(cNapis, "[%c]", ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12+46*FONT_SL, 230);

		//sprintf(cNapis, "K%cty %cyro 1:", ą, ż);
		sprintf(cNapis, "Kwaternion Akc:");
		RysujNapis(cNapis, KOL12, 250);
		//sprintf(cNapis, "K%cty %cyro 2:", ą, ż);
		sprintf(cNapis, "Kwaternion Mag:");
		RysujNapis(cNapis, KOL12, 270);

		setColor(SZARY50);
		RysujNapis((char*)cOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 300);	//"Wdus ekran i trzymaj aby zakonczyc"
	}

	//ICM42688
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(cNapis, "%.3f ", uDaneCM4.dane.fAkcel1[0]);
	RysujNapis(cNapis, KOL12+8*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.3f ", uDaneCM4.dane.fAkcel1[1]);
	RysujNapis(cNapis, KOL12+20*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.3f ", uDaneCM4.dane.fAkcel1[2]);
	RysujNapis(cNapis, KOL12+32*FONT_SL, 30);

	//LSM6DSV
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(cNapis, "%.3f ", uDaneCM4.dane.fAkcel2[0]);
	RysujNapis(cNapis, KOL12+8*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.3f ", uDaneCM4.dane.fAkcel2[1]);
	RysujNapis(cNapis, KOL12+20*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.3f ", uDaneCM4.dane.fAkcel2[2]);
	RysujNapis(cNapis, KOL12+32*FONT_SL, 50);

	//ICM42688
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fZyroKal1[0]);
	RysujNapis(cNapis, KOL12+8*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fZyroKal1[1]);
	RysujNapis(cNapis, KOL12+20*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fZyroKal1[2]);
	RysujNapis(cNapis, KOL12+32*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_IMU1] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	RysujNapis(cNapis, KOL12+49*FONT_SL, 70);

	//LSM6DSV
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fZyroKal2[0]);
	RysujNapis(cNapis, KOL12+8*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fZyroKal2[1]);
	RysujNapis(cNapis, KOL12+20*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fZyroKal2[2]);
	RysujNapis(cNapis, KOL12+32*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_IMU2] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	RysujNapis(cNapis, KOL12+49*FONT_SL, 90);

	//IIS2MDC
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(cNapis, "%.2f ", uDaneCM4.dane.fMagne1[0]*1e6);
	RysujNapis(cNapis, KOL12+8*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.2f ", uDaneCM4.dane.fMagne1[1]*1e6);
	RysujNapis(cNapis, KOL12+20*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.2f ", uDaneCM4.dane.fMagne1[2]*1e6);
	RysujNapis(cNapis, KOL12+32*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(POMARANCZ); 	else	setColor(SZARY50);
	fDlugosc = sqrtf(uDaneCM4.dane.fMagne1[0] * uDaneCM4.dane.fMagne1[0] + uDaneCM4.dane.fMagne1[1] * uDaneCM4.dane.fMagne1[1] + uDaneCM4.dane.fMagne1[2] * uDaneCM4.dane.fMagne1[2]);
	sprintf(cNapis, "%.1f %% ", fDlugosc / NOMINALNE_MAGN * 100);	//długość wektora [%]
	RysujNapis(cNapis, KOL12+49*FONT_SL, 110);

	//MMC34160
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(cNapis, "%.2f ", uDaneCM4.dane.fMagne2[0]*1e6);
	RysujNapis(cNapis, KOL12+8*FONT_SL, 130);
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.2f ", uDaneCM4.dane.fMagne2[1]*1e6);
	RysujNapis(cNapis, KOL12+20*FONT_SL, 130);
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.2f ", uDaneCM4.dane.fMagne2[2]*1e6);
	RysujNapis(cNapis, KOL12+32*FONT_SL, 130);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(POMARANCZ); 	else	setColor(SZARY50);
	fDlugosc = sqrtf(uDaneCM4.dane.fMagne2[0] * uDaneCM4.dane.fMagne2[0] + uDaneCM4.dane.fMagne2[1] * uDaneCM4.dane.fMagne2[1] + uDaneCM4.dane.fMagne2[2] * uDaneCM4.dane.fMagne2[2]);
	sprintf(cNapis, "%.1f %% ", fDlugosc / NOMINALNE_MAGN * 100);	//długość wektora [%]
	RysujNapis(cNapis, KOL12+49*FONT_SL, 130);

	//HMC5883
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_X); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(cNapis, "%.2f ", uDaneCM4.dane.fMagne3[0]*1e6);
	RysujNapis(cNapis, KOL12+8*FONT_SL, 150);
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_Y); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.2f ", uDaneCM4.dane.fMagne3[1]*1e6);
	RysujNapis(cNapis, KOL12+20*FONT_SL, 150);
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_Z); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.2f ", uDaneCM4.dane.fMagne3[2]*1e6);
	RysujNapis(cNapis, KOL12+32*FONT_SL, 150);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(POMARANCZ); 	else	setColor(SZARY50);
	fDlugosc = sqrtf(uDaneCM4.dane.fMagne3[0] * uDaneCM4.dane.fMagne3[0] + uDaneCM4.dane.fMagne3[1] * uDaneCM4.dane.fMagne3[1] + uDaneCM4.dane.fMagne3[2] * uDaneCM4.dane.fMagne3[2]);
	sprintf(cNapis, "%.1f %% ", fDlugosc / NOMINALNE_MAGN * 100);	//długość wektora [%]
	RysujNapis(cNapis, KOL12+49*FONT_SL, 150);

	//sygnalizacja tonem wartości osi Z magnetometru
	chTon = LICZBA_TONOW_WARIO/2 - (uDaneCM4.dane.fMagne3[2] / (NOMINALNE_MAGN / (LICZBA_TONOW_WARIO/2)));
	if (chTon > LICZBA_TONOW_WARIO)
		chTon = LICZBA_TONOW_WARIO;
	if (chTon < 0)
		chTon = 0;
	//UstawTon(chTon, 80);

	setColor(KOLOR_X);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatIMU1[0]);
	RysujNapis(cNapis, KOL12+8*FONT_SL, 170);
	setColor(KOLOR_Y);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatIMU1[1]);
	RysujNapis(cNapis, KOL12+20*FONT_SL, 170);
	setColor(KOLOR_Z);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatIMU1[2]);
	RysujNapis(cNapis, KOL12+32*FONT_SL, 170);

	setColor(KOLOR_X);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatIMU2[0]);
	RysujNapis(cNapis, KOL12+8*FONT_SL, 190);
	setColor(KOLOR_Y);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatIMU2[1]);
	RysujNapis(cNapis, KOL12+20*FONT_SL, 190);
	setColor(KOLOR_Z);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatIMU2[2]);
	RysujNapis(cNapis, KOL12+32*FONT_SL, 190);

	//kąty z akcelrometru 1
	setColor(KOLOR_X);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatAkcel1[0]);
	RysujNapis(cNapis, KOL12+11*FONT_SL, 210);
	setColor(KOLOR_Y);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatAkcel1[1]);
	RysujNapis(cNapis, KOL12+24*FONT_SL, 210);
	setColor(KOLOR_Z);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatAkcel1[2]);
	RysujNapis(cNapis, KOL12+36*FONT_SL, 210);

	//kąty z akcelrometru 2
	setColor(KOLOR_X);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatAkcel2[0]);
	RysujNapis(cNapis, KOL12+11*FONT_SL, 230);
	setColor(KOLOR_Y);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatAkcel2[1]);
	RysujNapis(cNapis, KOL12+24*FONT_SL, 230);
	setColor(KOLOR_Z);
	sprintf(cNapis, "%.2f ", RAD2DEG * uDaneCM4.dane.fKatAkcel2[2]);
	RysujNapis(cNapis, KOL12+36*FONT_SL, 230);

	/*/kąty z żyroskopu 1
	setColor(KOLOR_X);
	sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[0], ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12+13*FONT_SL, 250);
	setColor(KOLOR_Y);
	sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[1], ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12+25*FONT_SL, 250);
	setColor(KOLOR_Z);
	sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[2], ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12+37*FONT_SL, 250);

	//kąty z żyroskopu 2
	setColor(KOLOR_X);
	sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[0], ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12+13*FONT_SL, 270);
	setColor(KOLOR_Y);
	sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[1], ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12+25*FONT_SL, 270);
	setColor(KOLOR_Z);
	sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[2], ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12+37*FONT_SL, 270); */

	//kwaternion wektora przyspieszenia
	setColor(POMARANCZ);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[0]);
	RysujNapis(cNapis, KOL12+16*FONT_SL, 250);
	setColor(KOLOR_X);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[1]);
	RysujNapis(cNapis, KOL12+26*FONT_SL, 250);
	setColor(KOLOR_Y);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[2]);
	RysujNapis(cNapis, KOL12+36*FONT_SL, 250);
	setColor(KOLOR_Z);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[3]);
	RysujNapis(cNapis, KOL12+46*FONT_SL, 250);

	//kwaternion wektora magnetycznego
	setColor(POMARANCZ);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fKwaMag[0]);
	RysujNapis(cNapis, KOL12+16*FONT_SL, 270);
	setColor(KOLOR_X);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fKwaMag[1]);
	RysujNapis(cNapis, KOL12+26*FONT_SL, 270);
	setColor(KOLOR_Y);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fKwaMag[2]);
	RysujNapis(cNapis, KOL12+36*FONT_SL, 270);
	setColor(KOLOR_Z);
	sprintf(cNapis, "%.4f ", uDaneCM4.dane.fKwaMag[3]);
	RysujNapis(cNapis, KOL12+46*FONT_SL, 270);

	//Rysuj pasek postepu jeżeli trwa jakiś proces. Zakładam że czas procesu jest zmniejszany od wartości CZAS_KALIBRACJI do zera
	RysujPasekPostepu(CZAS_KALIBRACJI);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z damymi pomiarowymi czujników ciśnienia i GPS
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PomiaryCzujnikow(void)
{
	if (cRysujRaz)
	{
		cRysujRaz = 0;
		BelkaTytulu("Dane czujnikow");

		setColor(SZARY80);
		sprintf(cNapis, "Ci%cn1:           AGL1:           V1:            T:", ś);
		RysujNapis(cNapis, KOL12, 30);
		sprintf(cNapis, "Ci%cn2:           AGL2:           V2:            T:", ś);
		RysujNapis(cNapis, KOL12, 50);
		sprintf(cNapis, "Ci%cR%c%cn 1:          IAS1:", ś, ó, ż);
		RysujNapis(cNapis, KOL12, 70);
		sprintf(cNapis, "Ci%cR%c%cn 2:          IAS2:", ś, ó, ż);
		RysujNapis(cNapis, KOL12, 90);

		sprintf(cNapis, "GNSS D%cug:             Szer:             HDOP:", ł);
		RysujNapis(cNapis, KOL12, 120);
		sprintf(cNapis, "GNSS WysMSL:           Pred:             Kurs:");
		RysujNapis(cNapis, KOL12, 140);
		sprintf(cNapis, "GNSS Czas:             Data:              Sat:");
		RysujNapis(cNapis, KOL12, 160);

		//sprintf(cNapis, "Uaku 1:            Iaku 1:            Eaku 1:");
		sprintf(cNapis, "Uaku 1:            Iaku 1:            Vin 1:");
		RysujNapis(cNapis, KOL12, 190);
		//sprintf(cNapis, "Uaku 2:            Iaku 2:            Eaku 2:");
		sprintf(cNapis, "Uaku 2:            Iaku 2:            Vin 2:");
		RysujNapis(cNapis, KOL12, 210);
		sprintf(cNapis, "Uz.ser:            Uz.usb:            Ub.zeg:");
		RysujNapis(cNapis, KOL12, 230);
		sprintf(cNapis, "Ucz1.1:            Ucz1.2:            TmpCPU:");
		RysujNapis(cNapis, KOL12, 260);
		sprintf(cNapis, "Ucz2.1:            Ucz2.2:");
		RysujNapis(cNapis, KOL12, 280);


		setColor(SZARY50);
		RysujNapis((char*)cOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 300);	//"Wdus ekran i trzymaj aby zakonczyc"
	}

	//MS5611
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(BIALY); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(cNapis, "%.0f Pa ", uDaneCM4.dane.fCisnieBzw[0]);
	RysujNapis(cNapis, KOL12 + 7*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(CYJAN); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.2f m ", uDaneCM4.dane.fWysokoMSL[0]);
	RysujNapis(cNapis, KOL12 + 23*FONT_SL, 30);
	sprintf(cNapis, "%.2f m/s ", uDaneCM4.dane.fWariometr[0]);
	RysujNapis(cNapis, KOL12 + 37*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_BARO1] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	RysujNapis(cNapis, KOL12 + 51*FONT_SL, 30);

	//BMP581
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(BIALY); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(cNapis, "%.0f Pa ", uDaneCM4.dane.fCisnieBzw[1]);
	RysujNapis(cNapis, KOL12 + 7*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(CYJAN); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.2f m ", uDaneCM4.dane.fWysokoMSL[1]);
	RysujNapis(cNapis, KOL12 + 23*FONT_SL, 50);
	sprintf(cNapis, "%.2f m/s ", uDaneCM4.dane.fWariometr[1]);
	RysujNapis(cNapis, KOL12 + 37*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_BARO2] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	RysujNapis(cNapis, KOL12 + 51*FONT_SL, 50);

	//ND130
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(BIALY); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(cNapis, "%.0f Pa ", uDaneCM4.dane.fCisnRozn[0]);
	RysujNapis(cNapis, KOL12 + 11*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(MAGENTA); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.2f m/s ", uDaneCM4.dane.fPredkosc[0]);
	RysujNapis(cNapis, KOL12 + 26*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_CISR1] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	RysujNapis(cNapis, KOL12 + 40*FONT_SL, 70);

	//MS4525
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(BIALY); 	else	setColor(SZARY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(cNapis, "%.0f Pa ", uDaneCM4.dane.fCisnRozn[1]);
	RysujNapis(cNapis, KOL12 + 11*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(MAGENTA); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.2f m/s ", uDaneCM4.dane.fPredkosc[1]);
	RysujNapis(cNapis, KOL12 + 26*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(ZOLTY); 	else	setColor(SZARY50);
	sprintf(cNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_CISR2] - KELVIN , ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	RysujNapis(cNapis, KOL12 + 40*FONT_SL, 90);

	//dane z GNSS
	if (uDaneCM4.dane.stGnss1.chFix)
		setColor(BIALY);	//jest fix
	else
		setColor(SZARY70);	//nie ma fixa

	sprintf(cNapis, "%.7f ", uDaneCM4.dane.stGnss1.dDlugoscGeo);
	RysujNapis(cNapis, KOL12 + 11*FONT_SL, 120);
	sprintf(cNapis, "%.7f ", uDaneCM4.dane.stGnss1.dSzerokoscGeo);
	RysujNapis(cNapis, KOL12 + 29*FONT_SL, 120);
	sprintf(cNapis, "%.2f ", uDaneCM4.dane.stGnss1.fHdop);
	RysujNapis(cNapis, KOL12 + 47*FONT_SL, 120);

	sprintf(cNapis, "%.1f m ", uDaneCM4.dane.stGnss1.fWysokoscMSL);
	RysujNapis(cNapis, KOL12 + 13*FONT_SL, 140);
	sprintf(cNapis, "%.3f m/s ", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi);
	RysujNapis(cNapis, KOL12 + 29*FONT_SL, 140);
	sprintf(cNapis, "%3.2f%c ", uDaneCM4.dane.stGnss1.fKurs, ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12 + 47*FONT_SL, 140);

	sprintf(cNapis, "%02d:%02d:%02d ", uDaneCM4.dane.stGnss1.chGodz, uDaneCM4.dane.stGnss1.chMin, uDaneCM4.dane.stGnss1.chSek);
	RysujNapis(cNapis, KOL12 + 12*FONT_SL, 160);
	if  (uDaneCM4.dane.stGnss1.chMies > 12)	//ograniczenie aby nie pobierało nazwy miesiaca spoza tablicy cNazwyMies3Lit[]
		uDaneCM4.dane.stGnss1.chMies = 0;	//zerowy indeks jest pustą nazwą "---"
	sprintf(cNapis, "%02d %s %04d ", uDaneCM4.dane.stGnss1.chDzien, cNazwyMies3Lit[uDaneCM4.dane.stGnss1.chMies], uDaneCM4.dane.stGnss1.chRok + 2000);
	RysujNapis(cNapis, KOL12 + 29*FONT_SL, 160);
	sprintf(cNapis, "%d ", uDaneCM4.dane.stGnss1.chLiczbaSatelit);
	RysujNapis(cNapis, KOL12 + 47*FONT_SL, 160);

	//napięcie, prąd i energia obu pakietów
	setColor(BIALY);
	sprintf(cNapis, "%.2f V ", uDaneCM4.dane.fNapiecieAku[0]);
	RysujNapis(cNapis, KOL12 + 8*FONT_SL, 190);
	sprintf(cNapis, "%.2f A ", uDaneCM4.dane.fPradAku[0]);
	RysujNapis(cNapis, KOL12 + 27*FONT_SL, 190);
	//sprintf(cNapis, "%.1f mAh ", uDaneCM4.dane.fEnergiaPobr[0]);
	sprintf(cNapis, "%.2f V ", uDaneCM4.dane.fNapiecieWej[0]);			//tymczasowo pokaż napięcie wejściowe z własnego dzielnika
	RysujNapis(cNapis, KOL12 + 46*FONT_SL, 190);

	sprintf(cNapis, "%.2f V ", uDaneCM4.dane.fNapiecieAku[1]);
	RysujNapis(cNapis, KOL12 + 8*FONT_SL, 210);
	sprintf(cNapis, "%.2f A ", uDaneCM4.dane.fPradAku[1]);
	RysujNapis(cNapis, KOL12 + 27*FONT_SL, 210);
	//sprintf(cNapis, "%.1f mAh ", uDaneCM4.dane.fEnergiaPobr[1]);
	sprintf(cNapis, "%.2f V ", uDaneCM4.dane.fNapiecieWej[1]);			//tymczasowo pokaż napięcie wejściowe z własnego dzielnika
	RysujNapis(cNapis, KOL12 + 46*FONT_SL, 210);

	//napięcia pomocnicze
	sprintf(cNapis, "%.2f V ", uDaneCM4.dane.fNapiecieSerw);
	RysujNapis(cNapis, KOL12 + 8*FONT_SL, 230);
	sprintf(cNapis, "%.3f V ", uDaneCM4.dane.fNapiecieUSB);
	RysujNapis(cNapis, KOL12 + 27*FONT_SL, 230);
	sprintf(cNapis, "%.3f V " , uDaneCM4.dane.fNapiecieBatRTC);
	RysujNapis(cNapis, KOL12 + 46*FONT_SL, 230);

	//napięcie czujników zewnętrznych
	sprintf(cNapis, "%.3f V ", uDaneCM4.dane.fNapCzujZewn[0]);
	RysujNapis(cNapis, KOL12 + 8*FONT_SL, 260);
	sprintf(cNapis, "%.3f V ", uDaneCM4.dane.fNapCzujZewn[1]);
	RysujNapis(cNapis, KOL12 + 27*FONT_SL, 260);
	setColor(ZOLTY);
	sprintf(cNapis, "%.1f %cC ", uDaneCM4.dane.fTemperCPU, ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12 + 46*FONT_SL, 260);

	setColor(BIALY);
	sprintf(cNapis, "%.3f V ", uDaneCM4.dane.fNapCzujZewn[2]);
	RysujNapis(cNapis, KOL12 + 8*FONT_SL, 280);
	sprintf(cNapis, "%.3f V ", uDaneCM4.dane.fNapCzujZewn[3]);
	RysujNapis(cNapis, KOL12 + 27*FONT_SL, 280);

	//Rysuj pasek postepu jeżeli trwa jakiś proces. Zakładam że czas procesu jest zmniejszany od wartości CZAS_KALIBRACJI do zera
	RysujPasekPostepu(CZAS_KALIBRACJI);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z kanałami odbiornika RC albo serw
// Parametry:
// [we] chIndeksOpisu - indeks pozycji w zmiennej cNapisLcd zawierającej nazwę okna
// [we] sDane* - wskaźnik na tablicę zawierajacą dane wyświetlanych kanałów
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujPaskiKanalowRC(uint8_t chIndeksOpisu, uint16_t *sDane)
{
	uint16_t y, n, sSkorygowaneRC, sDlugoscPaska, sDlugoscTla;

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		BelkaTytulu((char*)cNapisLcd[chIndeksOpisu]);
		for (n=0; n<KANALY_ODB_RC; n++)
		{
			y = n * 17;
			setColor(SZARY80);
			sprintf(cNapis, "%2d:", n+1);
			RysujNapis(cNapis, KOL12, y+26);
			setColor(SZARY40);
			RysujProstokat(77, y+29, (79 + WE_RC_MAX / ROZDZIECZOSC_PASKA_RC), y+29+8);
		}
		setColor(SZARY50);
		RysujNapis((char*)cOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 300);	//"Wdus ekran i trzymaj aby zakonczyc"
	}

	for (n=0; n<KANALY_ODB_RC; n++)
	{
		y = n * 17;
		setColor(SZARY80);
		sprintf(cNapis, "%4d ", sDane[n]);
		RysujNapis(cNapis, KOL12+4*FONT_SL, y+26);

		//czasami długość kanału jest poza zakresem, więc skoryguj aby nie komplikować obliczeń
		if (sDane[n] < WE_RC_MIN)
			sSkorygowaneRC = WE_RC_MIN;
		else
		if (sDane[n] > WE_RC_MAX)
			sSkorygowaneRC = WE_RC_MAX;
		else
			sSkorygowaneRC = sDane[n];

		sDlugoscPaska = sSkorygowaneRC / ROZDZIECZOSC_PASKA_RC;
		if (sDlugoscPaska)
			RysujProstokatWypelniony(78, y+30, sDlugoscPaska, 6, NIEBIESKI);
		sDlugoscTla = (WE_RC_MAX / ROZDZIECZOSC_PASKA_RC) - sDlugoscPaska;
		if (sDlugoscTla)
			RysujProstokatWypelniony(79 + sDlugoscPaska, y+30, sDlugoscTla, 6, CZARNY);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje pasek postepu procesu rdzenia CM4 na dole ekranu
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujPasekPostepu(uint16_t sPelenZakres)
{
	uint16_t sDlugoscPaska = (uDaneCM4.dane.sPostepProcesu * DISP_X_SIZE) / sPelenZakres;
	if (sDlugoscPaska)	//nie rysuj paska jeżeli ma zerową długość
		RysujProstokatWypelniony(0, DISP_Y_SIZE - WYS_PASKA_POSTEPU, sDlugoscPaska, WYS_PASKA_POSTEPU, NIEBIESKI);		//Aktywna cześć paska
	RysujProstokatWypelniony(sDlugoscPaska, DISP_Y_SIZE - WYS_PASKA_POSTEPU, DISP_X_SIZE - sDlugoscPaska, WYS_PASKA_POSTEPU, CZARNY);	//tło za paskiem
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z testem generowania tonu
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void TestTonuAudio(void)
{
	extern uint8_t cNumerTonu;
	static uint16_t sLicznikTonu;
	if (cRysujRaz)
	{
		cRysujRaz = 0;
		BelkaTytulu("Test tonu wario");

		setColor(SZARY80);
		sprintf(cNapis, "Numer tonu:");
		RysujNapis(cNapis, KOL12, 30);
		sprintf(cNapis, "Czest. 1 harm.:");
		RysujNapis(cNapis, KOL12, 50);
		sprintf(cNapis, "Czest. 3 harm.:");
		RysujNapis(cNapis, KOL12, 70);
	}

	sLicznikTonu++;
	if (sLicznikTonu > 900)
	{
		sLicznikTonu = 0;
		cNumerTonu++;
		if (cNumerTonu >= LICZBA_TONOW_WARIO)
			cNumerTonu = 0;

		setColor(BIALY);
		sprintf(cNapis, "%d ", cNumerTonu);
		RysujNapis(cNapis, KOL12+16*FONT_SL, 30);
		sprintf(cNapis, "%.1f Hz ", 1.0f * CZESTOTLIWOSC_AUDIO / (MIN_OKRES_TONU + cNumerTonu * SKOK_TONU));
		RysujNapis(cNapis, KOL12+16*FONT_SL, 50);
		sprintf(cNapis, "%.1f Hz ", 3.0f * CZESTOTLIWOSC_AUDIO / (MIN_OKRES_TONU + cNumerTonu * SKOK_TONU));
		RysujNapis(cNapis, KOL12+16*FONT_SL, 70);
		UstawTon(cNumerTonu, 80);
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
	extern uint8_t cPort_exp_wysylany[];
	float fNapiecie;
	uint16_t sPozY;

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		BelkaTytulu("Parametry karty SD");

		//zamaż ewentualną pozostałość napisu o braku karty
		UstawCzcionke(BigFont);
		setColor(CZARNY);
		sprintf(cNapis, "                             ");
		RysujNapis(cNapis, CENTER, 50);
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
		sprintf(cNapis, "Typ: %ld ", CardInfo.CardType);
		RysujNapis(cNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(cNapis, "Pojemnosc: ");
		RysujNapis(cNapis, KOL12, sPozY);
		switch(pCSD.CSDStruct)
		{
		case 0:	sprintf(cNapis, "Standard");	break;
		case 1:	sprintf(cNapis, "Wysoka  ");	break;
		default:sprintf(cNapis, "Nieznana");	break;
		}
		RysujNapis(cNapis, KOL12 + 11*FONT_SL, sPozY);

		sPozY += 20;
		//sprintf(cNapis, "Klasy: %ld (0x%lX) ", CardInfo.Class, CardInfo.Class);
		sprintf(cNapis, "CCC: ");
		RysujNapis(cNapis, KOL12, sPozY);
		for (uint16_t n=0; n<12; n++)
		{
			if (CardInfo.Class & (1<<n))
				setColor(SZARY80);
			else
				setColor(SZARY40);
			sprintf(cNapis, "%X", n);
			RysujNapis(cNapis, KOL12 + ((5+n)*FONT_SL), sPozY);
		}
		sPozY += 20;

		setColor(SZARY70);
		sprintf(cNapis, "Klasa predk: %d ", pStatus.SpeedClass);
		RysujNapis(cNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(cNapis, "Klasa UHS: %d ", pStatus.UhsSpeedGrade);
		RysujNapis(cNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(cNapis, "Klasa Video: %d ", pStatus.VideoSpeedClass);
		RysujNapis(cNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(cNapis, "PerformanceMove: %d MB/s", pStatus.PerformanceMove);
		RysujNapis(cNapis, KOL12, sPozY);
		sPozY += 20;

		sprintf(cNapis, "Wsp pred. O/Z: %d ", pCSD.WrSpeedFact);
		RysujNapis(cNapis, KOL12, sPozY);
		sPozY += 20;

		if (cPort_exp_wysylany[0] & EXP02_LOG_VSELECT)	//LOG_SD1_VSEL: H=3,3V
			fNapiecie = 3.3;
		else
			fNapiecie = 1.8;
		sprintf(cNapis, "Napi%ccie I/O: %.1fV ", ę, fNapiecie);
		RysujNapis(cNapis, KOL12, sPozY);
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
		sprintf(cNapis, "Czas dost. async: %.1e s ", fPodstawaCzasu*fMnoznik);
		RysujNapis(cNapis, KOL12, sPozY);
		sPozY += 20;

		sprintf(cNapis, "Czas dost. sync: %d cyk.zeg ", pCSD.NSAC);
		RysujNapis(cNapis, KOL12, sPozY);
		sPozY += 20;

		sprintf(cNapis, "MaxBusClkFrec: %d", pCSD.MaxBusClkFrec);
		RysujNapis(cNapis, KOL12, sPozY);
		sPozY += 20;


		//druga kolumna
		sPozY = 30;
		sprintf(cNapis, "Liczba sektor%cw: %ld ", ó, CardInfo.BlockNbr);		//Specifies the Card Capacity in blocks
		RysujNapis(cNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(cNapis, "Rozmiar sektora: %ld B ", CardInfo.BlockSize);		//Specifies one block size in bytes
		RysujNapis(cNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(cNapis, "Pojemno%c%c karty: %ld MB ", ś, ć, CardInfo.BlockNbr * (CardInfo.BlockSize / 512) / 2048);		//Specifies one block size in bytes
		RysujNapis(cNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(cNapis, "Rozm Jedn Alok: %d kB", (8<<pStatus.AllocationUnitSize));		//Specifies one block size in bytes
		RysujNapis(cNapis, KOL22, sPozY);
		sPozY += 20;
		//sprintf(cNapis, "Liczba blok%cw log.: %ld ", ó, CardInfo.LogBlockNbr);		//Specifies the Card logical Capacity in blocks
		//RysujNapis(cNapis, KOL22, sPozY);
		//sPozY += 20;
		sprintf(cNapis, "RdBlockLen: %d ", (2<<pCSD.RdBlockLen));
		RysujNapis(cNapis, KOL22, sPozY);
		sPozY += 20;
		//sprintf(cNapis, "Rozmiar karty log: %ld MB ", CardInfo.LogBlockNbr * (CardInfo.LogBlockSize / 512) / 2048);		//Specifies one block size in bytes
		//RysujNapis(cNapis, KOL22, sPozY);
		//sPozY += 20;

		setColor(SZARY70);
		sprintf(cNapis, "Format: ");
		RysujNapis(cNapis, KOL22, sPozY);
		switch (pCSD.FileFormat)
		{
		case 0: sprintf(cNapis, "HDD z partycja ");	break;
		case 1: sprintf(cNapis, "DOS FAT ");			break;
		case 2: sprintf(cNapis, "UnivFileFormat ");	break;
		default: sprintf(cNapis, "Nieznany ");			break;
		}
		setColor(SZARY80);
		RysujNapis(cNapis, KOL22+8*FONT_SL, sPozY);
		sPozY += 20;

		setColor(SZARY70);
		sprintf(cNapis, "Manuf ID: ");
		RysujNapis(cNapis, KOL22, sPozY);
		switch (pCID.ManufacturerID)
		{
		case 0x02:	sprintf(cNapis, "Goodram/Toshiba ");	break;
		case 0x03:	sprintf(cNapis, "SandDisk ");	break;
		case 0xdF:
		case 0x05:	sprintf(cNapis, "Lenovo ");	break;
		case 0x09:	sprintf(cNapis, "APT ");	break;
		case 0x1B:	sprintf(cNapis, "Samsung ");	break;
		case 0x1D:	sprintf(cNapis, "ADATA ");	break;
		case 0x1F:
		case 0x41:	sprintf(cNapis, "Kingstone ");	break;
		case 0x6F:	sprintf(cNapis, "Kodak ");	break;
		case 0x74:	sprintf(cNapis, "Trnasced ");	break;
		case 0x82:	sprintf(cNapis, "Sony ");	break;
		default:	sprintf(cNapis, "%X ", pCID.ManufacturerID);
		}
		setColor(SZARY80);
		RysujNapis(cNapis, KOL22+10*FONT_SL, sPozY);
		sPozY += 20;

		setColor(SZARY70);
		chOEM[0] = (pCID.OEM_AppliID & 0xFF00)>>8;
		chOEM[1] = pCID.OEM_AppliID & 0x00FF;
		sprintf(cNapis, "OEM_AppliID: %c%c ", chOEM[0], chOEM[1]);
		RysujNapis(cNapis, KOL22, sPozY);
		sPozY += 20;

		sprintf(cNapis, "Nr seryjny: %ld ", pCID.ProdSN);
		RysujNapis(cNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(cNapis, "Data prod: %s %d ", cNazwyMies3Lit[(pCID.ManufactDate & 0xF)], ((pCID.ManufactDate>>4) & 0xFF) + 2000);
		RysujNapis(cNapis, KOL22, sPozY);
		sPozY += 20;

	}
	else
	{
		UstawCzcionke(BigFont);
		setColor(CZERWONY);
		sprintf(cNapis, "Wolne %carty, tu brak karty! ", ż);
		RysujNapis(cNapis, CENTER, 50);
		cRysujRaz = 1;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z parametrami rejestratora na karcie SD
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WyswietlRejestratorKartySD(void)
{
	extern uint8_t cKodBleduFAT;
	extern uint16_t sMaxDlugoscWierszaLogu;
	uint16_t sPozY;

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		BelkaTytulu("Rejestrator na karcie SD");

		setColor(SZARY80);
		sprintf(cNapis, "Karta SD: ");
		RysujNapis(cNapis, KOL12, 30);
		sprintf(cNapis, "Stan FATu: ");
		RysujNapis(cNapis, KOL12, 50);
		sprintf(cNapis, "Plik logu: ");
		RysujNapis(cNapis, KOL12, 70);
		sprintf(cNapis, "Rejestrator: ");
		RysujNapis(cNapis, KOL12, 90);
		sprintf(cNapis, "Zape%cnienie: ", ł);
		RysujNapis(cNapis, KOL12, 110);
	}

	sPozY = 30;
	if ((cPort_exp_odbierany[0] & EXP04_LOG_CARD_DET)	== 0)	//LOG_SD1_CDETECT - wejście detekcji obecności karty
	{
		setColor(KOLOR_Y);
		sprintf(cNapis, "Obecna");
	}
	else
	{
		setColor(KOLOR_X);
		sprintf(cNapis, "Brak! ");
	}
	RysujNapis(cNapis, KOL12 + 11*FONT_SL, sPozY);
	sPozY += 20;

	if (cStatusRejestratora & STATREJ_FAT_GOTOWY)
	{
		setColor(KOLOR_Y);
		sprintf(cNapis, "Gotowy                ");	//długością ma przykryć nadłuższy komunikat o błędzie
	}
	else
	{
		setColor(KOLOR_X);
		PobierzKodBleduFAT(cKodBleduFAT, cNapis);

	}
	RysujNapis(cNapis, KOL12 + 11*FONT_SL, sPozY);
	sPozY += 20;

	if (cStatusRejestratora & STATREJ_OTWARTY_PLIK)
	{
		setColor(KOLOR_Y);
		sprintf(cNapis, "Otwarty   ");
	}
	else
	{
		setColor(ZOLTY);
		if (cStatusRejestratora & STATREJ_BYL_OTWARTY)
			sprintf(cNapis, "Zatrzymany");
		else
			sprintf(cNapis, "Brak ");
	}
	RysujNapis(cNapis, KOL12 + 11*FONT_SL, sPozY);
	sPozY += 20;

	if (cStatusRejestratora & STATREJ_WLACZONY)
	{
		setColor(KOLOR_Y);
		sprintf(cNapis, "W%c%cczony  ", ł, ą);
	}
	else
	{
		setColor(ZOLTY);
		sprintf(cNapis, "Zatrzymany");
	}
	RysujNapis(cNapis, KOL12 + 13*FONT_SL, sPozY);
	sPozY += 20;


	float fZapelnienie = (float)sMaxDlugoscWierszaLogu / ROZMIAR_BUFORA_LOGU;
	if (fZapelnienie < 0.75)
		setColor(KOLOR_Y);	//zielony
	else
	if (fZapelnienie < 0.95)
		setColor(ZOLTY);
	else
		setColor(KOLOR_X);	//czerwony
	sprintf(cNapis, "%d / %d ", sMaxDlugoscWierszaLogu, ROZMIAR_BUFORA_LOGU);
	RysujNapis(cNapis, KOL12 + 13*FONT_SL, sPozY);
	sPozY += 20;
}



////////////////////////////////////////////////////////////////////////////////
// Parametry: cKodBleduFAT - numer kodu błędu typu FRESULT
// Zwraca: wskaźnik na string z kodem błędu FAT
////////////////////////////////////////////////////////////////////////////////
void PobierzKodBleduFAT(uint8_t cKodBledu, char* napis)
{
	//wersja krótsza z nazwą błędu
	switch (cKodBledu)
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
	/*switch (cKodBleduFAT)
	{
	case FR_DISK_ERR: 			sprintf(cNapis, "A hard error in low level disk I/O layer");	break;
	case FR_INT_ERR:			sprintf(cNapis, "Assertion failed");							break;
	case FR_NOT_READY: 			sprintf(cNapis, "The physical drive cannot work");				break;
	case FR_NO_FILE:			sprintf(cNapis, "Could not find the file");					break;
	case FR_NO_PATH:			sprintf(cNapis, "Could not find the path");					break;
	case FR_INVALID_NAME:		sprintf(cNapis, "The path name format is invalid");			break;
	case FR_DENIED:				sprintf(cNapis, "Access denied or directory full");			break;
	case FR_EXIST:				sprintf(cNapis, "Access denied due to prohibited access");		break;
	case FR_INVALID_OBJECT:		sprintf(cNapis, "The file/directory object is invalid");		break;
	case FR_WRITE_PROTECTED:	sprintf(cNapis, "The physical drive is write protected");		break;
	case FR_INVALID_DRIVE:		sprintf(cNapis, "The logical drive number is invalid");		break;
	case FR_NOT_ENABLED:		sprintf(cNapis, "The volume has no work area");				break;
	case FR_NO_FILESYSTEM:		sprintf(cNapis, "There is no valid FAT volume");				break;
	case FR_MKFS_ABORTED:		sprintf(cNapis, "The f_mkfs() aborted due to any problem");	break;
	case FR_TIMEOUT:			sprintf(cNapis, "Could not get a grant to access the volume");	break;
	case FR_LOCKED:				sprintf(cNapis, "Operation rejected due file sharing policy");	break;
	case FR_NOT_ENOUGH_CORE:	sprintf(cNapis, "LFN working buffer could not be allocated");	break;
	case FR_TOO_MANY_OPEN_FILES:sprintf(cNapis, "Number of open files > _FS_LOCK");			break;
	case FR_INVALID_PARAMETER:	sprintf(cNapis, "Given parameter is invalid");					break;
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
	sprintf(cNapis, "t=%ldus, phi=%.1f, theta=%.1f, psi=%.1f ", nCzas, fKat[0]*RAD2DEG, fKat[1]*RAD2DEG, fKat[2]*RAD2DEG);
	RysujNapis(cNapis, 0, 0);
	return nCzas;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran dla kalibracji wzmocnienia żyroskopów
// Parametry: *chSekwencer	- zmiennaWskazująca na kalibracje konktretnej osi
// Zwraca: BLAD_GOTOWE / BLAD_OK - informację o tym czy wyjść z trybu kalibracji czy nie
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibracjaWzmocnieniaZyroskopow(uint8_t *chSekwencer)
{
	char cNazwaOsi;
	float fPochylenie, fPrzechylenie;
	uint8_t cBłąd = BLAD_OK;

	if (cRysujRaz)
	{
		//cRysujRaz = 0;	będzie wyzerowane w funkcji rysowania poziomicy
		BelkaTytulu("Kalibr. wzmocnienia zyro");
		setColor(SZARY80);

		sprintf(cNapis, "Ca%cka %cyro 1:", ł, ż);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK, 100);
		sprintf(cNapis, "Ca%cka %cyro 2:", ł, ż);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK, 120);
		sprintf(cNapis, "K%cty w p%caszczy%cnie kalibracji", ą, ł, ź);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK, 140);
		sprintf(cNapis, "Pochylenie:");
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK, 160);
		sprintf(cNapis, "Przechylenie:");
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK, 180);
		sprintf(cNapis, "WspKal %cyro1:", ż);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK, 200);
		sprintf(cNapis, "WspKal %cyro2:", ż);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK, 220);

		setColor(SZARY40);
		stPrzycisk.sX1 = 10 + LIBELLA_BOK;
		stPrzycisk.sY1 = 240;
		stPrzycisk.sX2 = stPrzycisk.sX1 + 210;
		stPrzycisk.sY2 = stPrzycisk.sY1 + 75;
		RysujPrzycisk(stPrzycisk, "Odczyt", RYSUJ);

		cEtapKalibracji = 0;
		cStanPrzycisku = 0;
		setColor(ZOLTY);
		sprintf(cNapis, "B%cbelek powinien by%c w %crodku poziomicy", ą, ć, ś);
		RysujNapis(cNapis, CENTER, 50);
		setColor(SZARY60);
		sprintf(cNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		RysujNapis(cNapis, CENTER, 70);
		stStatusDotyku.chFlagi &= ~(DOTYK_ZWOLNONO | DOTYK_DOTKNIETO);	//czyść flagi ekranu dotykowego
	}

	//sekwencer kalibracji
	switch (*chSekwencer)
	{
	case SEKW_KAL_WZM_ZYRO_R:
		cNazwaOsi = 'Z';
		fPochylenie = uDaneCM4.dane.fKatAkcel1[1];
		fPrzechylenie = uDaneCM4.dane.fKatAkcel1[0];
		Poziomica(-fPrzechylenie, fPochylenie);	//przechylenie, pochylenie
		setColor(KOLOR_Z);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[2], ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[2], ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;

	case SEKW_KAL_WZM_ZYRO_Q:
		cNazwaOsi = 'Y';
		fPochylenie = atan2f(uDaneCM4.dane.fAkcel1[1], uDaneCM4.dane.fAkcel1[0]) + 90 * DEG2RAD;	//atan(y/x)
		fPrzechylenie = atan2f(uDaneCM4.dane.fAkcel1[1], uDaneCM4.dane.fAkcel1[2]) + 90 * DEG2RAD;	//atan(y/z)
		Poziomica(fPrzechylenie, -fPochylenie);	//przechylenie, pochylenie
		setColor(KOLOR_Y);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[1], ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[1], ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);	//przechylenie
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;

	case SEKW_KAL_WZM_ZYRO_P:
		cNazwaOsi = 'X';
		fPochylenie = atan2f(uDaneCM4.dane.fAkcel1[2], uDaneCM4.dane.fAkcel1[0]);	//atan(z/x)
		fPrzechylenie = atan2f(uDaneCM4.dane.fAkcel1[0], uDaneCM4.dane.fAkcel1[1]) - 90 * DEG2RAD;	//atan(x/y)
		Poziomica(-fPrzechylenie, fPochylenie);	//przechylenie, pochylenie
		setColor(KOLOR_X);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[0], ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[0], ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(cNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);	//przechylenie:
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;
	}

	setColor(ZOLTY);
	sprintf(cNapis, "Skieruj osia %c do dolu i wykonaj %d obroty w dowolna strone", cNazwaOsi, OBR_KAL_WZM);
	RysujNapis(cNapis, CENTER, 30);

	//wyświetl współczynnik kalibracji
	if ((uDaneCM4.dane.nZainicjowano & INIT_WYK_KAL_WZM_ZYRO) && (cEtapKalibracji >= 2))
	{
		setColor(BIALY);										//nowy współczynnik
		sprintf(cNapis, "%.3f", uDaneCM4.dane.uRozne.f32[0]);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 200);
		sprintf(cNapis, "%.3f", uDaneCM4.dane.uRozne.f32[1]);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 220);
	}
	else
	{
		setColor(SZARY80);										//stary współczynnik
		sprintf(cNapis, "%.3f", uDaneCM4.dane.uRozne.f32[2]);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 200);
		sprintf(cNapis, "%.3f", uDaneCM4.dane.uRozne.f32[3]);
		RysujNapis(cNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 220);
	}

	//napis dla przycisku
	setColor(ZOLTY);
	switch (cEtapKalibracji)
	{
	case 0:	//odczyt wartości bieżącej wzmocnienia
		uDaneCM7.dane.cWykonajPolecenie = POL7_CZYTAJ_WZM_ZYROP + *chSekwencer;
		sprintf(cNapis, "Odczyt");
		if (uDaneCM4.dane.cPotwierdzenieWykonania == POL7_CZYTAJ_WZM_ZYROP + *chSekwencer)	//jeżeli nastapił odczyt to przejdź do nastepnego etapu
			cEtapKalibracji = 1;
		break;

	case 1:	//zerowanie całki
		uDaneCM7.dane.cWykonajPolecenie = POL7_ZERUJ_CALKE_ZYRO;	//zeruje całkę żyroskopów przed kalibracją
		sprintf(cNapis, "Start");
		if (uDaneCM4.dane.cPotwierdzenieWykonania == POL7_ZERUJ_CALKE_ZYRO)	//jeżeli nastapiło wyzerowanie to nie czekaj na przycisk tylko od razu zacznij całkować
		{
			cEtapKalibracji = 2;
			uDaneCM7.dane.cWykonajPolecenie = POL7_CALKUJ_PRED_KAT;		//wykonaj całkowanie kąta podczas obrotu
		}
		break;

	case 2:	//całkowanie
		uDaneCM7.dane.cWykonajPolecenie = POL7_CALKUJ_PRED_KAT;		//wykonaj całkowanie kąta podczas obrotu
		sprintf(cNapis, "Oblicz");
		break;

	case 3:	//obliczenie kalibracji
		uDaneCM7.dane.cWykonajPolecenie = POL7_KALIBRUJ_ZYRO_WZMR + *chSekwencer;	//uruchom kalibrację dla scałkowanego kąta R, Q lub P
		if (uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZLE_OBLICZENIA)
			sprintf(cNapis, "Blad! ");
		else
			sprintf(cNapis, "Gotowe");
		break;

	default:	//wyjście lub przejscie do kalibracji kolejnej osi
		sprintf(cNapis, "Wyjdz ");
		cBłąd = BLAD_GOTOWE;	//zakończ kalibrację
		break;
	}

	/*setBackColor(SZARY40);	//kolor tła napisu kolorem przycisku
	UstawCzcionke(BigFont);
	RysujNapis(cNapis, stPrzycisk.sX1 + (stPrzycisk.sX2 - stPrzycisk.sX1)/2 - cRozmiar*FONT_BL/2 , stPrzycisk.sY1 + (stPrzycisk.sY2 - stPrzycisk.sY1)/2 - FONT_BH/2);
	setBackColor(CZARNY);
	UstawCzcionke(MidFont);*/


	//sprawdź czy jest naciskany przycisk
	if (stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
	{
		//czy naciśnięto na przycisk?
		if ((stStatusDotyku.sY > stPrzycisk.sY1) && (stStatusDotyku.sY < stPrzycisk.sY2) && (stStatusDotyku.sX > stPrzycisk.sX1) && (stStatusDotyku.sX < stPrzycisk.sX2))
		{
			cStanPrzycisku = 1;
			RysujPrzycisk(stPrzycisk, cNapis, ODSWIEZ);
		}
		else
			cBłąd = BLAD_GOTOWE;	//zakończ kabrację gdy nacięnięto poza przyciskiem

		stStatusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;
	}
	else	//DOTYK_DOTKNIETO
	{
		if (cStanPrzycisku)
		{
			cEtapKalibracji++;
			cEtapKalibracji &= 0x07;
			cStanPrzycisku = 0;
		}
	}
	return cBłąd;
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

	if (cRysujRaz)
	{
		RysujProstokatWypelniony(0, DISP_Y_SIZE - LIBELLA_BOK, LIBELLA_BOK, LIBELLA_BOK, LIBELLA1);
		cRysujRaz = 0;
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
// cNapis - tekst do wymisania na środku
// chCzynnosc - definiuje operacje: RYSUJ - pełne narysowanie przycisku, lub ODSWIEZ - podmianę samego napisu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujPrzycisk(prostokat_t prost, char *cNapis, uint8_t cCzynnosc)
{
	uint8_t cRozmiar;

	cRozmiar = strlen(cNapis);
	if (cCzynnosc == RYSUJ)
	{
		//setColor(SZARY40);
		//fillRect(prost.sX1, prost.sY1 ,prost.sX2, prost.sY2);	//rysuj przycisk
		RysujProstokatWypelniony(prost.sX1, prost.sY1, prost.sX2-prost.sX1, prost.sY2-prost.sY1, SZARY40);
	}

	setColor(ZOLTY);
	setBackColor(SZARY40);	//kolor tła napisu kolorem przycisku
	UstawCzcionke(BigFont);
	RysujNapis(cNapis, prost.sX1 + (prost.sX2 - prost.sX1)/2 - cRozmiar*FONT_BL/2 , prost.sY1 + (prost.sY2 - prost.sY1)/2 - FONT_BH/2);
	setBackColor(CZARNY);
	UstawCzcionke(MidFont);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran dla kalibracji zera (offsetu) magnetometru. Dla obrotu wokół X rysuje wykres YZ, dla obrotu wokół Y rysuje XZ dla Z rysuje XY
// Analizuje dane przekazane w zmiennej uDaneCM4.dane.uRozne.f32[6] przez CM4. Funkcja znajjduje i sygnalizuje znalezienia ich maksimów poprzez komunikaty głosowe: 1-minX, 2-maxX, 3-minY, 4-maxY, 5-minZ, 6-maxZ
// Wykres jest skalowany do maksimum wartosci bezwzględnej obu zmiennych
// Parametry: *cEtap	- wskaźnik na zmienną wskazującą na kalibracje konktretnej osi w konkretnym magnetometrze. Starszy półbajt koduje numer magnetometru, najmłodsze 2 bity oś obracaną a bit 4 procedurę kalibracji
// uDaneCM4.dane.uRozne.f32[6] - minima i maksima magnetometrów zbierana przez CM4
// Zwraca: BLAD_GOTOWE / BLAD_OK - informację o tym czy wyjść z trybu kalibracji czy nie
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibracjaZeraMagnetometru(uint8_t *cEtap)
{
	char cNazwaOsi;
	uint8_t cBłąd = BLAD_OK;
	float fWspSkal;		//współczynnik skalowania wykresu
	float fMag[3];		//dane bieżącego magnetometru
	uint16_t sX, sY;	//bieżace współrzędne na wykresie
	static uint16_t sPoprzedX, sPoprzedY;	//poprzednie współrzędne na wykresie
	static float fMin[3], fMax[3];	//minimum i  maksimum wskazań
	float fAbsMax;		//maksimum z wartości bezwzględnej minimum i maksimum
	uint8_t cCzujnik;

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		if (*cEtap & KALIBRUJ)
			sprintf(cNapis, "%s %s %d", cNapisLcd[STR_KALIBRACJA], cNapisLcd[STR_MAGNETOMETRU], (*cEtap & MASKA_CZUJNIKA)>>4);
		else
			sprintf(cNapis, "%s %s %d", cNapisLcd[STR_WERYFIKACJA], cNapisLcd[STR_MAGNETOMETRU], (*cEtap & MASKA_CZUJNIKA)>>4);
		BelkaTytulu(cNapis);

		setColor(SZARY80);
		sprintf(cNapis, "%s X:", cNapisLcd[STR_MAGN]);
		RysujNapis(cNapis, KOL12, 80);
		sprintf(cNapis, "%s Y:", cNapisLcd[STR_MAGN]);
		RysujNapis(cNapis, KOL12, 100);
		sprintf(cNapis, "%s Z:", cNapisLcd[STR_MAGN]);
		RysujNapis(cNapis, KOL12, 120);
		sprintf(cNapis, "Pochylenie:");
		RysujNapis(cNapis, KOL12, 140);
		sprintf(cNapis, "Przechylenie:");
		RysujNapis(cNapis, KOL12, 160);
		if (*cEtap & KALIBRUJ)
		{
			sprintf(cNapis, "%s X:", cNapisLcd[STR_EKSTREMA]);
			RysujNapis(cNapis, KOL12, 180);
			sprintf(cNapis, "%s Y:", cNapisLcd[STR_EKSTREMA]);
			RysujNapis(cNapis, KOL12, 200);
			sprintf(cNapis, "%s Z:", cNapisLcd[STR_EKSTREMA]);
			RysujNapis(cNapis, KOL12, 220);
		}
		else
		{
			sprintf(cNapis, "%s X:", cNapisLcd[STR_KAL]);
			RysujNapis(cNapis, KOL12, 180);
			sprintf(cNapis, "%s Y:", cNapisLcd[STR_KAL]);
			RysujNapis(cNapis, KOL12, 200);
			sprintf(cNapis, "%s Z:", cNapisLcd[STR_KAL]);
			RysujNapis(cNapis, KOL12, 220);
		}


		setColor(SZARY60);
		sprintf(cNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		RysujNapis(cNapis, CENTER, 45);

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
		cStanPrzycisku = 0;
		for (uint16_t n=0; n<3; n++)
		{
			if (*cEtap & KALIBRUJ)
				fMin[n] = fMax[n] = 0;		//podczas kalibracji zbieraj wartosci minimów i maksimów
			else
			{
				fMin[n] = -NOMINALNE_MAGN;
				fMax[n] =  NOMINALNE_MAGN;		//podczas sprawdzenia pracuj na stałym współczynniku
			}
		}
		*cEtap |= ZERUJ;	//przed pomiarem wystaw potrzebę wyzerowania znalezionych ekstemów magnetometru
		stStatusDotyku.chFlagi &= ~(DOTYK_ZWOLNONO | DOTYK_DOTKNIETO);	//czyść flagi ekranu dotykowego
	}

	if (*cEtap & ZERUJ)
	{
		uDaneCM7.dane.cWykonajPolecenie = POL7_ZERUJ_EKSTREMA;
		DodajProbkeDoMalejKolejki(PGA_00, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//komunikat "zero"
		if ( uDaneCM4.dane.cPotwierdzenieWykonania == POL7_ZERUJ_EKSTREMA)
		{
			*cEtap &= ~ZERUJ;
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
	if ((*cEtap & MASKA_OSI) < 3)	//pobieraj dane tylko dla konkretnych osi. Gdy obsłużone wszystkie to zakończ
	{
		switch (*cEtap & MASKA_CZUJNIKA)		//rodzaj magnetometru
		{
		case MAG1:
			if (*cEtap & KALIBRUJ)
				uDaneCM7.dane.cWykonajPolecenie = POL7_KAL_ZERO_MAGN1;
			else
				uDaneCM7.dane.cWykonajPolecenie = POL7_POBIERZ_KONF_MAGN1;

			for (uint16_t n=0; n<3; n++)
				fMag[n] = uDaneCM4.dane.fMagne1[n];
			break;

		case MAG2:
			if (*cEtap & KALIBRUJ)
				uDaneCM7.dane.cWykonajPolecenie = POL7_KAL_ZERO_MAGN2;
			else
				uDaneCM7.dane.cWykonajPolecenie = POL7_POBIERZ_KONF_MAGN2;
			for (uint16_t n=0; n<3; n++)
				fMag[n] = uDaneCM4.dane.fMagne2[n];
			break;

		case MAG3:
			if (*cEtap & KALIBRUJ)
				uDaneCM7.dane.cWykonajPolecenie = POL7_KAL_ZERO_MAGN3;
			else
				uDaneCM7.dane.cWykonajPolecenie = POL7_POBIERZ_KONF_MAGN3;
			for (uint16_t n=0; n<3; n++)
				fMag[n] = uDaneCM4.dane.fMagne3[n];
			break;
		}
	}

	//znajdowanie ekstremów, rysowanie wykresu i generowanie komunikatów głosowych
	switch (*cEtap & MASKA_OSI)		//rodzaj osi
	{
	case 0:	cNazwaOsi = 'X';
		if (*cEtap & KALIBRUJ)
		{
			if (uDaneCM4.dane.uRozne.f32[2] < fMin[1])	//minimum Y
			{
				fMin[1] = uDaneCM4.dane.uRozne.f32[2];
				DodajProbkeDoMalejKolejki(PGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
				DodajProbkeDoMalejKolejki(PGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[3] > fMax[1])	//maksimum Y
			{
				fMax[1] = uDaneCM4.dane.uRozne.f32[3];
				DodajProbkeDoMalejKolejki(PGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
				DodajProbkeDoMalejKolejki(PGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
			}
			else
			if (uDaneCM4.dane.uRozne.f32[4] < fMin[2])	//minimum Z
			{
				fMin[2] = uDaneCM4.dane.uRozne.f32[4];
				DodajProbkeDoMalejKolejki(PGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
				DodajProbkeDoMalejKolejki(PGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[5] > fMax[2])	//maksimum Z
			{
				fMax[2] = uDaneCM4.dane.uRozne.f32[5];
				DodajProbkeDoMalejKolejki(PGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
				DodajProbkeDoMalejKolejki(PGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
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

	case 1: cNazwaOsi = 'Y';
		if (*cEtap & KALIBRUJ)
		{
			if (uDaneCM4.dane.uRozne.f32[0] < fMin[0])	//minimum X
			{
				fMin[0] = uDaneCM4.dane.uRozne.f32[0];
				DodajProbkeDoMalejKolejki(PGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
				DodajProbkeDoMalejKolejki(PGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[1] > fMax[0])	//maksimum X
			{
				fMax[0] = uDaneCM4.dane.uRozne.f32[1];
				DodajProbkeDoMalejKolejki(PGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
				DodajProbkeDoMalejKolejki(PGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
			}
			else
			if (uDaneCM4.dane.uRozne.f32[4] < fMin[2])	//minimum Z
			{
				fMin[2] = uDaneCM4.dane.uRozne.f32[4];
				DodajProbkeDoMalejKolejki(PGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
				DodajProbkeDoMalejKolejki(PGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[5] > fMax[2])	//maksimum Z
			{
				fMax[2] = uDaneCM4.dane.uRozne.f32[5];
				DodajProbkeDoMalejKolejki(PGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
				DodajProbkeDoMalejKolejki(PGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
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

	case 2: cNazwaOsi = 'Z';
		if (*cEtap & KALIBRUJ)
		{
			if (uDaneCM4.dane.uRozne.f32[0] < fMin[0])	//minimum X
			{
				fMin[0] = uDaneCM4.dane.uRozne.f32[0];
				DodajProbkeDoMalejKolejki(PGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
				DodajProbkeDoMalejKolejki(PGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[1] > fMax[0])	//maksimum X
			{
				fMax[0] = uDaneCM4.dane.uRozne.f32[1];
				DodajProbkeDoMalejKolejki(PGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
				DodajProbkeDoMalejKolejki(PGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
			}
			else
			if (uDaneCM4.dane.uRozne.f32[2] < fMin[1])	//minimum Y
			{
				fMin[1] = uDaneCM4.dane.uRozne.f32[2];
				DodajProbkeDoMalejKolejki(PGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
				DodajProbkeDoMalejKolejki(PGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
			}
			else
			if (uDaneCM4.dane.uRozne.f32[3] > fMax[1])	//maksimum Y
			{
				fMax[1] = uDaneCM4.dane.uRozne.f32[3];
				DodajProbkeDoMalejKolejki(PGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
				DodajProbkeDoMalejKolejki(PGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
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
	sprintf(cNapis, "%.2f [uT] ", fMag[0]*1e6);
	RysujNapis(cNapis, KOL12 + 8*FONT_SL, 80);
	sprintf(cNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[0], ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12 + 12*FONT_SL, 140);
	if (*cEtap & KALIBRUJ)
	{
		sprintf(cNapis, "%.2f, %.2f ", uDaneCM4.dane.uRozne.f32[0]*1e6, uDaneCM4.dane.uRozne.f32[1]*1e6);	//ekstrema X
		RysujNapis(cNapis, KOL12 + 12*FONT_SL, 180);
	}
	else
	{
		sprintf(cNapis, "%.3e, %.4f ", uDaneCM4.dane.uRozne.f32[0], uDaneCM4.dane.uRozne.f32[1]);	//przesunięcie i skalowanie X
		RysujNapis(cNapis, KOL12 + 7*FONT_SL, 180);
	}

	setColor(KOLOR_Y);
	sprintf(cNapis, "%.2f [uT] ", fMag[1]*1e6);
	RysujNapis(cNapis, 10 + 8*FONT_SL, 100);
	sprintf(cNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[1], ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12 + 14*FONT_SL, 160);
	if (*cEtap & KALIBRUJ)
	{
		sprintf(cNapis, "%.2f, %.2f ", uDaneCM4.dane.uRozne.f32[2]*1e6, uDaneCM4.dane.uRozne.f32[3]*1e6);	//ekstrema Y
		RysujNapis(cNapis, KOL12 + 12*FONT_SL, 200);
	}
	else
	{
		sprintf(cNapis, "%.3e, %.4f ", uDaneCM4.dane.uRozne.f32[2], uDaneCM4.dane.uRozne.f32[3]);		//przesunięcie i skalowanie Y
		RysujNapis(cNapis, KOL12 + 7*FONT_SL, 200);
	}

	setColor(KOLOR_Z);
	sprintf(cNapis, "%.2f [uT] ", fMag[2]*1e6);
	RysujNapis(cNapis, 10 + 8*FONT_SL, 120);
	if (*cEtap & KALIBRUJ)
	{
		sprintf(cNapis, "%.2f, %.2f ", uDaneCM4.dane.uRozne.f32[4]*1e6, uDaneCM4.dane.uRozne.f32[5]*1e6);	//ekstrema Z
		RysujNapis(cNapis, KOL12 + 12*FONT_SL, 220);
	}
	else
	{
		sprintf(cNapis, "%.3e, %.4f ", uDaneCM4.dane.uRozne.f32[4], uDaneCM4.dane.uRozne.f32[5]);		//przesunięcie i skalowanie Z
		RysujNapis(cNapis, KOL12 + 7*FONT_SL, 220);
	}

	//sprawdź czy jest naciskany przycisk
	if (stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
	{
		//czy naciśnięto na przycisk?
		if ((stStatusDotyku.sY > stPrzycisk.sY1) && (stStatusDotyku.sY < stPrzycisk.sY2) && (stStatusDotyku.sX > stPrzycisk.sX1) && (stStatusDotyku.sX < stPrzycisk.sX2))
			cStanPrzycisku = 1;
		else
		{
			cBłąd = BLAD_GOTOWE;	//zakończ kabrację gdy nacięnięto poza przyciskiem
			uDaneCM7.dane.cWykonajPolecenie = POL7_NIC;	//neutralne polecenie kończy szukanie ekstremów w CM4
		}
		stStatusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;
	}
	else	//DOTYK_DOTKNIETO
	{
		if (cStanPrzycisku)	//został naciśniety
		{
			(*cEtap)++;
			cStanPrzycisku = 0;	//usuń ststus naciśnięcia
			sPoprzedX = stWykr.sX1 + SZER_WYKR_MAG/2;	//środek wykresu
			sPoprzedY = stWykr.sY1 + SZER_WYKR_MAG/2;
			DodajProbkeDoMalejKolejki(PGA_PRZYCISK, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//komunikat audio naciśnięcia przycisku
		}
	}

	cCzujnik = ((*cEtap & MASKA_CZUJNIKA) >> 4) - 1;	//oblicz indeks czujnika
	if ((*cEtap & MASKA_OSI) == 3)	//czy obsłużone już wszystkie 3 osie i jesteśmy w trybie kalibracji?
	{
		if (*cEtap & KALIBRUJ)
			uDaneCM7.dane.cWykonajPolecenie = POL7_ZAPISZ_KONF_MAGN1 + cCzujnik;	//zapisz konfigurację bieżącego magnetometru
		else
			cBłąd = BLAD_GOTOWE;		//koniec wertyfikacji
	}

	//wyjdź dopiero gdy dostanie potwierdzenie zapisu
	if (((*cEtap & MASKA_OSI) == 3) && (uDaneCM4.dane.cPotwierdzenieWykonania == POL7_ZAPISZ_KONF_MAGN1 + cCzujnik))
	{
		uDaneCM7.dane.cWykonajPolecenie = POL7_NIC;	//neutralne polecenie kończy szukanie ekstremów w CM4
		cBłąd = BLAD_GOTOWE;		//koniec kalibracji
		DodajProbkeDoMalejKolejki(PGA_GOTOWE, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//komunikat kończący: Gotowe
	}

	setColor(ZOLTY);
	sprintf(cNapis, "Ustaw UAV osi%c %c w kier. Wsch%cd-Zach%cd i obracaj wok%c%c %c", ą, cNazwaOsi, ó, ó, ó, ł, cNazwaOsi);
	RysujNapis(cNapis, CENTER, 25);
	return cBłąd;
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
// Parametry: *cEtap - wskaźnik na zmienną zawierającą bieżący etap procesu kalibracji
// uDaneCM4.dane.uRozne.fRozne[0] - pierwsze uśrednione ciśnienie czujnika 1
// uDaneCM4.dane.uRozne.fRozne[1] - pierwsze uśrednione ciśnienie czujnika 2
// uDaneCM4.dane.uRozne.fRozne[2] - drugie uśrednione ciśnienie czujnika 1
// uDaneCM4.dane.uRozne.fRozne[3] - drugie uśrednione ciśnienie czujnika 2
// uDaneCM4.dane.uRozne.fRozne[4] - współczynnik skalowania czujnika 1
// uDaneCM4.dane.uRozne.fRozne[5] - współczynnik skalowania czujnika 2
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibrujBaro(uint8_t *cEtap)
{
	uint8_t cBłąd = BLAD_OK;

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		BelkaTytulu("Kalibr. pomiaru cisnienia");

		setColor(ZOLTY);
		sprintf(cNapis, "U%crednij ci%cn. pocz. Pokonaj wys=27m i ponownie u%crednij", ś, ś, ś);
		RysujNapis(cNapis, CENTER, 30);

		setColor(SZARY60);
		sprintf(cNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		RysujNapis(cNapis, CENTER, 50);

		setColor(SZARY80);
		sprintf(cNapis, "Czujnik 1");
		RysujNapis(cNapis, KOL12 + 15*FONT_SL, 80);
		sprintf(cNapis, "Czujnik 2");
		RysujNapis(cNapis, KOL12 + 30*FONT_SL, 80);

		sprintf(cNapis, "Biezace cisn:");
		RysujNapis(cNapis, KOL12, 100);
		sprintf(cNapis, "Sredn cisn 1:");
		RysujNapis(cNapis, KOL12, 120);
		sprintf(cNapis, "Sredn.cisn 2:");
		RysujNapis(cNapis, KOL12, 140);
		sprintf(cNapis, "Skalowanie:");
		RysujNapis(cNapis, KOL12, 160);


		setColor(SZARY40);
		stPrzycisk.sX1 = 10;
		stPrzycisk.sY1 = 210;
		stPrzycisk.sX2 = stPrzycisk.sX1 + 250;
		stPrzycisk.sY2 = DISP_Y_SIZE - WYS_PASKA_POSTEPU - 1;
		RysujPrzycisk(stPrzycisk, "Start", RYSUJ);
		cStanPrzycisku = 0;
		*cEtap = 0;
	}

	// Zwraca: kod błędu
	setColor(BIALY);
	sprintf(cNapis, "%.0f Pa", uDaneCM4.dane.fCisnieBzw[0]);
	RysujNapis(cNapis, KOL12 + 14*FONT_SL, 100);
	sprintf(cNapis, "%.0f Pa", uDaneCM4.dane.fCisnieBzw[1]);
	RysujNapis(cNapis, KOL12 + 30*FONT_SL, 100);
	sprintf(cNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[0]);	//pierwsze uśrednione ciśnienie czujnika 1
	RysujNapis(cNapis, KOL12 + 14*FONT_SL, 120);
	sprintf(cNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[1]);	//pierwsze uśrednione ciśnienie czujnika 2
	RysujNapis(cNapis, KOL12 + 30*FONT_SL, 120);
	sprintf(cNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[2]);	//drugie uśrednione ciśnienie czujnika 1
	RysujNapis(cNapis, KOL12 + 14*FONT_SL, 140);
	sprintf(cNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[3]);	//drugie uśrednione ciśnienie czujnika 2
	RysujNapis(cNapis, KOL12 + 30*FONT_SL, 140);
	sprintf(cNapis, "%.6f ", uDaneCM4.dane.uRozne.f32[4]);	//współczynnik skalowania czujnika 1
	RysujNapis(cNapis, KOL12 + 14*FONT_SL, 160);
	sprintf(cNapis, "%.6f ", uDaneCM4.dane.uRozne.f32[5]);	//współczynnik skalowania czujnika 2
	RysujNapis(cNapis, KOL12 + 30*FONT_SL, 160);

	switch (*cEtap)
	{
	case 0:	//czyść zmienne robocze, załaduj bieżacy współczynnik
		uDaneCM7.dane.cWykonajPolecenie = POL7_INICJUJ_USREDN;	//zeruj licznik uśredniania
		if (uDaneCM4.dane.cPotwierdzenieWykonania == POL7_INICJUJ_USREDN)
			(*cEtap)++;
		break;

	case 1:
	case 3:
		uDaneCM7.dane.cWykonajPolecenie = POL7_ZERUJ_LICZNIK;	//zeruj licznik uśredniania
		if (uDaneCM4.dane.cPotwierdzenieWykonania == POL7_ZERUJ_LICZNIK)
		{
			sprintf(cNapis, "Usrednij %d", (*cEtap) / 2 + 1);
			RysujPrzycisk(stPrzycisk, cNapis, ODSWIEZ);
			if (cStanPrzycisku == 1)
				(*cEtap)++;	//wyzerowało się wiec przejdź do nastepnego etapu
		}
		break;

	case 2:
		uDaneCM7.dane.cWykonajPolecenie = POL7_USREDNIJ_CISN1;	//trwa uśrednianie ciśnienia 1
		if (uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_GOTOWE)
			(*cEtap)++;
		break;

	case 4:
		uDaneCM7.dane.cWykonajPolecenie = POL7_USREDNIJ_CISN2;	//trwa uśrednianie ciśnienia 2
		if (uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_GOTOWE)
			(*cEtap)++;
		break;

	case 5:
		uDaneCM7.dane.cWykonajPolecenie = POL7_NIC;	//Zakończ wykonywanie poleceń kalibracyjnych
		if (uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] == BLAD_ZLE_OBLICZENIA)
			RysujPrzycisk(stPrzycisk, "Blad!", ODSWIEZ);
		else
			RysujPrzycisk(stPrzycisk, "Gotowe", ODSWIEZ);
		sprintf(cNapis, "dP1 = %.2f Pa", fabs(uDaneCM4.dane.uRozne.f32[0] - uDaneCM4.dane.uRozne.f32[2]));	//Rożnica ciśnień czujnika 1
		RysujNapis(cNapis, 10 + stPrzycisk.sX2, 240);
		sprintf(cNapis, "dP2 = %.2f Pa", fabs(uDaneCM4.dane.uRozne.f32[1] - uDaneCM4.dane.uRozne.f32[3]));	//Rożnica ciśnień czujnika 2
		RysujNapis(cNapis, 10 + stPrzycisk.sX2, 260);
		if (cStanPrzycisku == 1)
			cBłąd = BLAD_GOTOWE;	//zakończ
		break;
	}

	//sprawdź czy jest naciskany przycisk
	if (stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
	{
		//czy naciśnięto na przycisk?
		if ((stStatusDotyku.sY > stPrzycisk.sY1) && (stStatusDotyku.sY < stPrzycisk.sY2) && (stStatusDotyku.sX > stPrzycisk.sX1) && (stStatusDotyku.sX < stPrzycisk.sX2))
		{
			cStanPrzycisku = 1;
			DodajProbkeDoMalejKolejki(PGA_PRZYCISK, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//komunikat audio naciśnięcia przycisku
		}
		else
		{
			cBłąd = BLAD_GOTOWE;	//zakończ kabrację gdy nacięnięto poza przyciskiem
			uDaneCM7.dane.cWykonajPolecenie = POL7_NIC;	//neutralne polecenie
		}
		stStatusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;
	}
	else	//DOTYK_DOTKNIETO
	{
		if (cStanPrzycisku)
			cStanPrzycisku = 0;
	}

	RysujPasekPostepu(CZAS_KALIBRACJI);
	return cBłąd;
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

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		sprintf(cNapis, "%s 2D %s", cNapisLcd[STR_WERYFIKACJA], cNapisLcd[STR_MAGNETOMETRU]);
		BelkaTytulu(cNapis);

		setColor(SZARY80);
		sprintf(cNapis, "Przechylenie:");
		RysujNapis(cNapis, KOL12, 80);
		sprintf(cNapis, "Pochylenie:");
		RysujNapis(cNapis, KOL12, 100);

		sprintf(cNapis, "%s 1:", cNapisLcd[STR_MAGN]);
		RysujNapis(cNapis, KOL12, 120);
		sprintf(cNapis, "%s 2:", cNapisLcd[STR_MAGN]);
		RysujNapis(cNapis, KOL12, 140);
		sprintf(cNapis, "%s 3:", cNapisLcd[STR_MAGN]);
		RysujNapis(cNapis, KOL12, 160);

		sprintf(cNapis, "Inklinacja: %.2f %c", INKLINACJA_MAG * RAD2DEG, ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12, 180);
		sprintf(cNapis, "Deklinacja: %.2f %c", DEKLINACJA_MAG * RAD2DEG, ZNAK_STOPIEN);
		RysujNapis(cNapis, KOL12, 200);

		setColor(SZARY60);
		sprintf(cNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		RysujNapis(cNapis, CENTER, 30);

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
	sprintf(cNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[0], ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12 + 14*FONT_SL, 80);

	setColor(KOLOR_Y);
	sprintf(cNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[1], ZNAK_STOPIEN);
	RysujNapis(cNapis, KOL12 + 12*FONT_SL, 100);


	setColor(CYJAN);
	sprintf(cNapis, "%.2f, %.2f [uT] ", uDaneCM4.dane.fMagne1[0]*1000, uDaneCM4.dane.fMagne1[1]*1e6f);
	RysujNapis(cNapis, KOL12 + 8*FONT_SL, 120);

	setColor(MAGENTA);
	sprintf(cNapis, "%.2f, %.2f [uT] ", uDaneCM4.dane.fMagne2[0]*1000, uDaneCM4.dane.fMagne2[1]*1e6f);
	RysujNapis(cNapis, KOL12 + 8*FONT_SL, 140);

	setColor(ZOLTY);
	sprintf(cNapis, "%.2f, %.2f [uT] ", uDaneCM4.dane.fMagne3[0]*1000, uDaneCM4.dane.fMagne3[1]*1e6f);
	RysujNapis(cNapis, KOL12 + 8*FONT_SL, 160);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran Nastaw regulatora PID
// Parametry: chKanal - numer kanału 0..11
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void NastawyPID(uint8_t chKanal)
{
	float fNastawy[ROZMIAR_REG_PID/4];
	uint8_t chTrybRegulatora[LICZBA_DRAZKOW];
	uint8_t cBłąd;
	unia8_32_t un8_32;

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		sprintf(cNapis, "%s %s", cNapisLcd[STR_NASTAWY_PID], cNapisLcd[STR_PRZECHYLENIA + chKanal/2]);
		BelkaTytulu(cNapis);

		setColor(SZARY80);
		sprintf(cNapis, "Regulator g%c%cwny", ł, ó);
		RysujNapis(cNapis, KOL12, 30);
		sprintf(cNapis, "Kp:");
		RysujNapis(cNapis, KOL12, 50);
		sprintf(cNapis, "Ti:");
		RysujNapis(cNapis, KOL12, 70);
		sprintf(cNapis, "Td:");
		RysujNapis(cNapis, KOL12, 90);
		sprintf(cNapis, "Max I:");
		RysujNapis(cNapis, KOL12, 110);
		sprintf(cNapis, "Min Wy:");
		RysujNapis(cNapis, KOL12, 130);
		sprintf(cNapis, "Max Wy:");
		RysujNapis(cNapis, KOL12, 150);
		sprintf(cNapis, "Skala w.zad:");
		RysujNapis(cNapis, KOL12, 170);
		sprintf(cNapis, "Przes w.zad:");
		RysujNapis(cNapis, KOL12, 190);
		sprintf(cNapis, "Filtr D:");
		RysujNapis(cNapis, KOL12, 210);
		sprintf(cNapis, "Filtr w.zad:");
		RysujNapis(cNapis, KOL12, 230);
		sprintf(cNapis, "Akcja wyprz:");
		RysujNapis(cNapis, KOL12, 250);
		sprintf(cNapis, "Reg k%ctowy:", ą);
		RysujNapis(cNapis, KOL12, 270);

		sprintf(cNapis, "Regulator pochodnej");
		RysujNapis(cNapis, KOL22, 30);
		sprintf(cNapis, "Kp:");
		RysujNapis(cNapis, KOL22, 50);
		sprintf(cNapis, "Ti:");
		RysujNapis(cNapis, KOL22, 70);
		sprintf(cNapis, "Td:");
		RysujNapis(cNapis, KOL22, 90);
		sprintf(cNapis, "Max I:");
		RysujNapis(cNapis, KOL22, 110);
		sprintf(cNapis, "Min Wy:");
		RysujNapis(cNapis, KOL22, 130);
		sprintf(cNapis, "Max Wy:");
		RysujNapis(cNapis, KOL22, 150);
		sprintf(cNapis, "Skala w.zad:");
		RysujNapis(cNapis, KOL22, 170);
		sprintf(cNapis, "Przes w.zad:");
		RysujNapis(cNapis, KOL22, 190);
		sprintf(cNapis, "Filtr D:");
		RysujNapis(cNapis, KOL22, 210);
		sprintf(cNapis, "Filtr w.zad:");
		RysujNapis(cNapis, KOL22, 230);
		sprintf(cNapis, "Akcja wyprz:");
		RysujNapis(cNapis, KOL22, 250);
		sprintf(cNapis, "Reg k%ctowy:", ą);
		RysujNapis(cNapis, KOL22, 270);
		setColor(SZARY60);
		RysujNapis((char*)cOpisBledow[KOMUNIKAT_DUS_I_TRZYMAJ], CENTER, 300);

		//odczytaj tryb pracy regulatorów
		cBłąd = CzytajFramChar(FAU_TRYB_REG, LICZBA_REG_PARAM, chTrybRegulatora);


		//odczytaj nastawy regulatorów
		cBłąd = CzytajFramFloat(FAU_PID_KP + (chKanal + 0) * ROZMIAR_REG_PID, ROZMIAR_REG_PID / 4, fNastawy);
		if (cBłąd == BLAD_OK)
		{
			//CM4 reaguje na zmianę polecenia, więc zmień na polecenie neutralne, aby można było ponowić odczyt drugiego regulatora
			uDaneCM7.dane.cWykonajPolecenie = POL7_NIC;	//nie odczytuj więcej danych
			osDelay(5);										//czas na propagację polecenia w innym wątku
			if (chTrybRegulatora[chKanal/2] > REG_AKRO)
				setColor(BIALY);	//regulator pracuje
			else
				setColor(SZARY60);	//regulator wyłączony
			sprintf(cNapis, "%.3f ", fNastawy[0]);	//Kp
			RysujNapis(cNapis, KOL12 + 4*FONT_SL, 50);
			sprintf(cNapis, "%.4f ", fNastawy[1]);	//Ti
			RysujNapis(cNapis, KOL12 + 4*FONT_SL, 70);
			sprintf(cNapis, "%.4f ", fNastawy[2]);	//Td
			RysujNapis(cNapis, KOL12 + 4*FONT_SL, 90);
			sprintf(cNapis, "%.1f ", fNastawy[3]);	//max całki
			RysujNapis(cNapis, KOL12 + 7*FONT_SL, 110);
			sprintf(cNapis, "%.1f ", fNastawy[4]);	//min wyjścia
			RysujNapis(cNapis, KOL12 + 8*FONT_SL, 130);
			sprintf(cNapis, "%.1f ", fNastawy[5]);	//max wyjścia
			RysujNapis(cNapis, KOL12 + 8*FONT_SL, 150);
			sprintf(cNapis, "%.4f ", fNastawy[6]);	//skalowanie wartości zadanej
			RysujNapis(cNapis, KOL12 + 13*FONT_SL, 170);
			sprintf(cNapis, "%.4f ", fNastawy[7]);	//stałe przesunięcie wartości zadanej
			RysujNapis(cNapis, KOL12 + 13*FONT_SL, 190);
			//fNastawy[8] = miejsce na 1 zmienną rezerwową
			un8_32.daneFloat =  fNastawy[9];
			sprintf(cNapis, "%d", un8_32.dane8[1]);
			RysujNapis(cNapis, KOL12 + 9*FONT_SL, 210);	//filtr D
			sprintf(cNapis, "%d", un8_32.dane8[2]);
			RysujNapis(cNapis, KOL12 + 13*FONT_SL, 230);	//filtr wartości zadanej
			sprintf(cNapis, "%d", un8_32.dane8[3]);
			RysujNapis(cNapis, KOL12 + 13*FONT_SL, 250);	//procent wyprzedzenia

			if (un8_32.dane8[0] & PID_KATOWY)
				sprintf(cNapis, cNapisLcd[STR_TAK]);
			else
				sprintf(cNapis, cNapisLcd[STR_NIE]);
			RysujNapis(cNapis, KOL12 + 13*FONT_SL, 270);	//flagi: kątowy
		}
		else
			cRysujRaz = 1;	//jeżeli się nie odczytało to wyświetl jeszcze raz

		cBłąd = CzytajFramFloat(FAU_PID_KP + (chKanal + 1) * ROZMIAR_REG_PID, ROZMIAR_REG_PID / 4, fNastawy);
		if (cBłąd == BLAD_OK)
		{
			uDaneCM7.dane.cWykonajPolecenie = POL7_NIC;	//nie odczytuj więcej danych - potrzebna zmiana polecenia aby mógł wykonać kolejne
			osDelay(5);
			if (chTrybRegulatora[chKanal/2] > REG_RECZNA)
				setColor(BIALY);	//regulator pracuje
			else
				setColor(SZARY60);	//regulator wyłączony
			sprintf(cNapis, "%.3f ", fNastawy[0]);	//Kp
			RysujNapis(cNapis, KOL22 + 4*FONT_SL, 50);
			sprintf(cNapis, "%.4f ", fNastawy[1]);	//Ti
			RysujNapis(cNapis, KOL22 + 4*FONT_SL, 70);
			sprintf(cNapis, "%.4f ", fNastawy[2]);	//Td
			RysujNapis(cNapis, KOL22 + 4*FONT_SL, 90);
			sprintf(cNapis, "%.1f ", fNastawy[3]);	//max całki
			RysujNapis(cNapis, KOL22 + 7*FONT_SL, 110);
			sprintf(cNapis, "%.1f ", fNastawy[4]);	//min wyjścia
			RysujNapis(cNapis, KOL22 + 8*FONT_SL, 130);
			sprintf(cNapis, "%.1f ", fNastawy[5]);	//max wyjścia
			RysujNapis(cNapis, KOL22 + 8*FONT_SL, 150);
			sprintf(cNapis, "%.4f ", fNastawy[6]);	//skalowanie wartości zadanej
			RysujNapis(cNapis, KOL22 + 13*FONT_SL, 170);
			sprintf(cNapis, "%.4f ", fNastawy[7]);	//stałe przesunięcie wartości zadanej
			RysujNapis(cNapis, KOL22 + 13*FONT_SL, 190);
			//fNastawy[8] = miejsce na 1 zmienną rezerwową
			un8_32.daneFloat =  fNastawy[9];
			sprintf(cNapis, "%d", un8_32.dane8[1]);
			RysujNapis(cNapis, KOL22 + 9*FONT_SL, 210);	//filtr D
			sprintf(cNapis, "%d", un8_32.dane8[2]);
			RysujNapis(cNapis, KOL22 + 13*FONT_SL, 230);	//filtr wzmocnienia
			sprintf(cNapis, "%d", un8_32.dane8[3]);
			RysujNapis(cNapis, KOL22 + 13*FONT_SL, 250);	//procent wyprzedzenia
			if (un8_32.dane8[0] & PID_KATOWY)
				sprintf(cNapis, cNapisLcd[STR_TAK]);
			else
				sprintf(cNapis, cNapisLcd[STR_NIE]);
			RysujNapis(cNapis, KOL22 + 13*FONT_SL, 270);	//flagi: kątowy
		}
		else
			cRysujRaz = 1;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje zawartość pamieci FRAM z CM4 jako liczby float
// Parametry:
// [we] sAdres - adres komórki pamieci FRAM
// [we] cRozmiar - ilość liczb float do odczytu
// [wy] *fDane - wskaźnik na odczytywaną strukturę danych float
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajFramFloat(uint16_t sAdres, uint8_t cRozmiar, float *fDane)
{
	cTimeout = 10;
	uDaneCM7.dane.cRozmiar = cRozmiar;
	uDaneCM7.dane.sAdres = sAdres;
	uDaneCM7.dane.cWykonajPolecenie = POL7_CZYTAJ_FRAM_FLOAT;
	do
	{
		osDelay(5);
		cTimeout--;
	}
	while ((uDaneCM4.dane.sAdres != uDaneCM7.dane.sAdres) && (cTimeout));
	for (uint8_t n=0; n<uDaneCM4.dane.cRozmiar; n++)
		*(fDane + n) = uDaneCM4.dane.uRozne.f32[n];

	if (cTimeout)
		return BLAD_OK;
	else
		return BLAD_TIMEOUT;
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje zawartość pamieci FRAM z CM4 jako liczby unsigned char
// Parametry:
// [we] sAdres - adres komórki pamieci FRAM
// [we] cRozmiar - ilość liczb float do odczytu
// [wy] *fDane - wskaźnik na odczytywaną strukturę danych float
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajFramChar(uint16_t sAdres, uint8_t cRozmiar, uint8_t *chDane)
{
	cTimeout = 10;
	uDaneCM7.dane.cRozmiar = cRozmiar;
	uDaneCM7.dane.sAdres = sAdres;
	uDaneCM7.dane.cWykonajPolecenie = POL7_CZYTAJ_FRAM_U8;
	do
	{
		osDelay(5);
		cTimeout--;
	}
	while ((uDaneCM4.dane.sAdres != uDaneCM7.dane.sAdres) && (cTimeout));
	for (uint8_t n=0; n<uDaneCM4.dane.cRozmiar; n++)
		*(chDane + n) = uDaneCM4.dane.uRozne.U8[n];

	if (cTimeout)
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



////////////////////////////////////////////////////////////////////////////////
// Liczy 6 FFT z żyroskopów i akcelerometrów ale wyświetla tylko 3 wybrane ze względu na czytelność.
// Na dole wykresu rysuje 100 linii wodospadu
// Pomiary mieszczące się w szerokości okna przedstawia w postaci wykresów, szersze obcina, węższe rozciaga do szerokości okna wykresu
// Parametry:
// [we] *stWynik - wskaźnik na dane wynikowe FFT
// [we] *stKonfig - wskaźnik na konfigurację FFT
// [we] chRodzajDanych - wskazuje na rodzaj danych z których robione jest FFT: 0 akcelerometry, 1 żyroskopy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujFFT(float *stWynik, stFFT_t *stKonfig, uint8_t chRodzajDanych)
{
	uint32_t nCzasFFT;
	float fWspWypX;	//współczynnik wypełnienie ekranu danymi pomiarowymi w poziomie
	float fZajetoscEkranu = 0;		//wypełnienie ekranu danymi dodawanymi jako zmiennoprzecinkowe
	float fOkno;
	float fMinY;	//ekstrema wyniku FFT potrzebne do skalowania wykresu
	int x1, x2, y1[LICZBA_WYKRESOW_FFT], y2;
	uint16_t sIndexDanych;	//indeky kolejnych pobieranych danych
	uint8_t chKolorRGB666[3];
	uint8_t chIndeksWybranychWykresow;	//przy iteracji po 3 wykresach wskazuje na akcelerometry lub żyroskopy

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		sprintf(cNapis, "FFT %sow", cNapisLcd[STR_AKCELETOMETR + (chRodzajDanych & 0x01)]);
		BelkaTytulu(cNapis);
		RysujProstokatWypelniony(0, AD_STARTY + AD_Y_SIZE, DISP_X_SIZE, AD_WSPADY, CZARNY);	//czyści ekran wodospadu
		WłączTelemetrię(TELEM_SZYBKA);	//włącz szybką telemetrię. Wyłączy się sama po przesłaniu wszystkich wyników
		stKonfig->sLiczbaProbek = 1 << stKonfig->chWykladnikPotegi;	//aktualizuj liczbę próbek
	}

	if (stKonfig->chStatus & FFT_NOWE_DANE)
	{
		nCzasFFT = PobierzCzasT6();
		for (uint8_t czujnik=0; czujnik<LICZBA_ZMIENNYCH_FFT; czujnik++)	//iteracja po czujnikach a nie po wykresach aby aplikacja mogła pobrać wszystkie dane
		{
			for (uint16_t n=0; n<stKonfig->sLiczbaProbek; n++)
			{
				switch(stKonfigFFT.chRodzajOkna)
				{
				case 1: fOkno = 0.53836 - 0.46164 * cos(2 * M_PI * n / (stKonfig->sLiczbaProbek - 1));	break;	//Okno_Hamminga
				case 2: fOkno = 0.5 * (1 - cos(2 * M_PI * n /(stKonfig->sLiczbaProbek - 1)));		break;		//Okno Hanna
				case 3: fOkno = 0.35875 - 0.48829 * cos(2 * M_PI * n / (stKonfig->sLiczbaProbek - 1)) +
										  0.14128 * cos(4 * M_PI * n / (stKonfig->sLiczbaProbek - 1)) -
										  0.01168 * cos(6 * M_PI * n / (stKonfig->sLiczbaProbek - 1)); 	break;	//Okno Blackmana-Harrisa
				default: fOkno = 1;	break;	//okno prostokątne
				}

				stWejscie[n].Re = fOkno * fBuforPomiarow[czujnik][n];		//przemnóż przez okno
				stWejscie[n].Im = 0.0f;
			}

			LiczFFT(stKonfig, czujnik);
		}
		fMinY = 2.5f;	//przyjmuję stały punkt odniesienia aby poziom zera był niezmienny

		nCzasFFT = MinalCzas(nCzasFFT);
		RysujProstokatWypelniony(0, MENU_NAG_WYS, DISP_X_SIZE, DISP_Y_SIZE - AD_WSPADY - AD_STARTY, CZARNY);	//czyści ekran wykresów
		setColor(SZARY80);
		sprintf(cNapis, "Czas FFT[6*%d]: %ld us, %d/%d", stKonfig->sLiczbaProbek, nCzasFFT, stKonfig->chIndeksTestu, LICZBA_TESTOW_FFT);
		RysujNapis(cNapis, 60, 20);

		fWspWypX = (float)(AD_X_SIZE)/((stKonfig->sLiczbaProbek/2)-1);	//współczynnik wypełnienia ekranu danymi pikseli / wynik

		sIndexDanych = 0;
		x1 = AD_STARTX;

		//rysuj podziałki i skalę pionową
		for (uint8_t n=0; n<AD_DZIALEK_SKALIY + 1; n++)	//iteracja po liczbie podziałek na skali
		{
			y2 = AD_STARTY + n * (AD_Y_SIZE / AD_DZIALEK_SKALIY);
			if (n == AD_DZIALEK_0Y)
				setColor(SZARY40);	//kolor podziałki 0
			else
				setColor(SZARY20);	//kolor pozostałych podziałek
			RysujLiniePozioma(x1, y2, AD_X_SIZE);

			sprintf(cNapis, "%d", (int8_t)AD_DZIALEK_SKALIY - (AD_DZIALEK_SKALIY - AD_DZIALEK_0Y) - n);
			RysujNapis(cNapis, 0, y2 - FONT_SH / 2);

		}

		for (uint8_t w=0; w<LICZBA_WYKRESOW_FFT; w++)	//iteracja po czujnikach
			y1[w] = AD_STARTY;

		for (uint16_t n=0; n<AD_X_SIZE; n++)
		{
			if (fWspWypX > 1.0)	//wykresy będą rozciagane do ekranu
			{
				fZajetoscEkranu -= 1;	//odejmij piksel obrazu
				if (fZajetoscEkranu < 0)
				{
					//fPixel = fWynikFFT[sIndexDanych] + fMinY;	//pobierz słowo danych przesuniete z minimum do zera
					sIndexDanych++;
					fZajetoscEkranu += fWspWypX;
				}
			}
			else	//wykresy będą ściskane po wiecej niż jedną liczbę na piksel
			{
				do	//pobierz dane z wyliczonych indeksów zmiennej
				{
					fZajetoscEkranu += fWspWypX;	//wypełnienie piksela danymi
					//fPixel = fWynikFFT[sIndexDanych] + fMinY;
					sIndexDanych++;
				}
				while (fZajetoscEkranu < 1);
				fZajetoscEkranu -= 1;	//odejmij piksel obrazu
			}

			//wykres FFT
			x2 = AD_STARTX + n;
			for (uint8_t w=0; w<LICZBA_WYKRESOW_FFT; w++)	//iteracja po czujnikach
			{
				chIndeksWybranychWykresow = w + chRodzajDanych * LICZBA_WYKRESOW_FFT;
				y2 = (AD_STARTY + AD_Y_SIZE - AD_POZIOM_0DB) - (int16_t)(fWynikFFT[stKonfig->chIndeksTestu][chIndeksWybranychWykresow][sIndexDanych] * AD_Y_DIV);

				if (y2 > AD_STARTY + AD_Y_SIZE - 1)
					y2 = AD_STARTY + AD_Y_SIZE - 1;
				else
				if (y2 < AD_STARTY)
					y2 = AD_STARTY;

				 //ustaw kolor wykresu
				switch (w)
				{
				case 0: setColor(KOLOR_X);	break;	//oś X acc
				case 1:	setColor(KOLOR_Y);	break;	//oś Y
				case 2: setColor(KOLOR_Z);	break;	//oś Z
				case 3: setColor(CYJAN);	break;	//oś X żyro
				case 4:	setColor(MAGENTA);	break;	//oś Y
				case 5: setColor(ZOLTY);	break;	//oś Z
				}

				RysujLinie(x1, y1[w], x2, y2);		//wykres FFT
				y1[w] = y2;
			}

			//rysuj wodospad o rozmiarze LICZBA_TESTOW_FFT
			y2 = AD_STARTY + AD_Y_SIZE + stKonfig->chIndeksTestu;

			//ustaw kolor RGB
			chKolorRGB666[0] = (fWynikFFT[stKonfig->chIndeksTestu][0 + chRodzajDanych * LICZBA_WYKRESOW_FFT][sIndexDanych] + fMinY) * WODOSPAD_SKALA_KOLORU;
			chKolorRGB666[1] = (fWynikFFT[stKonfig->chIndeksTestu][1 + chRodzajDanych * LICZBA_WYKRESOW_FFT][sIndexDanych] + fMinY) * WODOSPAD_SKALA_KOLORU;
			chKolorRGB666[2] = (fWynikFFT[stKonfig->chIndeksTestu][2 + chRodzajDanych * LICZBA_WYKRESOW_FFT][sIndexDanych] + fMinY) * WODOSPAD_SKALA_KOLORU;
			RysujPunkt(x2, y2, chKolorRGB666);
			x1 = x2;
		}
		stKonfig->chIndeksTestu++;
		if (stKonfig->chIndeksTestu > LICZBA_TESTOW_FFT+1)	//wartość większa o 1 jest potrzebna do detekcji końca testu
			stKonfig->chIndeksTestu = 0;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Wyśwetla czasy wykonania elementów skłądowych Petli Głównej AutoPilota
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PokazCzasOdcinkowPGAP(uint16_t *sCzasy)
{
	uint32_t nSuma = 0;
	float fCzestotliwosc;

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		BelkaTytulu("Czasy Petli Glownej AP");
		setColor(SZARY80);
		for (uint8_t n=0; n<4; n++)
		{
			sprintf(cNapis, "Modul %d:", n+1);
			RysujNapis(cNapis, 0, 30 + n*20);
		}
		sprintf(cNapis, "Odb. GNSS:");
		RysujNapis(cNapis, 0, 110);
		sprintf(cNapis, "Oper. I2C:");
		RysujNapis(cNapis, 0, 130);
		sprintf(cNapis, "f Kalmana:");
		RysujNapis(cNapis, 0, 150);
		sprintf(cNapis, "IMU trygon:");
		RysujNapis(cNapis, 0, 170);
		sprintf(cNapis, "IMU kwater:");
		RysujNapis(cNapis, 0, 190);
		sprintf(cNapis, "Odbiorn RC:");
		RysujNapis(cNapis, 0, 210);
		sprintf(cNapis, "Rozsz. I/O:");
		RysujNapis(cNapis, 0, 230);
		sprintf(cNapis, "Modul IMU:");
		RysujNapis(cNapis, 0, 250);
		sprintf(cNapis, "Wymiana:");
		RysujNapis(cNapis, 0, 270);
		sprintf(cNapis, "PolCM7:");
		RysujNapis(cNapis, 0, 290);

		//druga kolumna danych
		sprintf(cNapis, "LED WS2813:");
		RysujNapis(cNapis, KOL22, 30);
		sprintf(cNapis, "--15--");
		RysujNapis(cNapis, KOL22, 50);
		sprintf(cNapis, "--16--");
		RysujNapis(cNapis, KOL22, 70);
		sprintf(cNapis, "Kontr.Lotu:");
		RysujNapis(cNapis, KOL22, 90);
		sprintf(cNapis, "Mikser:");
		RysujNapis(cNapis, KOL22, 110);
		sprintf(cNapis, "Wyjscia RC:");
		RysujNapis(cNapis, KOL22, 130);
		sprintf(cNapis, "ADC:");
		RysujNapis(cNapis, KOL22, 150);
		sprintf(cNapis, "Suma Czasu:");
		RysujNapis(cNapis, KOL22, 170);
		sprintf(cNapis, "f Petli Gl:");
		RysujNapis(cNapis, KOL22, 190);
	}

	setColor(ZOLTY);
	for (uint8_t n=0; n<14; n++)
	{
		sprintf(cNapis, "%d ", sCzasy[n]);
		RysujNapis(cNapis, 12*FONT_SL, 30 + n*20);
	}

	for (uint8_t n=0; n<7; n++)
	{
		sprintf(cNapis, "%d ", sCzasy[n+14]);
		RysujNapis(cNapis, KOL22 + 12*FONT_SL, 30 + n*20);
	}

	setColor(ZIELONY);
	//licz sumę czasów
	for (uint8_t n=0; n<LICZBA_ODCINKOW_CZASU; n++)
		nSuma += sCzasy[n];
	sprintf(cNapis, "%ld us ", nSuma);
	RysujNapis(cNapis, KOL22 + 12*FONT_SL, 170);

	if (nSuma)
		fCzestotliwosc = 1000000.0f / nSuma;
	else
		fCzestotliwosc = 0;
	sprintf(cNapis, "%.1f Hz ", fCzestotliwosc);
	RysujNapis(cNapis, KOL22 + 12*FONT_SL, 190);


}

