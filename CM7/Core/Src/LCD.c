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
#include "RPi35B_480x320.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <W25Q128JV.h>
#include "dotyk.h"
#include "napisy.h"
#include "flash_nor.h"
#include "wymiana.h"
#include "audio.h"
#include "protokol_kom.h"
#include "ff.h"
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

//deklaracje zmiennych
extern uint8_t MidFont[];
extern uint8_t BigFont[];
const char *build_date = __DATE__;
const char *build_time = __TIME__;
extern const unsigned short obr_multimetr[];
extern const unsigned short obr_multitool[];
extern const unsigned short pitlab_logo18[];
extern const char *chNazwyMies3Lit[] ;
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
extern const unsigned short obr_wroc[];
extern const unsigned short obr_foto[];
extern const unsigned short obr_Mikołaj_Rey[];
extern const unsigned short obr_Wydajnosc[];
extern const unsigned short obr_mtest[];
extern const unsigned short obr_back[];
extern const unsigned short obr_volume[];
extern const unsigned short obr_touch[0xFFC];
extern const unsigned short obr_dotyk[0xFFC];
extern const unsigned short obr_dotyk_zolty[0xFFC];
extern const unsigned short obr_kartaSD[0xFFC];
extern const unsigned short obr_kartaSDneg[0xFFC];
extern const unsigned short obr_baczek[0xF80];
extern const unsigned short obr_kal_mag_n1[0xFFC];
extern const unsigned short obr_kontrolny[0xFFC];
extern const unsigned short obr_kostka3D[0xFFC];
extern const unsigned short obr_cisnienie[0xFFC];
extern const unsigned short obr_okregi[0xFFC];
extern const unsigned short obr_narzedzia[0xFFC];
extern const unsigned short obr_aparaturaRC[0xFFC];
extern const unsigned short obr_aparat[0xFFC];
extern const unsigned short obr_kamera[0xFFC];

//definicje zmiennych
uint8_t chTrybPracy;
uint8_t chNowyTrybPracy;
uint8_t chWrocDoTrybu;
uint8_t chRysujRaz;
uint8_t chMenuSelPos, chStarySelPos;	//wybrana pozycja menu i poprzednia pozycja

char chNapis[100], chNapisPodreczny[30];
float fTemperaturaKalibracji;
//float fZoom, fX, fY;
//float fReal, fImag;
//uint8_t chMnozPalety;
//uint8_t chDemoMode;
uint8_t chLiczIter;		//licznik iteracji wyświetlania
//extern uint16_t sBuforLCD[DISP_X_SIZE * DISP_Y_SIZE];
extern struct _statusDotyku statusDotyku;
extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
extern uint8_t chPort_exp_wysylany[];
extern uint8_t chPort_exp_odbierany[];
extern uint8_t chGlosnosc;		//regulacja głośności odtwarzania komunikatów w zakresie 0..SKALA_GLOSNOSCI
extern SD_HandleTypeDef hsd1;
extern uint8_t chStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern RTC_HandleTypeDef hrtc;
static uint8_t chOstatniCzas;
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
extern uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforKamery[ROZM_BUF16_KAM];
extern uint16_t sBuforKameryAxi[ROZM_BUF16_KAM];

//Definicje ekranów menu
struct tmenu stMenuGlowne[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Pomiary",		"Wyniki pomiarow czujnikow",				TP_POMIARY, 		obr_multimetr},
	{"Kalibracje", 	"Kalibracja czujnikow pokladowych",			TP_KALIBRACJE,		obr_kontrolny},
	{"Nastawy",  	"Ustawienia podsystemow",					TP_NASTAWY, 		obr_narzedzia},
	{"Wydajnosc",	"Pomiary wydajnosci systemow",				TP_WYDAJNOSC,		obr_Wydajnosc},
	{"Karta SD",	"Rejestrator i parametry karty SD",			TP_KARTA_SD,		obr_kartaSD},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"Multimedia",  "Obsluga multimediow: dzwiek i obraz",		TP_MULTIMEDIA, 		obr_glosnik2}};



struct tmenu stMenuKalibracje[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Kal IMU", 	"Kalibracja czujników IMU",					TP_KAL_IMU,			obr_baczek},
	{"Kal Magn", 	"Kalibracja magnetometrow",					TP_KAL_MAG,			obr_kal_mag_n1},
	{"Kal Baro", 	"Kalibracja cisnienia wg wzorca 10 pieter",	TP_KAL_BARO,		obr_cisnienie},
	{"Kal Dotyk", 	"Kalibracja panelu dotykowego na LCD",		TP_KAL_DOTYK,		obr_dotyk_zolty},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"nic",			"nic",										TP_W3,				obr_narzedzia},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_back}};

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
	{"TestDotyk",	"Testy panelu dotykowego",					TP_TESTY,			obr_dotyk_zolty},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_back}};

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
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_back}};


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
	{"SD18",		"nic",										TP_W4,				obr_Wydajnosc},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_back}};

struct tmenu stMenuMultiMedia[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Miki Rej",  	"Obsluga kamery, aparatu i obrobka obrazu",	TP_MMREJ,	 		obr_Mikołaj_Rey},
	{"Mikrofon",	"Wlacza mikrofon wylacza wzmacniacz",		TP_MM1,				obr_glosnik2},
	{"Wzmacniacz",	"Wlacza wzmacniacz, wylacza mikrofon",		TP_MM2,				obr_glosnik2},
	{"Test Tonu",	"Test tonu wario",							TP_MM_TEST_TONU,	obr_glosnik2},
	{"FFT Audio",	"FFT sygnału z mikrofonu",					TP_MM_AUDIO_FFT,	obr_fft},
	{"Zdjecie",		"Wykonuje kamerą statyczne zdjecie",		TP_MM_ZDJECIE,		obr_aparat},
	{"Kamera",		"Uruchamia kamerę w trybie ciagłem",		TP_MM_KAMERA,		obr_kamera},
	{"Test kom.",	"Test komunikatów audio",					TP_MM_KOM,			obr_glosnik2},
	{"Startowy",	"Ekran startowy",							TP_WITAJ,			obr_kontrolny},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_back}};

struct tmenu stMenuKartaSD[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Wlacz Rej",   "Wlacza rejestrator",   					TPKS_WLACZ_REJ,		obr_kartaSD},
	{"Wylacz Rej",  "Wylacza rejestrator",   					TPKS_WYLACZ_REJ,	obr_kartaSD},
	{"Parametry",	"Parametry karty SD",						TPKS_PARAMETRY,		obr_kartaSD},
	//{"Test trans",	"Wyniki pomiaru predkosci transferu",		TPKS_POMIAR, 		obr_multimetr},
	{"TPKS_4",		"nic  ",									TPKS_4,				obr_dotyk_zolty},
	{"TPKS_4",		"nic  ",									TPKS_4,				obr_dotyk_zolty},
	{"TPKS_5",		"nic  ",									TPKS_5,				obr_dotyk_zolty},
	{"TPKS_6",		"nic  ",									TPKS_6,				obr_dotyk_zolty},
	{"TPKS_7",		"nic  ",									TPKS_7,				obr_dotyk_zolty},
	{"TPKS_8",		"nic  ",									TPKS_8,				obr_dotyk_zolty},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_back}};


struct tmenu stMenuIMU[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Zyro Zimno", 	"Kalibracja zera zyroskopow w 10C",			TP_KAL_ZYRO_ZIM,	obr_baczek},
	{"Zyro Pokoj", 	"Kalibracja zera zyroskopow w 25C",			TP_KAL_ZYRO_POK,	obr_baczek},
	{"Zyro Gorac", 	"Kalibracja zera zyroskopow w 40C",			TP_KAL_ZYRO_GOR,	obr_baczek},
	{"Akcel 2D",	"Kalibracja akcelerometrow 2D",				TP_KAL_AKCEL_2D,	obr_kontrolny},
	{"Akcel 3D",	"Kalibracja akcelerometrow 3D",				TP_KAL_AKCEL_3D,	obr_kontrolny},
	{"Wzm ZyroP", 	"Kalibracja wzmocnienia zyroskopow P",		TP_KAL_WZM_ZYROP,	obr_baczek},
	{"Wzm ZyroQ", 	"Kalibracja wzmocnienia zyroskopow Q",		TP_KAL_WZM_ZYROQ,	obr_baczek},
	{"Wzm ZyroR", 	"Kalibracja wzmocnienia zyroskopow R",		TP_KAL_WZM_ZYROR,	obr_baczek},
	{"ZeroZyro", 	"Kasuj dryft scalkowanego kata z zyro.",	TP_KASUJ_DRYFT_ZYRO,obr_kostka3D},
	//{"Kostka 3D", 	"Rysuje kostke 3D w funkcji katow IMU",		TP_IMU_KOSTKA,	obr_kostka3D},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_back}};

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
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_back}};


////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran główny odświeżany w głównej pętli programu
// Parametry:
// Zwraca: nic
// Czas rysowania pełnego ekranu: 174ms
////////////////////////////////////////////////////////////////////////////////
void RysujEkran(void)
{
	if ((statusDotyku.chFlagi & DOTYK_SKALIBROWANY) != DOTYK_SKALIBROWANY)		//sprawdź czy ekran dotykowy jest skalibrowany
		chTrybPracy = TP_KAL_DOTYK;

	switch (chTrybPracy)
	{
	case TP_MENU_GLOWNE:	// wyświetla menu główne	MenuGlowne(&chNowyTrybPracy);
		Menu((char*)chNapisLcd[STR_MENU_MAIN], stMenuGlowne, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_KAL_BARO:	//kalibracja ciśnienia według wzorca
		if (KalibrujBaro(&chSekwencerKalibracji) == ERR_GOTOWE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MENU;
		}
		break;

	case TP_TESTY:
		if (TestDotyku() == ERR_GOTOWE)
			chNowyTrybPracy = TP_WROC_DO_MENU;
		break;


//***************************************************
	case TP_MULTIMEDIA:			//menu multimediow
		Menu((char*)chNapisLcd[STR_MENU_MULTI_MEDIA], stMenuMultiMedia, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;

	case TP_MMREJ:
		//OdtworzProbkeAudio((uint32_t)&sNiechajNarodowie[0], 129808);
		OdtworzProbkeAudioZeSpisu(PRGA_NIECHAJ_NARODO);
		chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		break;

	case TP_MM1:		//"Włącza mikrofon, włącza wzmacniacz
		chPort_exp_wysylany[1] |= EXP13_AUDIO_IN_SD;	//AUDIO_IN_SD - włącznika ShutDown mikrofonu, aktywny niski
		chPort_exp_wysylany[1] &= ~EXP14_AUDIO_OUT_SD;	//AUDIO_OUT_SD - włączniek ShutDown wzmacniacza audio, aktywny niski
		chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		break;

	case TP_MM2:	//Włącza wzmacniacz, włącza mikrofon
		chPort_exp_wysylany[1] &= ~EXP13_AUDIO_IN_SD;	//AUDIO_IN_SD - włącznika ShutDown mikrofonu, aktywny niski
		chPort_exp_wysylany[1] |= EXP14_AUDIO_OUT_SD;	//AUDIO_OUT_SD - włączniek ShutDown wzmacniacza audio, aktywny niski
		chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		break;

	case TP_MM_TEST_TONU:
		TestTonuAudio();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			ZatrzymajTon();
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		}
		break;

	case TP_MM_AUDIO_FFT:			//FFT sygnału z mikrofonu
		RejestrujAudio();
		chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		break;

	case TP_MM_ZDJECIE:	//pojedyncze zdjęcie
		//ZrobZdjecie(480, 320);
		ZrobZdjecie(160, 120);
		//ZrobZdjecie(320,240);
		//WyswietlZdjecie(320,240, sBuforKamery);
		//WyswietlZdjecie(480, 320, sBuforKamery);
		WyswietlZdjecie(160, 120, sBuforKameryAxi);
		chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		break;

	case TP_MM_KAMERA:	//ciagła praca kamery
		RozpocznijPraceDCMI(0);
		WyswietlZdjecie(480, 320, sBuforKamery);
		chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		break;

	case TP_MM_KOM:
		TestKomunikatow();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		}
		break;

	case TP_WITAJ:
		if (!chLiczIter)
			chLiczIter = 15;			//ustaw czas wyświetlania x 200ms
		Ekran_Powitalny(nZainicjowanoCM7);	//przywitaj użytkownika i prezentuj wykryty sprzęt
		if (!chLiczIter)				//jeżeli koniec odliczania to wyjdź
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		}
		break;

//***************************************************
	case TP_WYDAJNOSC:			///menu pomiarów wydajności
		Menu((char*)chNapisLcd[STR_MENU_WYDAJNOSC], stMenuWydajnosc, &chNowyTrybPracy);
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


	case TP_EMU_MAG_CAN:
		uDaneCM7.dane.chWykonajPolecenie = POL_KAL_ZERO_MAGN2;	//włącz tryb jak dla kalibracji aby nie uwzględniać w wyniku danych kalibracyjnych
		EmulujMagnetometrWizjerCan((float*)uDaneCM4.dane.fMagne2);
		setFont(BigFont);
		setColor(KOLOR_X);
		sprintf(chNapis, "Mag X: %.3f uT ", uDaneCM4.dane.fMagne2[0] * 1e6);
		print(chNapis, KOL12, 40);
		setColor(KOLOR_Y);
		sprintf(chNapis, "Mag Y: %.3f uT ", uDaneCM4.dane.fMagne2[1] * 1e6);
		print(chNapis, KOL12, 70);
		setColor(KOLOR_Z);
		sprintf(chNapis, "Mag Z: %.3f uT ", uDaneCM4.dane.fMagne2[2] * 1e6);
		print(chNapis, KOL12, 100);

		if (chRysujRaz)
		{
			BelkaTytulu("Emulacja magnetometru CAN");
			chRysujRaz = 0;
			setColor(GRAY50);
			sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
			print(chNapis, CENTER, 300);
		}

		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
			uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//zakończ tryb kalibracyjny
		}
	break;

	//***************************************************
	case TP_KARTA_SD:			///menu Karta SD
		Menu((char*)chNapisLcd[STR_MENU_KARTA_SD], stMenuKartaSD, &chNowyTrybPracy);
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
		TestKartySD();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_KARTA;
		}
		break;

	case TPKS_4:
	case TPKS_5:
	case TPKS_6:
	case TPKS_7:
	case TPKS_8:	break;

//*** IMU ************************************************
	case TP_KAL_IMU:			//menu kalibracji IMU
		Menu((char*)chNapisLcd[STR_MENU_IMU], stMenuIMU, &chNowyTrybPracy);
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
		break;

//*** Magnetometr ************************************************
	case TP_MAGNETOMETR:	//menu obsługi magnetometru
		Menu((char*)chNapisLcd[STR_MENU_MAGNETOMETR], stMenuMagnetometr, &chNowyTrybPracy);
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
		Menu((char*)chNapisLcd[STR_MENU_KALIBRACJE], stMenuKalibracje, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;


	case TP_KAL_DOTYK:
		chNowyTrybPracy = 0;			//nowy tryb jest ustawiany po starcie dla normalnej pracy ze skalibrowanym panelem. Jednak w przypadku potrzeby kalobracji,
										//powoduje to wyczyszczenie opisu i pierwszego krzyżyka i nie wiadomo o co chodzi, więc kasuję chNowyTrybPracy
		if (KalibrujDotyk() == ERR_GOTOWE)
			chTrybPracy = TP_TESTY;
		break;

//*** Pomiary ************************************************
	case TP_POMIARY:		//menu skupiające różne kalibracje
		Menu((char*)chNapisLcd[STR_MENU_POMIARY], stMenuPomiary, &chNowyTrybPracy);
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
		Menu((char*)chNapisLcd[STR_MENU_NASTAWY], stMenuNastawy, &chNowyTrybPracy);
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


	//rzeczy do zrobienia podczas uruchamiania nowego trybu pracy
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
		case TP_WROC_DO_MMEDIA:		chTrybPracy = TP_MULTIMEDIA;	break;	//powrót do menu MultiMedia
		case TP_WROC_DO_WYDAJN:		chTrybPracy = TP_WYDAJNOSC;		break;	//powrót do menu Wydajność
		case TP_WROC_DO_KARTA:		chTrybPracy = TP_KARTA_SD;		break;	//powrót do menu Karta SD
		case TP_WROC_KAL_IMU:		chTrybPracy = TP_KAL_IMU;		break;	//powrót do menu IMU
		case TP_WROC_DO_MAG:		chTrybPracy = TP_MAGNETOMETR;	break;	//powrót do menu Magnetometr
		case TP_WROC_DO_POMIARY:	chTrybPracy = TP_POMIARY;		break;	//powrót do menu Pomiary
		case TP_WROC_DO_NASTAWY:	chTrybPracy = TP_NASTAWY;		break;	//powrót do menu Nastawy
		case TP_FRAKTALE:			InitFraktal(0);		break;

		}

		LCD_clear(BLACK);
	}
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
		LCD_clear(WHITE);
		drawBitmap((DISP_X_SIZE-165)/2, 5, 165, 80, plogo165x80);

		setColor(GRAY20);
		setBackColor(WHITE);
		setFont(BigFont);
		sprintf(chNapis, "%s @%luMHz", (char*)chNapisLcd[STR_WITAJ_TYTUL], HAL_RCC_GetSysClockFreq()/1000000);
		print(chNapis, CENTER, 90);

		setColor(GRAY30);
		setFont(MidFont);
		//sprintf(chNapis, (char*)chNapisLcd[STR_WITAJ_MOTTO], ó, ć, ó, ó, ż);	//"By móc mieć w rój Wronów na pohybel wrażym hordom""
		sprintf(chNapis, (char*)chNapisLcd[STR_WITAJ_MOTTO2], ó, ó, ż, ó);	//"By móc zmóc wraże hordy rojem Wronów"	//STR_WITAJ_MOTTO2
		print(chNapis, CENTER, 115);

		sprintf(chNapis, "Adres: %d, IP: %d.%d.%d.%d, Nazwa: %s", stBSP.chAdres, stBSP.chAdrIP[0], stBSP.chAdrIP[1], stBSP.chAdrIP[2], stBSP.chAdrIP[3],  stBSP.chNazwa);
		print(chNapis, CENTER, 135);

		sprintf(chNapis, "(c) PitLab 2025 sv%d.%d.%d @ %s %s", WER_GLOWNA, WER_PODRZ, WER_REPO, build_date, build_time);
		print(chNapis, CENTER, 155);
		chRysujRaz = 0;
	}

	//pierwsza kolumna sprzętu wewnętrznego
	x = KOL12;
	y = WYKRYJ_GORA;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_FLASH_NOR]);		//"pamięć Flash NOR"
	print(chNapis, x, y);
	Wykrycie(x, y, n, (nZainicjowano & INIT_FLASH_NOR) == INIT_FLASH_NOR);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_FLASH_QSPI]);	//"pamięć Flash QSPI"
	print(chNapis, x, y);
	Wykrycie(x, y, n, (nZainicjowano & INIT_FLASH_QSPI) == INIT_FLASH_QSPI);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_KARTA_SD]);		//"Karta SD"
	print(chNapis, x, y);
	Wykrycie(x, y, n, BSP_SD_IsDetected());

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_KAMERA_OV5642]);	//"kamera "
	print(chNapis, x, y);
	Wykrycie(x, y, n, (nZainicjowano & INIT_KAMERA) == INIT_KAMERA);

	//dane z CM4
	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_GNSS]);		//GNSS
	if (uDaneCM4.dane.nZainicjowano & INIT_WYKR_UBLOX)
		n = sprintf(chNapis, "%s -> %s", (char*)chNapisLcd[STR_SPRAWDZ_GNSS], (char*)chNapisLcd[STR_SPRAWDZ_UBLOX]);	//GNSS -> uBlox
	if (uDaneCM4.dane.nZainicjowano & INIT_WYKR_MTK)
		n = sprintf(chNapis, "%s -> %s", (char*)chNapisLcd[STR_SPRAWDZ_GNSS], (char*)chNapisLcd[STR_SPRAWDZ_MTK]);		//GNSS -> MTK
	print(chNapis, x, y);
	Wykrycie(x, y, n,  (uDaneCM4.dane.nZainicjowano & INIT_GNSS_GOTOWY) == INIT_GNSS_GOTOWY);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, "%s -> %s", (char*)chNapisLcd[STR_SPRAWDZ_GNSS], (char*)chNapisLcd[STR_SPRAWDZ_HMC5883]);	//GNSS -> HMC5883
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_HMC5883) == INIT_HMC5883);

	//druga kolumna sprzętu zewnętrznego
	x = KOL22;
	y = WYKRYJ_GORA;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_MS5611]);		//moduł IMU -> MS5611
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_MS5611) == INIT_MS5611);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_BMP581]);		//moduł IMU -> BMP581
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_BMP581) == INIT_BMP581);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_ICM42688]);		//moduł IMU -> ICM42688
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_ICM42688) == INIT_ICM42688);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_LSM6DSV]);		//moduł IMU -> LSM6DSV
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV) == INIT_LSM6DSV);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_MMC34160]);		//moduł IMU -> MMC34160
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_MMC34160) == INIT_MMC34160);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_IIS2MDC]);		//moduł IMU -> IIS2MDC
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC) == INIT_IIS2MDC);

	y += WYKRYJ_WIERSZ;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_ND130]);		//moduł IMU -> ND130
	print(chNapis, x, y);
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
	{
		printChar('.', x+n*szer_fontu, y);
		//HAL_Delay(50);
	}
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
	print(chNapis, x , y);
	setColor(GRAY30);
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
		LCD_clear(BLACK);
		chRysujRaz = 0;
	}
//		setBackColor(BLACK);

	//nagłówek komunikatu
	setColor(RED);
	setFont(BigFont);
	sprintf(chNapis, (char*)chOpisBledow[KOMUNIKAT_NAGLOWEK]);	//"Blad wykonania polecenia!",
	print(chNapis, CENTER, 70);

	//stopka komunikatu
	setFont(MidFont);
	setColor(GRAY50);
	sprintf(chNapis, (char*)chOpisBledow[KOMUNIKAT_STOPKA]);	//"Wdus ekran i trzymaj aby zakonczyc"
	print(chNapis, CENTER, 250);

	//treść komunikatu
	setColor(YELLOW);
	switch(chKomunikatBledu)
	{
	case KOMUNIKAT_ZA_ZIMNO:	//sposób formatowania komunikatu taki sam jak dla BLAD_ZA_CIEPLO
	case KOMUNIKAT_ZA_CIEPLO:	//parametr1 to bieżąca temperatura, parametr 2 to nominalna temperatura kalibracji, parametr 3 to zakres tolerancji odchyłki temperatury
		sprintf(chNapis, (const char*)chOpisBledow[chKomunikatBledu], fParametr1 - KELVIN, ZNAK_STOPIEN, fParametr2 - fParametr3 - KELVIN, ZNAK_STOPIEN, fParametr2 + fParametr3 -  KELVIN, ZNAK_STOPIEN);	break;	//"Zbyt niska temeratura zyroskopow wynoszaca %d%cC. Musi miescic sie w granicach od %d%cC do %d%cC",
	}
	printRamka(chNapis, 20, 90, 440, 200);
}



////////////////////////////////////////////////////////////////////////////////
// Konwersja kolorów z HSV na RBG
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HSV2RGB(float hue, float sat, float val, float *red, float *grn, float *blu)
{
	int i;
	float f, p, q, t;
	if(val==0)
	{
		red=0;
		grn=0;
		val=0;
	}
	else
	{
		hue/=60;
		i = floor(hue);
		f = hue-i;
		p = val*(1-sat);
		q = val*(1-(sat*f));
		t = val*(1-(sat*(1-f)));
		if (i==0)
		{
			*red=val;
			*grn=t;
			*blu=p;
		}
		else if (i==1) {*red=q; 	*grn=val; 	*blu=p;}
		else if (i==2) {*red=p; 	*grn=val; 	*blu=t;}
		else if (i==3) {*red=p; 	*grn=q; 	*blu=val;}
		else if (i==4) {*red=t; 	*grn=p; 	*blu=val;}
		else if (i==5) {*red=val; 	*grn=p; 	*blu=q;}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran menu głównego
// Parametry:
// *tryb - wskaźnik na numer pozycji menu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void MenuGlowne(unsigned char *tryb)
{
	Menu((char*)chNapisLcd[STR_MENU_MAIN], stMenuGlowne, tryb);
	chWrocDoTrybu = TP_MENU_GLOWNE;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran parametryzowalnego menu
// Parametry:
// [i] *tytul - napis z nazwą menu wyświetlaną w górnym pasku
// [i] *menu - wskaźnik na strukturę menu
// [i] *napisy - wskaźnik na zmienną zawierajacą napisy
// [o] *tryb - wskaźnik na zwracany numer pozycji menu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void Menu(char *tytul, tmenu *menu, unsigned char *tryb)
{
	unsigned char chStarySelPos;
	unsigned char n, m;			//zapętlacze
	int x, x2, y;	//pomocnicze współrzędne ekranowe

	if (chRysujRaz)
	{
		BelkaTytulu(tytul);		//rysuje belkę tytułu ekranu

		//rysuje pasek podpowiedzi na dole ekranu
		//setColor(GRAY20);
		//fillRect(0, DISP_Y_SIZE - MENU_PASOP_WYS, DISP_X_SIZE, DISP_Y_SIZE);
		LCD_ProstokatWypelniony(0, DISP_Y_SIZE - MENU_PASOP_WYS, DISP_X_SIZE, MENU_PASOP_WYS, GRAY20);
		setBackColor(BLACK);

		//rysuj ikony poleceń
		setFont(MidFont);
		for (m=0; m<MENU_WIERSZE; m++)
		{
			for (n=0; n<MENU_KOLUMNY; n++)
			{
				//licz współrzedne środka ikony
				x = (DISP_X_SIZE/(2*MENU_KOLUMNY)) + n * (DISP_X_SIZE/MENU_KOLUMNY);
				y = ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS) / (2*MENU_WIERSZE)) + m * ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS) / MENU_WIERSZE) - MENU_OPIS_WYS + MENU_NAG_WYS;

				setColor(MENU_TLO_NAK);
				drawBitmap(x-MENU_ICO_WYS/2, y-MENU_ICO_DLG/2, MENU_ICO_DLG, MENU_ICO_WYS, menu[m*MENU_KOLUMNY+n].sIkona);

				setColor(GRAY60);
				x2 = FONT_SLEN * strlen(menu[m*MENU_KOLUMNY+n].chOpis);
				strcpy(chNapis, menu[m*MENU_KOLUMNY+n].chOpis);
				print(chNapis, x-x2/2, y+MENU_ICO_WYS/2+MENU_OPIS_WYS);
			}
		}
	}

	//sprawdź czy jest naciskany ekran
	if ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) || chRysujRaz)
	{
		chStarySelPos = chMenuSelPos;

		if (statusDotyku.sY < (DISP_Y_SIZE - MENU_NAG_WYS)/2)	//czy naciśniety górny rząd
			m = 0;
		else	//czy naciśniety dolny rząd
			m = 1;

		for (n=0; n<MENU_KOLUMNY; n++)
		{
			if ((statusDotyku.sX > n*(DISP_X_SIZE / MENU_KOLUMNY)) && (statusDotyku.sX < (n+1)*(DISP_X_SIZE / MENU_KOLUMNY)))
				chMenuSelPos = m * MENU_KOLUMNY + n;
		}


		if (chStarySelPos != chMenuSelPos)	//zamaż tylko gdy stara ramka jest inna od wybranej
		{
			//zamaż starą ramkę kolorem nieaktywnym
			for (m=0; m<MENU_WIERSZE; m++)
			{
				for (n=0; n<MENU_KOLUMNY; n++)
				{
					if (chStarySelPos == m*MENU_KOLUMNY+n)
					{
						//licz współrzedne środka ikony
						x = (DISP_X_SIZE /(2*MENU_KOLUMNY)) + n * (DISP_X_SIZE / MENU_KOLUMNY);
						y = ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS) / (2*MENU_WIERSZE)) + m * ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS) / MENU_WIERSZE) - MENU_OPIS_WYS + MENU_NAG_WYS;
						setColor(BLACK);
						drawRoundRect(x-MENU_ICO_DLG/2, y-MENU_ICO_WYS/2-2, x+MENU_ICO_DLG/2+2, y+MENU_ICO_WYS/2);
						setColor(GRAY60);
						x2 = FONT_SLEN * strlen(menu[m*MENU_KOLUMNY+n].chOpis);
						strcpy(chNapis, menu[m*MENU_KOLUMNY+n].chOpis);
						print(chNapis, x-x2/2, y+MENU_ICO_WYS/2+MENU_OPIS_WYS);
					}
				}
			}
			//setColor(GRAY20);
			//fillRect(0, DISP_X_SIZE-MENU_PASOP_WYS, DISP_Y_SIZE, DISP_X_SIZE);		//zamaż pasek podpowiedzi
			LCD_ProstokatWypelniony(0, DISP_Y_SIZE-MENU_PASOP_WYS, DISP_X_SIZE, MENU_PASOP_WYS, GRAY20);		//zamaż pasek podpowiedzi

		}

		//rysuj nową zaznaczoną ramkę
		for (m=0; m<MENU_WIERSZE; m++)
		{
			for (n=0; n<MENU_KOLUMNY; n++)
			{
				if (chMenuSelPos == m*MENU_KOLUMNY+n)
				{
					//licz współrzedne środka ikony
					x = (DISP_X_SIZE/(2*MENU_KOLUMNY)) + n * (DISP_X_SIZE/MENU_KOLUMNY);
					y = ((DISP_Y_SIZE-MENU_NAG_WYS-MENU_PASOP_WYS)/(2*MENU_WIERSZE)) + m * ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS)/MENU_WIERSZE) - MENU_OPIS_WYS + MENU_NAG_WYS;
					if  (statusDotyku.chFlagi == DOTYK_DOTKNIETO)	//czy naciśnięty ekran
						setColor(MENU_RAM_WYB);
					else
						setColor(MENU_RAM_AKT);
					drawRoundRect(x-MENU_ICO_DLG/2, y-MENU_ICO_WYS/2-2, x+MENU_ICO_DLG/2+2, y+MENU_ICO_WYS/2);
					setColor(GRAY80);
					x2 = FONT_SLEN * strlen(menu[m*MENU_KOLUMNY+n].chOpis);
					strcpy(chNapis, menu[m*MENU_KOLUMNY+n].chOpis);
					print(chNapis, x-x2/2, y+MENU_ICO_WYS/2+MENU_OPIS_WYS);
				}
			}
		}
		//rysuj pasek podpowiedzi
		if ((chStarySelPos != chMenuSelPos) || chRysujRaz)
		{
			setColor(MENU_RAM_AKT);
			setBackColor(GRAY20);
			strcpy(chNapis, menu[chMenuSelPos].chPomoc);
			print(chNapis, DW_SPACE, DISP_Y_SIZE - DW_SPACE - FONT_SH);
			setBackColor(BLACK);
			chRysujRaz = 0;
		}
	}

	//czy był naciśniety enkoder lub ekran
	if (statusDotyku.chFlagi & DOTYK_DOTKNIETO)
	{
		*tryb = menu[chMenuSelPos].chMode;
		statusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;	//kasuj flagę naciśnięcia ekranu
		DodajProbkeDoMalejKolejki(PRGA_PRZYCISK, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//odtwórz komunikat audio przycisku
		return;
	}
	*tryb = 0;

	//rysuj czas
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	if (sTime.Seconds != chOstatniCzas)
	{
		setColor(GRAY50);
		setColor(MENU_RAM_AKT);
		sprintf(chNapis, "%02d:%02d:%02d", sTime.Hours,  sTime.Minutes,  sTime.Seconds);
		print(chNapis, DISP_X_SIZE - 9*FONT_SL, DISP_Y_SIZE - DW_SPACE - FONT_SH);
		chOstatniCzas = sTime.Seconds;
	}
}


////////////////////////////////////////////////////////////////////////////////
// Rysuje belkę menu z logo i tytułem w poziomej orientacji ekranu
// Wychodzi z ustawionym czarnym tłem, białym kolorem i średnia czcionką
// Parametry: wskaźnik na zmienną z tytułem okna
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void BelkaTytulu(char* chTytul)
{
	//setColor(MENU_TLO_BAR);
	//fillRect(18, 0, DISP_X_SIZE, MENU_NAG_WYS);
	LCD_ProstokatWypelniony(18, 0, DISP_X_SIZE, MENU_NAG_WYS, MENU_TLO_BAR);
	drawBitmap(0, 0, 18, 18, pitlab_logo18);	//logo PitLab
	setColor(YELLOW);
	setBackColor(MENU_TLO_BAR);
	setFont(BigFont);
	print(chTytul, CENTER, UP_SPACE);
	setBackColor(BLACK);
	setColor(WHITE);
	setFont(MidFont);
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

		setColor(GRAY80);
		sprintf(chNapis, "Akcel1:");
		print(chNapis, KOL12, 30);
		sprintf(chNapis, "[m/s^2]");
		print(chNapis, KOL12+40*FONT_SL, 30);

		sprintf(chNapis, "Akcel2:");
		print(chNapis, KOL12, 50);
		sprintf(chNapis, "[m/s^2]");
		print(chNapis, KOL12+40*FONT_SL, 50);

		sprintf(chNapis, "Zyro 1:");
		print(chNapis, KOL12, 70);
		sprintf(chNapis, "[rad/s]");
		print(chNapis, KOL12+40*FONT_SL, 70);

		sprintf(chNapis, "Zyro 2:");
		print(chNapis, KOL12, 90);
		sprintf(chNapis, "[rad/s]");
		print(chNapis, KOL12+40*FONT_SL, 90);

		sprintf(chNapis, "Magn 1:");
		print(chNapis, KOL12, 110);
		sprintf(chNapis, "[uT]");
		print(chNapis, KOL12+40*FONT_SL, 110);

		sprintf(chNapis, "Magn 2:");
		print(chNapis, KOL12, 130);
		sprintf(chNapis, "[uT]");
		print(chNapis, KOL12+40*FONT_SL, 130);

		sprintf(chNapis, "Magn 3:");
		print(chNapis, KOL12, 150);
		sprintf(chNapis, "[uT]");
		print(chNapis, KOL12+40*FONT_SL, 150);

		sprintf(chNapis, "K%cty 1:", ą);
		print(chNapis, KOL12, 170);
		sprintf(chNapis, "K%cty 2:", ą);
		print(chNapis, KOL12, 190);
		sprintf(chNapis, "K%cty Akcel1:", ą);
		print(chNapis, KOL12, 210);
		sprintf(chNapis, "K%cty Akcel2:", ą);
		print(chNapis, KOL12, 230);
		//sprintf(chNapis, "K%cty %cyro 1:", ą, ż);
		sprintf(chNapis, "Kwaternion Akc:");
		print(chNapis, KOL12, 250);
		//sprintf(chNapis, "K%cty %cyro 2:", ą, ż);
		sprintf(chNapis, "Kwaternion Mag:");
		print(chNapis, KOL12, 270);

		setColor(GRAY50);
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		print(chNapis, CENTER, 300);
	}

	//ICM42688
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel1[0]);
	print(chNapis, KOL12+8*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel1[1]);
	print(chNapis, KOL12+20*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel1[2]);
	print(chNapis, KOL12+32*FONT_SL, 30);

	//LSM6DSV
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel2[0]);
	print(chNapis, KOL12+8*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel2[1]);
	print(chNapis, KOL12+20*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel2[2]);
	print(chNapis, KOL12+32*FONT_SL, 50);

	//ICM42688
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal1[0]);
	print(chNapis, KOL12+8*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal1[1]);
	print(chNapis, KOL12+20*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal1[2]);
	print(chNapis, KOL12+32*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_IMU1] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	print(chNapis, KOL12+49*FONT_SL, 70);

	//LSM6DSV
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal2[0]);
	print(chNapis, KOL12+8*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal2[1]);
	print(chNapis, KOL12+20*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal2[2]);
	print(chNapis, KOL12+32*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_IMU2] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	print(chNapis, KOL12+49*FONT_SL, 90);

	//IIS2MDC
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne1[0]*1e6);
	print(chNapis, KOL12+8*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne1[1]*1e6);
	print(chNapis, KOL12+20*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne1[2]*1e6);
	print(chNapis, KOL12+32*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(POMARANCZ); 	else	setColor(GRAY50);
	fDlugosc = sqrtf(uDaneCM4.dane.fMagne1[0] * uDaneCM4.dane.fMagne1[0] + uDaneCM4.dane.fMagne1[1] * uDaneCM4.dane.fMagne1[1] + uDaneCM4.dane.fMagne1[2] * uDaneCM4.dane.fMagne1[2]);
	sprintf(chNapis, "%.1f %% ", fDlugosc / NOMINALNE_MAGN * 100);	//długość wektora [%]
	print(chNapis, KOL12+49*FONT_SL, 110);

	//MMC34160
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne2[0]*1e6);
	print(chNapis, KOL12+8*FONT_SL, 130);
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne2[1]*1e6);
	print(chNapis, KOL12+20*FONT_SL, 130);
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne2[2]*1e6);
	print(chNapis, KOL12+32*FONT_SL, 130);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(POMARANCZ); 	else	setColor(GRAY50);
	fDlugosc = sqrtf(uDaneCM4.dane.fMagne2[0] * uDaneCM4.dane.fMagne2[0] + uDaneCM4.dane.fMagne2[1] * uDaneCM4.dane.fMagne2[1] + uDaneCM4.dane.fMagne2[2] * uDaneCM4.dane.fMagne2[2]);
	sprintf(chNapis, "%.1f %% ", fDlugosc / NOMINALNE_MAGN * 100);	//długość wektora [%]
	print(chNapis, KOL12+49*FONT_SL, 130);

	//HMC5883
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne3[0]*1e6);
	print(chNapis, KOL12+8*FONT_SL, 150);
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne3[1]*1e6);
	print(chNapis, KOL12+20*FONT_SL, 150);
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne3[2]*1e6);
	print(chNapis, KOL12+32*FONT_SL, 150);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(POMARANCZ); 	else	setColor(GRAY50);
	fDlugosc = sqrtf(uDaneCM4.dane.fMagne3[0] * uDaneCM4.dane.fMagne3[0] + uDaneCM4.dane.fMagne3[1] * uDaneCM4.dane.fMagne3[1] + uDaneCM4.dane.fMagne3[2] * uDaneCM4.dane.fMagne3[2]);
	sprintf(chNapis, "%.1f %% ", fDlugosc / NOMINALNE_MAGN * 100);	//długość wektora [%]
	print(chNapis, KOL12+49*FONT_SL, 150);

	//sygnalizacja tonem wartości osi Z magnetometru
	chTon = LICZBA_TONOW_WARIO/2 - (uDaneCM4.dane.fMagne3[2] / (NOMINALNE_MAGN / (LICZBA_TONOW_WARIO/2)));
	if (chTon > LICZBA_TONOW_WARIO)
		chTon = LICZBA_TONOW_WARIO;
	if (chTon < 0)
		chTon = 0;
	//UstawTon(chTon, 80);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU1[0], ZNAK_STOPIEN);
	print(chNapis, KOL12+8*FONT_SL, 170);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU1[1], ZNAK_STOPIEN);
	print(chNapis, KOL12+20*FONT_SL, 170);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU1[2], ZNAK_STOPIEN);
	print(chNapis, KOL12+32*FONT_SL, 170);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[0], ZNAK_STOPIEN);
	print(chNapis, KOL12+8*FONT_SL, 190);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[1], ZNAK_STOPIEN);
	print(chNapis, KOL12+20*FONT_SL, 190);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[2], ZNAK_STOPIEN);
	print(chNapis, KOL12+32*FONT_SL, 190);

	//kąty z akcelrometru 1
	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel1[0], ZNAK_STOPIEN);
	print(chNapis, KOL12+13*FONT_SL, 210);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel1[1], ZNAK_STOPIEN);
	print(chNapis, KOL12+25*FONT_SL, 210);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel1[2], ZNAK_STOPIEN);
	print(chNapis, KOL12+37*FONT_SL, 210);

	//kąty z akcelrometru 2
	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel2[0], ZNAK_STOPIEN);
	print(chNapis, KOL12+13*FONT_SL, 230);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel2[1], ZNAK_STOPIEN);
	print(chNapis, KOL12+25*FONT_SL, 230);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatAkcel2[2], ZNAK_STOPIEN);
	print(chNapis, KOL12+37*FONT_SL, 230);

	/*/kąty z żyroskopu 1
	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[0], ZNAK_STOPIEN);
	print(chNapis, KOL12+13*FONT_SL, 250);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[1], ZNAK_STOPIEN);
	print(chNapis, KOL12+25*FONT_SL, 250);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[2], ZNAK_STOPIEN);
	print(chNapis, KOL12+37*FONT_SL, 250);

	//kąty z żyroskopu 2
	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[0], ZNAK_STOPIEN);
	print(chNapis, KOL12+13*FONT_SL, 270);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[1], ZNAK_STOPIEN);
	print(chNapis, KOL12+25*FONT_SL, 270);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[2], ZNAK_STOPIEN);
	print(chNapis, KOL12+37*FONT_SL, 270); */

	//kwaternion wektora przyspieszenia
	setColor(POMARANCZ);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[0]);
	print(chNapis, KOL12+16*FONT_SL, 250);
	setColor(KOLOR_X);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[1]);
	print(chNapis, KOL12+26*FONT_SL, 250);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[2]);
	print(chNapis, KOL12+36*FONT_SL, 250);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaAkc[3]);
	print(chNapis, KOL12+46*FONT_SL, 250);

	//kwaternion wektora magnetycznego
	setColor(POMARANCZ);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaMag[0]);
	print(chNapis, KOL12+16*FONT_SL, 270);
	setColor(KOLOR_X);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaMag[1]);
	print(chNapis, KOL12+26*FONT_SL, 270);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaMag[2]);
	print(chNapis, KOL12+36*FONT_SL, 270);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.4f ", uDaneCM4.dane.fKwaMag[3]);
	print(chNapis, KOL12+46*FONT_SL, 270);

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

		setColor(GRAY80);
		sprintf(chNapis, "Ci%cn 1:             AGL1:", ś);
		print(chNapis, KOL12, 30);
		sprintf(chNapis, "Ci%cn 2:             AGL2:", ś);
		print(chNapis, KOL12, 50);
		sprintf(chNapis, "Ci%cR%c%cn 1:          IAS1:", ś, ó, ż);
		print(chNapis, KOL12, 70);
		sprintf(chNapis, "Ci%cR%c%cn 2:          IAS2:", ś, ó, ż);
		print(chNapis, KOL12, 90);

		sprintf(chNapis, "GNSS D%cug:             Szer:             HDOP:", ł);
		print(chNapis, KOL12, 140);
		sprintf(chNapis, "GNSS WysMSL:           Pred:             Kurs:");
		print(chNapis, KOL12, 160);
		sprintf(chNapis, "GNSS Czas:             Data:              Sat:");
		print(chNapis, KOL12, 180);

		setColor(GRAY50);
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		print(chNapis, CENTER, 300);
	}

	//MS5611
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(WHITE); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnieBzw[0]);
	print(chNapis, KOL12+8*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(CYAN); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f m ", uDaneCM4.dane.fWysokoMSL[0]);
	print(chNapis, KOL12+26*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_BARO1] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	print(chNapis, KOL12+40*FONT_SL, 30);

	//BMP581
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(WHITE); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnieBzw[1]);
	print(chNapis, KOL12+8*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(CYAN); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f m ", uDaneCM4.dane.fWysokoMSL[1]);
	print(chNapis, KOL12+26*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_BARO2] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	print(chNapis, KOL12+40*FONT_SL, 50);

	//ND130
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(WHITE); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnRozn[0]);
	print(chNapis, KOL12+11*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(MAGENTA); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f m/s ", uDaneCM4.dane.fPredkosc[0]);
	print(chNapis, KOL12+26*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_CISR1] - KELVIN, ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	print(chNapis, KOL12+40*FONT_SL, 70);

	//MS4525
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(WHITE); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnRozn[1]);
	print(chNapis, KOL12+11*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(MAGENTA); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f m/s ", uDaneCM4.dane.fPredkosc[1]);
	print(chNapis, KOL12+26*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_CISR2] - KELVIN , ZNAK_STOPIEN);	//temperatury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
	print(chNapis, KOL12+40*FONT_SL, 90);


	if (uDaneCM4.dane.stGnss1.chFix)
		setColor(WHITE);	//jest fix
	else
		setColor(GRAY70);	//nie ma fixa

	sprintf(chNapis, "%.7f ", uDaneCM4.dane.stGnss1.dDlugoscGeo);
	print(chNapis, KOL12+11*FONT_SL, 140);
	sprintf(chNapis, "%.7f ", uDaneCM4.dane.stGnss1.dSzerokoscGeo);
	print(chNapis, KOL12+29*FONT_SL, 140);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.stGnss1.fHdop);
	print(chNapis, KOL12+47*FONT_SL, 140);

	sprintf(chNapis, "%.1fm ", uDaneCM4.dane.stGnss1.fWysokoscMSL);
	print(chNapis, KOL12+13*FONT_SL, 160);
	sprintf(chNapis, "%.3fm/s ", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi);
	print(chNapis, KOL12+29*FONT_SL, 160);
	sprintf(chNapis, "%3.2f%c ", uDaneCM4.dane.stGnss1.fKurs, ZNAK_STOPIEN);
	print(chNapis, KOL12+47*FONT_SL, 160);

	sprintf(chNapis, "%02d:%02d:%02d ", uDaneCM4.dane.stGnss1.chGodz, uDaneCM4.dane.stGnss1.chMin, uDaneCM4.dane.stGnss1.chSek);
	print(chNapis, KOL12+12*FONT_SL, 180);
	if  (uDaneCM4.dane.stGnss1.chMies > 12)	//ograniczenie aby nie pobierało nazwy miesiaca spoza tablicy chNazwyMies3Lit[]
		uDaneCM4.dane.stGnss1.chMies = 0;	//zerowy indeks jest pustą nazwą "---"
	sprintf(chNapis, "%02d %s %04d ", uDaneCM4.dane.stGnss1.chDzien, chNazwyMies3Lit[uDaneCM4.dane.stGnss1.chMies], uDaneCM4.dane.stGnss1.chRok + 2000);
	print(chNapis, KOL12+29*FONT_SL, 180);
	sprintf(chNapis, "%d ", uDaneCM4.dane.stGnss1.chLiczbaSatelit);
	print(chNapis, KOL12+47*FONT_SL, 180);

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
			setColor(GRAY80);
			sprintf(chNapis, "Kan %2d:", n+1);
			print(chNapis, KOL12, y+26);
			setColor(GRAY40);
			drawRect(119, y+29, (121 + (PPM_MAX - PPM_MIN) / ROZDZIECZOSC_PASKA_RC), y+29+8);
		}
		setColor(GRAY50);
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		print(chNapis, CENTER, 300);
	}

	for (n=0; n<KANALY_ODB_RC; n++)
	{
		y = n * 17;
		setColor(GRAY80);
		sprintf(chNapis, "%4d ", uDaneCM4.dane.sKanalRC[n]);
		print(chNapis, KOL12+8*FONT_SL, y+26);

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
			LCD_ProstokatWypelniony(120, y+30, sDlugoscPaska, 6, BLUE);
		sDlugoscTla = ((PPM_MAX - PPM_MIN) / ROZDZIECZOSC_PASKA_RC) - sDlugoscPaska;
		if (sDlugoscTla)
			LCD_ProstokatWypelniony(121 + sDlugoscPaska, y+30, sDlugoscTla, 6, BLACK);
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
		LCD_ProstokatWypelniony(0, DISP_Y_SIZE - WYS_PASKA_POSTEPU, x, WYS_PASKA_POSTEPU, BLUE);
	}
	//setColor(BLACK);
	//fillRect(x, DISP_Y_SIZE - WYS_PASKA_POSTEPU, DISP_X_SIZE , DISP_Y_SIZE);
	LCD_ProstokatWypelniony(x, DISP_Y_SIZE - WYS_PASKA_POSTEPU, DISP_X_SIZE, WYS_PASKA_POSTEPU, BLUE);
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

		setColor(GRAY80);
		sprintf(chNapis, "Numer tonu:");
		print(chNapis, KOL12, 30);
		sprintf(chNapis, "Czest. 1 harm.:");
		print(chNapis, KOL12, 50);
		sprintf(chNapis, "Czest. 3 harm.:");
		print(chNapis, KOL12, 70);
	}

	sLicznikTonu++;
	if (sLicznikTonu > 900)
	{
		sLicznikTonu = 0;
		chNumerTonu++;
		if (chNumerTonu >= LICZBA_TONOW_WARIO)
			chNumerTonu = 0;

		setColor(WHITE);
		sprintf(chNapis, "%d ", chNumerTonu);
		print(chNapis, KOL12+16*FONT_SL, 30);
		sprintf(chNapis, "%.1f Hz ", 1.0f * CZESTOTLIWOSC_AUDIO / (MIN_OKRES_TONU + chNumerTonu * SKOK_TONU));
		print(chNapis, KOL12+16*FONT_SL, 50);
		sprintf(chNapis, "%.1f Hz ", 3.0f * CZESTOTLIWOSC_AUDIO / (MIN_OKRES_TONU + chNumerTonu * SKOK_TONU));
		print(chNapis, KOL12+16*FONT_SL, 70);
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
		setFont(BigFont);
		setColor(BLACK);
		sprintf(chNapis, "                             ");
		print(chNapis, CENTER, 50);
		setFont(MidFont);
	}

	if (BSP_SD_IsDetected())
	{
		BSP_SD_GetCardInfo(&CardInfo);
		HAL_SD_GetCardCID(&hsd1, &pCID);
		HAL_SD_GetCardCSD(&hsd1, &pCSD);
		HAL_SD_GetCardStatus(&hsd1, &pStatus);
		sPozY = 30;

		setColor(GRAY70);
		sprintf(chNapis, "Typ: %ld ", CardInfo.CardType);
		print(chNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Pojemnosc: ");
		print(chNapis, KOL12, sPozY);
		switch(pCSD.CSDStruct)
		{
		case 0:	sprintf(chNapis, "Standard");	break;
		case 1:	sprintf(chNapis, "Wysoka  ");	break;
		default:sprintf(chNapis, "Nieznana");	break;
		}
		print(chNapis, KOL12 + 11*FONT_SL, sPozY);

		sPozY += 20;
		//sprintf(chNapis, "Klasy: %ld (0x%lX) ", CardInfo.Class, CardInfo.Class);
		sprintf(chNapis, "CCC: ");
		print(chNapis, KOL12, sPozY);
		for (uint16_t n=0; n<12; n++)
		{
			if (CardInfo.Class & (1<<n))
				setColor(GRAY80);
			else
				setColor(GRAY40);
			sprintf(chNapis, "%X", n);
			print(chNapis, KOL12 + ((5+n)*FONT_SL), sPozY);
		}
		sPozY += 20;

		setColor(GRAY70);
		sprintf(chNapis, "Klasa predk: %d ", pStatus.SpeedClass);
		print(chNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Klasa UHS: %d ", pStatus.UhsSpeedGrade);
		print(chNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Klasa Video: %d ", pStatus.VideoSpeedClass);
		print(chNapis, KOL12, sPozY);
		sPozY += 20;
		sprintf(chNapis, "PerformanceMove: %d MB/s", pStatus.PerformanceMove);
		print(chNapis, KOL12, sPozY);
		sPozY += 20;

		sprintf(chNapis, "Wsp pred. O/Z: %d ", pCSD.WrSpeedFact);
		print(chNapis, KOL12, sPozY);
		sPozY += 20;

		if (chPort_exp_wysylany[0] & EXP02_LOG_VSELECT)	//LOG_SD1_VSEL: H=3,3V
			fNapiecie = 3.3;
		else
			fNapiecie = 1.8;
		sprintf(chNapis, "Napi%ccie I/O: %.1fV ", ę, fNapiecie);
		print(chNapis, KOL12, sPozY);
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
		print(chNapis, KOL12, sPozY);
		sPozY += 20;

		sprintf(chNapis, "Czas dost. sync: %d cyk.zeg ", pCSD.NSAC);
		print(chNapis, KOL12, sPozY);
		sPozY += 20;

		sprintf(chNapis, "MaxBusClkFrec: %d", pCSD.MaxBusClkFrec);
		print(chNapis, KOL12, sPozY);
		sPozY += 20;


		//druga kolumna
		sPozY = 30;
		sprintf(chNapis, "Liczba sektor%cw: %ld ", ó, CardInfo.BlockNbr);		//Specifies the Card Capacity in blocks
		print(chNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Rozmiar sektora: %ld B ", CardInfo.BlockSize);		//Specifies one block size in bytes
		print(chNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Pojemno%c%c karty: %ld MB ", ś, ć, CardInfo.BlockNbr * (CardInfo.BlockSize / 512) / 2048);		//Specifies one block size in bytes
		print(chNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Rozm Jedn Alok: %d kB", (8<<pStatus.AllocationUnitSize));		//Specifies one block size in bytes
		print(chNapis, KOL22, sPozY);
		sPozY += 20;
		//sprintf(chNapis, "Liczba blok%cw log.: %ld ", ó, CardInfo.LogBlockNbr);		//Specifies the Card logical Capacity in blocks
		//print(chNapis, KOL22, sPozY);
		//sPozY += 20;
		sprintf(chNapis, "RdBlockLen: %d ", (2<<pCSD.RdBlockLen));
		print(chNapis, KOL22, sPozY);
		sPozY += 20;
		//sprintf(chNapis, "Rozmiar karty log: %ld MB ", CardInfo.LogBlockNbr * (CardInfo.LogBlockSize / 512) / 2048);		//Specifies one block size in bytes
		//print(chNapis, KOL22, sPozY);
		//sPozY += 20;

		setColor(GRAY70);
		sprintf(chNapis, "Format: ");
		print(chNapis, KOL22, sPozY);
		switch (pCSD.FileFormat)
		{
		case 0: sprintf(chNapis, "HDD z partycja ");	break;
		case 1: sprintf(chNapis, "DOS FAT ");			break;
		case 2: sprintf(chNapis, "UnivFileFormat ");	break;
		default: sprintf(chNapis, "Nieznany ");			break;
		}
		setColor(GRAY80);
		print(chNapis, KOL22+8*FONT_SL, sPozY);
		sPozY += 20;

		setColor(GRAY70);
		sprintf(chNapis, "Manuf ID: ");
		print(chNapis, KOL22, sPozY);
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
		setColor(GRAY80);
		print(chNapis, KOL22+10*FONT_SL, sPozY);
		sPozY += 20;

		setColor(GRAY70);
		chOEM[0] = (pCID.OEM_AppliID & 0xFF00)>>8;
		chOEM[1] = pCID.OEM_AppliID & 0x00FF;
		sprintf(chNapis, "OEM_AppliID: %c%c ", chOEM[0], chOEM[1]);
		print(chNapis, KOL22, sPozY);
		sPozY += 20;

		sprintf(chNapis, "Nr seryjny: %ld ", pCID.ProdSN);
		print(chNapis, KOL22, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Data prod: %s %d ", chNazwyMies3Lit[(pCID.ManufactDate & 0xF)], ((pCID.ManufactDate>>4) & 0xFF) + 2000);
		print(chNapis, KOL22, sPozY);
		sPozY += 20;

	}
	else
	{
		setFont(BigFont);
		setColor(RED);
		sprintf(chNapis, "Wolne %carty, tu brak karty! ", ż);
		print(chNapis, CENTER, 50);
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
	setColor(GRAY80);

	sprintf(chNapis, "Karta SD: ");
	print(chNapis, KOL12, sPozY);
	setColor(YELLOW);
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
	print(chNapis, KOL12 + 11*FONT_SL, sPozY);
	sPozY += 20;


	setColor(GRAY80);
	sprintf(chNapis, "Stan FATu: ");
	print(chNapis, KOL12, sPozY);

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
	print(chNapis, KOL12 + 11*FONT_SL, sPozY);
	sPozY += 20;

	setColor(GRAY80);
	sprintf(chNapis, "Plik logu: ");
	print(chNapis, KOL12, sPozY);

	if (chStatusRejestratora & STATREJ_OTWARTY_PLIK)
	{
		setColor(KOLOR_Y);
		sprintf(chNapis, "Otwarty   ");
	}
	else
	{
		setColor(YELLOW);
		if (chStatusRejestratora & STATREJ_BYL_OTWARTY)
			sprintf(chNapis, "Zatrzymany");
		else
			sprintf(chNapis, "Brak ");
	}
	print(chNapis, KOL12 + 11*FONT_SL, sPozY);
	sPozY += 20;

	setColor(GRAY80);
	sprintf(chNapis, "Rejestrator: ");
	print(chNapis, KOL12, sPozY);
	setColor(YELLOW);
	if (chStatusRejestratora & STATREJ_WLACZONY)
	{
		setColor(KOLOR_Y);
		sprintf(chNapis, "W%c%cczony  ", ł, ą);
	}
	else
	{
		setColor(YELLOW);
		sprintf(chNapis, "Zatrzymany");
	}
	print(chNapis, KOL12 + 13*FONT_SL, sPozY);
	sPozY += 20;

	setColor(GRAY80);
	sprintf(chNapis, "Zapelnienie: ");
	print(chNapis, KOL12, sPozY);
	float fZapelnienie = (float)sMaxDlugoscWierszaLogu / ROZMIAR_BUFORA_LOGU;
	if (fZapelnienie < 0.75)
		setColor(KOLOR_Y);	//zielony
	else
	if (fZapelnienie < 0.95)
		setColor(YELLOW);
	else
		setColor(KOLOR_X);	//czerwony
	sprintf(chNapis, "%d/%d ", sMaxDlugoscWierszaLogu, ROZMIAR_BUFORA_LOGU);
	print(chNapis, KOL12 + 13*FONT_SL, sPozY);
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
	setColor(BLACK);
	drawLine(sKostkaPoprzednia[0][0] + DISP_X_SIZE/2, sKostkaPoprzednia[0][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[1][0] + DISP_X_SIZE/2, sKostkaPoprzednia[1][1] + DISP_Y_SIZE/2);
	drawLine(sKostkaPoprzednia[1][0] + DISP_X_SIZE/2, sKostkaPoprzednia[1][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[2][0] + DISP_X_SIZE/2, sKostkaPoprzednia[2][1] + DISP_Y_SIZE/2);
	drawLine(sKostkaPoprzednia[2][0] + DISP_X_SIZE/2, sKostkaPoprzednia[2][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[3][0] + DISP_X_SIZE/2, sKostkaPoprzednia[3][1] + DISP_Y_SIZE/2);
	drawLine(sKostkaPoprzednia[3][0] + DISP_X_SIZE/2, sKostkaPoprzednia[3][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[0][0] + DISP_X_SIZE/2, sKostkaPoprzednia[0][1] + DISP_Y_SIZE/2);

	//rysuj obrys kostki z dołu
	drawLine(sKostkaPoprzednia[4][0] + DISP_X_SIZE/2, sKostkaPoprzednia[4][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[5][0] + DISP_X_SIZE/2, sKostkaPoprzednia[5][1] + DISP_Y_SIZE/2);
	drawLine(sKostkaPoprzednia[5][0] + DISP_X_SIZE/2, sKostkaPoprzednia[5][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[6][0] + DISP_X_SIZE/2, sKostkaPoprzednia[6][1] + DISP_Y_SIZE/2);
	drawLine(sKostkaPoprzednia[6][0] + DISP_X_SIZE/2, sKostkaPoprzednia[6][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[7][0] + DISP_X_SIZE/2, sKostkaPoprzednia[7][1] + DISP_Y_SIZE/2);
	drawLine(sKostkaPoprzednia[7][0] + DISP_X_SIZE/2, sKostkaPoprzednia[7][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[4][0] + DISP_X_SIZE/2, sKostkaPoprzednia[4][1] + DISP_Y_SIZE/2);

	//rysuj linie pionowych ścianek
	drawLine(sKostkaPoprzednia[0][0] + DISP_X_SIZE/2, sKostkaPoprzednia[0][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[4][0] + DISP_X_SIZE/2, sKostkaPoprzednia[4][1] + DISP_Y_SIZE/2);
	drawLine(sKostkaPoprzednia[1][0] + DISP_X_SIZE/2, sKostkaPoprzednia[1][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[5][0] + DISP_X_SIZE/2, sKostkaPoprzednia[5][1] + DISP_Y_SIZE/2);
	drawLine(sKostkaPoprzednia[2][0] + DISP_X_SIZE/2, sKostkaPoprzednia[2][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[6][0] + DISP_X_SIZE/2, sKostkaPoprzednia[6][1] + DISP_Y_SIZE/2);
	drawLine(sKostkaPoprzednia[3][0] + DISP_X_SIZE/2, sKostkaPoprzednia[3][1] + DISP_Y_SIZE/2, sKostkaPoprzednia[7][0] + DISP_X_SIZE/2, sKostkaPoprzednia[7][1] + DISP_Y_SIZE/2);


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
	setColor(RED);
	//if (nIloczynSkalarny[0] <= 0)	//jeżeli odległość górnej płaszczyzny od ekrany jest większa niż dolnej płaszcyzny
	{
		drawLine(sKostka[0][0] + DISP_X_SIZE/2, sKostka[0][1] + DISP_Y_SIZE/2, sKostka[1][0] + DISP_X_SIZE/2, sKostka[1][1] + DISP_Y_SIZE/2);
		drawLine(sKostka[1][0] + DISP_X_SIZE/2, sKostka[1][1] + DISP_Y_SIZE/2, sKostka[2][0] + DISP_X_SIZE/2, sKostka[2][1] + DISP_Y_SIZE/2);
		drawLine(sKostka[2][0] + DISP_X_SIZE/2, sKostka[2][1] + DISP_Y_SIZE/2, sKostka[3][0] + DISP_X_SIZE/2, sKostka[3][1] + DISP_Y_SIZE/2);
		drawLine(sKostka[3][0] + DISP_X_SIZE/2, sKostka[3][1] + DISP_Y_SIZE/2, sKostka[0][0] + DISP_X_SIZE/2, sKostka[0][1] + DISP_Y_SIZE/2);
	}

	//rysuj obrys kostki z dołu
	setColor(GREEN);
	//if (nIloczynSkalarny[1] <= 0)	//jeżeli odległość dolnej płaszczyzny od ekranu jest większa niż górnej płaszczyzny
	{
		drawLine(sKostka[4][0] + DISP_X_SIZE/2, sKostka[4][1] + DISP_Y_SIZE/2, sKostka[5][0] + DISP_X_SIZE/2, sKostka[5][1] + DISP_Y_SIZE/2);
		drawLine(sKostka[5][0] + DISP_X_SIZE/2, sKostka[5][1] + DISP_Y_SIZE/2, sKostka[6][0] + DISP_X_SIZE/2, sKostka[6][1] + DISP_Y_SIZE/2);
		drawLine(sKostka[6][0] + DISP_X_SIZE/2, sKostka[6][1] + DISP_Y_SIZE/2, sKostka[7][0] + DISP_X_SIZE/2, sKostka[7][1] + DISP_Y_SIZE/2);
		drawLine(sKostka[7][0] + DISP_X_SIZE/2, sKostka[7][1] + DISP_Y_SIZE/2, sKostka[4][0] + DISP_X_SIZE/2, sKostka[4][1] + DISP_Y_SIZE/2);
	}

	//rysuj linie pionowych ścianek
	setColor(YELLOW);
	drawLine(sKostka[0][0] + DISP_X_SIZE/2, sKostka[0][1] + DISP_Y_SIZE/2, sKostka[4][0] + DISP_X_SIZE/2, sKostka[4][1] + DISP_Y_SIZE/2);
	drawLine(sKostka[1][0] + DISP_X_SIZE/2, sKostka[1][1] + DISP_Y_SIZE/2, sKostka[5][0] + DISP_X_SIZE/2, sKostka[5][1] + DISP_Y_SIZE/2);
	drawLine(sKostka[2][0] + DISP_X_SIZE/2, sKostka[2][1] + DISP_Y_SIZE/2, sKostka[6][0] + DISP_X_SIZE/2, sKostka[6][1] + DISP_Y_SIZE/2);
	drawLine(sKostka[3][0] + DISP_X_SIZE/2, sKostka[3][1] + DISP_Y_SIZE/2, sKostka[7][0] + DISP_X_SIZE/2, sKostka[7][1] + DISP_Y_SIZE/2);


	//zapamietaj współrzędne kostki aby w nastepnym cyklu ją wymazać
	for (uint16_t n=0; n<8; n++)
	{
		for (uint16_t m=0; m<2; m++)
			sKostkaPoprzednia[n][m] = sKostka[n][m];
	}
	setColor(GRAY50);
	sprintf(chNapis, "t=%ldus, phi=%.1f, theta=%.1f, psi=%.1f ", nCzas, fKat[0]*RAD2DEG, fKat[1]*RAD2DEG, fKat[2]*RAD2DEG);
	print(chNapis, 0, 0);
	return nCzas;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran dla kalibracji wzmocnienia żyroskopów
// Parametry: *chSekwencer	- zmiennaWskazująca na kalibracje konktretnej osi
// Zwraca: ERR_GOTOWE / ERR_OK - informację o tym czy wyjść z trybu kalibracji czy nie
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibracjaWzmocnieniaZyroskopow(uint8_t *chSekwencer)
{
	char chNazwaOsi;
	float fPochylenie, fPrzechylenie;
	uint8_t chErr = ERR_OK;
	//uint8_t chRozmiar;

	if (chRysujRaz)
	{
		//chRysujRaz = 0;	będzie wyzerowane w funkcji rysowania poziomicy
		BelkaTytulu("Kalibr. wzmocnienia zyro");
		setColor(GRAY80);

		sprintf(chNapis, "Ca%cka %cyro 1:", ł, ż);
		print(chNapis, KOL12 + LIBELLA_BOK, 100);
		sprintf(chNapis, "Ca%cka %cyro 2:", ł, ż);
		print(chNapis, KOL12 + LIBELLA_BOK, 120);
		sprintf(chNapis, "K%cty w p%caszczy%cnie kalibracji", ą, ł, ź);
		print(chNapis, KOL12 + LIBELLA_BOK, 140);
		sprintf(chNapis, "Pochylenie:");
		print(chNapis, KOL12 + LIBELLA_BOK, 160);
		sprintf(chNapis, "Przechylenie:");
		print(chNapis, KOL12 + LIBELLA_BOK, 180);
		sprintf(chNapis, "WspKal %cyro1:", ż);
		print(chNapis, KOL12 + LIBELLA_BOK, 200);
		sprintf(chNapis, "WspKal %cyro2:", ż);
		print(chNapis, KOL12 + LIBELLA_BOK, 220);

		setColor(GRAY40);
		stPrzycisk.sX1 = 10 + LIBELLA_BOK;
		stPrzycisk.sY1 = 240;
		stPrzycisk.sX2 = stPrzycisk.sX1 + 210;
		stPrzycisk.sY2 = stPrzycisk.sY1 + 75;
		RysujPrzycisk(stPrzycisk, "Odczyt", RYSUJ);

		chEtapKalibracji = 0;
		chStanPrzycisku = 0;
		setColor(YELLOW);
		sprintf(chNapis, "B%cbelek powinien by%c w %crodku poziomicy", ą, ć, ś);
		print(chNapis, CENTER, 50);
		setColor(GRAY60);
		sprintf(chNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		print(chNapis, CENTER, 70);
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
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[2], ZNAK_STOPIEN);
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;

	case SEKW_KAL_WZM_ZYRO_Q:
		chNazwaOsi = 'Y';
		fPochylenie = atan2f(uDaneCM4.dane.fAkcel1[1], uDaneCM4.dane.fAkcel1[0]) + 90 * DEG2RAD;	//atan(y/x)
		fPrzechylenie = atan2f(uDaneCM4.dane.fAkcel1[1], uDaneCM4.dane.fAkcel1[2]) + 90 * DEG2RAD;	//atan(y/z)
		Poziomica(fPrzechylenie, -fPochylenie);	//przechylenie, pochylenie
		setColor(KOLOR_Y);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[1], ZNAK_STOPIEN);
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[1], ZNAK_STOPIEN);
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);	//przechylenie
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;

	case SEKW_KAL_WZM_ZYRO_P:
		chNazwaOsi = 'X';
		fPochylenie = atan2f(uDaneCM4.dane.fAkcel1[2], uDaneCM4.dane.fAkcel1[0]);	//atan(z/x)
		fPrzechylenie = atan2f(uDaneCM4.dane.fAkcel1[0], uDaneCM4.dane.fAkcel1[1]) - 90 * DEG2RAD;	//atan(x/y)
		Poziomica(-fPrzechylenie, fPochylenie);	//przechylenie, pochylenie
		setColor(KOLOR_X);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro1[0], ZNAK_STOPIEN);
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatZyro2[0], ZNAK_STOPIEN);
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);	//przechylenie:
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;
	}

	setColor(YELLOW);
	sprintf(chNapis, "Skieruj osia %c do dolu i wykonaj %d obroty w dowolna strone", chNazwaOsi, OBR_KAL_WZM);
	print(chNapis, CENTER, 30);

	//wyświetl współczynnik kalibracji
	if ((uDaneCM4.dane.nZainicjowano & INIT_WYK_KAL_WZM_ZYRO) && (chEtapKalibracji >= 2))
	{
		setColor(WHITE);										//nowy współczynnik
		sprintf(chNapis, "%.3f", uDaneCM4.dane.uRozne.f32[0]);
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 200);
		sprintf(chNapis, "%.3f", uDaneCM4.dane.uRozne.f32[1]);
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 220);
	}
	else
	{
		setColor(GRAY80);										//stary współczynnik
		sprintf(chNapis, "%.3f", uDaneCM4.dane.uRozne.f32[2]);
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 200);
		sprintf(chNapis, "%.3f", uDaneCM4.dane.uRozne.f32[3]);
		print(chNapis, KOL12 + LIBELLA_BOK + 14*FONT_SL, 220);
	}

	//napis dla przycisku
	setColor(YELLOW);
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

	/*setBackColor(GRAY40);	//kolor tła napisu kolorem przycisku
	setFont(BigFont);
	print(chNapis, stPrzycisk.sX1 + (stPrzycisk.sX2 - stPrzycisk.sX1)/2 - chRozmiar*FONT_BL/2 , stPrzycisk.sY1 + (stPrzycisk.sY2 - stPrzycisk.sY1)/2 - FONT_BH/2);
	setBackColor(BLACK);
	setFont(MidFont);*/


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
		LCD_ProstokatWypelniony(0, DISP_Y_SIZE - LIBELLA_BOK, LIBELLA_BOK, LIBELLA_BOK, LIBELLA1);
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
		fillCircle(x2, y2, BABEL);

		//rysuj bąbelek
		setColor(LIBELLA3);
		fillCircle(x, y, BABEL-2);
		setColor(LIBELLA2);
		drawCircle(x, y, BABEL-1);	//obwódka bąbelka

		x2 = x;
		y2 = y;

		//skala 5°
		setColor(BLACK);
		drawCircle(LIBELLA_BOK / 2, DISP_Y_SIZE - LIBELLA_BOK / 2, 50);

		//skala 10°
		setColor(BLACK);
		//drawCircle(LIBELLA_BOK / 2, MENU_NAG_WYS + (DISP_Y_SIZE - MENU_NAG_WYS)/2, 75);
		drawCircle(LIBELLA_BOK / 2, DISP_Y_SIZE - LIBELLA_BOK / 2, 75);

		//skala 15°
		setColor(BLACK);
		//drawCircle(LIBELLA_BOK  / 2, MENU_NAG_WYS+(DISP_Y_SIZE - MENU_NAG_WYS)/2, 100);
		drawCircle(LIBELLA_BOK / 2, DISP_Y_SIZE - LIBELLA_BOK / 2, 100);

		drawHLine(0, DISP_Y_SIZE - LIBELLA_BOK/2, LIBELLA_BOK);
		drawVLine(LIBELLA_BOK / 2, DISP_Y_SIZE - LIBELLA_BOK, LIBELLA_BOK);
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
		//setColor(GRAY40);
		//fillRect(prost.sX1, prost.sY1 ,prost.sX2, prost.sY2);	//rysuj przycisk
		LCD_ProstokatWypelniony(prost.sX1, prost.sY1, prost.sX2-prost.sX1, prost.sY2-prost.sY1, GRAY40);
	}

	setColor(YELLOW);
	setBackColor(GRAY40);	//kolor tła napisu kolorem przycisku
	setFont(BigFont);
	print(chNapis, prost.sX1 + (prost.sX2 - prost.sX1)/2 - chRozmiar*FONT_BL/2 , prost.sY1 + (prost.sY2 - prost.sY1)/2 - FONT_BH/2);
	setBackColor(BLACK);
	setFont(MidFont);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran dla kalibracji zera (offsetu) magnetometru. Dla obrotu wokół X rysuje wykres YZ, dla obrotu wokół Y rysuje XZ dla Z rysuje XY
// Analizuje dane przekazane w zmiennej uDaneCM4.dane.uRozne.f32[6] przez CM4. Funkcja znajjduje i sygnalizuje znalezienia ich maksimów poprzez komunikaty głosowe: 1-minX, 2-maxX, 3-minY, 4-maxY, 5-minZ, 6-maxZ
// Wykres jest skalowany do maksimum wartosci bezwzględnej obu zmiennych
// Parametry: *chEtap	- wskaźnik na zmienną wskazującą na kalibracje konktretnej osi w konkretnym magnetometrze. Starszy półbajt koduje numer magnetometru, najmłodsze 2 bity oś obracaną a bit 4 procedurę kalibracji
// uDaneCM4.dane.uRozne.f32[6] - minima i maksima magnetometrów zbierana przez CM4
// Zwraca: ERR_GOTOWE / ERR_OK - informację o tym czy wyjść z trybu kalibracji czy nie
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibracjaZeraMagnetometru(uint8_t *chEtap)
{
	char chNazwaOsi;
	uint8_t chErr = ERR_OK;
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

		setColor(GRAY80);
		sprintf(chNapis, "%s X:", chNapisLcd[STR_MAGN]);
		print(chNapis, KOL12, 80);
		sprintf(chNapis, "%s Y:", chNapisLcd[STR_MAGN]);
		print(chNapis, KOL12, 100);
		sprintf(chNapis, "%s Z:", chNapisLcd[STR_MAGN]);
		print(chNapis, KOL12, 120);
		sprintf(chNapis, "Pochylenie:");
		print(chNapis, KOL12, 140);
		sprintf(chNapis, "Przechylenie:");
		print(chNapis, KOL12, 160);
		if (*chEtap & KALIBRUJ)
		{
			sprintf(chNapis, "%s X:", chNapisLcd[STR_EKSTREMA]);
			print(chNapis, KOL12, 180);
			sprintf(chNapis, "%s Y:", chNapisLcd[STR_EKSTREMA]);
			print(chNapis, KOL12, 200);
			sprintf(chNapis, "%s Z:", chNapisLcd[STR_EKSTREMA]);
			print(chNapis, KOL12, 220);
		}
		else
		{
			sprintf(chNapis, "%s X:", chNapisLcd[STR_KAL]);
			print(chNapis, KOL12, 180);
			sprintf(chNapis, "%s Y:", chNapisLcd[STR_KAL]);
			print(chNapis, KOL12, 200);
			sprintf(chNapis, "%s Z:", chNapisLcd[STR_KAL]);
			print(chNapis, KOL12, 220);
		}


		setColor(GRAY60);
		sprintf(chNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		print(chNapis, CENTER, 45);

		setColor(GRAY40);
		stPrzycisk.sX1 = 10;
		stPrzycisk.sY1 = 250;
		stPrzycisk.sX2 = stPrzycisk.sX1 + 210;
		stPrzycisk.sY2 = DISP_Y_SIZE;
		RysujPrzycisk(stPrzycisk, "Nastepna Os", RYSUJ);

		setColor(GRAY40);
		stWykr.sX1 = DISP_X_SIZE - SZER_WYKR_MAG;
		stWykr.sY1 = DISP_Y_SIZE - SZER_WYKR_MAG;
		stWykr.sX2 = DISP_X_SIZE;
		stWykr.sY2 = DISP_Y_SIZE;
		drawRect(stWykr.sX1, stWykr.sY1, stWykr.sX2, stWykr.sY2);	//rysuj ramkę wykresu
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
			return ERR_OK;	//nie idź dalej dopóki nie dostanie potwierdzenia że ekstrema zostały wyzerowane.
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
				drawLine(sX, sY, sPoprzedX, sPoprzedY);
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
				drawLine(sX, sY, sPoprzedX, sPoprzedY);
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
				drawLine(sX, sY, sPoprzedX, sPoprzedY);
				sPoprzedX = sX;
				sPoprzedY = sY;
			}
		}
		break;
	}

	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f [uT] ", fMag[0]*1e6);
	print(chNapis, KOL12 + 8*FONT_SL, 80);
	sprintf(chNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[0], ZNAK_STOPIEN);
	print(chNapis, KOL12 + 12*FONT_SL, 140);
	if (*chEtap & KALIBRUJ)
	{
		sprintf(chNapis, "%.2f, %.2f ", uDaneCM4.dane.uRozne.f32[0]*1e6, uDaneCM4.dane.uRozne.f32[1]*1e6);	//ekstrema X
		print(chNapis, KOL12 + 12*FONT_SL, 180);
	}
	else
	{
		sprintf(chNapis, "%.3e, %.4f ", uDaneCM4.dane.uRozne.f32[0], uDaneCM4.dane.uRozne.f32[1]);	//przesunięcie i skalowanie X
		print(chNapis, KOL12 + 7*FONT_SL, 180);
	}

	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f [uT] ", fMag[1]*1e6);
	print(chNapis, 10 + 8*FONT_SL, 100);
	sprintf(chNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[1], ZNAK_STOPIEN);
	print(chNapis, KOL12 + 14*FONT_SL, 160);
	if (*chEtap & KALIBRUJ)
	{
		sprintf(chNapis, "%.2f, %.2f ", uDaneCM4.dane.uRozne.f32[2]*1e6, uDaneCM4.dane.uRozne.f32[3]*1e6);	//ekstrema Y
		print(chNapis, KOL12 + 12*FONT_SL, 200);
	}
	else
	{
		sprintf(chNapis, "%.3e, %.4f ", uDaneCM4.dane.uRozne.f32[2], uDaneCM4.dane.uRozne.f32[3]);		//przesunięcie i skalowanie Y
		print(chNapis, KOL12 + 7*FONT_SL, 200);
	}

	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f [uT] ", fMag[2]*1e6);
	print(chNapis, 10 + 8*FONT_SL, 120);
	if (*chEtap & KALIBRUJ)
	{
		sprintf(chNapis, "%.2f, %.2f ", uDaneCM4.dane.uRozne.f32[4]*1e6, uDaneCM4.dane.uRozne.f32[5]*1e6);	//ekstrema Z
		print(chNapis, KOL12 + 12*FONT_SL, 220);
	}
	else
	{
		sprintf(chNapis, "%.3e, %.4f ", uDaneCM4.dane.uRozne.f32[4], uDaneCM4.dane.uRozne.f32[5]);		//przesunięcie i skalowanie Z
		print(chNapis, KOL12 + 7*FONT_SL, 220);
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

	setColor(YELLOW);
	sprintf(chNapis, "Ustaw UAV osi%c %c w kier. Wsch%cd-Zach%cd i obracaj wok%c%c %c", ą, chNazwaOsi, ó, ó, ó, ł, chNazwaOsi);
	print(chNapis, CENTER, 25);
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
	uint8_t chErr = ERR_OK;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Kalibr. pomiaru cisnienia");

		setColor(YELLOW);
		sprintf(chNapis, "U%crednij ci%cn. pocz. Pokonaj wys=27m i ponownie u%crednij", ś, ś, ś);
		print(chNapis, CENTER, 30);

		setColor(GRAY60);
		sprintf(chNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		print(chNapis, CENTER, 50);

		setColor(GRAY80);
		sprintf(chNapis, "Czujnik 1");
		print(chNapis, KOL12 + 15*FONT_SL, 80);
		sprintf(chNapis, "Czujnik 2");
		print(chNapis, KOL12 + 30*FONT_SL, 80);

		sprintf(chNapis, "Biezace cisn:");
		print(chNapis, KOL12, 100);
		sprintf(chNapis, "Sredn cisn 1:");
		print(chNapis, KOL12, 120);
		sprintf(chNapis, "Sredn.cisn 2:");
		print(chNapis, KOL12, 140);
		sprintf(chNapis, "Skalowanie:");
		print(chNapis, KOL12, 160);


		setColor(GRAY40);
		stPrzycisk.sX1 = 10;
		stPrzycisk.sY1 = 210;
		stPrzycisk.sX2 = stPrzycisk.sX1 + 250;
		stPrzycisk.sY2 = DISP_Y_SIZE - WYS_PASKA_POSTEPU - 1;
		RysujPrzycisk(stPrzycisk, "Start", RYSUJ);
		chStanPrzycisku = 0;
		*chEtap = 0;
	}

	// Zwraca: kod błędu
	setColor(WHITE);
	sprintf(chNapis, "%.0f Pa", uDaneCM4.dane.fCisnieBzw[0]);
	print(chNapis, KOL12 + 14*FONT_SL, 100);
	sprintf(chNapis, "%.0f Pa", uDaneCM4.dane.fCisnieBzw[1]);
	print(chNapis, KOL12 + 30*FONT_SL, 100);
	sprintf(chNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[0]);	//pierwsze uśrednione ciśnienie czujnika 1
	print(chNapis, KOL12 + 14*FONT_SL, 120);
	sprintf(chNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[1]);	//pierwsze uśrednione ciśnienie czujnika 2
	print(chNapis, KOL12 + 30*FONT_SL, 120);
	sprintf(chNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[2]);	//drugie uśrednione ciśnienie czujnika 1
	print(chNapis, KOL12 + 14*FONT_SL, 140);
	sprintf(chNapis, "%.2f Pa", uDaneCM4.dane.uRozne.f32[3]);	//drugie uśrednione ciśnienie czujnika 2
	print(chNapis, KOL12 + 30*FONT_SL, 140);
	sprintf(chNapis, "%.6f ", uDaneCM4.dane.uRozne.f32[4]);	//współczynnik skalowania czujnika 1
	print(chNapis, KOL12 + 14*FONT_SL, 160);
	sprintf(chNapis, "%.6f ", uDaneCM4.dane.uRozne.f32[5]);	//współczynnik skalowania czujnika 2
	print(chNapis, KOL12 + 30*FONT_SL, 160);

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
		print(chNapis, 10 + stPrzycisk.sX2, 240);
		sprintf(chNapis, "dP2 = %.2f Pa", fabs(uDaneCM4.dane.uRozne.f32[1] - uDaneCM4.dane.uRozne.f32[3]));	//Rożnica ciśnień czujnika 2
		print(chNapis, 10 + stPrzycisk.sX2, 260);
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

		setColor(GRAY80);
		sprintf(chNapis, "Przechylenie:");
		print(chNapis, KOL12, 80);
		sprintf(chNapis, "Pochylenie:");
		print(chNapis, KOL12, 100);

		sprintf(chNapis, "%s 1:", chNapisLcd[STR_MAGN]);
		print(chNapis, KOL12, 120);
		sprintf(chNapis, "%s 2:", chNapisLcd[STR_MAGN]);
		print(chNapis, KOL12, 140);
		sprintf(chNapis, "%s 3:", chNapisLcd[STR_MAGN]);
		print(chNapis, KOL12, 160);

		sprintf(chNapis, "Inklinacja: %.2f %c", INKLINACJA_MAG * RAD2DEG, ZNAK_STOPIEN);
		print(chNapis, KOL12, 180);
		sprintf(chNapis, "Deklinacja: %.2f %c", DEKLINACJA_MAG * RAD2DEG, ZNAK_STOPIEN);
		print(chNapis, KOL12, 200);

		setColor(GRAY60);
		sprintf(chNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		print(chNapis, CENTER, 30);

		setColor(GRAY40);
		stWykr.sX1 = DISP_X_SIZE - SZER_WYKR_MAG;
		stWykr.sY1 = DISP_Y_SIZE - SZER_WYKR_MAG;
		stWykr.sX2 = DISP_X_SIZE;
		stWykr.sY2 = DISP_Y_SIZE;
		drawRect(stWykr.sX1, stWykr.sY1, stWykr.sX2, stWykr.sY2);	//rysuj ramkę wykresu
		sPoprzed1X = sPoprzed2X = sPoprzed3X = stWykr.sX1 + SZER_WYKR_MAG/2;	//środek wykresu
		sPoprzed1Y = sPoprzed2Y = sPoprzed3Y = stWykr.sY1 + SZER_WYKR_MAG/2;
		//drawCircle(stWykr.sX1 + SZER_WYKR_MAG/2, stWykr.sY1 + SZER_WYKR_MAG/2, 2*SZER_WYKR_MAG/2 * cosf(INKLINACJA_MAG));	//promień okręgu jest rzutem wektora magnetycznego pochylonego pod kątem inklinacji
		drawCircle(stWykr.sX1 + SZER_WYKR_MAG/2, stWykr.sY1 + SZER_WYKR_MAG/2, PROMIEN_RZUTU_MAGN);

		//legenda kierunków  magnetycznych
		setColor(GRAY80);
		printChar('N', stWykr.sX1 + SZER_WYKR_MAG - 20, stWykr.sY1 + SZER_WYKR_MAG/2);
		printChar('E', stWykr.sX1 + SZER_WYKR_MAG/2, stWykr.sY1 + SZER_WYKR_MAG - 20);
		printChar('S', stWykr.sX1 + 20, stWykr.sY1 + SZER_WYKR_MAG/2);
		printChar('W', stWykr.sX1 + SZER_WYKR_MAG/2, stWykr.sY1 + 20 - FONT_SH);
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
		//setColor(KOLOR_X);
		setColor(CYAN);
		drawLine(sX, sY, sPoprzed1X, sPoprzed1Y);
		sPoprzed1X = sX;
		sPoprzed1Y = sY;
	}

	//rysuj wykres X-Y magnetometru 2
	sX = (int16_t)(uDaneCM4.dane.fMagne2[0] * fWspSkal) + stWykr.sX1 + SZER_WYKR_MAG/2;
	sY = (int16_t)(uDaneCM4.dane.fMagne2[1] * fWspSkal) + stWykr.sY1 + SZER_WYKR_MAG/2;
	if ((sX > stWykr.sX1) && (sX < stWykr.sX2) && (sY > stWykr.sY1) && (sY < stWykr.sY2) && ((sX != sPoprzed2X) && (sY != sPoprzed2Y)))
	{
		//setColor(KOLOR_Y);
		setColor(MAGENTA);
		drawLine(sX, sY, sPoprzed2X, sPoprzed2Y);
		sPoprzed2X = sX;
		sPoprzed2Y = sY;
	}

	//rysuj wykres X-Y magnetometru 3
	sX = (int16_t)(uDaneCM4.dane.fMagne3[0] * fWspSkal) + stWykr.sX1 + SZER_WYKR_MAG/2;
	sY = (int16_t)(uDaneCM4.dane.fMagne3[1] * fWspSkal) + stWykr.sY1 + SZER_WYKR_MAG/2;
	if ((sX > stWykr.sX1) && (sX < stWykr.sX2) && (sY > stWykr.sY1) && (sY < stWykr.sY2) && ((sX != sPoprzed3X) && (sY != sPoprzed3Y)))
	{
		//setColor(KOLOR_Z);
		setColor(YELLOW);
		drawLine(sX, sY, sPoprzed3X, sPoprzed3Y);
		sPoprzed3X = sX;
		sPoprzed3Y = sY;
	}

	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[0], ZNAK_STOPIEN);
	print(chNapis, KOL12 + 14*FONT_SL, 80);

	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU2[1], ZNAK_STOPIEN);
	print(chNapis, KOL12 + 12*FONT_SL, 100);


	setColor(CYAN);
	sprintf(chNapis, "%.2f, %.2f [uT] ", uDaneCM4.dane.fMagne1[0]*1000, uDaneCM4.dane.fMagne1[1]*1e6f);
	print(chNapis, KOL12 + 8*FONT_SL, 120);

	setColor(MAGENTA);
	sprintf(chNapis, "%.2f, %.2f [uT] ", uDaneCM4.dane.fMagne2[0]*1000, uDaneCM4.dane.fMagne2[1]*1e6f);
	print(chNapis, KOL12 + 8*FONT_SL, 140);

	setColor(YELLOW);
	sprintf(chNapis, "%.2f, %.2f [uT] ", uDaneCM4.dane.fMagne3[0]*1000, uDaneCM4.dane.fMagne3[1]*1e6f);
	print(chNapis, KOL12 + 8*FONT_SL, 160);
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

		setColor(GRAY80);
		sprintf(chNapis, "Regulator g%c%cwny", ł, ó);
		print(chNapis, KOL12, 60);
		sprintf(chNapis, "Kp:");
		print(chNapis, KOL12, 80);
		sprintf(chNapis, "Ti:");
		print(chNapis, KOL12, 100);
		sprintf(chNapis, "Td:");
		print(chNapis, KOL12, 120);
		sprintf(chNapis, "Max I:");
		print(chNapis, KOL12, 140);
		sprintf(chNapis, "Min Wy:");
		print(chNapis, KOL12, 160);
		sprintf(chNapis, "Max Wy:");
		print(chNapis, KOL12, 180);
		sprintf(chNapis, "Filtr D:");
		print(chNapis, KOL12, 200);
		sprintf(chNapis, "k%ctowy:", ą);
		print(chNapis, KOL12, 220);

		sprintf(chNapis, "Regulator pochodnej");
		print(chNapis, KOL22, 60);
		sprintf(chNapis, "Kp:");
		print(chNapis, KOL22, 80);
		sprintf(chNapis, "Ti:");
		print(chNapis, KOL22, 100);
		sprintf(chNapis, "Td:");
		print(chNapis, KOL22, 120);
		sprintf(chNapis, "Max I:");
		print(chNapis, KOL22, 140);
		sprintf(chNapis, "Min Wy:");
		print(chNapis, KOL22, 160);
		sprintf(chNapis, "Max Wy:");
		print(chNapis, KOL22, 180);
		sprintf(chNapis, "Filtr D:");
		print(chNapis, KOL22, 200);
		setColor(GRAY60);
		sprintf(chNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		print(chNapis, CENTER, 30);

		//odczytaj nastawy regulatorów
		chErr = CzytajFram(FAU_PID_P0 + (chKanal + 0) * ROZMIAR_REG_PID, ROZMIAR_REG_PID/4, fNastawy);
		if (chErr == ERR_OK)
		{
			un8_32.daneFloat = fNastawy[6];
			if (un8_32.dane8[0] & PID_WLACZONY)
				setColor(GRAY60);	//regulator wyłączony
			else
				setColor(WHITE);	//regulator pracuje

			sprintf(chNapis, "%.3f ", fNastawy[0]);	//Kp
			print(chNapis, KOL12 + 4*FONT_SL, 80);
			sprintf(chNapis, "%.3f ", fNastawy[1]);	//Ti
			print(chNapis, KOL12 + 4*FONT_SL, 100);
			sprintf(chNapis, "%.3f ", fNastawy[2]);	//Td
			print(chNapis, KOL12 + 4*FONT_SL, 120);
			sprintf(chNapis, "%.3f ", fNastawy[3]);	//max całki
			print(chNapis, KOL12 + 7*FONT_SL, 140);
			sprintf(chNapis, "%.3f ", fNastawy[4]);	//min wyjścia
			print(chNapis, KOL12 + 8*FONT_SL, 160);
			sprintf(chNapis, "%.3f ", fNastawy[5]);	//max wyjścia
			print(chNapis, KOL12 + 8*FONT_SL, 180);
			sprintf(chNapis, "%d", un8_32.dane8[0] & PID_MASKA_FILTRA_D);
			print(chNapis, KOL12 + 9*FONT_SL, 200);	//filtr D
			if (un8_32.dane8[0] & PID_KATOWY)
				sprintf(chNapis, "Tak");
			else
				sprintf(chNapis, "Nie");
			print(chNapis, KOL12 + 8*FONT_SL, 220);	//kątowy
		}
		else
			chRysujRaz = 1;	//jeżeli się nie odczytało to wyświetl jeszcze raz

		chErr = CzytajFram(FAU_PID_P0 + (chKanal + 1) * ROZMIAR_REG_PID, ROZMIAR_REG_PID/4, fNastawy);
		if (chErr == ERR_OK)
		{
			un8_32.daneFloat = fNastawy[6];
			if (un8_32.dane8[0] & PID_WLACZONY)
				setColor(GRAY60);	//regulator wyłączony
			else
				setColor(WHITE);	//regulator pracuje

			sprintf(chNapis, "%.3f ", fNastawy[0]);	//Kp
			print(chNapis, KOL22 + 4*FONT_SL, 80);
			sprintf(chNapis, "%.3f ", fNastawy[1]);	//Ti
			print(chNapis, KOL22 + 4*FONT_SL, 100);
			sprintf(chNapis, "%.3f ", fNastawy[2]);	//Td
			print(chNapis, KOL22 + 4*FONT_SL, 120);
			sprintf(chNapis, "%.3f ", fNastawy[3]);	//max całki
			print(chNapis, KOL22 + 7*FONT_SL, 140);
			sprintf(chNapis, "%.3f ", fNastawy[4]);	//min wyjścia
			print(chNapis, KOL22 + 8*FONT_SL, 160);
			sprintf(chNapis, "%.3f ", fNastawy[5]);	//max wyjścia
			print(chNapis, KOL22 + 8*FONT_SL, 180);
			sprintf(chNapis, "%d", un8_32.dane8[0] & PID_MASKA_FILTRA_D);
			print(chNapis, KOL22 + 9*FONT_SL, 200);	//filtr D
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
	uint8_t chTimeout = 10;
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
		return ERR_OK;
	else
		return ERR_TIMEOUT;
}



////////////////////////////////////////////////////////////////////////////////
// Wyświetla zawartość bufora kamery
// Parametry:
// [we] sSzerokosc - szerokość obrazu do wyświetlenia
// [we] sWysokosc - wysokość obrazu do wyświetlenia
// [we] *sObraz - wskaźnik na bufor obrazu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t WyswietlZdjecie(uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t* sObraz)
{
	uint8_t chErr = ERR_OK;

	if (sSzerokosc > DISP_X_SIZE)
		sSzerokosc = DISP_X_SIZE;
	if (sWysokosc > DISP_Y_SIZE)
		sWysokosc = DISP_Y_SIZE;
	drawBitmap(0, 0, sSzerokosc, sWysokosc, sObraz);
	return chErr;
}
