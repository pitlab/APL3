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

//deklaracje zmiennych
extern uint8_t MidFont[];
extern uint8_t BigFont[];
const char *build_date = __DATE__;
const char *build_time = __TIME__;
extern TIM_HandleTypeDef htim6;
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

//definicje zmiennych
uint8_t chTrybPracy;
uint8_t chNowyTrybPracy;
uint8_t chWrocDoTrybu;
uint8_t chRysujRaz;
uint8_t chMenuSelPos, chStarySelPos;	//wybrana pozycja menu i poprzednia pozycja

char chNapis[100], chNapisPodreczny[30];
float fTemperaturaKalibracji;
float fZoom, fX, fY;
float fReal, fImag;
unsigned char chMnozPalety;
unsigned char chDemoMode;
unsigned char chLiczIter;		//licznik iteracji fraktala
extern uint16_t sBuforLCD[DISP_X_SIZE * DISP_Y_SIZE];
extern struct _statusDotyku statusDotyku;
extern uint32_t nZainicjowano[2];		//flagi inicjalizacji sprzętu
extern uint8_t chPorty_exp_wysylane[];
extern uint8_t chPorty_exp_odbierane[];
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

//Definicje ekranów menu
struct tmenu stMenuGlowne[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Multimedia",  "Obsluga multimediow: dzwiek i obraz",		TP_MULTIMEDIA, 		obr_glosnik2},
	{"Wydajnosc",	"Pomiary wydajnosci systemow",				TP_WYDAJNOSC,		obr_Wydajnosc},
	{"Karta SD",	"Rejestrator i parametry karty SD",			TP_KARTA_SD,		obr_kartaSD},
	{"Kalib IMU", 	"Kalibracja zyroskopow i akclerometrow",	TP_IMU,				obr_baczek},
	{"Kalib Magn",	"Obsluga i kalibracja magnetometru",		TP_MAGNETOMETR,		obr_kal_mag_n1},
	{"Dane IMU",	"Wyniki pomiarow czujnikow IMU",			TP_POMIARY_IMU, 	obr_multimetr},
	{"nic 1", 		"nic",										TP_MG1,				obr_multitool},
	{"nic 2", 		"nic",										TP_MG2,				obr_multitool},
	{"Startowy",	"Ekran startowy",							TP_WITAJ,			obr_kontrolny},
	{"TestDotyk",	"Testy panelu dotykowego",					TP_TESTY,			obr_dotyk}};


struct tmenu stMenuWydajnosc[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Fraktale",	"Benchmark fraktalowy"	,					TP_FRAKTALE,		obr_fraktal},
	{"Zapis NOR", 	"Test zapisu do flash NOR",					TP_POM_ZAPISU_NOR,	obr_NOR},
	{"Trans NOR", 	"Pomiar predkosci flasha NOR 16-bit",		TP_POMIAR_FNOR,		obr_NOR},
	{"Trans QSPI",	"Pomiar predkosci flasha QSPI 4-bit",		TP_POMIAR_FQSPI,	obr_QSPI},
	{"Trans RAM",	"Pomiar predkosci SRAM i DRAM 16-bit",		TP_POMIAR_SRAM,		obr_RAM},
	{"W1",			"nic",										TP_W1,				obr_Wydajnosc},
	{"W2",			"nic",										TP_W2,				obr_Wydajnosc},
	{"SD33",		"nic",										TP_W3,				obr_Wydajnosc},
	{"SD18",		"nic",										TP_W4,				obr_Wydajnosc},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_back}};



struct tmenu stMenuMultiMedia[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Miki Rej",  	"Obsluga kamery, aparatu i obrobka obrazu",	TP_MMREJ,	 		obr_Mikołaj_Rey},
	{"Mikrofon",	"Wlacza mikrofon wylacza wzmacniacz",		TP_MM1,				obr_glosnik2},
	{"Wzmacniacz",	"Wlacza wzmacniacz, wylacza mikrofon",		TP_MM2,				obr_glosnik2},
	{"Test Tonu",	"Test tonu wario",							TP_MM_TEST_TONU,	obr_glosnik2},
	{"FFT Audio",	"FFT sygnału z mikrofonu",					TP_MM_AUDIO_FFT,	obr_fft},
	{"Komunikat1",	"Komunikat glosowy",						TP_MM_KOM1,			obr_volume},
	{"Komunikat2",	"Komunikat glosowy",						TP_MM_KOM2,			obr_volume},
	{"Komunikat3",	"Komunikat glosowy",						TP_MM_KOM3,			obr_volume},
	{"Test kom.",	"Test komunikatów audio",					TP_MM_KOM4,			obr_glosnik2},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_back}};


struct tmenu stMenuKartaSD[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Wlacz Rej",   "Wlacza rejestrator",   					TPKS_WLACZ_REJ,		obr_kartaSD},
	{"Wylacz Rej",  "Wylacza rejestrator",   					TPKS_WYLACZ_REJ,	obr_kartaSD},
	{"Parametry",	"Parametry karty SD",						TPKS_PARAMETRY,		obr_kartaSD},
	//{"Test trans",	"Wyniki pomiaru predkosci transferu",		TPKS_POMIAR, 		obr_multimetr},
	{"TPKS_4",		"nic  ",									TPKS_4,				obr_dotyk},
	{"TPKS_4",		"nic  ",									TPKS_4,				obr_dotyk},
	{"TPKS_5",		"nic  ",									TPKS_5,				obr_dotyk},
	{"TPKS_6",		"nic  ",									TPKS_6,				obr_dotyk},
	{"TPKS_7",		"nic  ",									TPKS_7,				obr_dotyk},
	{"TPKS_8",		"nic  ",									TPKS_8,				obr_dotyk},
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
	{"Kostka 3D", 	"Rysuje kostke 3D w funkcji katow IMU",		TP_IMU_KOSTKA,		obr_kostka3D},
	{"Powrot",		"Wraca do menu glownego",					TP_WROC_DO_MENU,	obr_back}};


struct tmenu stMenuMagnetometr[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Kal Magn1", 	"Kalibracja magnetometru 1",				TP_MAG_KAL1,		obr_kal_mag_n1},
	{"Kal Magn2", 	"Kalibracja magnetometru 2",				TP_MAG_KAL2,		obr_kal_mag_n1},
	{"Kal Magn3", 	"Kalibracja magnetometru 3",				TP_MAG_KAL3,		obr_kal_mag_n1},
	{"MAG4",		"nic  ",									TP_MAG4,			obr_dotyk},
	{"MAG5",		"nic  ",									TP_MAG5,			obr_dotyk},
	{"Spr Magn1",	"Sprawdz kalibracje magnetometru 1",		TP_SPR_MAG1,		obr_kal_mag_n1},
	{"Spr Magn2",	"Sprawdz kalibracje magnetometru 2",		TP_SPR_MAG2,		obr_kal_mag_n1},
	{"Spr Magn3",	"Sprawdz kalibracje magnetometru 3",		TP_SPR_MAG3,		obr_kal_mag_n1},
	{"Kal Dotyk", 	"Kalibracja panelu dotykowego na LCD",		TP_KAL_DOTYK,		obr_dotyk_zolty},
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
		chTrybPracy = TP_KALIB_DOTYK;

	switch (chTrybPracy)
	{
	case TP_MENU_GLOWNE:	// wyświetla menu główne	MenuGlowne(&chNowyTrybPracy);
		Menu((char*)chNapisLcd[STR_MENU_MAIN], stMenuGlowne, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;

	case TP_POMIARY_IMU:	PomiaryIMU();		//wyświetlaj wyniki pomiarów IMU pobrane z CM4
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chNowyTrybPracy = TP_WROC_DO_MENU;
			ZatrzymajTon();
		}
		break;

	case TP_MG1:
		chNowyTrybPracy = TP_WROC_DO_MENU;
		break;

	case TP_MG2:
		TestKomunikacjiSTD();	//wyślij komunikat tesktowy przez LPUART
		chNowyTrybPracy = TP_WROC_DO_MENU;
		break;

	case TP_MG4:
		TestKomunikacjiDMA();	//wyślij komunikat tesktowy przez LPUART
		chNowyTrybPracy = TP_WROC_DO_MENU;
		break;

	case TP_ZDJECIE:
		//chNowyTrybPracy = TP_WROC_DO_MENU;
		break;

	case TP_WITAJ:
		Ekran_Powitalny(nZainicjowano);	//przywitaj użytkownika i prezentuj wykryty sprzęt
		chNowyTrybPracy = TP_WROC_DO_MENU;
		break;

	case TP_TESTY:
		if (TestDotyku() == ERR_DONE)
			chNowyTrybPracy = TP_WROC_DO_MENU;
		break;

	case TP_USTAWIENIA:	break;


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
		chPorty_exp_wysylane[1] |= EXP13_AUDIO_IN_SD;	//AUDIO_IN_SD - włącznika ShutDown mikrofonu, aktywny niski
		chPorty_exp_wysylane[1] &= ~EXP14_AUDIO_OUT_SD;	//AUDIO_OUT_SD - włączniek ShutDown wzmacniacza audio, aktywny niski
		chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		break;

	case TP_MM2:	//Włącza wzmacniacz, włącza mikrofon
		chPorty_exp_wysylane[1] &= ~EXP13_AUDIO_IN_SD;	//AUDIO_IN_SD - włącznika ShutDown mikrofonu, aktywny niski
		chPorty_exp_wysylane[1] |= EXP14_AUDIO_OUT_SD;	//AUDIO_OUT_SD - włączniek ShutDown wzmacniacza audio, aktywny niski
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

	case TP_MM_KOM1:	//komunikat audio 1
		//OdtworzProbkeAudio((uint32_t)&sWszystkiegoNajlepszegoAda[0], 65126);
		OdtworzProbkeAudioZeSpisu(0);
		chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		break;

	case TP_MM_KOM2:	//komunikat audio 2
		OdtworzProbkeAudioZeSpisu(1);
		chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		break;

	case TP_MM_KOM3:	//komunikat audio 3
		//OdtworzProbkeAudio((uint32_t)&proba_mikrofonu[0], 30714);
		OdtworzProbkeAudioZeSpisu(2);
		chNowyTrybPracy = TP_WROC_DO_MMEDIA;
		break;

	case TP_MM_KOM4:	//komunikat audio 4
		TestKomunikatow();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
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

	case TP_POMIAR_FQSPI:	W25_TestTransferu();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_WYDAJN;
		}
		break;

	case TP_W1:		UstawTon(0, 60);	chTrybPracy = TP_WYDAJNOSC;	break;
	case TP_W2:		UstawTon(32, 60);	chTrybPracy = TP_WYDAJNOSC;	break;

	///LOG_SD1_VSEL - H=3,3V L=1,8V
	case TP_W3:		chPorty_exp_wysylane[0] |=  EXP02_LOG_VSELECT;	chTrybPracy = TP_WYDAJNOSC;	break;	//LOG_SD1_VSEL - H=3,3V
	case TP_W4:		chPorty_exp_wysylane[0] &= ~EXP02_LOG_VSELECT;	chTrybPracy = TP_WYDAJNOSC;	break;	//LOG_SD1_VSEL - L=1,8V

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
			chNowyTrybPracy = TPKSD_WROC;
		}
		break;

	case TPKS_WYLACZ_REJ:	//najpierw zakmnij plik a potem wyłacz rejestrator
		chStatusRejestratora|= STATREJ_ZAMKNIJ_PLIK | STATREJ_BYL_OTWARTY;
		WyswietlRejestratorKartySD();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TPKSD_WROC;
		}
		break;

	case TPKS_PARAMETRY:
		WyswietlParametryKartySD();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TPKSD_WROC;
		}
		break;

	case TPKS_POMIAR:
		TestKartySD();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TPKSD_WROC;
		}
		break;

	case TPKS_4:
	case TPKS_5:
	case TPKS_6:
	case TPKS_7:
	case TPKS_8:	break;

//*** IMU ************************************************
	case TP_IMU:			//menu IMU
		Menu((char*)chNapisLcd[STR_MENU_IMU], stMenuIMU, &chNowyTrybPracy);
		chWrocDoTrybu = TP_MENU_GLOWNE;
		break;

	case TP_KAL_ZYRO_ZIM:
		uDaneCM7.dane.chWykonajPolecenie = POL_KALIBRUJ_ZYRO_ZIM;	//uruchom kalibrację żyroskopów na zimno 10°C
		fTemperaturaKalibracji = TEMP_KAL_ZIMNO;
		if ((uDaneCM4.dane.sPostepProcesu > 0) && (uDaneCM4.dane.sPostepProcesu < CZAS_KALIBRACJI_ZYROSKOPU))
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
		if ((uDaneCM4.dane.sPostepProcesu > 0) && (uDaneCM4.dane.sPostepProcesu < CZAS_KALIBRACJI_ZYROSKOPU))
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
		if ((uDaneCM4.dane.sPostepProcesu > 0) && (uDaneCM4.dane.sPostepProcesu < CZAS_KALIBRACJI_ZYROSKOPU))
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
			chNowyTrybPracy = TP_IMU_WROC;
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
			chNowyTrybPracy = TP_IMU_WROC;
			uDaneCM7.dane.chWykonajPolecenie = POL_CZYSC_BLEDY;	//po zakończeniu wczyść zwrócony kod błędu
		}
		break;

	case TP_IMU_KOSTKA:	//rysuj kostkę 3D
		float fKat[3];
		for (uint8_t n=0; n<3; n++)
			fKat[n] = -1 *uDaneCM4.dane.fKatIMUZyro2[n];	//do rysowania przyjmij kąty z przeciwnym znakiem - jest OK
		RysujKostkeObrotu(fKat);
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_IMU_WROC;
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
			chNowyTrybPracy = TP_IMU_WROC;
		}
		break;



	case TP_KAL_WZM_ZYROR:
		chSekwencerKalibracji = SEKW_KAL_WZM_ZYRO_R;
		if (KalibracjaWzmocnieniaZyroskopow(&chSekwencerKalibracji) == ERR_DONE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_IMU_WROC;
		}
		break;

	case TP_KAL_WZM_ZYROQ:
		chSekwencerKalibracji = SEKW_KAL_WZM_ZYRO_Q;
		if (KalibracjaWzmocnieniaZyroskopow(&chSekwencerKalibracji) == ERR_DONE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_IMU_WROC;
		}
		break;

	case TP_KAL_WZM_ZYROP:
		chSekwencerKalibracji = SEKW_KAL_WZM_ZYRO_P;
		if (KalibracjaWzmocnieniaZyroskopow(&chSekwencerKalibracji) == ERR_DONE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_IMU_WROC;
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
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_DONE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_MAG_WROC;
		}
		break;

	case TP_MAG_KAL2:
		chSekwencerKalibracji |= MAG2 + KALIBRUJ;
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_DONE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_MAG_WROC;
		}
		break;

	case TP_MAG_KAL3:
		chSekwencerKalibracji |= MAG3 + KALIBRUJ;
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_DONE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_MAG_WROC;
		}
		break;

	case TP_MAG4:
	case TP_MAG5:
	case TP_SPR_MAG1:
		chSekwencerKalibracji |= MAG1;	//sprawdzenie, bez kalibracji
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_DONE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_MAG_WROC;
		}
		break;

	case TP_SPR_MAG2:
		chSekwencerKalibracji |= MAG2;	//sprawdzenie, bez kalibracji
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_DONE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_MAG_WROC;
		}
		break;

	case TP_SPR_MAG3:
		chSekwencerKalibracji |= MAG3;	//sprawdzenie, bez kalibracji
		if (KalibracjaZeraMagnetometru(&chSekwencerKalibracji) == ERR_DONE)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_MAG_WROC;
		}
		break;

	case TP_KAL_DOTYK:
		if (KalibrujDotyk() == ERR_DONE)
			chTrybPracy = TP_TESTY;
		break;

	//case TP_MAG_WROC:

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
		case TP_WROC_DO_MENU:	chTrybPracy = TP_MENU_GLOWNE;	break;	//powrót do menu głównego
		case TP_WROC_DO_MMEDIA:	chTrybPracy = TP_MULTIMEDIA;	break;	//powrót do menu MultiMedia
		case TP_WROC_DO_WYDAJN:	chTrybPracy = TP_WYDAJNOSC;		break;	//powrót do menu Wydajność
		case TPKSD_WROC:		chTrybPracy = TP_KARTA_SD;		break;	//powrót do menu Karta SD
		case TP_IMU_WROC:		chTrybPracy = TP_IMU;			break;	//powrót do menu IMU
		case TP_MAG_WROC:		chTrybPracy = TP_MAGNETOMETR;	break;	//powrót do menu Magnetometr
		case TP_FRAKTALE:		InitFraktal(0);		break;
		case TP_USTAWIENIA:		chTrybPracy = TP_KALIB_DOTYK;	break;
		}

		LCD_clear(BLACK);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran inicjalizacji sprzętu i wyświetla numer wersji
// Parametry: zainicjowano - wskaźnik na tablicę bitów z flagami zainicjowanego sprzętu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void Ekran_Powitalny(uint32_t* zainicjowano)
{
	uint8_t n;
	uint16_t x, y;

	extern const unsigned short plogo165x80[];
	extern uint8_t BSP_SD_IsDetected(void);

	if (chRysujRaz)
	{
		LCD_clear(WHITE);
		drawBitmap((DISP_X_SIZE-165)/2, 5, 165, 80, plogo165x80);

		setColor(GRAY20);
		setBackColor(WHITE);
		setFont(BigFont);
		sprintf(chNapis, "%s @%luMHz", (char*)chNapisLcd[STR_WITAJ_TYTUL], HAL_RCC_GetSysClockFreq()/1000000);
		print(chNapis, CENTER, 95);

		setColor(GRAY30);
		setFont(MidFont);
		sprintf(chNapis, (char*)chNapisLcd[STR_WITAJ_MOTTO], ó, ć, ó, ó, ż);	//"By móc mieć w rój Wronów na pohybel wrażym hordom""
		print(chNapis, CENTER, 118);

		sprintf(chNapis, "(c) PitLab 2025 sv%d.%d.%d @ %s %s", WER_GLOWNA, WER_PODRZ, WER_REPO, build_date, build_time);
		print(chNapis, CENTER, 135);
		chRysujRaz = 0;
	}

	//pierwsza kolumna sprzętu wewnętrznego
	x = 5;
	y = 160;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_FLASH_NOR]);		//"pamięć Flash NOR"
	print(chNapis, x, y);
	Wykrycie(x, y, n, (*(zainicjowano+0) & INIT0_FLASH_NOR) == INIT0_FLASH_NOR);

	y += 20;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_FLASH_QSPI]);	//"pamięć Flash QSPI"
	print(chNapis, x, y);
	Wykrycie(x, y, n, (*(zainicjowano+0) & INIT0_FLASH_NOR) == INIT0_FLASH_NOR);

	y += 20;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_KARTA_SD]);		//"Karta SD"
	print(chNapis, x, y);
	Wykrycie(x, y, n, BSP_SD_IsDetected());

	y += 20;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_KAMERA_OV5642]);	//"kamera "
	print(chNapis, x, y);
	Wykrycie(x, y, n, (*(zainicjowano+1) & INIT1_KAMERA) == INIT1_KAMERA);

	//dane z CM4
	y += 20;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_GNSS]);		//GNSS
	if (uDaneCM4.dane.nZainicjowano & INIT_WYKR_UBLOX)
		n = sprintf(chNapis, "%s -> %s", (char*)chNapisLcd[STR_SPRAWDZ_GNSS], (char*)chNapisLcd[STR_SPRAWDZ_UBLOX]);	//GNSS -> uBlox
	if (uDaneCM4.dane.nZainicjowano & INIT_WYKR_MTK)
		n = sprintf(chNapis, "%s -> %s", (char*)chNapisLcd[STR_SPRAWDZ_GNSS], (char*)chNapisLcd[STR_SPRAWDZ_MTK]);		//GNSS -> MTK
	print(chNapis, x, y);
	Wykrycie(x, y, n,  (uDaneCM4.dane.nZainicjowano & INIT_GNSS_GOTOWY) == INIT_GNSS_GOTOWY);

	y += 20;
	n = sprintf(chNapis, "%s -> %s", (char*)chNapisLcd[STR_SPRAWDZ_GNSS], (char*)chNapisLcd[STR_SPRAWDZ_HMC5883]);	//GNSS -> HMC5883
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_HMC5883) == INIT_HMC5883);

	//druga kolumna sprzętu zewnętrznego
	x = 5 + DISP_X_SIZE / 2;
	y = 160;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_MS5611]);		//moduł IMU -> MS5611
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_MS5611) == INIT_MS5611);

	y += 20;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_BMP581]);		//moduł IMU -> BMP581
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_BMP581) == INIT_BMP581);

	y += 20;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_ICM42688]);		//moduł IMU -> ICM42688
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_ICM42688) == INIT_ICM42688);

	y += 20;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_LSM6DSV]);		//moduł IMU -> LSM6DSV
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV) == INIT_LSM6DSV);

	y += 20;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_MMC34160]);		//moduł IMU -> MMC34160
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_MMC34160) == INIT_MMC34160);

	y += 20;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_IIS2MDC]);		//moduł IMU -> IIS2MDC
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC) == INIT_IIS2MDC);

	y += 20;
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1_ND130]);		//moduł IMU -> ND130
	print(chNapis, x, y);
	Wykrycie(x, y, n, (uDaneCM4.dane.nZainicjowano & INIT_ND130) == INIT_ND130);

	osDelay(2000);
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
// zmierz czas liczenia fraktala Julii
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void FraktalTest(uint8_t chTyp)
{
	uint32_t nCzas;

	nCzas = PobierzCzasT6();
	switch (chTyp)
	{
	case 0:	GenerateJulia(DISP_X_SIZE, DISP_Y_SIZE, DISP_X_SIZE/2, DISP_Y_SIZE/2, 135, sBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Julia: t=%ldus, c=%.3f ", nCzas, fImag);
		fImag -= 0.002;
		break;

			//ca�y fraktal - rotacja palety
	case 1: GenerateMandelbrot(fX, fY, fZoom, 30, sBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Mandelbrot: t=%ldus z=%.1f, p=%d", nCzas, fZoom, chMnozPalety);
		chMnozPalety += 1;
		break;

			//dolina konika x=-0,75, y=0,1
	case 2: GenerateMandelbrot(fX, fY, fZoom, 30, sBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Mandelbrot: t=%ldus z=%.1f, p=%d", nCzas, fZoom, chMnozPalety);
		fZoom /= 0.9;
		break;

			//dolina s�onia x=0,25-0,35, y=0,05, zoom=-0,6..-40
	case 3: GenerateMandelbrot(fX, fY, fZoom, 30, sBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Mandelbrot: t=%ldus z=%.1f, p=%d", nCzas, fZoom, chMnozPalety);
		fZoom /= 0.9;
		break;
	}

	//drawBitmap2(0, 0, DISP_X_SIZE, DISP_Y_SIZE, sBuforLCD);		//wywyła większymi paczkami - zła kolejność ale szybciej
	//drawBitmap(0, 0, DISP_X_SIZE, DISP_Y_SIZE, sBuforLCD);		//wysyła bajty parami we właściwej kolejności
	drawBitmap3(0, 0, DISP_X_SIZE, DISP_Y_SIZE, sBuforLCD);		//wyświetla bitmapę po 4 piksele z rotacją bajtów w wewnętrznym buforze. Dobre kolory i trochę szybciej niż po jednym pikselu
	//drawBitmap4(0, 0, DISP_X_SIZE, DISP_Y_SIZE, sBuforLCD);		//wyświetla bitmapę po 4 piksele przez DMA z rotacją bajtów w wewnętrznym buforze - nie działa
	setFont(MidFont);
	setColor(GREEN);
	print(chNapis, 0, 304);
}


////////////////////////////////////////////////////////////////////////////////
// wyświetl demo z fraktalami
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void FraktalDemo(void)
{
	switch (chDemoMode)
	{
	case 0:	FraktalTest(0);		//rysuj fraktala Julii
		if (chLiczIter > 40)
		{
			chLiczIter = 0;
			chDemoMode++;
			InitFraktal(1);
		}
		break;

	case 1:	FraktalTest(1);		//rysuj fraktala Mandelbrota - cały fraktal rotacja kolorów
		if (chLiczIter > 60)
		{
			chLiczIter = 0;
			chDemoMode++;
			InitFraktal(2);
		}
		break;

	case 2:	FraktalTest(2);		//rysuj fraktala Mandelbrota - dolina konika
		if (chLiczIter > 20)
		{
			chLiczIter = 0;
			chDemoMode++;
			InitFraktal(3);
		}
		break;

	case 3:	FraktalTest(3);		//rysuj fraktala Mandelbrota - dolina słonia
		if (chLiczIter > 20)
		{
			chLiczIter = 0;
			chDemoMode++;
			InitFraktal(2);
		}
		break;

	case 4:	chDemoMode++;	break;

	case 5:	//LCD_Test();		chMode++;
		InitFraktal(0);
		chDemoMode = 0;	break;
	}
	chLiczIter++;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjalizuj dane fraktali przed uruchomieniem
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InitFraktal(unsigned char chTyp)
{
#define ITERATION	80
//#define IMG_CONSTANT	0.285
//#define REAL_CONSTANT	0.005
//#define IMG_CONSTANT	-0.73
//#define REAL_CONSTANT	0.19
#define IMG_CONSTANT	-0.1
#define REAL_CONSTANT	0.65

	switch (chTyp)
	{
	case 0:	fReal = 0.38; 	fImag = -0.1;	break;	//Julia
	case 1:	fX=-0.70; 	fY=0.60;	fZoom = -0.6;	chMnozPalety = 2;	break;		//cały fraktal - rotacja palety -0.7, 0.6, -0.6,
	case 2:	fX=-0.75; 	fY=0.18;	fZoom = -0.6;	chMnozPalety = 15;	break;		//dolina konika x=-0,75, y=0,1
	case 3:	fX= 0.30; 	fY=0.05;	fZoom = -0.6;	chMnozPalety = 43;	break;		//dolina słonia x=0,25-0,35, y=0,05, zoom=-0,6..-40
	}
	//chMnozPalety = 43;		//8, 13, 21, 30, 34, 43, 48, 56, 61
}



////////////////////////////////////////////////////////////////////////////////
// Generuj fraktal Julii
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void GenerateJulia(unsigned short size_x, unsigned short size_y, unsigned short offset_x, unsigned short offset_y, unsigned short zoom, unsigned short * buffer)
{
	float tmp1, tmp2;
	float num_real, num_img;
	float radius;
	unsigned short i;
	unsigned short x,y;

	for (y=0; y<size_y; y++)
	{
		for (x=0; x<size_x; x++)
		{
			num_real = y - offset_y;
			num_real = num_real / zoom;
			num_img = x - offset_x;
			num_img = num_img / zoom;
			i=0;
			radius = 0;
			while ((i<ITERATION-1) && (radius < 2))
			{
				tmp1 = num_real * num_real;
				tmp2 = num_img * num_img;
				num_img = 2*num_real*num_img + fImag;
				num_real = tmp1 - tmp2 + fReal;
				radius = tmp1 + tmp2;
				i++;
			}
			/* Store the value in the buffer */
			buffer[x+y*size_x] = i*20;
		}
	}
}


#define CONTROL_SIZE_Y	128


////////////////////////////////////////////////////////////////////////////////
// Generuj fraktal Mandelbrota
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void GenerateMandelbrot(float centre_X, float centre_Y, float Zoom, unsigned short IterationMax, unsigned short * buffer)
{
	double X_Min = centre_X - 1.0/Zoom;
	double X_Max = centre_X + 1.0/Zoom;
	double Y_Min = centre_Y - (DISP_Y_SIZE-CONTROL_SIZE_Y) / (DISP_X_SIZE * Zoom);
	double Y_Max = centre_Y + (DISP_Y_SIZE-CONTROL_SIZE_Y) / (DISP_X_SIZE * Zoom);
	double dx = (X_Max - X_Min) / DISP_X_SIZE;
	double dy = (Y_Max - Y_Min) / (DISP_Y_SIZE-CONTROL_SIZE_Y) ;
	double y = Y_Min;
	unsigned short j, i;
	int n;
	double x, Zx, Zy, Zx2, Zy2, Zxy;

	for (j=0; j<DISP_Y_SIZE; j++)
	{
		x = X_Min;
		for (i = 0; i < DISP_X_SIZE; i++)
		{
			Zx = x;
			Zy = y;
			n = 0;
			while (n < IterationMax)
			{
				Zx2 = Zx * Zx;
				Zy2 = Zy * Zy;
				Zxy = 2.0 * Zx * Zy;
				Zx = Zx2 - Zy2 + x;
				Zy = Zxy + y;
				if(Zx2 + Zy2 > 16.0)
				{
					break;
				}
				n++;
			}
			x += dx;

			buffer[i+j*DISP_X_SIZE] = n*chMnozPalety;
		}
		y += dy;
	}
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
// Pobiera stan licznika pracującego na 200MHz/200
// Parametry: brak
// Zwraca: stan licznika w mikrosekundach
////////////////////////////////////////////////////////////////////////////////
uint32_t PobierzCzasT6(void)
{
	extern volatile uint16_t sCzasH;
	return htim6.Instance->CNT + ((uint32_t)sCzasH <<16);
}



////////////////////////////////////////////////////////////////////////////////
// Liczy upływ czasu
// Parametry: nStart - licznik czasu na na początku pomiaru
// Zwraca: ilość czasu w milisekundach jaki upłynął do podanego czasu startu
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas(uint32_t nPoczatek)
{
	uint32_t nCzas, nCzasAkt;

	nCzasAkt = PobierzCzasT6();
	if (nCzasAkt >= nPoczatek)
		nCzas = nCzasAkt - nPoczatek;
	else
		nCzas = 0xFFFFFFFF - nPoczatek + nCzasAkt;
	return nCzas;
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
		setColor(GRAY20);
		fillRect(0, DISP_Y_SIZE - MENU_PASOP_WYS, DISP_X_SIZE, DISP_Y_SIZE);
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
			setColor(GRAY20);
			//fillRect(0, DISP_X_SIZE-MENU_PASOP_WYS, DISP_Y_SIZE, DISP_X_SIZE);		//zamaż pasek podpowiedzi
			fillRect(0, DISP_Y_SIZE-MENU_PASOP_WYS, DISP_X_SIZE, DISP_Y_SIZE);		//zamaż pasek podpowiedzi

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
	setColor(MENU_TLO_BAR);
	fillRect(18, 0, DISP_X_SIZE, MENU_NAG_WYS);
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

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Dane pomiarowe");

		setColor(GRAY80);
		sprintf(chNapis, "Akcel1:");
		print(chNapis, 10, 30);
		sprintf(chNapis, "Akcel2:");
		print(chNapis, 10, 50);
		sprintf(chNapis, "Zyro 1:");
		print(chNapis, 10, 70);
		sprintf(chNapis, "Zyro 2:");
		print(chNapis, 10, 90);
		sprintf(chNapis, "Magn 1:");
		print(chNapis, 10, 110);
		sprintf(chNapis, "Magn 2:");
		print(chNapis, 10, 130);
		sprintf(chNapis, "Magn 3:");
		print(chNapis, 10, 150);
		sprintf(chNapis, "Przech:             Pochyl:          Odchyl:");
		print(chNapis, 10, 170);
		sprintf(chNapis, "Przech:             Pochyl:          Odchyl:");
		print(chNapis, 10, 190);
		sprintf(chNapis, "Ci%cn 1:             AGL1:            Temper:", ś);
		print(chNapis, 10, 210);
		sprintf(chNapis, "Ci%cn 2:             AGL2:            Temper:", ś);
		print(chNapis, 10, 230);
		sprintf(chNapis, "Ci%cR%c%cn 1:          IAS1:            Temper:", ś, ó, ż);
		print(chNapis, 10, 250);
		sprintf(chNapis, "Ci%cR%c%cn 2:          IAS2:            Temper:", ś, ó, ż);
		print(chNapis, 10, 270);

		//sprintf(chNapis, "GNSS D%cug:             Szer:             HDOP:", ł);
		//print(chNapis, 10, 260);
		//sprintf(chNapis, "GNSS WysMSL:           Pred:             Kurs:");
		//print(chNapis, 10, 280);
		//sprintf(chNapis, "GNSS Czas:             Data:              Sat:");
		//print(chNapis, 10, 300);

		setColor(GRAY50);
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		print(chNapis, CENTER, 300);
	}

	//ICM42688
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel1[0]);
	print(chNapis, 10+8*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel1[1]);
	print(chNapis, 10+20*FONT_SL, 30);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel1[2]);
	print(chNapis, 10+32*FONT_SL, 30);

	//LSM6DSV
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel2[0]);
	print(chNapis, 10+8*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel2[1]);
	print(chNapis, 10+20*FONT_SL, 50);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fAkcel2[2]);
	print(chNapis, 10+32*FONT_SL, 50);

	//ICM42688
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal1[0]);
	print(chNapis, 10+8*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal1[1]);
	print(chNapis, 10+20*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal1[2]);
	print(chNapis, 10+32*FONT_SL, 70);
	if (uDaneCM4.dane.nZainicjowano & INIT_ICM42688)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_IMU1] - KELVIN, ZNAK_STOPIEN);	//temepratury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	print(chNapis, 10+45*FONT_SL, 70);

	//LSM6DSV
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal2[0]);
	print(chNapis, 10+8*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal2[1]);
	print(chNapis, 10+20*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyroKal2[2]);
	print(chNapis, 10+32*FONT_SL, 90);
	if (uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_IMU2] - KELVIN, ZNAK_STOPIEN);	//temepratury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	print(chNapis, 10+45*FONT_SL, 90);

	//IIS2MDC
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fMagne1[0]);
	print(chNapis, 10+8*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne1[1]);
	print(chNapis, 10+20*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne1[2]);
	print(chNapis, 10+32*FONT_SL, 110);
	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f%cC ", uDaneCM4.dane.fTemper[TEMP_MAG1] - KELVIN, ZNAK_STOPIEN);	//temepratury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	print(chNapis, 10+45*FONT_SL, 110);

	//MMC34160
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne2[0]);
	print(chNapis, 10+8*FONT_SL, 130);
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne2[1]);
	print(chNapis, 10+20*FONT_SL, 130);
	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne2[2]);
	print(chNapis, 10+32*FONT_SL, 130);

	//HMC5883
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_X); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne3[0]);
	print(chNapis, 10+8*FONT_SL, 150);
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_Y); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne3[1]);
	print(chNapis, 10+20*FONT_SL, 150);
	if (uDaneCM4.dane.nZainicjowano & INIT_HMC5883)	setColor(KOLOR_Z); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.fMagne3[2]);
	print(chNapis, 10+32*FONT_SL, 150);

	//sygnalizacja tonem wartości osi Z magnetometru
	chTon = LICZBA_TONOW_WARIO/2 - (uDaneCM4.dane.fMagne3[2] / (NORM_AMPL_MAG / (LICZBA_TONOW_WARIO/2)));
	if (chTon > LICZBA_TONOW_WARIO)
		chTon = LICZBA_TONOW_WARIO;
	if (chTon < 0)
		chTon = 0;
	//UstawTon(chTon, 80);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro1[0], ZNAK_STOPIEN);
	print(chNapis, 10+8*FONT_SL, 170);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro1[1], ZNAK_STOPIEN);
	print(chNapis, 10+28*FONT_SL, 170);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro1[2], ZNAK_STOPIEN);
	print(chNapis, 10+45*FONT_SL, 170);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro2[0], ZNAK_STOPIEN);
	print(chNapis, 10+8*FONT_SL, 190);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro2[1], ZNAK_STOPIEN);
	print(chNapis, 10+28*FONT_SL, 190);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro2[2], ZNAK_STOPIEN);
	print(chNapis, 10+45*FONT_SL, 190);

	//MS5611
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(WHITE); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnie[0]);
	print(chNapis, 10+8*FONT_SL, 210);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(CYAN); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f m ", uDaneCM4.dane.fWysoko[0]);
	print(chNapis, 10+26*FONT_SL, 210);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS5611)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_BARO1] - KELVIN, ZNAK_STOPIEN);	//temepratury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	print(chNapis, 10+45*FONT_SL, 210);

	//BMP581
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(WHITE); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnie[1]);
	print(chNapis, 10+8*FONT_SL, 230);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(CYAN); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f m ", uDaneCM4.dane.fWysoko[1]);
	print(chNapis, 10+26*FONT_SL, 230);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_BMP851)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_BARO2] - KELVIN, ZNAK_STOPIEN);	//temepratury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	print(chNapis, 10+45*FONT_SL, 230);

	//ND130
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(WHITE); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnRozn[0]);
	print(chNapis, 10+11*FONT_SL, 250);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(MAGENTA); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f m/s ", uDaneCM4.dane.fPredkosc[0]);
	print(chNapis, 10+26*FONT_SL, 250);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_ND140)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_CISR1] - KELVIN, ZNAK_STOPIEN);	//temepratury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	print(chNapis, 10+45*FONT_SL, 250);

	//MS4525
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(WHITE); 	else	setColor(GRAY50);	//stan wyzerowania sygnalizuj kolorem
	sprintf(chNapis, "%.0f Pa ", uDaneCM4.dane.fCisnRozn[1]);
	print(chNapis, 10+11*FONT_SL, 270);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(MAGENTA); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.2f m/s ", uDaneCM4.dane.fPredkosc[1]);
	print(chNapis, 10+26*FONT_SL, 270);
	if (uDaneCM4.dane.nZainicjowano & INIT_P0_MS4525)	setColor(YELLOW); 	else	setColor(GRAY50);
	sprintf(chNapis, "%.1f %cC ", uDaneCM4.dane.fTemper[TEMP_CISR2] - KELVIN , ZNAK_STOPIEN);	//temepratury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
	print(chNapis, 10+45*FONT_SL, 270);


	/*if (uDaneCM4.dane.stGnss1.chFix)
		setColor(WHITE);	//jest fix
	else
		setColor(GRAY70);	//nie ma fixa

	sprintf(chNapis, "%.7f ", uDaneCM4.dane.stGnss1.dDlugoscGeo);
	print(chNapis, 10+11*FONT_SL, 260);
	sprintf(chNapis, "%.7f ", uDaneCM4.dane.stGnss1.dSzerokoscGeo);
	print(chNapis, 10+29*FONT_SL, 260);
	sprintf(chNapis, "%.2f ", uDaneCM4.dane.stGnss1.fHdop);
	print(chNapis, 10+47*FONT_SL, 260);

	sprintf(chNapis, "%.1fm ", uDaneCM4.dane.stGnss1.fWysokoscMSL);
	print(chNapis, 10+13*FONT_SL, 280);
	sprintf(chNapis, "%.3fm/s ", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi);
	print(chNapis, 10+29*FONT_SL, 280);
	sprintf(chNapis, "%3.2f%c ", uDaneCM4.dane.stGnss1.fKurs, ZNAK_STOPIEN);
	print(chNapis, 10+47*FONT_SL, 280);

	sprintf(chNapis, "%02d:%02d:%02d ", uDaneCM4.dane.stGnss1.chGodz, uDaneCM4.dane.stGnss1.chMin, uDaneCM4.dane.stGnss1.chSek);
	print(chNapis, 10+12*FONT_SL, 300);
	if  (uDaneCM4.dane.stGnss1.chMies > 12)	//ograniczenie aby nie pobierało nazwy miesiaca spoza tablicy chNazwyMies3Lit[]
		uDaneCM4.dane.stGnss1.chMies = 0;	//zerowy indeks jest pustą nazwą "---"
	sprintf(chNapis, "%02d %s %04d ", uDaneCM4.dane.stGnss1.chDzien, chNazwyMies3Lit[uDaneCM4.dane.stGnss1.chMies], uDaneCM4.dane.stGnss1.chRok + 2000);
	print(chNapis, 10+29*FONT_SL, 300);
	sprintf(chNapis, "%d ", uDaneCM4.dane.stGnss1.chLiczbaSatelit);
	print(chNapis, 10+47*FONT_SL, 300); */

	//sprintf(chNapis, "Serwa:  9 = %d, 10 = %d, 11 = %d, 12 = %d", uDaneCM4.dane.sSerwa[8], uDaneCM4.dane.sSerwa[9], uDaneCM4.dane.sSerwa[10], uDaneCM4.dane.sSerwa[11]);
	//sprintf(chNapis, "Serwa:  1 = %d,  2 = %d,  3 = %d,  4 = %d", uDaneCM4.dane.sSerwa[0], uDaneCM4.dane.sSerwa[1], uDaneCM4.dane.sSerwa[2], uDaneCM4.dane.sSerwa[3]);
	//sprintf(chNapis, "Serwa: 13 = %d, 14 = %d, 15 = %d, 16 = %d", uDaneCM4.dane.sSerwa[12], uDaneCM4.dane.sSerwa[13], uDaneCM4.dane.sSerwa[14], uDaneCM4.dane.sSerwa[15]);
	//sprintf(chNapis, "Serwa:  5 = %d,  6 = %d,  7 = %d,  8 = %d", uDaneCM4.dane.sSerwa[4], uDaneCM4.dane.sSerwa[5], uDaneCM4.dane.sSerwa[6], uDaneCM4.dane.sSerwa[7]);
	//print(chNapis, 10, 300);

	//Rysuj pasek postepu jeżeli trwa jakiś proces. Zakładam że czas procesu jest zmniejszany od wartości CZAS_KALIBRACJI_ZYROSKOPU do zera
	RysujPasekPostepu(CZAS_KALIBRACJI_ZYROSKOPU);
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
		setColor(BLUE);
		fillRect(0, DISP_Y_SIZE - 5, x , DISP_Y_SIZE);
	}
	setColor(BLACK);
	fillRect(x, DISP_Y_SIZE - 5, DISP_X_SIZE , DISP_Y_SIZE);
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
		print(chNapis, 10, 30);
		sprintf(chNapis, "Czest. 1 harm.:");
		print(chNapis, 10, 50);
		sprintf(chNapis, "Czest. 3 harm.:");
		print(chNapis, 10, 70);
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
		print(chNapis, 10+16*FONT_SL, 30);
		sprintf(chNapis, "%.1f Hz ", 1.0f * CZESTOTLIWOSC_AUDIO / (MIN_OKRES_TONU + chNumerTonu * SKOK_TONU));
		print(chNapis, 10+16*FONT_SL, 50);
		sprintf(chNapis, "%.1f Hz ", 3.0f * CZESTOTLIWOSC_AUDIO / (MIN_OKRES_TONU + chNumerTonu * SKOK_TONU));
		print(chNapis, 10+16*FONT_SL, 70);
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
	extern uint8_t chPorty_exp_wysylane[];
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
		print(chNapis, 10, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Pojemnosc: ");
		print(chNapis, 10, sPozY);
		switch(pCSD.CSDStruct)
		{
		case 0:	sprintf(chNapis, "Standard");	break;
		case 1:	sprintf(chNapis, "Wysoka  ");	break;
		default:sprintf(chNapis, "Nieznana");	break;
		}
		print(chNapis, 10 + 11*FONT_SL, sPozY);

		sPozY += 20;
		//sprintf(chNapis, "Klasy: %ld (0x%lX) ", CardInfo.Class, CardInfo.Class);
		sprintf(chNapis, "CCC: ");
		print(chNapis, 10, sPozY);
		for (uint16_t n=0; n<12; n++)
		{
			if (CardInfo.Class & (1<<n))
				setColor(GRAY80);
			else
				setColor(GRAY40);
			sprintf(chNapis, "%X", n);
			print(chNapis, 10 + ((5+n)*FONT_SL), sPozY);
		}
		sPozY += 20;

		setColor(GRAY70);
		sprintf(chNapis, "Klasa predk: %d ", pStatus.SpeedClass);
		print(chNapis, 10, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Klasa UHS: %d ", pStatus.UhsSpeedGrade);
		print(chNapis, 10, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Klasa Video: %d ", pStatus.VideoSpeedClass);
		print(chNapis, 10, sPozY);
		sPozY += 20;
		sprintf(chNapis, "PerformanceMove: %d MB/s", pStatus.PerformanceMove);
		print(chNapis, 10, sPozY);
		sPozY += 20;

		sprintf(chNapis, "Wsp pred. O/Z: %d ", pCSD.WrSpeedFact);
		print(chNapis, 10, sPozY);
		sPozY += 20;

		if (chPorty_exp_wysylane[0] & EXP02_LOG_VSELECT)	//LOG_SD1_VSEL: H=3,3V
			fNapiecie = 3.3;
		else
			fNapiecie = 1.8;
		sprintf(chNapis, "Napi%ccie I/O: %.1fV ", ę, fNapiecie);
		print(chNapis, 10, sPozY);
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
		print(chNapis, 10, sPozY);
		sPozY += 20;

		sprintf(chNapis, "Czas dost. sync: %d cyk.zeg ", pCSD.NSAC);
		print(chNapis, 10, sPozY);
		sPozY += 20;

		sprintf(chNapis, "MaxBusClkFrec: %d", pCSD.MaxBusClkFrec);
		print(chNapis, 10, sPozY);
		sPozY += 20;


		//druga kolumna
		sPozY = 30;
		sprintf(chNapis, "Liczba sektor%cw: %ld ", ó, CardInfo.BlockNbr);		//Specifies the Card Capacity in blocks
		print(chNapis, 240, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Rozmiar sektora: %ld B ", CardInfo.BlockSize);		//Specifies one block size in bytes
		print(chNapis, 240, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Pojemno%c%c karty: %ld MB ", ś, ć, CardInfo.BlockNbr * (CardInfo.BlockSize / 512) / 2048);		//Specifies one block size in bytes
		print(chNapis, 240, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Rozm Jedn Alok: %d kB", (8<<pStatus.AllocationUnitSize));		//Specifies one block size in bytes
		print(chNapis, 240, sPozY);
		sPozY += 20;
		//sprintf(chNapis, "Liczba blok%cw log.: %ld ", ó, CardInfo.LogBlockNbr);		//Specifies the Card logical Capacity in blocks
		//print(chNapis, 240, sPozY);
		//sPozY += 20;
		sprintf(chNapis, "RdBlockLen: %d ", (2<<pCSD.RdBlockLen));
		print(chNapis, 240, sPozY);
		sPozY += 20;
		//sprintf(chNapis, "Rozmiar karty log: %ld MB ", CardInfo.LogBlockNbr * (CardInfo.LogBlockSize / 512) / 2048);		//Specifies one block size in bytes
		//print(chNapis, 240, sPozY);
		//sPozY += 20;

		setColor(GRAY70);
		sprintf(chNapis, "Format: ");
		print(chNapis, 240, sPozY);
		switch (pCSD.FileFormat)
		{
		case 0: sprintf(chNapis, "HDD z partycja ");	break;
		case 1: sprintf(chNapis, "DOS FAT ");			break;
		case 2: sprintf(chNapis, "UnivFileFormat ");	break;
		default: sprintf(chNapis, "Nieznany ");			break;
		}
		setColor(GRAY80);
		print(chNapis, 240+8*FONT_SL, sPozY);
		sPozY += 20;

		setColor(GRAY70);
		sprintf(chNapis, "Manuf ID: ");
		print(chNapis, 240, sPozY);
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
		print(chNapis, 240+10*FONT_SL, sPozY);
		sPozY += 20;

		setColor(GRAY70);
		chOEM[0] = (pCID.OEM_AppliID & 0xFF00)>>8;
		chOEM[1] = pCID.OEM_AppliID & 0x00FF;
		sprintf(chNapis, "OEM_AppliID: %c%c ", chOEM[0], chOEM[1]);
		print(chNapis, 240, sPozY);
		sPozY += 20;

		sprintf(chNapis, "Nr seryjny: %ld ", pCID.ProdSN);
		print(chNapis, 240, sPozY);
		sPozY += 20;
		sprintf(chNapis, "Data prod: %s %d ", chNazwyMies3Lit[(pCID.ManufactDate & 0xF)], ((pCID.ManufactDate>>4) & 0xFF) + 2000);
		print(chNapis, 240, sPozY);
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
	print(chNapis, 10, sPozY);
	setColor(YELLOW);
	if ((chPorty_exp_odbierane[0] & EXP04_LOG_CARD_DET)	== 0)	//LOG_SD1_CDETECT - wejście detekcji obecności karty
	{
		setColor(KOLOR_Y);
		sprintf(chNapis, "Obecna");
	}
	else
	{
		setColor(KOLOR_X);
		sprintf(chNapis, "Brak! ");
	}
	print(chNapis, 10 + 11*FONT_SL, sPozY);
	sPozY += 20;


	setColor(GRAY80);
	sprintf(chNapis, "Stan FATu: ");
	print(chNapis, 10, sPozY);

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
	print(chNapis, 10 + 11*FONT_SL, sPozY);
	sPozY += 20;

	setColor(GRAY80);
	sprintf(chNapis, "Plik logu: ");
	print(chNapis, 10, sPozY);

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
	print(chNapis, 10 + 11*FONT_SL, sPozY);
	sPozY += 20;

	setColor(GRAY80);
	sprintf(chNapis, "Rejestrator: ");
	print(chNapis, 10, sPozY);
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
	print(chNapis, 10 + 13*FONT_SL, sPozY);
	sPozY += 20;

	setColor(GRAY80);
	sprintf(chNapis, "Zapelnienie: ");
	print(chNapis, 10, sPozY);
	float fZapelnienie = (float)sMaxDlugoscWierszaLogu / ROZMIAR_BUFORA_LOGU;
	if (fZapelnienie < 0.75)
		setColor(KOLOR_Y);	//zielony
	else
	if (fZapelnienie < 0.95)
		setColor(YELLOW);
	else
		setColor(KOLOR_X);	//czerwony
	sprintf(chNapis, "%d/%d ", sMaxDlugoscWierszaLogu, ROZMIAR_BUFORA_LOGU);
	print(chNapis, 10 + 13*FONT_SL, sPozY);
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
// Zwraca: ERR_DONE / ERR_OK - informację o tym czy wyjść z trybu kalibracji czy nie
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
		print(chNapis, 10 + LIBELLA_BOK, 100);
		sprintf(chNapis, "Ca%cka %cyro 2:", ł, ż);
		print(chNapis, 10 + LIBELLA_BOK, 120);
		sprintf(chNapis, "K%cty w p%caszczy%cnie kalibracji", ą, ł, ź);
		print(chNapis, 10 + LIBELLA_BOK, 140);
		sprintf(chNapis, "Pochylenie:");
		print(chNapis, 10 + LIBELLA_BOK, 160);
		sprintf(chNapis, "Przechylenie:");
		print(chNapis, 10 + LIBELLA_BOK, 180);
		sprintf(chNapis, "WspKal %cyro1:", ż);
		print(chNapis, 10 + LIBELLA_BOK, 200);
		sprintf(chNapis, "WspKal %cyro2:", ż);
		print(chNapis, 10 + LIBELLA_BOK, 220);

		setColor(GRAY40);
		stPrzycisk.sX1 = 10 + LIBELLA_BOK;
		stPrzycisk.sY1 = 240;
		stPrzycisk.sX2 = stPrzycisk.sX1 + 210;
		stPrzycisk.sY2 = stPrzycisk.sY1 + 75;
		RysujPrzycisk(stPrzycisk, "Odczyt");

		//fillRect(stPrzycisk.sX1, stPrzycisk.sY1 ,stPrzycisk.sX2, stPrzycisk.sY2);	//rysuj przycisk
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
		fPochylenie = uDaneCM4.dane.fKatIMUAkcel1[1];
		fPrzechylenie = uDaneCM4.dane.fKatIMUAkcel1[0];
		Poziomica(-fPrzechylenie, fPochylenie);	//przechylenie, pochylenie
		setColor(KOLOR_Z);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro1[2], ZNAK_STOPIEN);
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro2[2], ZNAK_STOPIEN);
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;

	case SEKW_KAL_WZM_ZYRO_Q:
		chNazwaOsi = 'Y';
		fPochylenie = atan2f(uDaneCM4.dane.fAkcel1[1], uDaneCM4.dane.fAkcel1[0]) + 90 * DEG2RAD;	//atan(y/x)
		fPrzechylenie = atan2f(uDaneCM4.dane.fAkcel1[1], uDaneCM4.dane.fAkcel1[2]) + 90 * DEG2RAD;	//atan(y/z)
		Poziomica(fPrzechylenie, -fPochylenie);	//przechylenie, pochylenie
		setColor(KOLOR_Y);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro1[1], ZNAK_STOPIEN);
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro2[1], ZNAK_STOPIEN);
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);	//przechylenie
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;

	case SEKW_KAL_WZM_ZYRO_P:
		chNazwaOsi = 'X';
		fPochylenie = atan2f(uDaneCM4.dane.fAkcel1[2], uDaneCM4.dane.fAkcel1[0]);	//atan(z/x)
		fPrzechylenie = atan2f(uDaneCM4.dane.fAkcel1[0], uDaneCM4.dane.fAkcel1[1]) - 90 * DEG2RAD;	//atan(x/y)
		Poziomica(-fPrzechylenie, fPochylenie);	//przechylenie, pochylenie
		setColor(KOLOR_X);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro1[0], ZNAK_STOPIEN);
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 100);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * uDaneCM4.dane.fKatIMUZyro2[0], ZNAK_STOPIEN);
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 120);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPochylenie, ZNAK_STOPIEN);	//pochylenie
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 160);
		sprintf(chNapis, "%.2f %c ", RAD2DEG * fPrzechylenie, ZNAK_STOPIEN);	//przechylenie:
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 180);
		break;
	}

	setColor(YELLOW);
	sprintf(chNapis, "Skieruj osia %c do dolu i wykonaj %d obroty w dowolna strone", chNazwaOsi, OBR_KAL_WZM);
	print(chNapis, CENTER, 30);

	//wyświetl współczynnik kalibracji
	if ((uDaneCM4.dane.nZainicjowano & INIT_WYK_KAL_WZM_ZYRO) && (chEtapKalibracji >= 2))
	{
		setColor(WHITE);										//nowy współczynnik
		sprintf(chNapis, "%.3f", uDaneCM4.dane.fRozne[0]);
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 200);
		sprintf(chNapis, "%.3f", uDaneCM4.dane.fRozne[1]);
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 220);
	}
	else
	{
		setColor(GRAY80);										//stary współczynnik
		sprintf(chNapis, "%.3f", uDaneCM4.dane.fRozne[2]);
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 200);
		sprintf(chNapis, "%.3f", uDaneCM4.dane.fRozne[3]);
		print(chNapis, 10 + LIBELLA_BOK + 14*FONT_SL, 220);
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
		if (uDaneCM4.dane.chOdpowiedzNaPolecenie == ERR_ZLE_WZMOC_ZYRO)
			sprintf(chNapis, "Blad! ");
		else
			sprintf(chNapis, "Gotowe");
		break;

	default:	//wyjście lub przejscie do kalibracji kolejnej osi
		sprintf(chNapis, "Wyjdz ");
		chErr = ERR_DONE;	//zakończ kalibrację
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
			RysujPrzycisk(stPrzycisk, chNapis);
			//setColor(GRAY40);
			//fillRect(stPrzycisk.sX1, stPrzycisk.sY1 ,stPrzycisk.sX2, stPrzycisk.sY2);	//rysuj przycisk
		}
		else
			chErr = ERR_DONE;	//zakończ kabrację gdy nacięnięto poza przyciskiem

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
		setColor(LIBELLA1);
		fillRect(0, DISP_Y_SIZE - LIBELLA_BOK, LIBELLA_BOK, DISP_Y_SIZE);	//wypełnienie płynem
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
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujPrzycisk(prostokat_t prost, char *chNapis)
{
	uint8_t chRozmiar;

	chRozmiar = strlen(chNapis);
	setColor(GRAY40);
	fillRect(prost.sX1, prost.sY1 ,prost.sX2, prost.sY2);	//rysuj przycisk

	setColor(YELLOW);
	setBackColor(GRAY40);	//kolor tła napisu kolorem przycisku
	setFont(BigFont);
	print(chNapis, prost.sX1 + (prost.sX2 - prost.sX1)/2 - chRozmiar*FONT_BL/2 , prost.sY1 + (prost.sY2 - prost.sY1)/2 - FONT_BH/2);
	setBackColor(BLACK);
	setFont(MidFont);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran dla kalibracji zera (offsetu) magnetometru. Dla obrotu wokół X rysuje wykres YZ, dla obrotu wokół Y rysuje XZ dla Z rysuje XY
// Analizuje dane przekazane w zmiennej uDaneCM4.dane.fRozne[6] przez CM4. Funkcja znajjduje i sygnalizuje znalezienia ich maksimów poprzez komunikaty głosowe: 1-minX, 2-maxX, 3-minY, 4-maxY, 5-minZ, 6-maxZ
// Wykres jest skalowany do maksimum wartosci bezwzględnej obu zmiennych
// Parametry: *chEtap	- wskaźnik na zmienną wskazującą na kalibracje konktretnej osi w konkretnym magnetometrze. Starszy półbajt koduje numer magnetometru, najmłodsze 2 bity oś obracaną a bit 4 procedurę kalibracji
// uDaneCM4.dane.fRozne[6] - minima i maksima magnetometrów zbierana przez CM4
// Zwraca: ERR_DONE / ERR_OK - informację o tym czy wyjść z trybu kalibracji czy nie
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibracjaZeraMagnetometru(uint8_t *chEtap)
{
	char chNazwaOsi;
	uint8_t chErr = ERR_OK;
	float fWspSkal;		//współczynnik skalowania wykresu
	float fMag[3];	//dane bieżącego magnetometru
	uint16_t sX, sY;	//bieżace współrzędne na wykresie
	static uint16_t sPopX, sPopY;	//poprzednie współrzędne na wykresie
	static int16_t sMin[3], sMax[3];	//minimum i  maksimum wskazań
	uint16_t sAbsMax;		//maksimum z wartości bezwzględnej minimum i maksimum
	uint8_t chCzujnik;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		if (*chEtap & KALIBRUJ)
			BelkaTytulu("Kalibr. zera magnetometru");
		else
			BelkaTytulu("Weryf. zera magnetometru");
		setColor(GRAY80);
		sprintf(chNapis, "Magnetometr X:");
		print(chNapis, 10, 80);
		sprintf(chNapis, "Magnetometr Y:");
		print(chNapis, 10, 100);
		sprintf(chNapis, "Magnetometr Z:");
		print(chNapis, 10, 120);
		sprintf(chNapis, "Pochylenie:");
		print(chNapis, 10, 140);
		sprintf(chNapis, "Przechylenie:");
		print(chNapis, 10, 160);
		//if (*chEtap & KALIBRUJ)
		{
			sprintf(chNapis, "Ekstrema X:");
			print(chNapis, 10, 180);
			sprintf(chNapis, "Ekstrema Y:");
			print(chNapis, 10, 200);
			sprintf(chNapis, "Ekstrema Z:");
			print(chNapis, 10, 220);
		}

		setColor(GRAY60);
		sprintf(chNapis, "Wci%cnij ekran poza przyciskiem by wyj%c%c", ś, ś, ć);
		print(chNapis, CENTER, 50);

		setColor(GRAY40);
		stPrzycisk.sX1 = 10;
		stPrzycisk.sY1 = 250;
		stPrzycisk.sX2 = stPrzycisk.sX1 + 210;
		stPrzycisk.sY2 = DISP_Y_SIZE;
		RysujPrzycisk(stPrzycisk, "Nastepna Os");

		setColor(GRAY40);
		stWykr.sX1 = DISP_X_SIZE - SZER_WYKR_MAG;
		stWykr.sY1 = DISP_Y_SIZE - SZER_WYKR_MAG;
		stWykr.sX2 = DISP_X_SIZE;
		stWykr.sY2 = DISP_Y_SIZE;
		drawRect(stWykr.sX1, stWykr.sY1, stWykr.sX2, stWykr.sY2);	//rysuj ramkę wykresu
		sPopX = stWykr.sX1 + SZER_WYKR_MAG/2;	//środek wykresu
		sPopY = stWykr.sY1 + SZER_WYKR_MAG/2;
		chStanPrzycisku = 0;
		for (uint16_t n=0; n<3; n++)
		{
			if (*chEtap & KALIBRUJ)
				sMin[n] = sMax[n] = 0;		//podczas kalibracji zbieraj wartosci minimów i maksimów
			else
			{
				sMin[n] = -NORM_AMPL_MAG;
				sMax[n] =  NORM_AMPL_MAG;		//podczas sprawdzenia pracuj na stałym współczynniku
			}
		}
		*chEtap |= ZERUJ;	//przed pomiarem  wystaw potrzebę wyzerowania wskazań magnetometru
		statusDotyku.chFlagi &= ~(DOTYK_ZWOLNONO | DOTYK_DOTKNIETO);	//czyść flagi ekranu dotykowego
	}

	if (*chEtap & ZERUJ)
	{
		uDaneCM7.dane.chWykonajPolecenie = POL_ZERUJ_EKSTREMA;
		DodajProbkeDoMalejKolejki(PRGA_00, ROZM_MALEJ_KOLEJKI_KOMUNIK);
		if ( uDaneCM4.dane.chOdpowiedzNaPolecenie == POL_ZERUJ_EKSTREMA)
		{
			*chEtap &= ~ZERUJ;
			for (uint16_t n=0; n<3; n++)
			{
				sMin[n] = 0;
				sMax[n] = 0;
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
				uDaneCM7.dane.chWykonajPolecenie = POL_SPRAWDZ_MAGN1;
			for (uint16_t n=0; n<3; n++)
				fMag[n] = uDaneCM4.dane.fMagne1[n];
			break;

		case MAG2:
			if (*chEtap & KALIBRUJ)
				uDaneCM7.dane.chWykonajPolecenie = POL_KAL_ZERO_MAGN2;
			else
				uDaneCM7.dane.chWykonajPolecenie = POL_SPRAWDZ_MAGN2;
			for (uint16_t n=0; n<3; n++)
				fMag[n] = uDaneCM4.dane.fMagne2[n];
			break;

		case MAG3:
			if (*chEtap & KALIBRUJ)
				uDaneCM7.dane.chWykonajPolecenie = POL_KAL_ZERO_MAGN3;
			else
				uDaneCM7.dane.chWykonajPolecenie = POL_SPRAWDZ_MAGN3;
			for (uint16_t n=0; n<3; n++)
				fMag[n] = uDaneCM4.dane.fMagne3[n];
			break;
		}
	}

	switch (*chEtap & MASKA_OSI)		//rodzaj osi
	{
	case 0:	chNazwaOsi = 'X';
		if ((int16_t)uDaneCM4.dane.fRozne[2] < sMin[1])	//minimum Y
		{
			sMin[1] = (int16_t)uDaneCM4.dane.fRozne[2];
			DodajProbkeDoMalejKolejki(PRGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
			DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
		}
		else
		if ((int16_t)uDaneCM4.dane.fRozne[3] > sMax[1])	//maksimum Y
		{
			sMax[1] = (int16_t)uDaneCM4.dane.fRozne[3];
			DodajProbkeDoMalejKolejki(PRGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
			DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
		}
		else
		if ((int16_t)uDaneCM4.dane.fRozne[4] < sMin[2])	//minimum Z
		{
			sMin[2] = (int16_t)uDaneCM4.dane.fRozne[4];
			DodajProbkeDoMalejKolejki(PRGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
			DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
		}
		else
		if ((int16_t)uDaneCM4.dane.fRozne[5] > sMax[2])	//maksimum Z
		{
			sMax[2] = (int16_t)uDaneCM4.dane.fRozne[5];
			DodajProbkeDoMalejKolejki(PRGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
			DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
		}

		//rysuj wykres biegunowy Y-Z
		sAbsMax = MaximumGlobalne(sMin, sMax);	//znajdź maksimum globalne wszystkich osi
		if (sAbsMax > MIN_MAG_WYKR)
		{
			setColor(KOLOR_X);
			fWspSkal = (float)(SZER_WYKR_MAG / 2) / sAbsMax;
			sX = (int16_t)(fMag[1] * fWspSkal) + stWykr.sX1 + SZER_WYKR_MAG/2;
			sY = (int16_t)(fMag[2] * fWspSkal) + stWykr.sY1 + SZER_WYKR_MAG/2;
			drawLine(sX, sY, sPopX, sPopY);
			sPopX = sX;
			sPopY = sY;
		}
		break;

	case 1: chNazwaOsi = 'Y';
		if ((int16_t)uDaneCM4.dane.fRozne[0] < sMin[0])	//minimum X
		{
			sMin[0] = (int16_t)uDaneCM4.dane.fRozne[0];
			DodajProbkeDoMalejKolejki(PRGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
			DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
		}
		else
		if ((int16_t)uDaneCM4.dane.fRozne[1] > sMax[0])	//maksimum X
		{
			sMax[0] = (int16_t)uDaneCM4.dane.fRozne[1];
			DodajProbkeDoMalejKolejki(PRGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
			DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
		}
		else
		if ((int16_t)uDaneCM4.dane.fRozne[4] < sMin[2])	//minimum Z
		{
			sMin[2] = (int16_t)uDaneCM4.dane.fRozne[4];
			DodajProbkeDoMalejKolejki(PRGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
			DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
		}
		else
		if ((int16_t)uDaneCM4.dane.fRozne[5] > sMax[2])	//maksimum Z
		{
			sMax[2] = (int16_t)uDaneCM4.dane.fRozne[5];
			DodajProbkeDoMalejKolejki(PRGA_Z, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Z
			DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
		}

		//rysuj wykres biegunowy X-Z
		sAbsMax = MaximumGlobalne(sMin, sMax);	//znajdź maksimum globalne wszystkich osi
		if (sAbsMax > MIN_MAG_WYKR)
		{
   			setColor(KOLOR_Y);
			fWspSkal = (float)(SZER_WYKR_MAG / 2) / sAbsMax;
			sX = (int16_t)(fMag[0] * fWspSkal) + stWykr.sX1 + SZER_WYKR_MAG/2;
			sY = (int16_t)(fMag[2] * fWspSkal) + stWykr.sY1 + SZER_WYKR_MAG/2;
			drawLine(sX, sY, sPopX, sPopY);
			sPopX = sX;
			sPopY = sY;
		}
		break;

	case 2: chNazwaOsi = 'Z';
		if ((int16_t)uDaneCM4.dane.fRozne[0] < sMin[0])	//minimum X
		{
			sMin[0] = (int16_t)uDaneCM4.dane.fRozne[0];
			DodajProbkeDoMalejKolejki(PRGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
			DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
		}
		else
		if ((int16_t)uDaneCM4.dane.fRozne[1] > sMax[0])	//maksimum X
		{
			sMax[0] = (int16_t)uDaneCM4.dane.fRozne[1];
			DodajProbkeDoMalejKolejki(PRGA_X, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//X
			DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
		}
		else
		if ((int16_t)uDaneCM4.dane.fRozne[2] < sMin[1])	//minimum Y
		{
			sMin[1] = (int16_t)uDaneCM4.dane.fRozne[2];
			DodajProbkeDoMalejKolejki(PRGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
			DodajProbkeDoMalejKolejki(PRGA_MIN, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MIN
		}
		else
		if ((int16_t)uDaneCM4.dane.fRozne[3] > sMax[1])	//maksimum Y
		{
			sMax[1] = (int16_t)uDaneCM4.dane.fRozne[3];
			DodajProbkeDoMalejKolejki(PRGA_Y, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//Y
			DodajProbkeDoMalejKolejki(PRGA_MAX, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//MAX
		}

		//rysuj wykres biegunowy X-Y
		sAbsMax = MaximumGlobalne(sMin, sMax);	//znajdź maksimum globalne wszystkich osi
		if (sAbsMax > MIN_MAG_WYKR)
		{
			setColor(KOLOR_Z);
			fWspSkal = (float)(SZER_WYKR_MAG / 2) / sAbsMax;
			sX = (int16_t)(fMag[0] * fWspSkal) + stWykr.sX1 + SZER_WYKR_MAG/2;
			sY = (int16_t)(fMag[1] * fWspSkal) + stWykr.sY1 + SZER_WYKR_MAG/2;
			drawLine(sX, sY, sPopX, sPopY);
			sPopX = sX;
			sPopY = sY;
		}
		break;
	}

	setColor(KOLOR_X);
	sprintf(chNapis, "%.1f ", fMag[0]);
	print(chNapis, 10 + 15*FONT_SL, 80);
	sprintf(chNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU1[0], ZNAK_STOPIEN);
	print(chNapis, 10 + 12*FONT_SL, 140);
	sprintf(chNapis, "%.0f, %.0f ", uDaneCM4.dane.fRozne[0], uDaneCM4.dane.fRozne[1]);
	print(chNapis, 10 + 13*FONT_SL, 180);

	setColor(KOLOR_Y);
	sprintf(chNapis, "%.1f ", fMag[1]);
	print(chNapis, 10 + 15*FONT_SL, 100);
	sprintf(chNapis, "%.2f%c ", RAD2DEG * uDaneCM4.dane.fKatIMU1[1], ZNAK_STOPIEN);
	print(chNapis, 10 + 14*FONT_SL, 160);
	sprintf(chNapis, "%.0f, %.0f ", uDaneCM4.dane.fRozne[2], uDaneCM4.dane.fRozne[3]);
	print(chNapis, 10 + 13*FONT_SL, 200);

	setColor(KOLOR_Z);
	sprintf(chNapis, "%.1f ", fMag[2]);
	print(chNapis, 10 + 15*FONT_SL, 120);
	sprintf(chNapis, "%.0f, %.0f ", uDaneCM4.dane.fRozne[4], uDaneCM4.dane.fRozne[5]);
	print(chNapis, 10 + 13*FONT_SL, 220);

	//sprawdź czy jest naciskany przycisk
	if (statusDotyku.chFlagi & DOTYK_DOTKNIETO)
	{
		//czy naciśnięto na przycisk?
		if ((statusDotyku.sY > stPrzycisk.sY1) && (statusDotyku.sY < stPrzycisk.sY2) && (statusDotyku.sX > stPrzycisk.sX1) && (statusDotyku.sX < stPrzycisk.sX2))
			chStanPrzycisku = 1;
		else
		{
			chErr = ERR_DONE;	//zakończ kabrację gdy nacięnięto poza przyciskiem
			uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//neutralne polecenie kończy szukanie ekstremów w CM4
		}
		statusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;
	}
	else	//DOTYK_DOTKNIETO
	{
		if (chStanPrzycisku)
		{
			(*chEtap)++;
			chStanPrzycisku = 0;
			sPopX = stWykr.sX1 + SZER_WYKR_MAG/2;	//środek wykresu
			sPopY = stWykr.sY1 + SZER_WYKR_MAG/2;
			DodajProbkeDoMalejKolejki(PRGA_PRZYCISK1, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//komunikat audio naciśnięcia przycisku
		}
	}

	chCzujnik = ((*chEtap & MASKA_CZUJNIKA) >> 4) - 1;
	if ((*chEtap & MASKA_OSI) == 3)
		uDaneCM7.dane.chWykonajPolecenie = POL_ZAPISZ_KONF_MAGN1 + chCzujnik;	//zapisz konfigurację bieżącego magnetometru

	//wyjdź dopiero gdy dostanie potwierdzenie zapisu
	if (((*chEtap & MASKA_OSI) == 3) && (uDaneCM4.dane.chOdpowiedzNaPolecenie == POL_ZAPISZ_KONF_MAGN1 + chCzujnik))
	{
		uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//neutralne polecenie kończy szukanie ekstremów w CM4
		chErr = ERR_DONE;		//koniec kalibracji
		DodajProbkeDoMalejKolejki(PRGA_GOTOWE, ROZM_MALEJ_KOLEJKI_KOMUNIK);	//komunikat kończący: Gotowe
	}

	setColor(YELLOW);
	sprintf(chNapis, "Ustaw UAV osi%c %c w kier. Wsch%cd-Zach%cd i obracaj wok%c%c %c", ą, chNazwaOsi, ó, ó, ó, ł, chNazwaOsi);
	print(chNapis, CENTER, 30);
	return chErr;
}


////////////////////////////////////////////////////////////////////////////////
// Znajduje maksimum globalne wszystkich osi
// Parametry: *sMin, *sMax - wskaźniki na minima i maksima wszystkich 3 osi
// Zwraca: globalne maksimum będące wartoscią bezwzgledną wszyskich danych wejściowych
////////////////////////////////////////////////////////////////////////////////
uint16_t MaximumGlobalne(int16_t* sMin, int16_t* sMax)
{
	uint16_t sMaxGlob = 0;

	for (uint16_t n=0; n<3; n++)
	{
		if (sMaxGlob < *(sMax + n))
			sMaxGlob = *(sMax + n);
		if (sMaxGlob < abs(*(sMin + n)))
			sMaxGlob = abs(*(sMin + n));
	}
	return sMaxGlob;
}
