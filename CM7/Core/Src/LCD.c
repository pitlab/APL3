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

//deklaracje zmiennych
extern uint8_t MidFont[];
extern uint8_t BigFont[];
const char *build_date = __DATE__;
const char *build_time = __TIME__;
extern TIM_HandleTypeDef htim6;
extern const unsigned short obr_ppm[];
extern const unsigned short obr_sbus[];
extern const unsigned short obr_multimetr[];
extern const unsigned short obr_multitool[];
extern const unsigned short obr_oscyloskop[];
extern const unsigned short obr_mtest[];
extern const unsigned short obr_calibration[];
extern const unsigned short pitlab_logo18[];

//definicje zmiennych
uint8_t chTrybPracy;
uint8_t chNowyTrybPracy;
uint8_t chWrocDoTrybu;
uint8_t chRysujRaz;
uint8_t chMenuSelPos, chStarySelPos;	//wybrana pozycja menu i poprzednia pozycja
char chNapis[60];
float fZoom, fX, fY;
float fReal, fImag;
unsigned char chMnozPalety;
unsigned char chDemoMode;
unsigned char chLiczIter;		//licznik iteracji fraktala
extern uint16_t sBuforLCD[DISP_X_SIZE * DISP_Y_SIZE];
extern struct _statusDotyku statusDotyku;
extern uint32_t nZainicjowano[2];		//flagi inicjalizacji sprzętu

//Definicje ekranów menu
struct tmenu stMenuGlowne[MENU_WIERSZE * MENU_KOLUMNY]  = {
	//1234567890     1234567890123456789012345678901234567890   TrybPracy			Obrazek
	{"Kamera",  	"Obsluga kamery, aparatu i obrobka obrazu",	TP_KAMERA,	 		obr_ppm},
	{"Fraktale",	"Benchmark fraktalowy"	,					TP_FRAKTALE,		obr_sbus},
	{"Dane IMU",	"Wyniki pomiarow czujnikow IMU",			TP_POMIARY_IMU, 	obr_multimetr},
	{"Zapis NOR", 	"Test zapisu do flash NOR",					TP_MULTITOOL,		obr_calibration},
	{"Trans NOR", 	"Pomiar predkosci flasha NOR 16-bit",		TP_POMIAR_FNOR,		obr_calibration},
	{"Trans QSPI",	"Pomiar predkosci flasha QSPI 4-bit",		TP_POMIAR_FQSPI,	obr_calibration},
	{"Trans SRAM",	"Pomiar predkosci Static RAM 16-bit",		TP_POMIAR_SRAM,		obr_calibration},
	{"Startowy",	"Ekran startowy",							TP_ZDJECIE,			obr_multitool},
	{"TestDotyk",	"Testy panelu dotykowego",					TP_TESTY,			obr_oscyloskop},
	{"Kal Dotyk", 	"Kalibracja panelu dotykowego na LCD",		TP_USTAWIENIA,		obr_mtest}};


uint32_t MinalCzas2(uint32_t nPoczatek);

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
	case TP_MENU_GLOWNE:	MenuGlowne(&chNowyTrybPracy);	break;

	case TP_KAMERA:	break;
	case TP_FRAKTALE:		FraktalDemo();	break;
	case TP_KALIB_DOTYK:
		if (KalibrujDotyk() == ERR_DONE)
			chTrybPracy = TP_TESTY;
		break;

	case TP_MULTITOOL:		Test_Flash();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MENU;
		}
		break;

	case TP_POMIAR_FNOR:	TestPredkosciOdczytuNOR();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MENU;
		}
		break;

	case TP_POMIAR_FQSPI:	W25_TestTransferu();
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MENU;
		}
		break;

	case TP_POMIAR_SRAM:	TestPredkosciOdczytuRAM();
	if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MENU;
		}
		break;

	case TP_ZDJECIE:
		Ekran_Powitalny(nZainicjowano);	//przywitaj użytkownika i prezentuj wykryty sprzęt
		chNowyTrybPracy = TP_WROC_DO_MENU;
		break;

	case TP_TESTY:
		if (TestDotyku() == ERR_DONE)
			chNowyTrybPracy = TP_WROC_DO_MENU;
		break;

	case TP_USTAWIENIA:	break;

	case TP_POMIARY_IMU:	PomiaryIMU();		//wyświetlaj wyniki pomiarów IMU pobrane z CM4
		if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			chTrybPracy = chWrocDoTrybu;
			chNowyTrybPracy = TP_WROC_DO_MENU;
		}
		break;

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
		case TP_WROC_DO_MENU:	chTrybPracy = TP_MENU_GLOWNE;	break;
		case TP_FRAKTALE:		InitFraktal(0);		break;
		case TP_USTAWIENIA:		chTrybPracy = TP_KALIB_DOTYK;	break;
		}

		LCD_clear(BLACK);
	}



	//extern struct _statusDotyku statusDotyku;
	/*LCD_clear(BLACK);
	//LCD_rect(0, 0, DISP_X_SIZE, DISP_Y_SIZE, GRAY60);
	setColor(BLUE);
	drawHLine(10, 10, 470);
	setColor(RED);
	drawVLine(10, 10, 310);*/

	//fillRect(200, 100, 400, 200);

	//LCD_Orient(POZIOMO);

	//setColor(BLUE);
	//drawCircle(300, 200, 50);

	//setFont(MidFont);
	//setColor(GRAY60);
	//sprintf(chNapis, "APL3  SysCLK = %lu MHz", HAL_RCC_GetSysClockFreq()/1000000);
	//sprintf(chNapis, "APL3 v%d.%d.%d @ %s %s SysCLK = %lu MHz", WER_GLOWNA, WER_PODRZ, WER_REPO, build_date, build_time,  HAL_RCC_GetSysClockFreq()/1000000);	//numer wersji w repozytorium i czas kompilacji
	//print(chNapis, 10, 0);
	//sprintf(chNapis, "v%d.%d.%d @ %s %s", WER_GLOWNA, WER_PODRZ, WER_REPO, build_date, build_time);	//numer wersji w repozytorium i czas kompilacji
	//print(chNapis, 10, 20);

	/*setColor(GREEN);
	sprintf(chNapis, "Dotyk: X=%d Y=%d F1=%d F2=%d ", statusDotyku.sAdc[0], statusDotyku.sAdc[1], statusDotyku.sAdc[2], statusDotyku.sAdc[3]);
	print(chNapis, 10, 50);*/

	//

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
	extern volatile unia_wymianyCM4_t uDaneCM4;
	extern const unsigned short plogo165x80[];

	if (chRysujRaz)
	{
		LCD_clear(WHITE);
		drawBitmap((DISP_HX_SIZE-165)/2, 10, 165, 80, plogo165x80);

		setColor(GRAY20);
		setBackColor(WHITE);
		setFont(BigFont);
		sprintf(chNapis, "%s @%luMHz", (char*)chNapisLcd[STR_WITAJ_TYTUL], HAL_RCC_GetSysClockFreq()/1000000);
		print(chNapis, CENTER, 100);

		setColor(GRAY30);
		setFont(MidFont);
		//sprintf(chNapis, (char*)chNapisLcd[STR_WITAJ_KASZANA]);	//"z technologia \"--Kaszana_OFF\""
		sprintf(chNapis, (char*)chNapisLcd[STR_WITAJ_MOTTO], ó, ć, ó, ó, ż);	//"By móc mieć w rój Wronów na pohybel wrażym hordom""
		print(chNapis, CENTER, 120);

		sprintf(chNapis, "(c) PitLab 2024 sv%d.%d.%d @ %s %s", WER_GLOWNA, WER_PODRZ, WER_REPO, build_date, build_time);
		print(chNapis, CENTER, 140);
		chRysujRaz = 0;
	}

	//pierwsza kolumna sprzętu wewnętrznego
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_FLASH_NOR]);		//"pamięć Flash NOR"
	x = 0;
	y = 170;
	print(chNapis, x, y);
	Wykrycie(x, y, n, *(zainicjowano+0) && INIT0_FLASH_NOR);

	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_FLASH_QSPI]);	//"pamięć Flash QSPI"
	y += 20;
	print(chNapis, x, y);
	Wykrycie(x, y, n, *(zainicjowano+0) && INIT0_FLASH_NOR);

	//druga kolumna sprzętu zewnętrznego
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_KAMERA_OV5642]);	//"kamera "
	x = DISP_X_SIZE / 2;
	y = 170;
	print(chNapis, x, y);
	Wykrycie(x, y, n, *(zainicjowano+1) && INIT1_KAMERA);

	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_IMU1]);		//moduł IMU
	y += 20;
	print(chNapis, x, y);
	Wykrycie(x, y, n, *(zainicjowano+1) && INIT1_MOD_IMU);

	//dane z CM4
	n = sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_GNSS]);		//moduł GPS
	y += 20;
	print(chNapis, x, y);
	Wykrycie(x, y, n, uDaneCM4.dane.nZainicjowano && INITCM4_GNSS);

	HAL_Delay(2000);	//czekaj
	LCD_clear(BLACK);
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
		HAL_Delay(50);
	}
	if (wykryto)
		sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_WYKR]);	//"wykryto"
	else
		sprintf(chNapis, (char*)chNapisLcd[STR_SPRAWDZ_BRAK]);	//"brakuje"
	x += kropek * szer_fontu;
	print(chNapis, x , y);
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
	case 1:	fX=-0.70; 	fY=0.60;	fZoom = -0.6;	chMnozPalety = 2;	break;		//ca�y fraktal - rotacja palety -0.7, 0.6, -0.6,
	case 2:	fX=-0.75; 	fY=0.18;	fZoom = -0.6;	chMnozPalety = 15;	break;		//dolina konika x=-0,75, y=0,1
	case 3:	fX= 0.30; 	fY=0.05;	fZoom = -0.6;	chMnozPalety = 43;	break;		//dolina s�onia x=0,25-0,35, y=0,05, zoom=-0,6..-40
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
	//double c;
	double x, Zx, Zy, Zx2, Zy2, Zxy;

	//for (j = 0; j < (DISP_Y_SIZE-CONTROL_SIZE_Y); j++)
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

uint32_t MinalCzas2(uint32_t nPoczatek)
{
	uint32_t nCzas, nCzasAkt;

	nCzasAkt = HAL_GetTick();
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
		fillRect(0, DISP_HY_SIZE-MENU_PASOP_WYS, DISP_HX_SIZE, DISP_HY_SIZE);
		setBackColor(BLACK);

		//rysuj ikony poleceń
		setFont(MidFont);
		for (m=0; m<MENU_WIERSZE; m++)
		{
			for (n=0; n<MENU_KOLUMNY; n++)
			{
				//licz współrzedne środka ikony
				x = (DISP_HX_SIZE/(2*MENU_KOLUMNY)) + n * (DISP_HX_SIZE/MENU_KOLUMNY);
				y = ((DISP_HY_SIZE-MENU_NAG_WYS-MENU_PASOP_WYS)/(2*MENU_WIERSZE)) + m * ((DISP_HY_SIZE-MENU_NAG_WYS-MENU_PASOP_WYS)/MENU_WIERSZE) - MENU_OPIS_WYS + MENU_NAG_WYS;

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

		if (statusDotyku.sY < (DISP_HY_SIZE-MENU_NAG_WYS)/2)	//czy naciśniety górny rząd
			m = 0;
		else	//czy naciśniety dolny rząd
			m = 1;

		for (n=0; n<MENU_KOLUMNY; n++)
		{
			if ((statusDotyku.sX > n*(DISP_HX_SIZE/MENU_KOLUMNY)) && (statusDotyku.sX < (n+1)*(DISP_HX_SIZE/MENU_KOLUMNY)))
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
						x = (DISP_HX_SIZE/(2*MENU_KOLUMNY)) + n * (DISP_HX_SIZE/MENU_KOLUMNY);
						y = ((DISP_HY_SIZE-MENU_NAG_WYS-MENU_PASOP_WYS)/(2*MENU_WIERSZE)) + m * ((DISP_HY_SIZE-MENU_NAG_WYS-MENU_PASOP_WYS)/MENU_WIERSZE) - MENU_OPIS_WYS + MENU_NAG_WYS;
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
			fillRect(0, DISP_HY_SIZE-MENU_PASOP_WYS, DISP_HX_SIZE, DISP_HY_SIZE);		//zamaż pasek podpowiedzi

		}

		//rysuj nową zaznaczoną ramkę
		for (m=0; m<MENU_WIERSZE; m++)
		{
			for (n=0; n<MENU_KOLUMNY; n++)
			{
				if (chMenuSelPos == m*MENU_KOLUMNY+n)
				{
					//licz współrzedne środka ikony
					x = (DISP_HX_SIZE/(2*MENU_KOLUMNY)) + n * (DISP_HX_SIZE/MENU_KOLUMNY);
					y = ((DISP_HY_SIZE-MENU_NAG_WYS-MENU_PASOP_WYS)/(2*MENU_WIERSZE)) + m * ((DISP_HY_SIZE-MENU_NAG_WYS-MENU_PASOP_WYS)/MENU_WIERSZE) - MENU_OPIS_WYS + MENU_NAG_WYS;
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
			print(chNapis, DW_SPACE, DISP_HY_SIZE-DW_SPACE-FONT_SH);
			setBackColor(BLACK);
		}
		chRysujRaz = 0;
	}

	//czy był naciśniety enkoder lub ekran
	if (statusDotyku.chFlagi & DOTYK_DOTKNIETO)
	{
		*tryb = menu[chMenuSelPos].chMode;
		statusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;	//kasuj flagę naciśnięcia ekranu
		return;
	}
	*tryb = 0;
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
	fillRect(18, 0, DISP_HX_SIZE, MENU_NAG_WYS);
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
	extern volatile unia_wymianyCM4_t uDaneCM4;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Dane pomiarowe");

		setColor(GRAY80);
		sprintf(chNapis, "Wysoko%c%c baro1:           baro2:", ś, ć);
		print(chNapis, 10, 30);
		sprintf(chNapis, "Akcel1:");
		print(chNapis, 10, 50);
		sprintf(chNapis, "Akcel2:");
		print(chNapis, 10, 70);
		sprintf(chNapis, "Zyro 1:");
		print(chNapis, 10, 90);
		sprintf(chNapis, "Zyro 2:");
		print(chNapis, 10, 110);
		sprintf(chNapis, "Magn 1:");
		print(chNapis, 10, 130);
		sprintf(chNapis, "Magn 2:");
		print(chNapis, 10, 150);
		sprintf(chNapis, "Przechyl:           Pochyl:          Odchyl:");
		print(chNapis, 10, 170);

		sprintf(chNapis, "GNSS D%cug:             Szer:             Sat:", ł);
		print(chNapis, 10, 240);
		sprintf(chNapis, "GNSS WysMSL:           Pred:           Kurs:");
		print(chNapis, 10, 260);
		sprintf(chNapis, "GNSS Czas:             Data:");
		print(chNapis, 10, 280);

		setColor(GRAY50);
		sprintf(chNapis, "Wdu%c ekran i przytrzymaj aby zako%cczy%c", ś, ń, ć);
		print(chNapis, CENTER, 300);
	}

	setColor(WHITE);
	sprintf(chNapis, "%.2fm ", uDaneCM4.dane.fWysokosc[0]);
	print(chNapis, 10+16*FONT_SL, 30);
	sprintf(chNapis, "%.2fm ", uDaneCM4.dane.fWysokosc[1]);
	print(chNapis, 10+33*FONT_SL, 30);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.3fg ", uDaneCM4.dane.fAkcel1[0]);
	print(chNapis, 10+8*FONT_SL, 50);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.3fg ", uDaneCM4.dane.fAkcel1[1]);
	print(chNapis, 10+20*FONT_SL, 50);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.3fg ", uDaneCM4.dane.fAkcel1[2]);
	print(chNapis, 10+32*FONT_SL, 50);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.3fg ", uDaneCM4.dane.fAkcel2[0]);
	print(chNapis, 10+8*FONT_SL, 70);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.3fg ", uDaneCM4.dane.fAkcel2[1]);
	print(chNapis, 10+20*FONT_SL, 70);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.3fg ", uDaneCM4.dane.fAkcel2[2]);
	print(chNapis, 10+32*FONT_SL, 70);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyro1[0]);
	print(chNapis, 10+8*FONT_SL, 90);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyro1[1]);
	print(chNapis, 10+20*FONT_SL, 90);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyro1[2]);
	print(chNapis, 10+32*FONT_SL, 90);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyro2[0]);
	print(chNapis, 10+8*FONT_SL, 110);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyro2[1]);
	print(chNapis, 10+20*FONT_SL, 110);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fZyro2[2]);
	print(chNapis, 10+32*FONT_SL, 110);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fMagn1[0]);
	print(chNapis, 10+8*FONT_SL, 130);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fMagn1[1]);
	print(chNapis, 10+20*FONT_SL, 130);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fMagn1[2]);
	print(chNapis, 10+32*FONT_SL, 130);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fMagn2[0]);
	print(chNapis, 10+8*FONT_SL, 150);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fMagn2[1]);
	print(chNapis, 10+20*FONT_SL, 150);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.3f ", uDaneCM4.dane.fMagn2[2]);
	print(chNapis, 10+32*FONT_SL, 150);

	setColor(KOLOR_X);
	sprintf(chNapis, "%.3f%c ", uDaneCM4.dane.fKatyIMU[0], ZNAK_STOPIEN);
	print(chNapis, 10+10*FONT_SL, 170);
	setColor(KOLOR_Y);
	sprintf(chNapis, "%.3f%c ", uDaneCM4.dane.fKatyIMU[1], ZNAK_STOPIEN);
	print(chNapis, 10+28*FONT_SL, 170);
	setColor(KOLOR_Z);
	sprintf(chNapis, "%.3f%c ", uDaneCM4.dane.fKatyIMU[2], ZNAK_STOPIEN);
	print(chNapis, 10+45*FONT_SL, 170);

	setColor(WHITE);
	sprintf(chNapis, "%.7f ", uDaneCM4.dane.stGnss1.dDlugoscGeo);
	print(chNapis, 10+11*FONT_SL, 240);
	sprintf(chNapis, "%.7f ", uDaneCM4.dane.stGnss1.dSzerokoscGeo);
	print(chNapis, 10+29*FONT_SL, 240);
	sprintf(chNapis, "%d ", uDaneCM4.dane.stGnss1.chLiczbaSatelit);
	print(chNapis, 10+46*FONT_SL, 240);

	sprintf(chNapis, "%.1fm ", uDaneCM4.dane.stGnss1.fWysokoscMSL);
	print(chNapis, 10+13*FONT_SL, 260);
	sprintf(chNapis, "%.3fm/s ", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi);
	print(chNapis, 10+29*FONT_SL, 260);
	sprintf(chNapis, "%.2f%c ", uDaneCM4.dane.stGnss1.fKurs, ZNAK_STOPIEN);
	print(chNapis, 10+45*FONT_SL, 260);

	sprintf(chNapis, "%02d:%02d:%02d ", uDaneCM4.dane.stGnss1.chGodz, uDaneCM4.dane.stGnss1.chMin, uDaneCM4.dane.stGnss1.chSek);
	print(chNapis, 10+12*FONT_SL, 280);
	sprintf(chNapis, "%02d-%02d-%04d ", uDaneCM4.dane.stGnss1.chDzien, uDaneCM4.dane.stGnss1.chMies, uDaneCM4.dane.stGnss1.chRok + 2000);
	print(chNapis, 10+29*FONT_SL, 280);

	//sprintf(chNapis, "Serwa:  9 = %d, 10 = %d, 11 = %d, 12 = %d", uDaneCM4.dane.sSerwa[8], uDaneCM4.dane.sSerwa[9], uDaneCM4.dane.sSerwa[10], uDaneCM4.dane.sSerwa[11]);
	//sprintf(chNapis, "Serwa:  1 = %d,  2 = %d,  3 = %d,  4 = %d", uDaneCM4.dane.sSerwa[0], uDaneCM4.dane.sSerwa[1], uDaneCM4.dane.sSerwa[2], uDaneCM4.dane.sSerwa[3]);
	//sprintf(chNapis, "Serwa: 13 = %d, 14 = %d, 15 = %d, 16 = %d", uDaneCM4.dane.sSerwa[12], uDaneCM4.dane.sSerwa[13], uDaneCM4.dane.sSerwa[14], uDaneCM4.dane.sSerwa[15]);
	//sprintf(chNapis, "Serwa:  5 = %d,  6 = %d,  7 = %d,  8 = %d", uDaneCM4.dane.sSerwa[4], uDaneCM4.dane.sSerwa[5], uDaneCM4.dane.sSerwa[6], uDaneCM4.dane.sSerwa[7]);
	//print(chNapis, 10, 300);
}
