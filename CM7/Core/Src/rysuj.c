//////////////////////////////////////////////////////////////////////////////
//
// Biblioteka funkcju rysujących na ekranie 480x320
// Abstrahuje od sprzętu, może przynajmniej teoretycznie pracować z różnymi wyświetlaczami
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
//#include <RPi35B_480x320.h>
#include "rysuj.h"
#include <string.h>
#include <stdio.h>
#include "dotyk.h"
#include "audio.h"
#include <RPi35B_480x320.h>
#include <ili9488.h>

//deklaracje zmiennych
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
struct current_font cfont;
extern uint8_t MidFont[];
extern uint8_t BigFont[];
extern uint8_t chRysujRaz;
extern char chNapis[];
extern struct _statusDotyku statusDotyku;
extern uint8_t chMenuSelPos, chStarySelPos;	//wybrana pozycja menu i poprzednia pozycja
static uint8_t chOstatniCzas;
uint16_t sBuforLCD[DISP_X_SIZE * DISP_Y_SIZE];
uint8_t chOrientacja;
uint8_t fch, fcl, bch, bcl;	//kolory czcionki i tła (bajt starszy i młodszy)
uint8_t _transparent;	//flaga określająca czy mamy rysować tło czy rysujemy na istniejącym

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
		RysujProstokatWypelniony(0, DISP_Y_SIZE - MENU_PASOP_WYS, DISP_X_SIZE, MENU_PASOP_WYS, GRAY20);
		setBackColor(BLACK);

		//rysuj ikony poleceń
		UstawCzcionke(MidFont);
		for (m=0; m<MENU_WIERSZE; m++)
		{
			for (n=0; n<MENU_KOLUMNY; n++)
			{
				//licz współrzedne środka ikony
				x = (DISP_X_SIZE/(2*MENU_KOLUMNY)) + n * (DISP_X_SIZE/MENU_KOLUMNY);
				y = ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS) / (2*MENU_WIERSZE)) + m * ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS) / MENU_WIERSZE) - MENU_OPIS_WYS + MENU_NAG_WYS;

				setColor(MENU_TLO_NAK);
				RysujBitmape(x-MENU_ICO_WYS/2, y-MENU_ICO_DLG/2, MENU_ICO_DLG, MENU_ICO_WYS, menu[m*MENU_KOLUMNY+n].sIkona);

				setColor(GRAY60);
				x2 = FONT_SLEN * strlen(menu[m*MENU_KOLUMNY+n].chOpis);
				strcpy(chNapis, menu[m*MENU_KOLUMNY+n].chOpis);
				RysujNapis(chNapis, x-x2/2, y+MENU_ICO_WYS/2+MENU_OPIS_WYS);
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
						RysujProstokatZaokraglony(x-MENU_ICO_DLG/2, y-MENU_ICO_WYS/2-2, x+MENU_ICO_DLG/2+2, y+MENU_ICO_WYS/2);
						setColor(GRAY60);
						x2 = FONT_SLEN * strlen(menu[m*MENU_KOLUMNY+n].chOpis);
						strcpy(chNapis, menu[m*MENU_KOLUMNY+n].chOpis);
						RysujNapis(chNapis, x-x2/2, y+MENU_ICO_WYS/2+MENU_OPIS_WYS);
					}
				}
			}
			//setColor(GRAY20);
			//fillRect(0, DISP_X_SIZE-MENU_PASOP_WYS, DISP_Y_SIZE, DISP_X_SIZE);		//zamaż pasek podpowiedzi
			RysujProstokatWypelniony(0, DISP_Y_SIZE-MENU_PASOP_WYS, DISP_X_SIZE, MENU_PASOP_WYS, GRAY20);		//zamaż pasek podpowiedzi

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
					RysujProstokatZaokraglony(x-MENU_ICO_DLG/2, y-MENU_ICO_WYS/2-2, x+MENU_ICO_DLG/2+2, y+MENU_ICO_WYS/2);
					setColor(GRAY80);
					x2 = FONT_SLEN * strlen(menu[m*MENU_KOLUMNY+n].chOpis);
					strcpy(chNapis, menu[m*MENU_KOLUMNY+n].chOpis);
					RysujNapis(chNapis, x-x2/2, y+MENU_ICO_WYS/2+MENU_OPIS_WYS);
				}
			}
		}
		//rysuj pasek podpowiedzi
		if ((chStarySelPos != chMenuSelPos) || chRysujRaz)
		{
			setColor(MENU_RAM_AKT);
			setBackColor(GRAY20);
			strcpy(chNapis, menu[chMenuSelPos].chPomoc);
			RysujNapis(chNapis, DW_SPACE, DISP_Y_SIZE - DW_SPACE - FONT_SH);
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
		RysujNapis(chNapis, DISP_X_SIZE - 9*FONT_SL, DISP_Y_SIZE - DW_SPACE - FONT_SH);
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
	extern const unsigned short pitlab_logo18[];

	RysujProstokatWypelniony(18, 0, DISP_X_SIZE, MENU_NAG_WYS, MENU_TLO_BAR);
	RysujBitmape(0, 0, 18, 18, pitlab_logo18);	//logo PitLab
	setColor(YELLOW);
	setBackColor(MENU_TLO_BAR);
	UstawCzcionke(BigFont);
	RysujNapis(chTytul, CENTER, UP_SPACE);
	setBackColor(BLACK);
	setColor(WHITE);
	UstawCzcionke(MidFont);
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
	RysujBitmape(0, 0, sSzerokosc, sWysokosc, sObraz);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje histogram na dole ekranu
// Parametry: wskaźnik na zmienną z tytułem okna
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujHistogramRGB16(uint8_t *histR, uint8_t *histG, uint8_t *histB)
{
	uint8_t chErr = ERR_OK;
	uint16_t sPoczatek;

	for (uint8_t x=0; x<ROZDZIECZOSC_HISTOGRAMU; x++)
	{
		sPoczatek = x * SZER_PASKA_HISTOGRAMU;
		RysujProstokatWypelniony(sPoczatek, DISP_Y_SIZE - *(histR+x), SZER_PASKA_HISTOGRAMU, *(histR+x), RED);
		sPoczatek = x * SZER_PASKA_HISTOGRAMU + SZER_PASKA_HISTOGRAMU * ROZDZIECZOSC_HISTOGRAMU;
		RysujProstokatWypelniony(sPoczatek, DISP_Y_SIZE - *(histG+x), SZER_PASKA_HISTOGRAMU, *(histG+x), GREEN);
		sPoczatek = x * SZER_PASKA_HISTOGRAMU + (2 * SZER_PASKA_HISTOGRAMU * ROZDZIECZOSC_HISTOGRAMU);
		RysujProstokatWypelniony(sPoczatek, DISP_Y_SIZE - *(histB+x), SZER_PASKA_HISTOGRAMU, *(histB+x), BLUE);
	}
	return chErr;
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
// rysuj prostokąt o współrzędnych x1,y1, x2,y2 we wcześniej zdefiniowanym kolorze
// Parametry: x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujProstokat(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	int nTemp;

	if (x1>x2)
	{
		nTemp = x1;
		x1 = x2;
		x2 = nTemp;
	}
	if (y1>y2)
	{
		nTemp = y1;
		y1 = y2;
		y2 = nTemp;
	}

	RysujLiniePozioma(x1, y1, x2-x1);
	RysujLiniePozioma(x1, y2, x2-x1);
	RysujLiniePionowa(x1, y1, y2-y1);
	RysujLiniePionowa(x2, y1, y2-y1);
}



////////////////////////////////////////////////////////////////////////////////
// rysuj zaokrąglony prostokąt o współprzędnych (x1,y1), (x2,y2) we wcześniej zdefiniowanym kolorze
// Parametry: x, y - współrzędne krawędzi
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujProstokatZaokraglony(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t nTemp;

	if (x1>x2)
	{
		nTemp = x1;
		x1 = x2;
		x2 = nTemp;
	}
	if (y1>y2)
	{
		nTemp = y1;
		y1 = y2;
		y2 = nTemp;
	}
	if ((x2-x1)>4 && (y2-y1)>4)
	{
		drawPixel(x1+1,y1+1);
		drawPixel(x2-1,y1+1);
		drawPixel(x1+1,y2-1);
		drawPixel(x2-1,y2-1);
		RysujLiniePozioma(x1+2, y1, x2-x1-4);
		RysujLiniePozioma(x1+2, y2, x2-x1-4);
		RysujLiniePionowa(x1, y1+2, y2-y1-4);
		RysujLiniePionowa(x2, y1+2, y2-y1-4);
	}
}



////////////////////////////////////////////////////////////////////////////////
// ustawia aktualną czcionkę
// Parametry: *chCzcionka - wskaźnik na tablicę znaków z nagłówkiem identyfikującym czcionkę
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void UstawCzcionke(uint8_t* chCzcionka)
{
	cfont.font = chCzcionka;
	cfont.x_size = *(chCzcionka + 0);
	cfont.y_size = *(chCzcionka + 1);
	cfont.offset = *(chCzcionka + 2);
	cfont.numchars = *(chCzcionka + 3);
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera szerokość aktualnej czcionki
// Parametry: nic
// Zwraca: szerokość znaku
////////////////////////////////////////////////////////////////////////////////
uint8_t GetFontX(void)
{
	return cfont.x_size;
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera wysokość aktualnej czcionki
// Parametry: nic
// Zwraca: wysokość znaku
////////////////////////////////////////////////////////////////////////////////
uint8_t GetFontY(void)
{
	return cfont.y_size;
}



////////////////////////////////////////////////////////////////////////////////
// pisze napis na miejscu o podanych współrzędnych
// Parametry:
// *st - ciąg do wypisania
//  x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujNapis(char *str, uint16_t x, uint16_t y)
{
	int stl;

	stl = strlen((char*)str);

	if (chOrientacja == POZIOMO)
	{
	if (x == RIGHT)
		x = (DISP_X_SIZE+1)-(stl*cfont.x_size);
	if (x == CENTER)
		x = ((DISP_X_SIZE+1)-(stl*cfont.x_size))/2;
	}
	else	//wersja dla pionowego układu ekranu
	{
	if (x == RIGHT)
		x = (DISP_VX_SIZE+1)-(stl*cfont.x_size);
	if (x == CENTER)
		x = ((DISP_VX_SIZE+1)-(stl*cfont.x_size))/2;
	}

	for (uint16_t i=0; i<stl; i++)
		RysujZnak(*str++, x + (i*(cfont.x_size)), y);
}



////////////////////////////////////////////////////////////////////////////////
// pisze zawijający się napis w ramce o podanych współrzędnych
// Parametry:
// *st - ciąg do wypisania
//  x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujNapiswRamce(char *str, uint16_t x, uint16_t y, uint16_t sx, uint16_t sy)
{
	int dlugoscNapisu, dlugoscWiersza;

	dlugoscNapisu = strlen((char*)str);

	do
	{
		if ((dlugoscNapisu * cfont.x_size) > sx)	//czy napis dłuższy niż szerokość ramki
		{
			//znajdź spację czyli miejsce do złamania napisu zaczynając od ostatniego znaku mieszcząceogo się w ramce
			for (uint16_t n = sx / cfont.x_size; n > 0; n--)
			{
				if (*(str+n) == ' ')
				{
					dlugoscWiersza = n;
					break;
				}
			}
		}
		else
			dlugoscWiersza = dlugoscNapisu;


		//if (chOrientacja == POZIOMO)		//na razie obsługuję tylko poziomo
		{
			if (x == RIGHT)
				x = (DISP_X_SIZE - sx + 1) - (dlugoscWiersza * cfont.x_size);
			if (x == CENTER)
				x = ((DISP_X_SIZE - sx) / 2)  + (sx - (dlugoscWiersza * cfont.x_size)) / 2;
		}
		for (uint16_t i=0; i<dlugoscWiersza; i++)
			RysujZnak(*str++, x + (i*(cfont.x_size)), y);

		dlugoscNapisu -= dlugoscWiersza;
		y += cfont.y_size;
	} 	while  (dlugoscNapisu && (y < (y + sy)));
}



////////////////////////////////////////////////////////////////////////////////
// rysuje koło
// Parametry:
//  x, y - współrzdne środka
//  radius - promień
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujKolo(uint16_t x, uint16_t y, uint16_t promien)
{
	int16_t y1, x1;

	for(y1=-promien; y1<=0; y1++)
		for(x1=-promien; x1<=0; x1++)
			if(x1*x1+y1*y1 <= promien*promien)
			{
				RysujLiniePozioma(x+x1, y+y1, 2*(-x1));
				RysujLiniePozioma(x+x1, y-y1, 2*(-x1));
				break;
			}
}
