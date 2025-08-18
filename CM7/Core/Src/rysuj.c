//////////////////////////////////////////////////////////////////////////////
//
// Biblioteka funkcju rysujących na ekranie 480x320
// Abstrahuje od sprzętu, może przynajmniej teoretycznie pracować z różnymi wyświetlaczami
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "rysuj.h"
#include "RPi35B_480x320.h"
#include <string.h>
#include <stdio.h>
#include "dotyk.h"
#include "audio.h"

//deklaracje zmiennych
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern uint8_t MidFont[];
extern uint8_t BigFont[];
extern uint8_t chRysujRaz;
extern char chNapis[];
extern struct _statusDotyku statusDotyku;
extern uint8_t chMenuSelPos, chStarySelPos;	//wybrana pozycja menu i poprzednia pozycja
static uint8_t chOstatniCzas;

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
	extern const unsigned short pitlab_logo18[];

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



////////////////////////////////////////////////////////////////////////////////
// Rysuje histogram na dole ekranu
// Parametry: wskaźnik na zmienną z tytułem okna
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujHistogramRGB16(uint8_t *histR, uint8_t *histG, uint8_t *histB)
{
	uint8_t chErr = ERR_OK;

	for (uint8_t x=0; x<32; x++)
		LCD_ProstokatWypelniony(x*2, DISP_Y_SIZE - *(histR+x), 2, *(histR+x), RED);

	for (uint8_t x=0; x<64; x++)
		LCD_ProstokatWypelniony(x*2+64, DISP_Y_SIZE - *(histG+x), 2, *(histG+x), GREEN);

	for (uint8_t x=0; x<32; x++)
		LCD_ProstokatWypelniony(x*2+192, DISP_Y_SIZE - *(histB+x), 2, *(histB+x), BLUE);
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
