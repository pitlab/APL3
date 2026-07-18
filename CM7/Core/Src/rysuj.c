//////////////////////////////////////////////////////////////////////////////
//
// Biblioteka funkcju rysujących na ekranie 480x320
// Abstrahuje od sprzętu, może przynajmniej teoretycznie pracować z różnymi wyświetlaczami
// Nie wymaga ochrony semaforem SPI, gdyż używa wyłącznie funkcji chronionych
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
//#include <RPi35B_480x320.h>
#include <Audio.h>
#include <Czas.h>
#include <Dotyk.h>
#include <LCD/ILI9488.h>
#include <string.h>
#include <stdio.h>
#include <LCD/RPi35B_480x320.h>
#include <Rejestrator.h>
#include <Rysuj.h>
#include "AnalizaObrazu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "SampleAudio.h"


//deklaracje zmiennych
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef stTime;
extern RTC_DateTypeDef stDate;
struct current_font cfont;
extern uint8_t MidFont[];
extern uint8_t BigFont[];
extern uint8_t cRysujRaz;
extern char cNapis[];
extern stStatusDotyku_t stStatusDotyku;
uint8_t cMenuSelPos, cStarySelPos;	//wybrana pozycja menu i poprzednia pozycja
static uint8_t cOstatniCzas;
extern volatile uint8_t cStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
extern uint8_t cPort_exp_odbierany[];
uint8_t cStatusPolaczenia;		//każde 2 kolejne bity oznaczają status połaczenia: LPUART, USB, TCP, RTSP
static uint8_t cPoprzedniStatusPolaczenia = 0xFF;	//sluży do wykrycia zmiany statusu
uint8_t cOrientacja;
uint8_t _transparent;	//flaga określająca czy mamy rysować tło czy rysujemy na istniejącym
extern uint8_t cKolor666[3];		//tablica kolorów RGB pierwszego planu w formacie RGB 6-6-6



////////////////////////////////////////////////////////////////////////////////
// Rysuje ekran parametryzowalnego menu
// Parametry:
// [i] *tytul - napis z nazwą menu wyświetlaną w górnym pasku
// [i] *menu - wskaźnik na strukturę menu
// [o] *cPozycjaMenu - wskaźnik na zwracany numer pozycji menu
// Zwraca: kod błądu
////////////////////////////////////////////////////////////////////////////////
uint8_t Menu(char *tytul, menu_t *menu, uint8_t *cPozycjaMenu)
{
	uint8_t cStarySelPos;
	uint8_t chRząd;
	uint16_t x, x2, y;	//pomocnicze współrzędne ekranowe
	uint8_t cBłąd = BLAD_OK;
	uint8_t cStatus;

	if (cRysujRaz)
	{
		cBłąd |= BelkaTytulu(tytul);		//rysuje belkę tytułu ekranu
		RysujProstokatWypelniony(0, MENU_NAG_WYS, DISP_X_SIZE, DISP_Y_SIZE - MENU_PASOP_WYS - MENU_NAG_WYS, CZARNY);	//czyści ekran
		RysujProstokatWypelniony(0, DISP_Y_SIZE - MENU_PASOP_WYS, DISP_X_SIZE, MENU_PASOP_WYS, SZARY20);	//rysuje pasek podpowiedzi na dole ekranu
		setBackColor(CZARNY);

		//rysuj ikony poleceń
		UstawCzcionke(MidFont);
		for (uint8_t m=0; m<MENU_WIERSZE; m++)
		{
			for (uint8_t n=0; n<MENU_KOLUMNY; n++)
			{
				//licz współrzedne środka ikony
				x = (DISP_X_SIZE/(2*MENU_KOLUMNY)) + n * (DISP_X_SIZE/MENU_KOLUMNY);
				y = ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS) / (2*MENU_WIERSZE)) + m * ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS) / MENU_WIERSZE) - MENU_OPIS_WYS + MENU_NAG_WYS;

				setColor(MENU_TLO_NAK);
				RysujBitmape(x-MENU_ICO_WYS/2, y-MENU_ICO_DLG/2, MENU_ICO_DLG, MENU_ICO_WYS, menu[m*MENU_KOLUMNY+n].sIkona);

				setColor(SZARY60);
				x2 = FONT_SLEN * strlen(menu[m*MENU_KOLUMNY+n].chOpis);
				strcpy(cNapis, menu[m*MENU_KOLUMNY+n].chOpis);
				cBłąd |= RysujNapis(cNapis, x-x2/2, y+MENU_ICO_WYS/2+MENU_OPIS_WYS);
			}
		}
		cPoprzedniStatusPolaczenia = 0xFF;	//wymuś przerysowanie statusu połączenia
	}

	//sprawdź czy jest naciskany ekran
	if ((stStatusDotyku.chFlagi & DOTYK_DOTKNIETO) || cRysujRaz)
	{
		cStarySelPos = cMenuSelPos;

		if (stStatusDotyku.sY < (DISP_Y_SIZE - MENU_NAG_WYS)/2)	//czy naciśniety górny rząd
			chRząd = 0;
		else	//czy naciśniety dolny rząd
			chRząd = 1;

		for (uint8_t n=0; n<MENU_KOLUMNY; n++)
		{
			if ((stStatusDotyku.sX > n*(DISP_X_SIZE / MENU_KOLUMNY)) && (stStatusDotyku.sX < (n+1)*(DISP_X_SIZE / MENU_KOLUMNY)))
				cMenuSelPos = chRząd * MENU_KOLUMNY + n;
		}

		if (cStarySelPos != cMenuSelPos)	//zamaż tylko gdy stara ramka jest inna od wybranej
		{
			//zamaż starą ramkę kolorem nieaktywnym
			for (uint8_t m=0; m<MENU_WIERSZE; m++)
			{
				for (uint8_t n=0; n<MENU_KOLUMNY; n++)
				{
					if (cStarySelPos == m*MENU_KOLUMNY+n)
					{
						//licz współrzedne środka ikony
						x = (DISP_X_SIZE /(2*MENU_KOLUMNY)) + n * (DISP_X_SIZE / MENU_KOLUMNY);
						y = ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS) / (2*MENU_WIERSZE)) + m * ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS) / MENU_WIERSZE) - MENU_OPIS_WYS + MENU_NAG_WYS;
						setColor(CZARNY);
						RysujProstokatZaokraglony(x-MENU_ICO_DLG/2, y-MENU_ICO_WYS/2-2, x+MENU_ICO_DLG/2+2, y+MENU_ICO_WYS/2);
						setColor(SZARY60);
						x2 = FONT_SLEN * strlen(menu[m*MENU_KOLUMNY+n].chOpis);
						strcpy(cNapis, menu[m*MENU_KOLUMNY+n].chOpis);
						cBłąd |= RysujNapis(cNapis, x-x2/2, y+MENU_ICO_WYS/2+MENU_OPIS_WYS);
					}
				}
			}
			RysujProstokatWypelniony(0, DISP_Y_SIZE-MENU_PASOP_WYS, DISP_X_SIZE, MENU_PASOP_WYS, SZARY20);		//zamaż pasek podpowiedzi
		}

		//rysuj nową zaznaczoną ramkę
		for (uint8_t m=0; m<MENU_WIERSZE; m++)
		{
			for (uint8_t n=0; n<MENU_KOLUMNY; n++)
			{
				if (cMenuSelPos == m*MENU_KOLUMNY+n)
				{
					//licz współrzedne środka ikony
					x = (DISP_X_SIZE/(2*MENU_KOLUMNY)) + n * (DISP_X_SIZE/MENU_KOLUMNY);
					y = ((DISP_Y_SIZE-MENU_NAG_WYS-MENU_PASOP_WYS)/(2*MENU_WIERSZE)) + m * ((DISP_Y_SIZE - MENU_NAG_WYS - MENU_PASOP_WYS)/MENU_WIERSZE) - MENU_OPIS_WYS + MENU_NAG_WYS;
					if  (stStatusDotyku.chFlagi == DOTYK_DOTKNIETO)	//czy naciśnięty ekran
						setColor(MENU_RAM_WYB);
					else
						setColor(MENU_RAM_AKT);
					RysujProstokatZaokraglony(x-MENU_ICO_DLG/2, y-MENU_ICO_WYS/2-2, x+MENU_ICO_DLG/2+2, y+MENU_ICO_WYS/2);
					setColor(SZARY80);
					x2 = FONT_SLEN * strlen(menu[m*MENU_KOLUMNY+n].chOpis);
					strcpy(cNapis, menu[m*MENU_KOLUMNY+n].chOpis);
					cBłąd |= RysujNapis(cNapis, x-x2/2, y+MENU_ICO_WYS/2+MENU_OPIS_WYS);
				}
			}
		}
		//rysuj pasek podpowiedzi
		if ((cStarySelPos != cMenuSelPos) || cRysujRaz)
		{
			setColor(MENU_RAM_AKT);
			setBackColor(SZARY20);
			strcpy(cNapis, menu[cMenuSelPos].chPomoc);
			cBłąd |= RysujNapis(cNapis, DW_SPACE, DISP_Y_SIZE - 2 * (DW_SPACE - FONT_SH));
			setBackColor(CZARNY);
			cRysujRaz = 0;
		}
	}

	//czy był naciśniety enkoder lub ekran
	if (stStatusDotyku.chFlagi & DOTYK_DOTKNIETO)
	{
		*cPozycjaMenu = menu[cMenuSelPos].chMode;
		stStatusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;	//kasuj flagę naciśnięcia ekranu
		DodajProbkeDoMalejKolejki(PGA_PRZYCISK, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//odtwórz komunikat audio przycisku
		return cBłąd;
	}
	*cPozycjaMenu = 0;

	//rysuj czas
	PobierzDateCzas(&stDate, &stTime);
	if (stTime.Seconds != cOstatniCzas)
	{
		extern uint8_t chStanSynchronizacjiCzasu;
		if (chStanSynchronizacjiCzasu == (SSC_GODZ_SYNCHR + SSC_MIN_SYNCHR + SSC_SEK_SYNCHR + SSC_ROK_SYNCHR + SSC_MIES_SYNCHR + SSC_DZIEN_SYNCHR))
			setColor(BIALY);	//czas i data zsynchroniozwane
		else
		{
			if (chStanSynchronizacjiCzasu == (SSC_GODZ_SYNCHR + SSC_MIN_SYNCHR + SSC_SEK_SYNCHR))
					setColor(SZARY70);	//tylko czas jest zsynchroniozwany
			else
				setColor(SZARY50);	//czas i data niezsynchronizowane
		}

		setBackColor(SZARY20);
		sprintf(cNapis, "%02d:%02d:%02d", stTime.Hours,  stTime.Minutes,  stTime.Seconds);
		cBłąd |= RysujNapis(cNapis, DISP_X_SIZE - 8*FONT_SL, DISP_Y_SIZE - DW_SPACE - FONT_SH);
		cOstatniCzas = stTime.Seconds;
		setBackColor(CZARNY);
	}

	//odśwież status połączenia jeżeli się zmieniło
	setBackColor(SZARY20);
	for (uint8_t n=0; n<4; n++)		//rozpatruj każde połaczenie osobno
	{
		cStatus = (cStatusPolaczenia >> (2*n)) & 0x03;		//wyodrebnij bity danego połączenia
		if (cStatus != ((cPoprzedniStatusPolaczenia >> (2*n)) & 0x03))
		{
			//Połączenie może mieć max 4 stany: 0=brak gotowości do odbioru, 1=gotowe do odbioru, 2=połączone, 3=aktywnie transmituje lub odbiera
			switch (cStatus)
			{
			case 0: setColor(SZARY60);	break;
			case 1: setColor(ZOLTY);	break;
			case 2: setColor(ZIELONY);	break;
			case 3: setColor(CZERWONY);	break;
			}

			//w wybranym kolorze napisz nazwe interfejsu
			switch (n)
			{
			case 0:	cBłąd |= RysujNapis(" USB", DISP_X_SIZE - 27*FONT_SL, DISP_Y_SIZE - DW_SPACE - FONT_SH);	break;	//napis zaczyna się od spacji aby nie zlewało się z poprzednią treścią
			case 1:	cBłąd |= RysujNapis("UART", DISP_X_SIZE - 22*FONT_SL, DISP_Y_SIZE - DW_SPACE - FONT_SH);	break;
			case 2:	cBłąd |= RysujNapis("TCP",  DISP_X_SIZE - 17*FONT_SL, DISP_Y_SIZE - DW_SPACE - FONT_SH);	break;
			case 3:	cBłąd |= RysujNapis("RTSP", DISP_X_SIZE - 13*FONT_SL, DISP_Y_SIZE - DW_SPACE - FONT_SH);	break;
			}
		}
	}
	cPoprzedniStatusPolaczenia = cStatusPolaczenia;

	//status karty
	if ((cPort_exp_odbierany[0] & EXP04_LOG_CARD_DET) == 0)	//LOG_SD1_CDETECT - wejście detekcji obecności karty, aktywny niski
	{
		if (cStatusRejestratora & STATREJ_WLACZONY)
		{
			setColor(ZIELONY);
			cBłąd |= RysujNapis("SD Loguje", 0, DISP_Y_SIZE - DW_SPACE - FONT_SH);
		}
		else
		if (cStatusRejestratora & STATREJ_FAT_GOTOWY)
		{
			setColor(ZOLTY);
			cBłąd |= RysujNapis("SD Gotowe", 0, DISP_Y_SIZE - DW_SPACE - FONT_SH);
		}
		else
		{
			setColor(BLAD);
			cBłąd |= RysujNapis("Blad SD! ", 0, DISP_Y_SIZE - DW_SPACE - FONT_SH);
		}
	}
	else
	{
		setColor(POMARANCZ);
		cBłąd |= RysujNapis("Brak SD! ", 0, DISP_Y_SIZE - DW_SPACE - FONT_SH);
	}

	//wypisz rozmiar sterty
	size_t stosWHM = uxTaskGetStackHighWaterMark(NULL);
	size_t freeHeap = xPortGetFreeHeapSize();
	setColor(CYJAN);
	sprintf(cNapis, " %d/%d ", freeHeap, stosWHM);
	cBłąd |= RysujNapis(cNapis, DISP_X_SIZE - 39*FONT_SL, DISP_Y_SIZE - DW_SPACE - FONT_SH);
	setBackColor(CZARNY);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje belkę menu z logo i tytułem w poziomej orientacji ekranu
// Wychodzi z ustawionym czarnym tłem, białym kolorem i śCZERWONYnia czcionką
// Parametry: wskaźnik na zmienną z tytułem okna
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t BelkaTytulu(char* chTytul)
{
	extern const unsigned short pitlab_logo18[];
	uint8_t cBłąd = BLAD_OK;

	cBłąd |= RysujProstokatWypelniony(18, 0, DISP_X_SIZE, MENU_NAG_WYS, MENU_TLO_BAR);
	cBłąd |= RysujBitmape(0, 0, 18, 18, pitlab_logo18);	//logo PitLab
	setColor(ZOLTY);
	setBackColor(MENU_TLO_BAR);
	UstawCzcionke(BigFont);
	cBłąd |= RysujNapis(chTytul, CENTER, UP_SPACE);
	setColor(BIALY);
	UstawCzcionke(MidFont);
	setBackColor(CZARNY);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wyświetla zawartość 16 bitowego bufora kamery w formacie RGB565
// Parametry:
// [we] sSzerokosc - szerokość obrazu do wyświetlenia
// [we] sWysokosc - wysokość obrazu do wyświetlenia
// [we] *sObraz - wskaźnik na bufor obrazu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t WyswietlZdjecie(uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t* sObraz)
{
	uint8_t cBłąd = BLAD_OK;

	if (sSzerokosc > DISP_X_SIZE)
		sSzerokosc = DISP_X_SIZE;
	if (sWysokosc > DISP_Y_SIZE)
		sWysokosc = DISP_Y_SIZE;
	RysujBitmape(0, 0, sSzerokosc, sWysokosc, sObraz);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wyświetla zawartość 3*8 bitowego bufora kamery w formacie RGB666
// Parametry:
// [we] sSzerokosc - szerokość obrazu do wyświetlenia
// [we] sWysokosc - wysokość obrazu do wyświetlenia
// [we] *sObraz - wskaźnik na bufor obrazu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t WyswietlZdjecieRGB666(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t* chObraz)
{
	uint8_t cBłąd = BLAD_OK;
	//uint32_t nCzas;
	//extern uint32_t nRozmiarObrazuJPEG;
	//extern uint32_t nRozmiarObrazuKamery;

	//nCzas = PobierzCzasT6();
	if (sSzerokosc > DISP_X_SIZE)
		sSzerokosc = DISP_X_SIZE;
	if (sWysokosc > DISP_Y_SIZE)
		sWysokosc = DISP_Y_SIZE;
#ifdef 	LCD_ILI9488
	RysujBitmape888(0, 0, sSzerokosc, sWysokosc, chObraz);
#endif
	//nCzas = MinalCzas(nCzas);
	//sprintf(cNapis, "%.2f fps, kompr: %.1f", 1.0/(nCzas/1000000.0), (float)nRozmiarObrazuKamery / nRozmiarObrazuJPEG);
	//setColor(ZOLTY);
	//RysujNapis(cNapis, 0, DISP_Y_SIZE - FONT_BH);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje histogram obrazu kolorowego RGB565 na dole ekranu
// Parametry: *histR, *histG, *histB - wskaźniki na składowe histogramu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void RysujHistogramRGB32(uint8_t *histR, uint8_t *histG, uint8_t *histB)
{
	uint16_t sPoczatek;	//wskazuje na początek kolejnego histogramu

	for (uint8_t x=0; x<ROZMIAR_HIST_KOLOR; x++)
	{
		sPoczatek = (x * SZER_PASKA_HISTOGRAMU) + (DISP_X_SIZE - (3 * ROZMIAR_HIST_KOLOR * SZER_PASKA_HISTOGRAMU));
		RysujProstokatWypelniony(sPoczatek, DISP_Y_SIZE - *(histR + x), SZER_PASKA_HISTOGRAMU, *(histR + x), CZERWONY);

		sPoczatek = (x * SZER_PASKA_HISTOGRAMU) + (DISP_X_SIZE - (3 * ROZMIAR_HIST_KOLOR * SZER_PASKA_HISTOGRAMU)) + (ROZMIAR_HIST_KOLOR * SZER_PASKA_HISTOGRAMU);
		RysujProstokatWypelniony(sPoczatek, DISP_Y_SIZE - *(histG + x), SZER_PASKA_HISTOGRAMU, *(histG + x), ZIELONY);

		sPoczatek = (x * SZER_PASKA_HISTOGRAMU) + (DISP_X_SIZE - (3 * ROZMIAR_HIST_KOLOR * SZER_PASKA_HISTOGRAMU)) + (2 * ROZMIAR_HIST_KOLOR * SZER_PASKA_HISTOGRAMU);
		RysujProstokatWypelniony(sPoczatek, DISP_Y_SIZE - *(histB + x), SZER_PASKA_HISTOGRAMU, *(histB + x), NIEBIESKI);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje histogram obrazu czarno-białego na dole ekranu wyrównany do prawej
// Parametry: *histCB8 - wskaźnik na histogram
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void RysujHistogramCB8(uint8_t *histCB8)
{
	//uint16_t xprzes;

	for (uint16_t x=0; x<ROZMIAR_HIST_CB8; x++)
	{
		//xprzes = x + DISP_X_SIZE - ROZMIAR_HIST_CB8;
		//RysujProstokatWypelniony(xprzes, DISP_Y_SIZE - *(histCB8 + x), 1, *(histCB8 + x), ZOLTY);
		RysujProstokatWypelniony(x + DISP_X_SIZE - ROZMIAR_HIST_CB8, DISP_Y_SIZE - *(histCB8 + x), 1, *(histCB8 + x), ZOLTY);
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

	if (x1 > x2)
	{
		nTemp = x1;
		x1 = x2;
		x2 = nTemp;
	}
	if (y1 > y2)
	{
		nTemp = y1;
		y1 = y2;
		y2 = nTemp;
	}
	if ((x2-x1) > 4 && (y2-y1) > 4)
	{
		RysujPunkt(x1+1, y1+1, cKolor666);
		RysujPunkt(x2-1, y1+1, cKolor666);
		RysujPunkt(x1+1, y2-1, cKolor666);
		RysujPunkt(x2-1, y2-1, cKolor666);

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
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujNapis(char *str, uint16_t x, uint16_t y)
{
	int stl;
	uint8_t cBłąd = BLAD_OK;;

	stl = strlen((char*)str);

	if (cOrientacja == POZIOMO)
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
		cBłąd |= RysujZnak(*str++, x + (i*(cfont.x_size)), y);
	return cBłąd;
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


		//if (cOrientacja == POZIOMO)		//na razie obsługuję tylko poziomo
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
	{
		for(x1=-promien; x1<=0; x1++)
		{
			if(x1*x1+y1*y1 <= promien*promien)
			{
				RysujLiniePozioma(x+x1, y+y1, 2*(-x1));
				RysujLiniePozioma(x+x1, y-y1, 2*(-x1));
				break;
			}
		}
	}
}
