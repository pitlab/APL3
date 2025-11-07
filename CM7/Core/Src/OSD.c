//////////////////////////////////////////////////////////////////////////////
//
// Obsługa wyświetlania na obrazie cyfrowym
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "osd.h"
#include "display.h"
#include "LCD_mem.h"
#include "cmsis_os.h"
#include <math.h>
#include "czas.h"

uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforOSD[DISP_X_SIZE * DISP_Y_SIZE * ROZMIAR_KOLORU_OSD];	//pamięć obrazu OSD
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforLCD[DISP_X_SIZE * DISP_Y_SIZE * 3];	//pamięć obrazu wyświetlacza w formacie RGB888
extern uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforKamerySRAM[SZER_ZDJECIA * WYS_ZDJECIA / 2];
extern DMA2D_HandleTypeDef hdma2d;
static char chNapisOSD[60];
static uint8_t chTransferDMA2DZakonczony;
uint32_t nFlagiObiektowOsd;	//flagi obecnosci poszczególnych obiektów  na ekranie OSD
stKonfOsd_t stKonfOSD;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern const char *chNazwyMies3Lit[13];
uint16_t LicznikLiniiCzyszczeniaOSD;

////////////////////////////////////////////////////////////////////////////////
// Inicjuje zasoby używane przez OSD
// Parametry:
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujOSD(void)
{
	uint8_t chErr = BLAD_OK;

	//wstępna konfiguracja. Właściwa będzie kiedyś odczytana z pamieci konfiguracji
	stKonfOSD.stHoryzont.sKolorObiektu = KOLOSD_CZER1 + PRZEZR_50;	//czerwony 50% przezroczystości
	//stKonfOSD.stHoryzont.sKolorTla = KOLOSD_CZER3 + PRZEZR_75;
	stKonfOSD.stHoryzont.sPozycjaX = POZ_SRODEK;
	stKonfOSD.stHoryzont.sPozycjaY = POZ_SRODEK;
	stKonfOSD.stHoryzont.chFlagi = FO_WIDOCZNY;

	stKonfOSD.stPredWys.sKolorObiektu = KOLOSD_NIEB1 + PRZEZR_50;	//niebieski 50% przezroczystości
	//stKonfOSD.stPredWys.sKolorTla = KOLOSD_NIEB3 + PRZEZR_75;
	stKonfOSD.stPredWys.chFlagi = FO_WIDOCZNY;

	//współzędne geograficzne
	stKonfOSD.stSzerGeo.sKolorObiektu = KOLOSD_ZIELONY + PRZEZR_50;
	stKonfOSD.stSzerGeo.sKolorTla = KOLOSD_ZIEL3 + PRZEZR_75;
	stKonfOSD.stSzerGeo.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stSzerGeo.sPozycjaX = POZ_LEWO1;
	stKonfOSD.stSzerGeo.sPozycjaY = POZ_DOL1;

	stKonfOSD.stDlugGeo.sKolorObiektu = KOLOSD_ZIEL1 + PRZEZR_50;
	stKonfOSD.stDlugGeo.sKolorTla = KOLOSD_ZOLTY + PRZEZR_75;
	stKonfOSD.stDlugGeo.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stDlugGeo.sPozycjaX = POZ_LEWO1;
	stKonfOSD.stDlugGeo.sPozycjaY = POZ_DOL2;

	stKonfOSD.stData.sKolorObiektu = KOLOSD_ZOLTY + PRZEZR_50;
	stKonfOSD.stData.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stData.sPozycjaX = POZ_PRAWO2;
	stKonfOSD.stData.sPozycjaY = POZ_DOL1;

	stKonfOSD.stCzas.sKolorObiektu = KOLOSD_CYJAN + PRZEZR_50;
	stKonfOSD.stCzas.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stCzas.sPozycjaX = POZ_PRAWO1;
	stKonfOSD.stCzas.sPozycjaY = POZ_DOL1;

	//napięcie baterii
	stKonfOSD.stNapiBat.sKolorObiektu = KOLOSD_CZER1 + PRZEZR_50;
	stKonfOSD.stNapiBat.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stNapiBat.sPozycjaX = POZ_PRAWO3;
	stKonfOSD.stNapiBat.sPozycjaY = POZ_GORA2;

	//prad pobierany z baterii
	stKonfOSD.stPradBat.sKolorObiektu = KOLOSD_CZER2 + PRZEZR_50;
	stKonfOSD.stPradBat.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stPradBat.sPozycjaX = POZ_PRAWO2;
	stKonfOSD.stPradBat.sPozycjaY = POZ_GORA2;

	//energia baterii: pobrana lub pozostała
	stKonfOSD.stEnerBat.sKolorObiektu = KOLOSD_CZER3 + PRZEZR_50;
	stKonfOSD.stEnerBat.chFlagi = FO_WIDOCZNY + FO_E_POBRANA;
	stKonfOSD.stEnerBat.sPozycjaX = POZ_PRAWO1;
	stKonfOSD.stEnerBat.sPozycjaY = POZ_GORA2;


	//obiekty dagnostyczne
	//czas wypełniania tłem
	stKonfOSD.stCzasWyp.sKolorObiektu = KOLOSD_SZARY75 + PRZEZR_25;
	stKonfOSD.stCzasWyp.sKolorTla = KOLOSD_SZARY75 + PRZEZR_75;
	stKonfOSD.stCzasWyp.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stCzasWyp.sPozycjaX = POZ_LEWO1;
	stKonfOSD.stCzasWyp.sPozycjaY = POZ_GORA1;

	stKonfOSD.stCzasRys.sKolorObiektu = KOLOSD_SZARY75 + PRZEZR_75;
	stKonfOSD.stCzasRys.sKolorTla = KOLOSD_SZARY75 + PRZEZR_25;
	stKonfOSD.stCzasRys.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stCzasRys.sPozycjaX = POZ_LEWO1;
	stKonfOSD.stCzasRys.sPozycjaY = POZ_GORA2;

	//czas rysowania horyzontu
	stKonfOSD.stCzasHor.sKolorObiektu = KOLOSD_BIALY + PRZEZR_0;	;
	//stKonfOSD.stCzasHor.sKolorTla = KOLOSD_SZARY75 + PRZEZR_50;
	stKonfOSD.stCzasHor.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stCzasHor.sPozycjaX = POZ_PRAWO4;
	stKonfOSD.stCzasHor.sPozycjaY = POZ_GORA1;

	//czas rysowania wysokości i prędkosci
	stKonfOSD.stCzasPiW.sKolorObiektu = KOLOSD_BIALY + PRZEZR_25;
	//stKonfOSD.stCzasPiW.sKolorTla = KOLOSD_SZARY50 + PRZEZR_50;
	stKonfOSD.stCzasPiW.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stCzasPiW.sPozycjaX = POZ_PRAWO3;
	stKonfOSD.stCzasPiW.sPozycjaY = POZ_GORA1;

	//czas rysowania daty
	stKonfOSD.stCzasDaty.sKolorObiektu = KOLOSD_BIALY + PRZEZR_50;
	//stKonfOSD.stCzasDaty.sKolorTla = PRZEZR_50;
	stKonfOSD.stCzasDaty.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stCzasDaty.sPozycjaX = POZ_PRAWO2;
	stKonfOSD.stCzasDaty.sPozycjaY = POZ_GORA1;

	//czas rysowania pozycji GNSS
	stKonfOSD.stCzasGNSS.sKolorObiektu = KOLOSD_BIALY + PRZEZR_75;
	//stKonfOSD.stCzasGNSS.sKolorTla = KOLOSD_SZARY50 + PRZEZR_50;
	stKonfOSD.stCzasGNSS.chFlagi = FO_WIDOCZNY;
	stKonfOSD.stCzasGNSS.sPozycjaX = POZ_PRAWO1;
	stKonfOSD.stCzasGNSS.sPozycjaY = POZ_GORA1;

	//wypełnij tło w buforze OSD
	//WypelnijEkranwBuforze(DISP_X_SIZE, DISP_Y_SIZE, chBuforOSD, PRZEZR_0); - jest wypełniane przed uruchomieniem z menu

	hdma2d.XferCpltCallback = XferCpltCallback;
	hdma2d.XferErrorCallback = XferErrorCallback;

	hdma2d.Instance = DMA2D;
	hdma2d.Init.Mode = DMA2D_M2M_BLEND;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB888;
	hdma2d.Init.OutputOffset = 0;
	hdma2d.Init.RedBlueSwap = DMA2D_RB_SWAP;
	//1=foreground: OSD
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB4444;
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
	hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
	hdma2d.LayerCfg[1].ChromaSubSampling = DMA2D_NO_CSS;
	//background: kamera
	hdma2d.LayerCfg[0].InputOffset = 0;
	hdma2d.LayerCfg[0].InputColorMode = DMA2D_INPUT_RGB565;
	hdma2d.LayerCfg[0].AlphaMode = DMA2D_REPLACE_ALPHA;
	hdma2d.LayerCfg[0].InputAlpha = 0xFF;		//nieprzezroczysty
	chErr |= HAL_DMA2D_Init(&hdma2d);
	chErr |= HAL_DMA2D_ConfigLayer(&hdma2d, 0);
	chErr |= HAL_DMA2D_ConfigLayer(&hdma2d, 1);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja wykonuje zmieszanie zawartosci bufora OSD z obrazem z kamery
// Parametry:
//	*chObrazFront - adres bufora foreground
//	*chObrazTlo - adre bufora backbground
//	*chBuforWyjsciowy - adres bufora wyjściowego
//	sSzerokosc, sWysokosc - rozmiary obrazu
// Zwraca: kod błędu
// Czas operacji 16,5ms dla obrazu 480x320
////////////////////////////////////////////////////////////////////////////////
uint8_t PolaczBuforOSDzObrazem(uint8_t *chObrazFront, uint8_t *chObrazTlo, uint8_t *chBuforWyjsciowy, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	uint8_t chErr;
	uint8_t chTimeout = 100;
	uint32_t nErr;

	if (hdma2d.State == HAL_DMA2D_STATE_BUSY)
		HAL_DMA2D_Abort(&hdma2d);
	chTransferDMA2DZakonczony = 0;
	chErr = HAL_DMA2D_BlendingStart_IT(&hdma2d, (uint32_t)chObrazFront, (uint32_t)chObrazTlo, (uint32_t)chBuforWyjsciowy, (uint32_t)sSzerokosc, (uint32_t)sWysokosc);
	do
	{
		osDelay(1);
		chTimeout--;
	}
	while (!chTransferDMA2DZakonczony && chTimeout);
	if (chTimeout == 0)
	{
		//chErr = BLAD_TIMEOUT;
		nErr = HAL_DMA2D_GetError(&hdma2d);
		chErr = nErr & 0xFF;
	}

	return chErr;
}



//callback for transfer complete.
void XferCpltCallback(DMA2D_HandleTypeDef *hdma2d)
{
	chTransferDMA2DZakonczony = 1;
}


//callback for transfer error.
void XferErrorCallback(DMA2D_HandleTypeDef *hdma2d)
{
	chTransferDMA2DZakonczony = 2;
}


//callback for line event.
void LineEventCallback(struct __DMA2D_HandleTypeDef *hdma2d)
{
	chTransferDMA2DZakonczony = 0;
}

//callback for CLUT loading completion.
void CLUTLoadingCpltCallback(struct __DMA2D_HandleTypeDef *hdma2d)
{

}



////////////////////////////////////////////////////////////////////////////////
// Funkcja rysuje w buforze pamieci elementy ekranu OSD używajac kolorów ARGB4444
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujTestoweOSD(void)
{
	uint16_t sKolor, sTlo, sBrakTla;

	//sKolor = 0x0000;
	//WypelnijEkranwBuforze1(chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
	WypelnijEkranwBuforze(DISP_X_SIZE, DISP_Y_SIZE, chBuforOSD, KOLOSD_CZARNY + PRZEZR_0);

	sBrakTla = 0;
	sprintf(chNapisOSD, "Abc");

	//zwiększaj przezroczystość w połnej skali 16 kroków
	for (uint8_t n=0; n<16; n++)
	{
		sKolor = 0x0F00 | n << 12;	//czerwony
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 0, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
		sKolor = 0x00F0 | n << 12;	//zielony
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 20, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
		sKolor = 0x000F | n << 12;	//niebieski
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 40, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
		sKolor = 0x00FF | n << 12;	//cyjan
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 60, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
		sKolor = 0x0F0F | n << 12;	//magenta
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 80, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
		sKolor = 0x0FF0 | n << 12;	//żółty
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 100, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

		sKolor = 0x7F0F;	//magenta
		sTlo = 0x0999 | n << 12;	//szary
		RysujNapiswBuforze(chNapisOSD, n * DISP_X_SIZE/16, 120, DISP_X_SIZE, chBuforOSD, (uint8_t*)&sKolor, (uint8_t*)&sTlo, ROZMIAR_KOLORU_OSD);
		sKolor = 0x0F0F | n << 12;	//magenta
		RysujNapiswBuforze(chNapisOSD, n * DISP_X_SIZE/16, 140, DISP_X_SIZE, chBuforOSD, (uint8_t*)&sKolor, (uint8_t*)&sBrakTla, ROZMIAR_KOLORU_OSD);
	}

	//podwójna linia ukośna
	sKolor = 0x70F0;	//zielony
	RysujLiniewBuforze(20, 100, 400, 300, DISP_X_SIZE, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
	sKolor = 0x7F0F;	//przeciwstawny do zielonego
	RysujLiniewBuforze(21, 101, 401, 301, DISP_X_SIZE, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

	sKolor = 0x7F00;	//czerwony
	RysujLiniePoziomawBuforze(20, 400, 160, DISP_X_SIZE, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
	sKolor = 0x700F;	//niebieski
	RysujLiniePionowawBuforze(20, 160, 300, DISP_X_SIZE,  chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

	//podwójny okrąg o przeciwstawnych kolorach
	sKolor = 0x7707;	//fioletowy
	RysujOkragwBuforze(240, 180, 80, DISP_X_SIZE, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
	sKolor = 0x77F7;	//
	RysujOkragwBuforze(240, 180, 81, DISP_X_SIZE, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja rysuje w buforze pamieci elementy ekranu OSD używajac kolorów ARGB4444
// Parametry:
//	sSzerokosc, sWysokosc - rozmiary obrazu OSD
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujOSD(stKonfOsd_t *stKonf, volatile stWymianyCM4_t *stDane)
{
	uint16_t sKolor;
	int16_t x, y, x1, y1, x2, y2;	//współrzędne

	uint32_t nCykleStartGlob, nCykleStartLok, nCykleStop;
	prostokat_t stWspXY;	//współrzędne ekranowe
	uint8_t chZnak;

	StartPomiaruCykli();	//włącza licznik pomiaru cykli
	nCykleStartGlob = DWT->CYCCNT;	//globalnego czas początku pomiaru
	//WypelnijEkranwBuforze(stKonf->sSzerokosc, stKonf->sWysokosc, chBuforOSD, PRZEZR_100);

	//systematycznie zamazuj po jednej linii ekranu aby czyścić ewentualne śmieci
	sKolor = PRZEZR_100;
	RysujLiniePoziomawBuforze(0, stKonf->sSzerokosc, LicznikLiniiCzyszczeniaOSD, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
	LicznikLiniiCzyszczeniaOSD++;
	if (LicznikLiniiCzyszczeniaOSD >= stKonf->sWysokosc)
		LicznikLiniiCzyszczeniaOSD = 0;

	//Czas wypełniania ekranu tłem
	if (stKonfOSD.stCzasWyp.chFlagi & FO_WIDOCZNY)
	{
		nCykleStop = DWT->CYCCNT;
		nCykleStop -= nCykleStartGlob;
		sprintf(chNapisOSD, "Twyp %ld ", nCykleStop / (HAL_RCC_GetSysClockFreq()/1000000));
		PobierzPozycjeObiektu(&stKonf->stCzasWyp, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stCzasWyp.sKolorObiektu, (uint8_t*)&stKonf->stCzasWyp.sKolorTla, ROZMIAR_KOLORU_OSD);
	}

	nCykleStartLok = DWT->CYCCNT;

	if (stKonf->stHoryzont.chFlagi & FO_WIDOCZNY)
	{
		//rysuj poprzedni horyzont kolorem przezroczystym
		RysujHoryzont(stKonf, stKonf->stHoryzont.fPoprzWartosc1, stKonf->stHoryzont.fPoprzWartosc2, PRZEZR_100);

		//rysuj bieżący horyzont właściwym kolorem
		RysujHoryzont(stKonf, stDane->fKatIMU1[POCH], stDane->fKatIMU1[PRZE], stKonf->stHoryzont.sKolorObiektu);
	}

	//czas rysowania horyzontu
	if (stKonfOSD.stCzasHor.chFlagi & FO_WIDOCZNY)
	{
		nCykleStop = DWT->CYCCNT;
		nCykleStop -= nCykleStartLok;
		sprintf(chNapisOSD, "Thor %ld ", nCykleStop / (HAL_RCC_GetSysClockFreq()/1000000));
		PobierzPozycjeObiektu(&stKonf->stCzasHor, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stCzasHor.sKolorObiektu, (uint8_t*)&stKonf->stCzasHor.sKolorTla, ROZMIAR_KOLORU_OSD);
	}

	nCykleStartLok = DWT->CYCCNT;
	//rysuj wskaźniki prędkości po lewej i wysokości po prawej wygledem środka ekranu
	if (stKonf->stPredWys.chFlagi & FO_WIDOCZNY)
	{
		sKolor = stKonf->stPredWys.sKolorObiektu;

		//linia skali prędkosci
		x = stKonf->sSzerokosc * PIW_SZER_PROC / 100;
		y1 = stKonf->sWysokosc * (100 - PIW_WYS_PROC) / 200;
		y2 = stKonf->sWysokosc - y1;
		RysujLiniewBuforze(x, y1, x, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
		sprintf(chNapisOSD, "%.3d", (uint16_t)(stDane->fPredkosc[0]));

		x1 = x - 5 * OSD_CZCION_SZER;
		y = stKonf->sWysokosc / 2 - OSD_CZCION_WYS / 2;
		RysujNapiswBuforze(chNapisOSD, x1, y-PIW_KOR_WYS_CZC, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, (uint8_t*)&stKonf->stPredWys.sKolorTla, ROZMIAR_KOLORU_OSD);

		//rysuj strzałkę dookoła pomiaru prędkości
		x1 = x - PIW_DYST_NAPISU - PIW_ZNAK_STRZAL * OSD_CZCION_SZER - PIW_ODL_RAMKI;
		x2 = x - PIW_DYST_NAPISU;
		y1 = stKonf->sWysokosc / 2 - OSD_CZCION_WYS / 2 - PIW_ODL_RAMKI;
		y2 = stKonf->sWysokosc / 2 + OSD_CZCION_WYS / 2 + PIW_ODL_RAMKI;
		RysujLiniePoziomawBuforze(x1, x2, y1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);					//góra
		RysujLiniePoziomawBuforze(x1, x2, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);					//dół
		RysujLiniewBuforze(x2, y1, x, stKonf->sWysokosc / 2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);	//ostrze góra
		RysujLiniewBuforze(x2, y2, x, stKonf->sWysokosc / 2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);	//ostrze dół
		RysujLiniePionowawBuforze(x1, y1, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);					//tył strzałki

		//linia skali wysokości
		x = stKonf->sSzerokosc - x;
		y1 = stKonf->sWysokosc * (100 - PIW_WYS_PROC) / 200;
		y2 = stKonf->sWysokosc - y1;
		RysujLiniewBuforze(x, y1, x, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

		sprintf(chNapisOSD, "%.3d", (uint16_t)(stDane->fWysokoMSL[0]));
		x1 = x + 2 * OSD_CZCION_SZER;
		RysujNapiswBuforze(chNapisOSD, x1, y-PIW_KOR_WYS_CZC, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, (uint8_t*)&stKonf->stPredWys.sKolorTla, ROZMIAR_KOLORU_OSD);

		//rysuj strzałkę dookoła pomiaru wysokości
		//x1 = x + 2 * OSD_CZCION_SZER;
		//x2 = x + 6 * OSD_CZCION_SZER + PIW_ODL_RAMKI;
		x1 = x + PIW_DYST_NAPISU;
		x2 = x + PIW_DYST_NAPISU + PIW_ZNAK_STRZAL * OSD_CZCION_SZER + PIW_ODL_RAMKI;
		y1 = stKonf->sWysokosc / 2 - OSD_CZCION_WYS / 2 - PIW_ODL_RAMKI;
		y2 = stKonf->sWysokosc / 2 + OSD_CZCION_WYS / 2 + PIW_ODL_RAMKI;
		RysujLiniePoziomawBuforze(x1, x2, y1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);					//góra
		RysujLiniePoziomawBuforze(x1, x2, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);					//dół
		RysujLiniewBuforze(x, stKonf->sWysokosc / 2, x1, y1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);	//ostrze góra
		RysujLiniewBuforze(x, stKonf->sWysokosc / 2, x1, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);	//ostrze dół
		RysujLiniePionowawBuforze(x2, y1, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);					//tył strzałki
	}

	//czas rysowania prędkosci i wysokości
	if (stKonfOSD.stCzasPiW.chFlagi & FO_WIDOCZNY)
	{
		nCykleStop = DWT->CYCCNT;
		nCykleStop -= nCykleStartLok;
		sprintf(chNapisOSD, "Tpiw %ld ", nCykleStop / (HAL_RCC_GetSysClockFreq()/1000000));
		PobierzPozycjeObiektu(&stKonf->stCzasPiW, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stCzasPiW.sKolorObiektu, (uint8_t*)&stKonf->stCzasPiW.sKolorTla, ROZMIAR_KOLORU_OSD);
	}

	//szerokość geograficzna
	if (stKonf->stSzerGeo.chFlagi & FO_WIDOCZNY)
	{
		if (stDane->stGnss1.dSzerokoscGeo >= 0.0)
			chZnak = 'N';
		else
			chZnak = 'S';
		sprintf(chNapisOSD, "%.6f%c", stDane->stGnss1.dSzerokoscGeo, chZnak);
		PobierzPozycjeObiektu(&stKonf->stSzerGeo, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stSzerGeo.sKolorObiektu, (uint8_t*)&stKonf->stSzerGeo.sKolorTla, ROZMIAR_KOLORU_OSD);
	}

	nCykleStartLok = DWT->CYCCNT;
	//długość geograficzna
	if (stKonf->stDlugGeo.chFlagi & FO_WIDOCZNY)
	{
		if (stDane->stGnss1.dDlugoscGeo >= 0.0)
			chZnak = 'E';
		else
			chZnak = 'W';
		sprintf(chNapisOSD, "%.6f%c", stDane->stGnss1.dDlugoscGeo, chZnak);
		PobierzPozycjeObiektu(&stKonf->stDlugGeo, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stDlugGeo.sKolorObiektu, (uint8_t*)&stKonf->stDlugGeo.sKolorTla, ROZMIAR_KOLORU_OSD);
	}


	//czas rysowania pozycji GNSS
	if (stKonfOSD.stCzasGNSS.chFlagi & FO_WIDOCZNY)
	{
		nCykleStop = DWT->CYCCNT;
		nCykleStop -= nCykleStartLok;
		sprintf(chNapisOSD, "Tgnss %ld ", nCykleStop / (HAL_RCC_GetSysClockFreq()/1000000));
		PobierzPozycjeObiektu(&stKonf->stCzasGNSS, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stCzasGNSS.sKolorObiektu, (uint8_t*)&stKonf->stCzasGNSS.sKolorTla, ROZMIAR_KOLORU_OSD);
	}

	nCykleStartLok = DWT->CYCCNT;
	//datę i czas trzeba pobrać razem, nawet jeżeli tylko jeden składnik jest potrzebny
	if ((stKonf->stData.chFlagi & FO_WIDOCZNY) || (stKonf->stCzas.chFlagi & FO_WIDOCZNY))
		PobierzDateCzas(&sDate, &sTime);

	//bieżąca data
	if (stKonf->stData.chFlagi & FO_WIDOCZNY)
	{
		if (sDate.Month <= 12)	//zabezpieczenie przed
			sprintf(chNapisOSD, "%2d %s %.2d", sDate.Date, chNazwyMies3Lit[sDate.Month], sDate.Year);
		else
			sprintf(chNapisOSD, "-- --- --");
		PobierzPozycjeObiektu(&stKonf->stData, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stData.sKolorObiektu, (uint8_t*)&stKonf->stData.sKolorTla, ROZMIAR_KOLORU_OSD);
	}

	//czas rysowania daty
	if (stKonfOSD.stCzasDaty.chFlagi & FO_WIDOCZNY)
	{
		nCykleStop = DWT->CYCCNT;
		nCykleStop -= nCykleStartLok;
		sprintf(chNapisOSD, "Tdat %ld ", nCykleStop / (HAL_RCC_GetSysClockFreq()/1000000));
		PobierzPozycjeObiektu(&stKonf->stCzasDaty, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stCzasDaty.sKolorObiektu, (uint8_t*)&stKonf->stCzasDaty.sKolorTla, ROZMIAR_KOLORU_OSD);
	}

	//bieżacy czas
	if (stKonf->stCzas.chFlagi & FO_WIDOCZNY)
	{
		sprintf(chNapisOSD, "%2d:%.2d:%.2d", sTime.Hours, sTime.Minutes, sTime.Seconds);
		PobierzPozycjeObiektu(&stKonf->stCzas, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stCzas.sKolorObiektu, (uint8_t*)&stKonf->stCzas.sKolorTla, ROZMIAR_KOLORU_OSD);
	}


	//napięcie baterii
	if (stKonfOSD.stNapiBat.chFlagi & FO_WIDOCZNY)
	{
		sprintf(chNapisOSD, "Ub %.1fV ", stDane->fTemper[0]);	//Dodać zmienną napiecia
		PobierzPozycjeObiektu(&stKonf->stNapiBat, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stNapiBat.sKolorObiektu, (uint8_t*)&stKonf->stNapiBat.sKolorTla, ROZMIAR_KOLORU_OSD);
	}

	//prad pobierany z baterii
	if (stKonfOSD.stPradBat.chFlagi & FO_WIDOCZNY)
	{
		sprintf(chNapisOSD, "Ib %.1fA ", stDane->fTemper[3]);	//Dodać zmienną pradu
		PobierzPozycjeObiektu(&stKonf->stPradBat, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stPradBat.sKolorObiektu, (uint8_t*)&stKonf->stPradBat.sKolorTla, ROZMIAR_KOLORU_OSD);
	}

	//energia baterii: pobrana lub pozostała
	if (stKonfOSD.stEnerBat.chFlagi & FO_WIDOCZNY)
	{
		uint8_t chSymbol;
		if (stKonfOSD.stEnerBat.chFlagi & FO_E_POBRANA)
			chSymbol = 'p';	//energia pobrana
		else
			chSymbol = 'z';	//energia zostająca w pakiecie
		sprintf(chNapisOSD, "E%c %ldmAh ", chSymbol, (uint32_t)stDane->fTemper[2]);	//Dodać zmienną energii
		PobierzPozycjeObiektu(&stKonf->stEnerBat, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stEnerBat.sKolorObiektu, (uint8_t*)&stKonf->stEnerBat.sKolorTla, ROZMIAR_KOLORU_OSD);
	}

	//całkowity czas przygotowania ekranu OSD
	if (stKonfOSD.stCzasRys.chFlagi & FO_WIDOCZNY)
	{
		nCykleStop = DWT->CYCCNT;
		nCykleStop -= nCykleStartGlob;
		sprintf(chNapisOSD, "Trys %ld ", nCykleStop / (HAL_RCC_GetSysClockFreq()/1000000));
		PobierzPozycjeObiektu(&stKonf->stCzasRys, stKonf, &stWspXY);
		RysujNapiswBuforze(chNapisOSD, stWspXY.sX1, stWspXY.sY1, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&stKonf->stCzasRys.sKolorObiektu, (uint8_t*)&stKonf->stCzasRys.sKolorTla, ROZMIAR_KOLORU_OSD);
	}
	StopPrintfCykle();		//drukuje na konsoli liczbę wykonanych cykli
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje kontrolkę sztucznego horyzontu jako belkę o długości 50% szerokosci ekranu w zerze pochylenia i krótsze belki 40% szerokości ekranu co 10°
// Parametry:
//	*stKonf - wskaźnik na konfigurację OSD
//	fPochylenie, fPrzechylenie - bieżące pochylenie i przechylenie
//	sKolor - kolor obiektu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujHoryzont(stKonfOsd_t *stKonf, float fPochylenie, float fPrzechylenie, uint16_t sKolor)
{
	int16_t sXprzechyl, sYprzechyl;	//korekta położenia końców belki wynikająca z przechylenia
	int16_t sXSrodkaBelki, sYSrodkaBelki;
	int16_t x1, y1, x2, y2;	//współrzędne

	//zachowaj wartości bieżące na nastepne rysowanie
	stKonf->stHoryzont.fPoprzWartosc1 = fPochylenie;
	stKonf->stHoryzont.fPoprzWartosc2 = fPrzechylenie;

	float fSinPrze = sin(fPrzechylenie);
	float fCosPrze = cos(fPrzechylenie);

	uint16_t sDlugoscPolowyBelki = stKonf->sSzerokosc * HOR_SZER00_PROC / 200;
	//licz współrzędne środka prawego końca belki zerowego kąta względem środka ekranu
	sXprzechyl = (int16_t)(sDlugoscPolowyBelki * fCosPrze);
	sYprzechyl = (int16_t)(sDlugoscPolowyBelki * fSinPrze);

	uint16_t sWysokoscPolowyOkna = stKonf->sWysokosc * HOR_WYS_PROC / 200;
	int16_t sWysPochylenia = (int16_t)(sWysokoscPolowyOkna * fPochylenie / Deg2Rad(HOR_SKALA_POCH));	//przesunięcie w pionie środka horyzontu w zależnosci od pochylenia

	x1 = x2 = stKonf->sSzerokosc / 2;
	x1 -= sXprzechyl;
	x2 += sXprzechyl;
	y1 = y2 = stKonf->sWysokosc / 2 + sWysPochylenia;
	y1 += sYprzechyl;
	y2 -= sYprzechyl;
	RysujLiniewBuforze(x1, y1, x2, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

	//belki krótsze dla kolejnych 10°
	sDlugoscPolowyBelki = stKonf->sSzerokosc * HOR_SZER10_PROC / 200;
	sXprzechyl = (int16_t)(sDlugoscPolowyBelki * fCosPrze);
	sYprzechyl = (int16_t)(sDlugoscPolowyBelki * fSinPrze);

	//belka +20°, policz współrzędne końca krótszej belki
	sWysPochylenia = (int16_t)(sWysokoscPolowyOkna * (fPochylenie - Deg2Rad(20)) / Deg2Rad(HOR_SKALA_POCH));
	//Środki wszystkich belek łączy niewidoczna, prostopadła do nich linia. Kolejne belki przecinaja ją co odległość: sWysPochylenia
	//Dla każdej belki trzeba wyznaczyć miejsce na tej linii i rozpocząć rysowanie belki symetrycznie wzgledem tego punktu
	sXSrodkaBelki = sWysPochylenia * fSinPrze;
	sYSrodkaBelki = sWysPochylenia * fCosPrze;

	x1 = x2 = stKonf->sSzerokosc / 2 + sXSrodkaBelki;
	x1 -= sXprzechyl;
	x2 += sXprzechyl;
	y1 = y2 = stKonf->sWysokosc / 2 + sYSrodkaBelki;	//część wspólna obu współrzędnych
	y1 += sYprzechyl;
	y2 -= sYprzechyl;
	RysujLiniewBuforze(x1, y1, x2, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

	//belka +10°, policz współrzędne końca krótszej belki
	sWysPochylenia = (int16_t)(sWysokoscPolowyOkna * (fPochylenie - Deg2Rad(10)) / Deg2Rad(HOR_SKALA_POCH));
	sXSrodkaBelki = sWysPochylenia * fSinPrze;
	sYSrodkaBelki = sWysPochylenia * fCosPrze;

	x1 = x2 = stKonf->sSzerokosc / 2 + sXSrodkaBelki;
	x1 -= sXprzechyl;
	x2 += sXprzechyl;
	y1 = y2 = stKonf->sWysokosc / 2 + sYSrodkaBelki;	//część wspólna obu współrzędnych
	y1 += sYprzechyl;
	y2 -= sYprzechyl;
	RysujLiniewBuforze(x1, y1, x2, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

	//belka -10°
	sWysPochylenia = (int16_t)(sWysokoscPolowyOkna * (fPochylenie + Deg2Rad(10)) / Deg2Rad(HOR_SKALA_POCH));
	sXSrodkaBelki = sWysPochylenia * fSinPrze;
	sYSrodkaBelki = sWysPochylenia * fCosPrze;

	x1 = x2 = stKonf->sSzerokosc / 2 + sXSrodkaBelki;
	x1 -= sXprzechyl;
	x2 += sXprzechyl;
	y1 = y2 = stKonf->sWysokosc / 2 + sYSrodkaBelki;	//część wspólna obu współrzędnych
	y1 += sYprzechyl;
	y2 -= sYprzechyl;
	RysujLiniewBuforze(x1, y1, x2, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

	//belka -20°
	sWysPochylenia = (int16_t)(sWysokoscPolowyOkna * (fPochylenie + Deg2Rad(20)) / Deg2Rad(HOR_SKALA_POCH));
	sXSrodkaBelki = sWysPochylenia * fSinPrze;
	sYSrodkaBelki = sWysPochylenia * fCosPrze;

	x1 = x2 = stKonf->sSzerokosc / 2 + sXSrodkaBelki;
	x1 -= sXprzechyl;
	x2 += sXprzechyl;
	y1 = y2 = stKonf->sWysokosc / 2 + sYSrodkaBelki;	//część wspólna obu współrzędnych
	y1 += sYprzechyl;
	y2 -= sYprzechyl;
	RysujLiniewBuforze(x1, y1, x2, y2, stKonf->sSzerokosc, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
}



////////////////////////////////////////////////////////////////////////////////
// Konwertuje stopnie na radiany
// Parametry: stopnie
// Zwraca: radiany
////////////////////////////////////////////////////////////////////////////////
float Deg2Rad(float stopnie)
{
	return stopnie / 180.0f * M_PI;
}



////////////////////////////////////////////////////////////////////////////////
// Konwertuje radiany na stopnie
// Parametry: radiany
// Zwraca: stopnie
////////////////////////////////////////////////////////////////////////////////
float Rad2Deg(float radiany)
{
	return radiany / M_PI * 180.0f;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja pobiera współrzędne obiektu wykonując ewentualne przeliczenie współrzędnych
// względnych na bezwzgledne w zależnosci od bieżącej konfiguracji ekranu OSD
// Parametry:
// 	*stObiekt - wskaźnik na strukturę obiektu do wyswietlenia na OSD, zawierający współrzędne obiektu
//	*stKonf - wskaźnik na strukturę konfiguracji OSD zawierajacą rozmiar ekranu OSD
//	*stWspolrzedne - wskaźnik na zwracane bezwzględne współrzedne obiektu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PobierzPozycjeObiektu(stObiektOsd_t *stObiekt, stKonfOsd_t *stKonf, prostokat_t *stWspolrzedne)
{
	switch (stObiekt->sPozycjaX)
	{
	case POZ_SRODEK:stWspolrzedne->sX1 = stKonf->sSzerokosc / 2;				break;	//środek ekranu poziomo lub pionowo
	case POZ_LEWO1:	stWspolrzedne->sX1 = OSD_MARGINES;							break;	//pierwszy obiekt od lewej
	case POZ_LEWO2:	stWspolrzedne->sX1 = OSD_SZER_OBIEKTU + OSD_MARGINES;		break;	//drugi obiekt od lewej
	case POZ_LEWO3:	stWspolrzedne->sX1 = 2 * OSD_SZER_OBIEKTU + OSD_MARGINES;	break;	//trzeci obiekt od lewej
	case POZ_LEWO4:	stWspolrzedne->sX1 = 3 * OSD_SZER_OBIEKTU + OSD_MARGINES;	break;	//czwarty obiekt od lewej
	case POZ_PRAWO1:stWspolrzedne->sX1 = stKonf->sSzerokosc - OSD_SZER_OBIEKTU - OSD_MARGINES;	break;	//pierwszy obiekt od prawej
	case POZ_PRAWO2:stWspolrzedne->sX1 = stKonf->sSzerokosc - 2 * OSD_SZER_OBIEKTU - OSD_MARGINES;	break;	//drugi obiekt od prawej
	case POZ_PRAWO3:stWspolrzedne->sX1 = stKonf->sSzerokosc - 3 * OSD_SZER_OBIEKTU - OSD_MARGINES;	break;	//trzeci obiekt od prawej
	case POZ_PRAWO4:stWspolrzedne->sX1 = stKonf->sSzerokosc - 4 * OSD_SZER_OBIEKTU - OSD_MARGINES;	break;	//czwarty obiekt od prawej
	default:		stWspolrzedne->sX1 = stObiekt->sPozycjaX;
	}

	switch (stObiekt->sPozycjaY)
	{
	case POZ_SRODEK:stWspolrzedne->sY1 = stKonf->sWysokosc / 2;					break;	//środek ekranu poziomo lub pionowo
	case POZ_GORA1: stWspolrzedne->sY1 = OSD_MARGINES;							break;	//pierwszy obiekt od góry
	case POZ_GORA2:	stWspolrzedne->sY1 = OSD_WYS_OBIEKTU + OSD_MARGINES;		break;	//drugi obiekt od góry
	case POZ_GORA3:	stWspolrzedne->sY1 = 2 * OSD_WYS_OBIEKTU  + OSD_MARGINES;	break;	//trzeci obiekt od góry
	case POZ_GORA4:	stWspolrzedne->sY1 = 3 * OSD_WYS_OBIEKTU  + OSD_MARGINES;	break;	//czwarty obiekt od góry
	case POZ_DOL1:	stWspolrzedne->sY1 = stKonf->sWysokosc - OSD_WYS_OBIEKTU - OSD_MARGINES;	break;	//pierwszy obiekt od dołu
	case POZ_DOL2:	stWspolrzedne->sY1 = stKonf->sWysokosc - 2 * OSD_WYS_OBIEKTU - OSD_MARGINES;	break;	//drugi obiekt od dołu
	case POZ_DOL3:	stWspolrzedne->sY1 = stKonf->sWysokosc - 3 * OSD_WYS_OBIEKTU - OSD_MARGINES;	break;	//trzeci obiekt od dołu
	case POZ_DOL4:	stWspolrzedne->sY1 = stKonf->sWysokosc - 4 * OSD_WYS_OBIEKTU - OSD_MARGINES;	break;	//czwarty obiekt od dołu
	default:		stWspolrzedne->sY1 = stObiekt->sPozycjaY;
	}
}

