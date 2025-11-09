/*
 * OSD.h
 *
 *  Created on: Oct 28, 2025
 *      Author: PitLab
 */

#ifndef INC_OSD_H_
#define INC_OSD_H_
#include "sys_def_CM7.h"
#include "wymiana.h"
#include "rysuj.h"

#define ROZMIAR_KOLORU_OSD	2

//definicje kolorów OSD w formacie ARGB4444 bez ustawionej przerzroczystości
//#define KOLOSD_PRZEZR	0x0000
#define KOLOSD_CZARNY	0x0000
#define KOLOSD_SZARY75	0x0CCC
#define KOLOSD_SZARY50	0x0777
#define KOLOSD_SZARY25	0x0333
#define KOLOSD_BIALY	0x0FFF
#define KOLOSD_CZER0	0x0F00		//pełen kolor
#define KOLOSD_CZER1	0x0F33		//pełen kolor + 1/4 jasności
#define KOLOSD_CZER2	0x0F77		//pełen kolor + 1/2 jasności
#define KOLOSD_CZER3	0x0FCC		//pełen kolor + 3/4 jasności
#define KOLOSD_CZER9	0x0C00		//3/4 koloru
#define KOLOSD_CZER8	0x0700		//1/2 koloru
#define KOLOSD_CZER7	0x0300		//1/4 koloru
#define KOLOSD_ZIEL0	0x00F0
#define KOLOSD_ZIEL1	0x03F3
#define KOLOSD_ZIEL2	0x07F7
#define KOLOSD_ZIEL3	0x0CFC
#define KOLOSD_ZIEL9	0x00C0
#define KOLOSD_ZIEL8	0x0070
#define KOLOSD_ZIEL7	0x0030
#define KOLOSD_NIEB0	0x000F
#define KOLOSD_NIEB1	0x033F
#define KOLOSD_NIEB2	0x077F
#define KOLOSD_NIEB3	0x0CCF
#define KOLOSD_NIEB9	0x000C
#define KOLOSD_NIEB8	0x0007
#define KOLOSD_NIEB7	0x0003
#define KOLOSD_ZOLTY0	0x0FF0
#define KOLOSD_ZOLTY1	0x0FF3
#define KOLOSD_ZOLTY2	0x0FF7
#define KOLOSD_ZOLTY3	0x0FFC
#define KOLOSD_ZOLTY9	0x0CC0
#define KOLOSD_ZOLTY8	0x0770
#define KOLOSD_ZOLTY7	0x0330
#define KOLOSD_CYJAN0	0x00FF		//pełen kolor
#define KOLOSD_CYJAN1	0x03FF		//pełen kolor + 1/4 jasności
#define KOLOSD_CYJAN2	0x07FF		//pełen kolor + 1/2 jasności
#define KOLOSD_CYJAN3	0x0CFF		//pełen kolor + 3/4 jasności
#define KOLOSD_CYJAN9	0x00CC		//3/4 koloru
#define KOLOSD_CYJAN8	0x0077		//1/2 koloru
#define KOLOSD_CYJAN7	0x0033		//1/4 koloru
#define KOLOSD_MAGEN0	0x0F0F
#define KOLOSD_MAGEN1	0x0F3F
#define KOLOSD_MAGEN2	0x0F7F
#define KOLOSD_MAGEN3	0x0FCF
#define KOLOSD_MAGEN9	0x0C0C
#define KOLOSD_MAGEN8	0x0707
#define KOLOSD_MAGEN7	0x0303

//definicje poziomu przezroczystości
#define PRZEZR_0		0xF000
#define PRZEZR_25		0xC000
#define PRZEZR_50		0x7000
#define PRZEZR_75		0x3000
#define PRZEZR_100		0x0000

//definicje określajace położenie wzgledne obiektow ekranowych OSD
#define	POZ_SRODEK		0xFFFF		//środek ekranu poziomo lub pionowo
#define	POZ_GORA1		0xFFFE		//pierwszy wiersz od góry
#define	POZ_GORA2		0xFFFD		//drugi wiersz od góry
#define	POZ_GORA3		0xFFFC
#define	POZ_GORA4		0xFFFB
#define	POZ_DOL1		0xFFFA		//pierwszy wiersz od dołu ekranu
#define	POZ_DOL2		0xFFF9		//drugi wiersz od dołu ekranu
#define	POZ_DOL3		0xFFF8
#define	POZ_DOL4		0xFFF7
#define	POZ_LEWO1		0xFFF6		//pierwszy obiekt od lewej
#define	POZ_LEWO2		0xFFF5		//drugi obiekt od lewej
#define	POZ_LEWO3		0xFFF4
#define	POZ_LEWO4		0xFFF3
#define	POZ_PRAWO1		0xFFF2		//pierwszy obiekt od prawej
#define	POZ_PRAWO2		0xFFF1
#define	POZ_PRAWO3		0xFFF0
#define	POZ_PRAWO4		0xFFEF


#define OSD_CZCION_SZER	FONT_SL		//szerokość czcionki użytej w OSD
#define OSD_CZCION_WYS	FONT_SH		//wysokość czcionki użytej w OSD
#define OSD_MARGINES	10			//zamiast rysować od brzegu zacznij od tej wartosci
#define OSD_SZER_OBIEKTU	(10 * OSD_CZCION_SZER)
#define OSD_WYS_OBIEKTU	(3 * OSD_CZCION_WYS / 2)

//definicje wizualizacji belki sztucznego horyzontu
#define HOR_SZER00_PROC	50		//szerokość belki 0° pochylenia horyzontu w procentach szerokosci ekranu
#define HOR_SZER10_PROC	20		//szerokość belek co 10° pochylenia horyzontu w procentach szerokosci ekranu
#define HOR_WYS_PROC	40		//wysokość pola w którym jest widoczny horyzont w procentach wysokości ekranu
#define HOR_GRUBOSC		5		//grubość belki horyzontu w pikselach
#define HOR_SKALA_POCH	30		//stopni pochylenia pokazanych w oknie horyzontu
#define PIW_ODL_RAMKI	1		//odległość między napisem wyniku pomiaru a ramkę strzałki
#define PIW_KOR_WYS_CZC	1		//korekta położenia środka wysokości czcionki
#define PIW_SZER_PROC	20		//Prędkość i Wysokość - pionowa skala jest w tylu procentach ekranu od brzegów
#define PIW_WYS_PROC	40		//wysokość pola w którym są widoczne pionowe skale prędkosci i wysokosci
#define PIW_ZNAK_STRZAL	4		//liczba znaków mieszczących się w strzałce (wysokość 4 cyfrowa)
#define PIW_DYST_NAPISU	16		//odległość między osią a napisem, miejsce na ostrze strzałki

//struktura obiektów ekranowych OSD
typedef struct
{
	uint8_t chFlagi;
	uint16_t sPozycjaX;		//położenie poziome (bezwzględne w pikselach lub stała okreslająca położenie wzgledne)
	uint16_t sPozycjaY;		//położenie pionowe
	uint16_t sKolorObiektu;	//ARGB4444 Podstawowy kolor kontrolki
	uint16_t sKolorJasny;	//ARGB4444 Kolor jasnej obwiedni
	uint16_t sKolorCiemny;	//ARGB4444 Kolor ciemnej obwiedni
	uint16_t sKolorTla;		//ARGB4444 Kolor wypełniania tła
} stObiektOsd_t;

typedef struct
{
	uint8_t chFlagi;
	uint16_t sPozycjaX;		//położenie poziome (bezwzględne w pikselach lub stała okreslająca położenie wzgledne)
	uint16_t sPozycjaY;		//położenie pionowe
	uint16_t sKolorObiektu;	//ARGB4444
	uint16_t sKolorJasny;	//ARGB4444 Kolor jasnej obwiedni
	uint16_t sKolorCiemny;	//ARGB4444 Kolor ciemnej obwiedni
	uint16_t sKolorTla;		//ARGB4444
	float fPoprzWartosc1;	//wartość poprzednia 1 potrzebna do określenia potrzeby przerysowania obiektu
	float fPoprzWartosc2;	//wartość poprzednia 2
} stObOsd_F2_t;			//typ obiektu OSD zawierajacego 2 zmienne float

//struktura konfiguracji OSD
typedef struct st_KonfOsd
{
	uint16_t sSzerokosc;		//szerokość obrazu
	uint16_t sWysokosc;			//wysokość obrazu
	stObOsd_F2_t stHoryzont;	//obiekt sztucznego horyzontu
	stObiektOsd_t stKursMag;	//kurs lotu: magnetyczny lub geograficzny
	stObiektOsd_t stPredWys;	//prędkość i wysokość lotu
	stObiektOsd_t stDlugGeo;	//długość geograficzna
	stObiektOsd_t stSzerGeo;	//szerokość geograficzna
	stObiektOsd_t stData;		//bieżąca data
	stObiektOsd_t stCzas;		//bieżący czas
	stObiektOsd_t stNapiBat;	//napięcie baterii
	stObiektOsd_t stPradBat;	//prad pobierany z baterii
	stObiektOsd_t stEnerBat;	//energia baterii: pobrana lub pozostała
	//stObiektOsd_t stLogo;		//logo producenta
	//obiekty diagnostyczne
	stObiektOsd_t stCzasWyp;	//czas wypełniania ekranu tłem
	stObiektOsd_t stCzasRys;	//czas rysowania całego ekranu
	stObiektOsd_t stCzasBlend;	//czas blendowania
	stObiektOsd_t stCzasLCD;	//czas rysowania na LCD

	stObiektOsd_t stCzasHor;	//czas rysowania horyzontu
	stObiektOsd_t stCzasPiW;	//czas rysowania wysokości i prędkosci
	stObiektOsd_t stCzasDaty;	//czas rysowania daty
	stObiektOsd_t stCzasGNSS;	//czas rysowania pozycji GNSS
} stKonfOsd_t;

//definicje flag obiektów OSD
#define FO_WIDOCZNY		0x80	//obiekt widoczny
#define FO_E_POBRANA	0x01	//wskazuje na energię pobraną lub pozostałą
#define FO_E_POZOSTALA	0x00	//lub pozostałą (brak bitu)





uint8_t InicjujOSD(void);
uint8_t PolaczBuforOSDzObrazem(uint8_t *chObrazFront, uint8_t *chObrazTlo, uint8_t *chBuforWyjsciowy, uint16_t sSzerokosc, uint16_t sWysokosc);
void XferCpltCallback(DMA2D_HandleTypeDef *hdma2d);
void XferErrorCallback(DMA2D_HandleTypeDef *hdma2d);
void RysujTestoweOSD();
void RysujOSD(stKonfOsd_t *stKonf, volatile stWymianyCM4_t *stDane);
//void RysujHoryzont(stKonfOsd_t *stKonf, float fPochylenie, float fPrzechylenie, uint16_t sKolor);
void RysujHoryzont(stKonfOsd_t *stKonf, float fPochylenie, float fPrzechylenie, uint16_t sKolor, uint16_t sKolorJasny, uint16_t sKolorCiemny);
void PobierzPozycjeObiektu(stObiektOsd_t *stObiekt, stKonfOsd_t *stKonf, prostokat_t *stWspolrzedne);

float Deg2Rad(float stopnie);
float Rad2Deg(float radiany);

#endif /* INC_OSD_H_ */
