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
#define KOLOSD_CZARNY	0x0000
#define KOLOSD_PRZEZR	0x0000
#define KOLOSD_BIALY	0x0FFF
#define KOLOSD_CZERWONY	0x0F00
#define KOLOSD_ZIELONY	0x00F0
#define KOLOSD_NIEBIES	0x000F
#define KOLOSD_ZOLTY	0x0FF0
#define KOLOSD_CYJAN	0x00FF
#define KOLOSD_MAGENTA	0x0F0F

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
#define OSD_MARGINES	20			//zamiast rysować od brzegu zacznij od tej wartosci
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
#define PIW_DYST_NAPISU	12		//odległość między osią a napisem, miejsce na ostrze strzałki

//struktura obiektów ekranowych OSD
typedef struct st_ObiektOsd
{
	uint8_t chFlagi;
	uint16_t sPozycjaX;		//położenie poziome (bezwzględne w pikselach lub stała okreslająca położenie wzgledne)
	uint16_t sPozycjaY;		//położenie pionowe
	uint16_t sKolorObiektu;	//ARGB4444
	uint16_t sKolorTla;		//ARGB4444
	float fPoprzWartosc;	//wartość poprzednia potrzebna do określenia potrzeby przerysowania obiektu
} stObiektOsd_t;

//struktura konfiguracji OSD
typedef struct st_KonfOsd
{
	uint16_t sSzerokosc;		//szerokość obrazu
	uint16_t sWysokosc;			//wysokość obrazu
	stObiektOsd_t stHoryzont;	//obiekt sztucznego horyzontu
	stObiektOsd_t stKursMag;	//kurs lotu: magnetyczny lub geograficzny
	stObiektOsd_t stPredWys;	//prędkość i wysokość lotu
	stObiektOsd_t stDlugGeo;	//długość geograficzna
	stObiektOsd_t stSzerGeo;	//szerokość geograficzna
	stObiektOsd_t stData;		//bieżąca data
	stObiektOsd_t stCzas;		//bieżący czas
	stObiektOsd_t stNapiBat;	//napięcie baterii
	stObiektOsd_t stPradBat;	//prad pobierany z baterii
	stObiektOsd_t stEnerBat;	//energia baterii: pobrana lub pozostała
} stKonfOsd_t;

//definicje flag obiektów OSD
#define FO_WIDOCZNY	0x01	//obiekt widoczny






uint8_t InicjujOSD(void);
uint8_t PolaczBuforOSDzObrazem(uint8_t *chObrazFront, uint8_t *chObrazTlo, uint8_t *chBuforWyjsciowy, uint16_t sSzerokosc, uint16_t sWysokosc);
void XferCpltCallback(DMA2D_HandleTypeDef *hdma2d);
void XferErrorCallback(DMA2D_HandleTypeDef *hdma2d);
void RysujTestoweOSD();
void RysujOSD(stKonfOsd_t *stKonf, volatile stWymianyCM4_t *stDane);
void PobierzPozycjeObiektu(stObiektOsd_t *stObiekt, stKonfOsd_t *stKonf, prostokat_t *stWspolrzedne);

float Deg2Rad(float stopnie);
float Rad2Deg(float radiany);

#endif /* INC_OSD_H_ */
