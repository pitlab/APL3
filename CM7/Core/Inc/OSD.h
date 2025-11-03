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

//definicje kolorów OSD w formacie ARGB4444
#define KOLOSD_CZARNY	0xF000


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

//definicje wizualizacji belki sztucznego horyzontu
#define HOR_SZER00_PROC	50		//szerokość belki 0° pochylenia horyzontu w procentach szerokosci ekranu
#define HOR_SZER10_PROC	40		//szerokość belki 10° pochylenia horyzontu w procentach szerokosci ekranu
#define HOR_WYS_PROC	40		//wysokość pola w którym jest widoczny horyzont w procentach wysokości ekranu
#define HOR_GRUBOSC		5		//grubość belki horyzontu w pikselach
#define HOR_SKALA_POCH	30		//stopni pochylenia pokazanych w oknie horyzontu

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
	stObiektOsd_t stNapiBat;	//napięcie baterii
	stObiektOsd_t stPradBat;	//prad pobierany z baterii
	stObiektOsd_t stEnerBat;	//energia baterii: pobrana lub pozostała
} stKonfOsd_t;

//definicje flag obiektów OSD
#define FLO_WIDOCZNY	0x01	//obiekt widoczny






uint8_t InicjujOSD(void);
uint8_t PolaczBuforOSDzObrazem(uint8_t *chObrazFront, uint8_t *chObrazTlo, uint8_t *chBuforWyjsciowy, uint16_t sSzerokosc, uint16_t sWysokosc);
void XferCpltCallback(DMA2D_HandleTypeDef *hdma2d);
void XferErrorCallback(DMA2D_HandleTypeDef *hdma2d);
void RysujTestoweOSD();
void RysujOSD(stKonfOsd_t stKonf, stWymianyCM4_t stDane);
void PobierzPozycjeObiektu(stKonfOsd_t stKonf, prostokat_t *stWspolrzedne);

#endif /* INC_OSD_H_ */
