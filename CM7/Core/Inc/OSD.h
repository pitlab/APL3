/*
 * OSD.h
 *
 *  Created on: Oct 28, 2025
 *      Author: PitLab
 */

#ifndef INC_OSD_H_
#define INC_OSD_H_
#include "sys_def_CM7.h"

#define ROZMIAR_KOLORU_OSD	2

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

//struktura konfiguracji OSD
typedef struct st_KonfOsd
{
	uint16_t sSzerokosc;	//szerokość obrazu
	uint16_t sWysokosc;		//wysokość obrazu
} stKonfOsd_t;


//struktura obiektów ekranowych OSD
typedef struct st_ObiektOsd
{
	uint16_t sPozycjaX;		//położenie poziome (bezwzględne w pikselach lub stała okreslajaca położenie wzgledne)
	uint16_t sPozycjaY;		//położenie pionowe
	uint16_t sKolorObiektu;	//ARGB4444
	uint16_t sKolorTla;		//ARGB4444
	float fPoprzWartosc;	//wartość poprzednia
} stObiektOsd_t;



//definicje flag obiektów OSD
#define OSD_HORYZONT	0x00000001	//obecność sztucznego horyzontu
#define OSD_WYS_PRED	0x00000002	//obecność wskaźników wysokości i predkości
#define OSD_	0x00000004
#define OSD_	0x00000008
#define OSD_	0x00000010
#define OSD_	0x00000020

uint8_t InicjujOSD(void);
void RysujTestoweOSD();
void RysujOSD(uint16_t sSzerokosc, uint16_t sWysokosc);
uint8_t PolaczBuforOSDzObrazem(uint8_t *chObrazFront, uint8_t *chObrazTlo, uint8_t *chBuforWyjsciowy, uint16_t sSzerokosc, uint16_t sWysokosc);
void XferCpltCallback(DMA2D_HandleTypeDef *hdma2d);
void XferErrorCallback(DMA2D_HandleTypeDef *hdma2d);

#endif /* INC_OSD_H_ */
