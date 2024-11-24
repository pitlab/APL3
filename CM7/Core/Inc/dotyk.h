/*
 * dotyk.h
 *
 *  Created on: Nov 17, 2024
 *      Author: PitLab
 */

#ifndef INC_DOTYK_H_
#define INC_DOTYK_H_
#include "stm32h7xx_hal.h"



//definicje kanałów przetwornika
#define TPCHX	5
#define TPCHY	1
#define TPCHZ1	3
#define TPCHZ2	4

//definicje elementów ekranowych
#define BRZEG	50		//odległość od brzegu ekranu
#define KRZYZ	16		//długość linii krzyzyka

//definicje parametrów dotknięcia ekranu

#define MIN_Z	50		//minimalna siła nacisku na ekran
#define MIN_T	2		//minimalny czas trwania dotknięcia
#define PKT_KAL	3		//punktów kalibracji

//flagi statusu
#define DOTYK_DOTKNIETO		0x01	//nacisnięto przycisk ekranowy
#define DOTYK_ZWOLNONO		0x02	//puszczono przycisk ekranowy
#define DOTYK_ZAPISANO		0x04	//zapisano dane konfiguracyjne
#define DOTYK_SKALIBROWANY	0x08	//została wykonana kalibracja


struct _statusDotyku
{
	uint16_t sAdc[4];
	uint16_t sX;
	uint16_t sY;
	uint8_t chCzasDotkniecia;	//czas utrzymania dotknięcia
	uint8_t chFlagi;			//flagi określające naciśnięcie i zwolnienie przycisku ekranowego
	uint32_t nOstCzasPomiaru;	//ostatni pomiar czasu między kolejnymi dotknięciami ekranu
};

struct _kalibracjaDotyku
{
	float fAx;
	float fBx;
	float fAy;
	float fBy;
	float fDeltaX;
	float fDeltaY;
};


uint16_t CzytajKanalDotyku(uint8_t  chKanal);
void CzytajDotyk(void);
uint8_t KalibrujDotyk(void);
void ObliczKalibracjeDotykuWielopunktowa(void);
void ObliczKalibracjeDotyku3Punktowa(void);
uint8_t TestDotyku(void);
uint8_t TestObliczenKalibracji(void);
uint8_t InicjujDotyk(void);

#endif /* INC_DOTYK_H_ */
