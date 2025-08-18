/*
 * LCD.h
 *
 *  Created on: Nov 16, 2024
 *      Author: PitLab
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_
#include "sys_def_CM7.h"
#include "display.h"

//#define MIN_MAG_WYKR	NOMINALNE_MAGN / 600		//minimalna wartość danych w danej osi aby zacząć rysować wykres biegunowy magnetometru
#define MIN_MAG_WYKR	NOMINALNE_MAGN * 0.8f		//minimalna wartość danych w danej osi aby zacząć rysować wykres biegunowy magnetometru

//czynności związane z obsługą przycisku graficznego
#define RYSUJ		0
#define ODSWIEZ		1

#define WYKRYJ_GORA	175
#define WYKRYJ_WIERSZ	18
#define KOL12	5	//współrzędne x początku pierwszej z dwu kolumn danych
#define KOL22	245	//współrzędne x początku drugiej z dwu kolumn danych


#define ROZDZIECZOSC_PASKA_RC		3	//zakres RC = 1000us / szerokość ekranu [us/pixel]

typedef struct
{
	uint16_t sX1;
	uint16_t sY1;
	uint16_t sX2;
	uint16_t sY2;
} prostokat_t;

void RysujEkran(void);
void Ekran_Powitalny(uint32_t nZainicjowano);
void Wykrycie(uint16_t x, uint16_t y, uint8_t dopelnij_znakow, uint8_t wynik);
void WyswietlKomunikatBledu(uint8_t chKomunikatBledu, float fParametr1, float fParametr2, float fParametr3);
void HSV2RGB(float hue, float sat, float val, float *red, float *grn, float *blu);
void MenuGlowne(unsigned char *tryb);

void PomiaryIMU(void);
void PomiaryCisnieniowe(void);
void DaneOdbiornikaRC(void);
void TestTonuAudio(void);
void WyswietlParametryKartySD(void);
void TestKartySD(void);
void WyswietlRejestratorKartySD(void);
void PobierzKodBleduFAT(uint8_t chKodBleduFAT, char* chNapis);
void RysujPasekPostepu(uint16_t sPelenZakres);
uint32_t RysujKostkeObrotu(float *fKat);
uint8_t KalibracjaWzmocnieniaZyroskopow(uint8_t *chSekwencer);
void Poziomica(float fKatAkcelX, float fKatAkcelY);
void RysujPrzycisk(prostokat_t prost, char *chNapis, uint8_t chCzynnosc);
uint8_t KalibracjaZeraMagnetometru(uint8_t *chEtap);
float MaximumGlobalne(float* fMin, float* fMax);
uint8_t KalibrujBaro(uint8_t *chEtap);
void PlaskiObrotMagnetometrow(void);
void NastawyPID(uint8_t chKanal);
uint8_t CzytajFram(uint16_t sAdres, uint8_t chRozmiar, float* fDane);
#endif /* INC_LCD_H_ */
