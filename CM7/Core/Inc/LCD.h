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
#include "rysuj.h"
#include "fft.h"

//#define MIN_MAG_WYKR	NOMINALNE_MAGN / 600		//minimalna wartość danych w danej osi aby zacząć rysować wykres biegunowy magnetometru
#define MIN_MAG_WYKR	NOMINALNE_MAGN * 0.8f		//minimalna wartość danych w danej osi aby zacząć rysować wykres biegunowy magnetometru

//czynności związane z obsługą przycisku graficznego
#define RYSUJ		0
#define ODSWIEZ		1

#define WYKRYJ_GORA	175
#define WYKRYJ_WIERSZ	18
#define KOL12	5	//współrzędne x początku pierwszej z dwu kolumn danych
#define KOL22	245	//współrzędne x początku drugiej z dwu kolumn danych

//dane do FFT
#define FFT_ACC		0		//przychodzące dane to akceletrometry
#define FFT_ZYR		1		//przychodzące datne to żyroskopy

//wykresy analizatora drgań FFT
#define AD_STARTX			15		//współrzedna X początku wykresu
#define AD_STARTY			20		//współrzedna Y początku wykresu
#define AD_WSPADY			100		//wysokość wodospadu na dole ekranu
#define AD_X_SIZE			(DISP_X_SIZE - AD_STARTX)	//szerokość wykresu (niezależna od rozmiaru FFT)
#define	AD_Y_SIZE			(DISP_Y_SIZE - AD_STARTY - AD_WSPADY)	//wysokość wykresu
#define AD_DZIALEK_SKALIY	6	//podziel całą skalę Y na tyle części odpowoadających dB
#define AD_DZIALEK_0Y		4	//poziom 0 dB na skali Y jest na tylu działkach od góry
#define AD_Y_DIV			(AD_Y_SIZE / AD_DZIALEK_SKALIY)	//skala piskeli na dB
#define AD_POZIOM_0DB		(AD_Y_DIV * (AD_DZIALEK_SKALIY - AD_DZIALEK_0Y))

#define WODOSPAD_SKALA_KOLORU	20

#define ROZDZIECZOSC_PASKA_RC		5	//zakres WE_RC_MAX = 2000 / szerokość ekranu = 400


uint8_t RysujEkran(void);
void Ekran_Powitalny(uint32_t nZainicjowano);
void Wykrycie(uint16_t x, uint16_t y, uint8_t dopelnij_znakow, uint8_t wynik);
void WyswietlKomunikatBledu(uint8_t chKomunikatBledu, float fParametr1, float fParametr2, float fParametr3);
void HSV2RGB(float hue, float sat, float val, float *red, float *grn, float *blu);
void MenuGlowne(uint8_t *chTryb);

void PomiaryIMU(void);
void PomiaryCisnieniowe(void);
//void DaneOdbiornikaRC(void);
void RysujPaskiKanalowRC(uint8_t chIndeksOpisu, uint16_t *sDane);
void TestTonuAudio(void);
void WyswietlParametryKartySD(void);
void TestKartySD(void);
void WyswietlRejestratorKartySD(void);
void PobierzKodBleduFAT(uint8_t chKodBleduFAT, char *chNapis);
void RysujPasekPostepu(uint16_t sPelenZakres);
uint32_t RysujKostkeObrotu(float *fKat);
uint8_t KalibracjaWzmocnieniaZyroskopow(uint8_t *chSekwencer);
void Poziomica(float fKatAkcelX, float fKatAkcelY);
void RysujPrzycisk(prostokat_t prost, char *chNapis, uint8_t chCzynnosc);
uint8_t KalibracjaZeraMagnetometru(uint8_t *chEtap);
float MaximumGlobalne(float *fMin, float *fMax);
uint8_t KalibrujBaro(uint8_t *chEtap);
void PlaskiObrotMagnetometrow(void);
void NastawyPID(uint8_t chKanal);
uint8_t CzytajFramFoat(uint16_t sAdres, uint8_t chRozmiar, float *fDane);
uint8_t CzytajFramChar(uint16_t sAdres, uint8_t chRozmiar, uint8_t *chDane);
void RysujFFT(float *stWynik, stFFT_t *stKonfig, uint8_t chRodzajDanych);
#endif /* INC_LCD_H_ */
