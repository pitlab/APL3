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


typedef struct
{
	uint16_t sX1;
	uint16_t sY1;
	uint16_t sX2;
	uint16_t sY2;
} prostokat_t;

void RysujEkran(void);
void Ekran_Powitalny(uint32_t * zainicjowano);
void Wykrycie(uint16_t x, uint16_t y, uint8_t dopelnij_znakow, uint8_t wynik);
void WyswietlKomunikatBledu(uint8_t chKomunikatBledu, float fParametr1, float fParametr2, float fParametr3);
void FraktalTest(unsigned char chTyp);
void FraktalDemo(void);
void GenerateJulia(unsigned short size_x, unsigned short size_y, unsigned short offset_x, unsigned short offset_y, unsigned short zoom, unsigned short * buffer);
void GenerateMandelbrot(float centre_X, float centre_Y, float Zoom, unsigned short IterationMax, unsigned short * buffer);
void HSV2RGB(float hue, float sat, float val, float *red, float *grn, float *blu);
void InitFraktal(unsigned char chTyp);
void MenuGlowne(unsigned char *tryb);
void Menu(char *tytul, tmenu *menu, unsigned char *tryb);
void BelkaTytulu(char* chTytul);
void PomiaryIMU(void);
void PomiaryCisnieniowe(void);
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
#endif /* INC_LCD_H_ */
