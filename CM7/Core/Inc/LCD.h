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

void RysujEkran(void);
void Ekran_Powitalny(uint32_t * zainicjowano);
void Wykrycie(uint16_t x, uint16_t y, uint8_t dopelnij_znakow, uint8_t wynik);
void WyswietlKomunikatBledu(uint8_t chKomunikatBledu, float fParametr1, float fParametr2, float fParametr3);
void FraktalTest(unsigned char chTyp);
void FraktalDemo(void);
void GenerateJulia(unsigned short size_x, unsigned short size_y, unsigned short offset_x, unsigned short offset_y, unsigned short zoom, unsigned short * buffer);
void GenerateMandelbrot(float centre_X, float centre_Y, float Zoom, unsigned short IterationMax, unsigned short * buffer);
void HSV2RGB(float hue, float sat, float val, float *red, float *grn, float *blu);
uint32_t PobierzCzasT6(void);
uint32_t MinalCzas(uint32_t nPoczatek);
void InitFraktal(unsigned char chTyp);
void MenuGlowne(unsigned char *tryb);
void Menu(char *tytul, tmenu *menu, unsigned char *tryb);
void BelkaTytulu(char* chTytul);
void PomiaryIMU(void);
void TestTonuAudio(void);
void WyswietlParametryKartySD(void);
void TestKartySD(void);
void WyswietlRejestratorKartySD(void);
void PobierzKodBleduFAT(uint8_t chKodBleduFAT, char* chNapis);
uint32_t RysujKostkeObrotu(float *fKat);
#endif /* INC_LCD_H_ */
