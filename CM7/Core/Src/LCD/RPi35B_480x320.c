//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi wyświetlacza TFT 320x480 RPI 3,5" B https://www.waveshare.com/wiki/3.5inch_RPi_LCD_(B)
//
// Opracowano na podstawie analizy działania systemu z obrazu "RPi LCD Bookworm_32bit only for pi5&pi4" pobranego ze strony waveshare
// Wersja B najprawdopodobniej oparta o sterownik ILI9486
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "display.h"
#include "main.h"
#include "moduly_SPI.h"
#include <math.h>
#include <RPi35B_480x320.h>
#include <stdio.h>
#include <string.h>
#include "semafory.h"
#include "cmsis_os.h"
#include "LCD_SPI.h"

// Wyświetlacz pracował na 25MHz ale później zaczął śmiecić na ekranie. Próbuję na 22,2MHz - jest OK


//deklaracje zmiennych
//extern SPI_HandleTypeDef hspi5;
extern uint8_t chRysujRaz;
extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu

extern uint16_t sBuforLCD[DISP_X_SIZE * DISP_Y_SIZE];
extern uint8_t chOrientacja;
extern uint8_t fch, fcl, bch, bcl;	//kolory czcionki i tła (bajt starszy i młodszy)
extern uint8_t _transparent;	//flaga określająca czy mamy rysować tło czy rysujemy na istniejącym
volatile uint8_t chDMAgotowe;

//#ifdef LCD_RPI35B
extern struct current_font cfont;
//#endif

////////////////////////////////////////////////////////////////////////////////
// Konfiguracja wyświetlacza 3.5inch RPi LCD (B) https://www.waveshare.com/wiki/3.5inch_RPi_LCD_(B)
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLCD_35B_16bit(void)
{
	extern uint8_t chPort_exp_wysylany[LICZBA_EXP_SPI_ZEWN];

	// LCD_RESET 1 - 0 - 1
	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(10);

	chPort_exp_wysylany[0] &= ~EXP01_LCD_RESET;	//RES=0
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

	LCD_write_command16(0xB0);
	LCD_write_data16(0x00, 0x00);
	LCD_write_command16(0x11);

	HAL_Delay(285); 			//285ms przerwy

	LCD_write_command16(0x21);
	LCD_write_command16(0x3A);
	LCD_write_data16(0x00, 0x55);

	LCD_write_command16(0xC2);
	LCD_write_data16(0x00, 0x33);

	LCD_write_command16(0xC5);
	LCD_write_dat_pie16( 0x00);
	LCD_write_dat_sro16(0x1E);
	LCD_write_dat_ost16(0x80);

	LCD_write_command16(0x36);
	LCD_write_data16(0x00, 0x28);

	LCD_write_command16(0xB1);
	LCD_write_data16(0x00, 0xB0);

	LCD_write_command16(0xE0);	//Positive Gamma Correction
	LCD_write_dat_pie16(0x00);
	LCD_write_dat_sro16(0x13);
	LCD_write_dat_sro16(0x18);
	LCD_write_dat_sro16(0x04);
	LCD_write_dat_sro16(0x0F);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x3A);
	LCD_write_dat_sro16(0x56);
	LCD_write_dat_sro16(0x4D);
	LCD_write_dat_sro16(0x03);
	LCD_write_dat_sro16(0x0A);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x30);
	LCD_write_dat_sro16(0x3E);
	LCD_write_dat_ost16(0x0F);

	LCD_write_command16(0xE1);	//Negative Gamma Correction
	LCD_write_dat_pie16(0x00);
	LCD_write_dat_sro16(0x13);
	LCD_write_dat_sro16(0x18);
	LCD_write_dat_sro16(0x01);
	LCD_write_dat_sro16(0x11);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x38);
	LCD_write_dat_sro16(0x34);
	LCD_write_dat_sro16(0x4D);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x0D);
	LCD_write_dat_sro16(0x0B);
	LCD_write_dat_sro16(0x31);
	LCD_write_dat_sro16(0x37);
	LCD_write_dat_ost16(0x0F);

	LCD_write_command16(0x11);
	HAL_Delay(302); 		//302ms

	LCD_write_command16(0x29);
	LCD_write_command16(0x36);	//orientacja ekranu
	LCD_write_data16(0x00, 0xE8);

	LCD_write_command16(0x2A);	//Column Address Set
	LCD_write_dat_pie16(0x00);
	LCD_write_dat_sro16(0x00);
	LCD_write_dat_sro16(0x01);
	LCD_write_dat_ost16(0xDF);

	LCD_write_command16(0x2B);	//Page Address Set
	LCD_write_dat_pie16(0x00);
	LCD_write_dat_sro16(0x00);
	LCD_write_dat_sro16(0x01);
	LCD_write_dat_ost16(0x3F);

	chRysujRaz = 1;
	nZainicjowanoCM7 |= INIT_LCD480x320;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Konfiguracja wyświetlacza 3.5inch RPi LCD (C) https://www.waveshare.com/wiki/3.5inch_RPi_LCD_(C)
// Nie działa
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLCD_35C_8bit(void)
{
	extern uint8_t chPort_exp_wysylany[LICZBA_EXP_SPI_ZEWN];

	// LCD_RESET 1 - 0 - 1
	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(10);

	chPort_exp_wysylany[0] &= ~EXP01_LCD_RESET;	//RES=0
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

	LCD_write_command8(0xFF);
	LCD_write_command8(0xFF);
	HAL_Delay(5);

	LCD_write_command8(0xFF);
	LCD_write_command8(0xFF);
	LCD_write_command8(0xFF);
	LCD_write_command8(0xFF);
	HAL_Delay(10);

	LCD_write_command8(0xB0);
	LCD_write_dat_jed8(0x00);

	LCD_write_command8(0xB3);
	LCD_write_dat_pie8(0x02);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_ost8(0x10);

	LCD_write_command8(0xB4);
	LCD_write_dat_jed8(0x11);

	LCD_write_command8(0xC0);
	LCD_write_dat_pie8(0x13);
	LCD_write_dat_sro8(0x3B);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x01);
	LCD_write_dat_sro8(0x00);	//NW
	LCD_write_dat_ost8(0x43);

	LCD_write_command8(0xC1);
	LCD_write_dat_pie8(0x08);
	LCD_write_dat_sro8(0x15);	//CLOCK
	LCD_write_dat_sro8(0x08);
	LCD_write_dat_ost8(0x08);

	LCD_write_command8(0xC4);
	LCD_write_dat_pie8(0x15);
	LCD_write_dat_sro8(0x03);
	LCD_write_dat_sro8(0x03);
	LCD_write_dat_ost8(0x01);

	LCD_write_command8(0xC6);
	LCD_write_dat_jed8(0x02);

	LCD_write_command8(0xC8);
	LCD_write_dat_pie8(0x0C);
	LCD_write_dat_sro8(0x05);
	LCD_write_dat_sro8(0x0A);
	LCD_write_dat_sro8(0x6B);
	LCD_write_dat_sro8(0x04);
	LCD_write_dat_sro8(0x06);
	LCD_write_dat_sro8(0x15);
	LCD_write_dat_sro8(0x10);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x31);
	LCD_write_dat_sro8(0x10);
	LCD_write_dat_sro8(0x15);
	LCD_write_dat_sro8(0x06);
	LCD_write_dat_sro8(0x64);
	LCD_write_dat_sro8(0x0D);
	LCD_write_dat_sro8(0x0A);
	LCD_write_dat_sro8(0x05);
	LCD_write_dat_sro8(0x0C);
	LCD_write_dat_sro8(0x31);
	LCD_write_dat_ost8(0x00);

	LCD_write_command8(0x35);
	LCD_write_dat_jed8(0x00);

	LCD_write_command8(0x0C);
	LCD_write_dat_jed8(0x66);

	LCD_write_command8(0x3A);
	LCD_write_dat_jed8(0x66);

	LCD_write_command8(0x44);
	LCD_write_dat_pie8(0x00);
	LCD_write_dat_ost8(0x01);

	LCD_write_command8(0xD0);
	LCD_write_dat_pie8(0x07);
	LCD_write_dat_sro8(0x07);	//VCI1
	LCD_write_dat_sro8(0x14);	//VRH 0x1D
	LCD_write_dat_ost8(0xA2);	//BT

	LCD_write_command8(0xD1);
	LCD_write_dat_pie8(0x03);
	LCD_write_dat_sro8(0x5A);	//VCM
	LCD_write_dat_ost8(0x10);	//VDV

	LCD_write_command8(0xD2);
	LCD_write_dat_pie8(0x03);
	LCD_write_dat_sro8(0x04);
	LCD_write_dat_ost8(0x04);

	LCD_write_command8(0x11);
	HAL_Delay(150);

	LCD_write_command8(0x2A);
	LCD_write_dat_pie8(0x00);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x01);
	LCD_write_dat_ost8(0x3F);	//320

	LCD_write_command8(0x2B);
	LCD_write_dat_pie8(0x00);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x01);
	LCD_write_dat_ost8(0xDF);	//480

	//write_SPI_commond(0xB4);
	//write_SPI_data(0x00);
	HAL_Delay(100);
	LCD_write_command8(0x29);

	HAL_Delay(30);
	LCD_write_command8(0x2C);

	chRysujRaz = 1;
	nZainicjowanoCM7 |= INIT_LCD480x320;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Konfiguracja wyświetlacza 3.5inch RPi LCD (C) https://www.waveshare.com/wiki/3.5inch_RPi_LCD_(C)
// Na podstawie sterownika NoTro: https://github.com/notro/fbtft/blob/master/fb_ili9486.c
// Nie działa
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLCD_35C_16bit(void)
{
	extern uint8_t chPort_exp_wysylany[LICZBA_EXP_SPI_ZEWN];

	// LCD_RESET 1 - 0 - 1
	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(10);

	chPort_exp_wysylany[0] &= ~EXP01_LCD_RESET;	//RES=0
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

//	LCD_write_command16(0xFF);
	//LCD_write_command16(0xFF);
	//HAL_Delay(5);		//delay_nms(5);

	//LCD_write_command16(0xFF);
	//LCD_write_command16(0xFF);
	//LCD_write_command16(0xFF);
	//LCD_write_command16(0xFF);
//	HAL_Delay(10);		//delay_nms(10);

	LCD_write_command16(0xB0);	// Interface Mode Control
	LCD_write_dat_jed16(0x00);

	LCD_write_command16(0x11);	// Sleep OUT
	HAL_Delay(250);

	LCD_write_command16(0x3A);	// Interface Pixel Format, 16 bits / pixel
	LCD_write_dat_jed16(0x55);

	LCD_write_command16(0x36);	// Memory Access Control
	LCD_write_dat_jed16(0x28);

	LCD_write_command16(0xC2);	// Power Control 3
	LCD_write_dat_jed16(0x44);

	LCD_write_command16(0xC5);	// VCOM Control 1
	LCD_write_dat_pie16(0x00);
	LCD_write_dat_sro16(0x00);
	LCD_write_dat_sro16(0x00);
	LCD_write_dat_ost16(0x00);

	LCD_write_command16(0xE0);	// PGAMCTRL(Positive Gamma Control)
	LCD_write_dat_pie16(0x0F);
	LCD_write_dat_sro16(0x1F);
	LCD_write_dat_sro16(0x1C);
	LCD_write_dat_sro16(0x0C);
	LCD_write_dat_sro16(0x0F);
	LCD_write_dat_sro16(0x08);
	LCD_write_dat_sro16(0x48);
	LCD_write_dat_sro16(0x98);
	LCD_write_dat_sro16(0x37);
	LCD_write_dat_sro16(0x0A);
	LCD_write_dat_sro16(0x13);
	LCD_write_dat_sro16(0x04);
	LCD_write_dat_sro16(0x11);
	LCD_write_dat_sro16(0x0D);
	LCD_write_dat_ost16(0x00);

	LCD_write_command16(0xE1);	// NGAMCTRL(Negative Gamma Control)
	LCD_write_dat_pie16(0x0F);
	LCD_write_dat_sro16(0x32);
	LCD_write_dat_sro16(0x2E);
	LCD_write_dat_sro16(0x0B);
	LCD_write_dat_sro16(0x0D);
	LCD_write_dat_sro16(0x05);
	LCD_write_dat_sro16(0x47);
	LCD_write_dat_sro16(0x75);
	LCD_write_dat_sro16(0x37);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x10);
	LCD_write_dat_sro16(0x03);
	LCD_write_dat_sro16(0x24);
	LCD_write_dat_sro16(0x20);
	LCD_write_dat_ost16(0x00);

	LCD_write_command16(0xE2);	// Digital Gamma Control 1
	LCD_write_dat_pie16(0x0F);
	LCD_write_dat_sro16(0x32);
	LCD_write_dat_sro16(0x2E);
	LCD_write_dat_sro16(0x0B);
	LCD_write_dat_sro16(0x0D);
	LCD_write_dat_sro16(0x05);
	LCD_write_dat_sro16(0x47);
	LCD_write_dat_sro16(0x75);
	LCD_write_dat_sro16(0x37);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x10);
	LCD_write_dat_sro16(0x03);
	LCD_write_dat_sro16(0x24);
	LCD_write_dat_sro16(0x20);
	LCD_write_dat_ost16(0x00);

	LCD_write_command16(0x36);	// Memory Access Control, BGR
	LCD_write_dat_jed16(0x28);

	LCD_write_command16(0x11);	// Sleep OUT
	HAL_Delay(250);

	LCD_write_command16(0x29);		// Display ON
	HAL_Delay(250);

	chRysujRaz = 1;
	nZainicjowanoCM7 |= INIT_LCD480x320;
	return ERR_OK;
}


#ifdef LCD_RPI35B
////////////////////////////////////////////////////////////////////////////////
// ustawia orientację ekranu
// Parametry: orientacja - orientacja układu ekranu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void OrientacjaEkranu(uint8_t orientacja)
{
	LCD_write_command16(0x36); // Memory Access Control
	if (orientacja)	//PIONOWO
	{
		LCD_write_data16(0x00,
			(0<<2)|	//MH Horizontal Refresh ORDER
			(1<<3)|	//BGR RGB-BGR Order: 0: RGB=RGB; 1:RGB=BGR
			(0<<4)|	//ML Vertical Refresh Order
			(0<<5)|	//MV Row / Column Exchange
			(1<<6)|	//MX Column Address Order
			(0<<7));	//MY Row Address Order
	}
	else	//POZIOMO
	{
		LCD_write_data16(0x00,
			(0<<2)|	//MH Horizontal Refresh ORDER
			(1<<3)|	//BGR RGB-BGR Order: 0: RGB=RGB; 1:RGB=BGR
			(0<<4)|	//ML Vertical Refresh Order
			(1<<5)|	//MV Row / Column Exchange
			(1<<6)|	//MX Column Address Order
			(1<<7));	//MY Row Address Order
	}
	chOrientacja = orientacja;
}



////////////////////////////////////////////////////////////////////////////////
// Zapełnij cały ekran kolorem
// Parametry: sKolor565 - kolor w formacie RGB 5-6-5
// Zwraca: nic
// Czas czyszczenia ekranu: 378ms @25MHz
////////////////////////////////////////////////////////////////////////////////
void WypelnijEkran(uint16_t sKolor565)
{
	uint32_t y;
	uint8_t x, dane[8];

	//for(y=0; y<DISP_X_SIZE * DISP_Y_SIZE; y++)
		//sBuforLCD[y] = GRAY20;
	if (!sKolor565)
		sKolor565 = BLACK;

	chRysujRaz = 1;
	setColor(sKolor565);
	setBackColor(sKolor565);

	LCD_write_command16(0x2A);	//Column Address Set
	for (x=0; x<8; x++)
		dane[x] = 0;
	dane[5] = 0x01;
	dane[7] = 0xDF;		//479 = 0x1DF
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2B);	//Page Address Set
	dane[5] = 0x01;
	dane[7] = 0x3F;		//319 = 0x13F
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2C);	//Memory Write
	for (x=0; x<4; x++)
	{
		dane[2*x+0] = bch;
		dane[2*x+1] = bcl;
	}
	for(y=0; y<320*480/4; y++)
		LCD_WrData(dane, 8);

	/*/ilość transmitowanych danych jest liczbą 16-bitową, więc całość danych dzielę na mniejsze kawałki
	for (x=0; x<8; x++)
		LCD_WrDataDMA((uint8_t*) sBuforLCD + (x * DISP_X_SIZE * DISP_Y_SIZE / 4), DISP_X_SIZE * DISP_Y_SIZE /4);*/
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor rysowania jako RGB
// Parametry: r, g, b - składowe RGB koloru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setColorRGB(uint8_t r, uint8_t g, uint8_t b)
{
	fch = ((r & 0xF8) | g>>5);
	fcl = ((g & 0x1C)<<3 | b>>3);
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor rysowania jako natywny dla wyświetlacza 5R+6G+5B
// Parametry: sKolor565 - kolor w formacie RGB 5-6-5
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setColor(uint16_t sKolor565)
{
	fch = (uint8_t)(sKolor565>>8);
	fcl = (uint8_t)(sKolor565 & 0xFF);
}



////////////////////////////////////////////////////////////////////////////////
// pobiera aktywny kolor
// Parametry: nic
// Zwraca: kolor
////////////////////////////////////////////////////////////////////////////////
uint16_t getColor(void)
{
	return (fch<<8) | fcl;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor tła jako RGB
// Parametry: r, g, b - składowe RGB koloru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setBackColorRGB(uint8_t r, uint8_t g, uint8_t b)
{
	bch = ((r&248) | g>>5);
	bcl = ((g&28)<<3 | b>>3);
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor tła jako natywny dla wyświetlacza 5R+6G+5B
// Parametry: sKolor565 - kolor w formacie RGB 5-6-5
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setBackColor(uint16_t sKolor565)
{
	if (color == TRANSPARENT)
		_transparent = 1;
	else
	{
		bch = (uint8_t)(sKolor565>>8);
		bcl = (uint8_t)(sKolor565 & 0xFF);
		_transparent = 0;
	}
}



////////////////////////////////////////////////////////////////////////////////
// pobiera aktywny kolor tła
// Parametry: nic
// Zwraca: kolor
////////////////////////////////////////////////////////////////////////////////
uint16_t getBackColor(void)
{
	return (bch<<8) | bcl;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuj prostokąt wypełniony kolorem
// Parametry:
// Zwraca: nic
// Czas rysowania pełnego ekranu: 372ms @25MHz
////////////////////////////////////////////////////////////////////////////////
void RysujProstokatWypelniony(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t kolor)
{
	uint16_t i, j, k;
	uint8_t n, dane[8];

	for (n=0; n<8; n++)
		dane[n] = 0;

	LCD_write_command16(0x2A);	//Column Address Set
	dane[1] = sStartX >> 8;
	dane[3] = sStartX;
	dane[5] = (sStartX + sSzerokosc - 1) >> 8;
	dane[7] = sStartX + sSzerokosc - 1;
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2B);	//Page Address Set
	dane[1] = sStartY >> 8;
	dane[3] = sStartY;
	dane[5] = (sStartY +  sWysokosc - 1) >> 8;
	dane[7] =  sStartY + sWysokosc - 1;
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2C);	//Memory Write
	for(n=0; n<4; n++)
	{
		dane[2*n+0] = kolor >> 8;
		dane[2*n+1] = kolor;
	}

	for(i=0; i<sWysokosc; i++)
	{
		for(j=0; j<sSzerokosc/4; j++)
			LCD_WrData(dane, 8);

		//ponieważ wypełnianie odbywa się paczkami po 4 pixele, może zdarzyć się że nie dopełni wszystkich danych
		k = sSzerokosc - j*4;	//policz czy jest reszta
		if (k)
		{
			for(j=0; j<k; j++)
				LCD_WrData(dane, 2);	//dopełnij reszte
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje poziomą linią
// Parametry: x, y współrzędne początku
// len - długośc linii
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujLiniePozioma(int16_t x, int16_t y, int16_t len)
{
	int i;

	if (len < 0)
	{
		len = -len;
		x -= len;
	}
	setXY(x, y, x+len, y);

	for (i=0; i<len+1; i++)
		LCD_write_data16(fch, fcl);
	clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje pionową linię
// Parametry: x, y współrzędne początku
// len - długość linii
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujLiniePionowa(int16_t x, int16_t y, int16_t len)
{
	int i;

	if (len < 0)
	{
		len = -len;
		y -= len;
	}
	setXY(x, y, x, y+len);

	for (i=0; i<len+1; i++)
		LCD_write_data16(fch, fcl);
	clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// ustawia parametry pamięci do rysowania linii
// Parametry:
// x1, y1 współrzędne początku obszaru
// x2, y2 współrzędne końca obszaru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t sTemp;
	uint8_t x, dane[8];

	for (x=0; x<8; x++)
		dane[x] = 0;

	if (chOrient == PIONOWO)
	{
		sTemp = x1;
		x1 = y1;
		y1 = sTemp;
		sTemp = x2;
		x2 = y2;
		y2 = sTemp;
		y1=DISP_Y_SIZE - y1;
		y2=DISP_Y_SIZE - y2;
		sTemp = y1;
		y1 = y2;
		y2 = sTemp;
	}

	LCD_write_command16(0x2A);	//Column Address Set
	dane[1] = x1>>8;
	dane[3] = x1;
	dane[5] = x2>>8;
	dane[7] = x2;
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2B);	//Page Address Set
	dane[1] = y1>>8;
	dane[3] = y1;
	dane[5] = y2>>8;
	dane[7] = y2;
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2C);	//Memory Write
}




////////////////////////////////////////////////////////////////////////////////
// zeruje parametry pamięci do rysowania linii
// Parametry:nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void clrXY(void)
{
	if (chOrient == PIONOWO)
		setXY(0, 0, DISP_X_SIZE, DISP_Y_SIZE);
	else
		setXY(0, 0, DISP_Y_SIZE, DISP_X_SIZE);
}



////////////////////////////////////////////////////////////////////////////////
// rysuje piksel o współprzędnych x,y we wcześniej zdefiniowanym kolorze
// Parametry: x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawPixel(uint16_t x, uint16_t y)
{
	setXY(x, y, x, y);
	LCD_write_dat_pie16(fch);
	LCD_write_dat_ost16(fcl);
	//clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// rysuj linię o współrzędnych x1,y1, x2,y3 we wcześniej zdefiniowanym kolorze
// Parametry: x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujLinie(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	if (y1==y2)
		drawHLine(x1, y1, x2-x1);
	else if (x1==x2)
		drawVLine(x1, y1, y2-y1);
	else
	{
		uint16_t	dx = (x2 > x1 ? x2 - x1 : x1 - x2);
		int16_t		xstep =  x2 > x1 ? 1 : -1;
		uint16_t	dy = (y2 > y1 ? y2 - y1 : y1 - y2);
		int16_t		ystep =  y2 > y1 ? 1 : -1;
		int16_t	col = x1, row = y1;

		if (dx < dy)
		{
			int16_t t = - (dy >> 1);
			while (1)
			{
				setXY (col, row, col, row);
				LCD_write_data16(fch, fcl);
				if (row == y2)
					return;
				row += ystep;
				t += dx;
				if (t >= 0)
				{
					col += xstep;
					t   -= dy;
				}
			}
		}
		else
		{
			int16_t t = - (dx >> 1);
			while (1)
			{
				setXY (col, row, col, row);
				LCD_write_data16(fch, fcl);
				if (col == x2)
					return;
				col += xstep;
				t += dy;
				if (t >= 0)
				{
					row += ystep;
					t   -= dx;
				}
			}
		}
	}
	clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// pisze znak na miejscu o podanych współrzędnych
// Parametry: c - znak; x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujZnak(uint8_t c, uint16_t x, uint16_t y)
{
	uint8_t i,ch;
	uint16_t j;
	uint16_t temp;
	uint16_t zz;

	if (!_transparent)
	{
		if (chOrient==POZIOMO)
		{
			setXY(x,y,x+cfont.x_size-1,y+cfont.y_size-1);

			temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
			for(j=0;j<((cfont.x_size/8)*cfont.y_size);j++)
			{
				ch = cfont.font[temp];
				for(i=0;i<8;i++)
				{
					if((ch&(1<<(7-i)))!=0)
						LCD_write_data16(fch, fcl);
					else
						LCD_write_data16(bch, bcl);
				}
				temp++;
			}
		}
		else
		{
			temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;

			for(j=0;j<((cfont.x_size/8)*cfont.y_size);j+=(cfont.x_size/8))
			{
				setXY(x,y+(j/(cfont.x_size/8)),x+cfont.x_size-1,y+(j/(cfont.x_size/8)));
				for (zz=(cfont.x_size/8)-1; zz>=0; zz--)
				{
					ch=cfont.font[temp+zz];
					for(i=0;i<8;i++)
					{
						if((ch&(1<<i))!=0)
							LCD_write_data16(fch, fcl);
						else
							LCD_write_data16(bch, bcl);
					}
				}
				temp+=(cfont.x_size/8);
			}
		}
	}
	else
	{
		temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
		for(j=0;j<cfont.y_size;j++)
		{
			for (zz=0; zz<(cfont.x_size/8); zz++)
			{
				ch = cfont.font[temp+zz];
				for(i=0;i<8;i++)
				{
					if((ch&(1<<(7-i)))!=0)
					{
						setXY(x+i+(zz*8),y+j,x+i+(zz*8)+1,y+j+1);
						LCD_write_data16(fch, fcl);
					}
				}
			}
			temp+=(cfont.x_size/8);
		}
	}
	//clrXY();
}






////////////////////////////////////////////////////////////////////////////////
// rysuje okrąg
// Parametry:
//  x, y - współrzdne środka
//  radius - promień
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujOkrag(uint16_t x, uint16_t y, uint16_t promien)
{
	int16_t f = 1 - promien;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * promien;
	int16_t x1 = 0;
	int16_t y1 = promien;

	//cbi(P_CS, B_CS);
	setXY(x, y + promien, x, y + promien);
	LCD_write_data16(fch, fcl);

	setXY(x, y - promien, x, y - promien);
	LCD_write_data16(fch, fcl);

	setXY(x + promien, y, x + promien, y);
	LCD_write_data16(fch, fcl);

	setXY(x - promien, y, x - promien, y);
	LCD_write_data16(fch, fcl);

	while(x1 < y1)
	{
		if(f >= 0)
		{
			y1--;
			ddF_y += 2;
			f += ddF_y;
		}
		x1++;
		ddF_x += 2;
		f += ddF_x;
		setXY(x + x1, y + y1, x + x1, y + y1);
		LCD_write_data16(fch, fcl);

		setXY(x - x1, y + y1, x - x1, y + y1);
		LCD_write_data16(fch, fcl);

		setXY(x + x1, y - y1, x + x1, y - y1);
		LCD_write_data16(fch, fcl);

		setXY(x - x1, y - y1, x - x1, y - y1);
		LCD_write_data16(fch, fcl);

		setXY(x + y1, y + x1, x + y1, y + x1);
		LCD_write_data16(fch, fcl);

		setXY(x - y1, y + x1, x - y1, y + x1);
		LCD_write_data16(fch, fcl);

		setXY(x + y1, y - x1, x + y1, y - x1);
		LCD_write_data16(fch, fcl);

		setXY(x - y1, y - x1, x - y1, y - x1);
		LCD_write_data16(fch, fcl);
	}
	//sbi(P_CS, B_CS);
	//clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// wyświetla bitmapę po jednym pikselu
// Parametry:
//  x, y - współrzędne ekranu
//  sx, sy - rozmiar bitmapy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujBitmape(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data)
{
	uint16_t col;
	uint32_t tx, ty, tc;

	if (chOrient == POZIOMO)
	{
		setXY(x, y, x+sx-1, y+sy-1);
		for (tc=0; tc<(sx*sy); tc++)
		{
			col = data[tc];
			LCD_write_data16(col>>8, col);
		}
	}
	else
	{
		for (ty=0; ty<sy; ty++)
		{
			setXY(x, y+ty, x+sx-1, y+ty);
			for (tx=sx-1; tx>=0; tx--)
			{
				col = data[(ty*sx)+tx];
				LCD_write_data16(col>>8, col);
			}
		}
	}
	//clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// wyświetla bitmapę przesyłajac po 16 bajtów bufora - zła kolejność kolorów
// Parametry:
//  x, y - współrzędne ekranu
//  sx, sy - rozmiar bitmapy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawBitmap2(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, uint16_t* data)
{
//	uint16_t col;
	uint32_t tc;

	if (chOrient == POZIOMO)
	{
		setXY(x, y, x+sx-1, y+sy-1);
		for (tc=0; tc<(sx*sy/8); tc++)
		{
			//col = data[tc];
			//LCD_write_data16(col>>8, col & 0xff);
			LCD_WrData((uint8_t*)data + (tc*16), 16);
			//LCD_WrData16(data+(tc*8), 8);
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// wyświetla bitmapę po 4 piksele z rotacją bajtów w wewnętrznym buforze. Dobre kolory i trochę szybciej niż po jednym pikselu
// Parametry:
//  x, y - współrzędne ekranu
//  sx, sy - rozmiar bitmapy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawBitmap3(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data)
{
	uint16_t piksel;
	uint32_t tc;
	uint8_t n, bufor[8];

	if (chOrient == POZIOMO)
	{
		setXY(x, y, x+sx-1, y+sy-1);
		for (tc=0; tc<(sx*sy/4); tc++)
		{
			for (n=0; n<4; n++)
			{
				piksel = data[tc*4+n];
				bufor[n*2+0] = piksel >>8;
				bufor[n*2+1] = piksel;
			}
			LCD_WrData(bufor, 8);
		}
	}

}



////////////////////////////////////////////////////////////////////////////////
// wyświetla bitmapę po 4 piksele przez DMA z rotacją bajtów w wewnętrznym buforze. Dobre kolory i trochę szybciej niż po jednym pikselu
// Parametry:
//  x, y - współrzędne ekranu
//  sx, sy - rozmiar bitmapy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawBitmap4(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data)
{
	uint16_t piksel;
	uint32_t tc;
	uint8_t n, bufor[16];

	chDMAgotowe = 1;
	if (chOrient == POZIOMO)
	{
		setXY(x, y, x+sx-1, y+sy-1);
		for (tc=0; tc<(sx*sy/8); tc++)
		{
			for (n=0; n<8; n++)
			{
				piksel = data[tc*8+n];
				bufor[n*2+0] = piksel >>8;
				bufor[n*2+1] = piksel;
			}
			do {}
			while (!chDMAgotowe);
			chDMAgotowe = 0;
			LCD_WrDataDMA(bufor, 16);
		}
	}
}
#endif



////////////////////////////////////////////////////////////////////////////////
// Odczytuje pamięć obrazu wyświetlacza
// Parametry:
// x1, y1 współrzędne początku obszaru
// x2, y2 współrzędne końca obszaru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void CzytajPamiecObrazu(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t* bufor)
{
	setXY(x1, y1, x2, y2);

	LCD_write_command16(0x2E);	//Memory Read

	for (uint16_t n=y1; n<y2; n++)
	{
		LCD_data_read(bufor+n*(x2-x1), x2-x1);
	}
}
