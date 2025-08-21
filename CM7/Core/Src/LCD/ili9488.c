//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi wyświetlacza TFT 320x480 ILI9488 pracującego na magistrali SPI (max 20MHz)
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <ili9488.h>
#include "moduly_SPI.h"
#include "cmsis_os.h"
#include "LCD_SPI.h"
#include "display.h"
#include "rysuj.h"

// Wyświetlacz pracował na 25MHz ale później zaczął śmiecić na ekranie. Próbuję na 22,2MHz - jest OK


//deklaracje zmiennych
extern uint8_t chRysujRaz;
extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
extern uint8_t chOrient;
extern uint8_t fch, fcl, bch, bcl;	//kolory czcionki i tła (bajt starszy i młodszy)
extern uint8_t _transparent;	//flaga określająca czy mamy rysować tło czy rysujemy na istniejącym

////////////////////////////////////////////////////////////////////////////////
// Konfiguracja wyświetlacza
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLCD_ILI9488(void)
{
	extern uint8_t chPort_exp_wysylany[LICZBA_EXP_SPI_ZEWN];
	uint8_t chBuf[4];

	// LCD_RESET 1 - 0 - 1
	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(10);

	chPort_exp_wysylany[0] &= ~EXP01_LCD_RESET;	//RES=0
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(200);

	LCD_write_command8(ILI9488_RDDID);	//Read display identification information
	LCD_data_read(chBuf, 4);

	LCD_write_command8(ILI9488_GMCTRP1);
	LCD_write_dat_pie8(0x00);
	LCD_write_dat_sro8(0x03);
	LCD_write_dat_sro8(0x09);
	LCD_write_dat_sro8(0x08);
	LCD_write_dat_sro8(0x16);
	LCD_write_dat_sro8(0x0A);
	LCD_write_dat_sro8(0x3F);
	LCD_write_dat_sro8(0x78);
	LCD_write_dat_sro8(0x4C);
	LCD_write_dat_sro8(0x09);
	LCD_write_dat_sro8(0x0A);
	LCD_write_dat_sro8(0x08);
	LCD_write_dat_sro8(0x16);
	LCD_write_dat_sro8(0x1A);
	LCD_write_dat_ost8(0x0F);

	LCD_write_command8(ILI9488_GMCTRN1);
	LCD_write_dat_pie8(0x00);
	LCD_write_dat_sro8(0x16);
	LCD_write_dat_sro8(0x19);
	LCD_write_dat_sro8(0x03);
	LCD_write_dat_sro8(0x0F);
	LCD_write_dat_sro8(0x05);
	LCD_write_dat_sro8(0x32);
	LCD_write_dat_sro8(0x45);
	LCD_write_dat_sro8(0x0E);
	LCD_write_dat_sro8(0x0D);
	LCD_write_dat_sro8(0x35);
	LCD_write_dat_sro8(0x37);
	LCD_write_dat_ost8(0x0F);

	LCD_write_command8(ILI9488_PWCTR1);	//Power Control 1
	LCD_write_dat_pie8(0x17);	//Vreg1out
	LCD_write_dat_ost8(0x15);	//Verg2out

	LCD_write_command8(ILI9488_PWCTR1); 	//Power Control 2
	LCD_write_dat_jed8(0x41);   //VGH,VGL

	LCD_write_command8(ILI9488_VMCTR1);   //Power Control 3
	LCD_write_dat_pie8(0x00);
	LCD_write_dat_sro8(0x12);   //Vcom
	LCD_write_dat_ost8(0x80);

	LCD_write_command8(ILI9488_MADCTL);	//Memory Access Control
	LCD_write_dat_jed8(
		(1 << 7)|	//MY Row Address Order
		(1 << 6)|	//MX Column Address Order
		(1 << 5)|	//MV Row/Column Exchange
		(0 << 4)|	//ML Vertical Refresh Order
		(1 << 3)|	//BGR RGB-BGR Order
		(0 << 2));	//MH Horizontal Refresh ORDER

	LCD_write_command8(ILI9488_PIXFMT);	// Interface Pixel Format
	LCD_write_dat_jed8(0x66);	//18 bit

	LCD_write_command8(0xB0);      // Interface Mode Control
	LCD_write_dat_jed8(
		(0 << 7)|	//SDA_EN: 3/4 wire serial interface selection: 0=DIN and SDO pins are used, 1=DIN/SDA pin is used for 3/4 wire serial interface and SDO pin is not used.
		(0 << 3)|	//VSPL: VSYNC polarity (0 = Low level sync clock, 1 = High level sync clock)
		(0 << 2)|	//HSPL: HSYNC polarity (0 = Low level sync clock, 1 = High level sync clock)
		(0 << 1)|	//DPL: DOTCLK polarity set (0 = data fetched at the rising time, 1 = data fetched at the falling time)
		(0 << 0));	//EPL: ENABLE polarity (0 = High enable for RGB interface, 1 = Low enable for RGB interface)

	LCD_write_command8(ILI9488_FRMCTR1);      //Frame rate
	LCD_write_dat_jed8(0xA0);    //60Hz
	//powiny być 2 parametry

	LCD_write_command8(ILI9488_INVCTR);      //Display Inversion Control
	LCD_write_dat_jed8(0x02);    //2-dot

	LCD_write_command8(ILI9488_DFUNCTR);      //Display Function Control  RGB/MCU Interface Control
	LCD_write_dat_pie8(0x02);    //MCU
	LCD_write_dat_ost8(0x02);    //Source,Gate scan dieection
	//powinien być trzeci parametr

	LCD_write_command8(0xE9);      // Set Image Functio
	LCD_write_dat_jed8(0x00);    // Disable 24 bit data

	LCD_write_command8(0xF7);      // Adjust Control
	LCD_write_dat_pie8(0xA9);
	LCD_write_dat_sro8(0x51);
	LCD_write_dat_sro8(0x2C);
	LCD_write_dat_ost8(0x82);    // D7 stream, loose


	LCD_write_command8(ILI9488_SLPOUT);    //Exit Sleep
	HAL_Delay(120);

	LCD_write_command8(ILI9488_DISPON);    //Display on

	LCD_Orient(POZIOMO);

	LCD_write_command8(ILI9488_RDDID);	//Read display identification information
	LCD_data_read(chBuf, 4);

	chRysujRaz = 1;
	nZainicjowanoCM7 |= INIT_LCD480x320;
	return ERR_OK;
}



#ifdef LCD_ILI9488
////////////////////////////////////////////////////////////////////////////////
// ustawia orientację ekranu
// Parametry: orient - orientacja
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void LCD_Orient(uint8_t orient)
{
	/*uint8_t chDane[8];

	LCD_write_command16(0x04);
	LCD_data_read(chDane, 8);*/


	LCD_write_command8(0x36); // Memory Access Control
	if (orient)	//PIONOWO
	{
		LCD_write_dat_jed8(
			(0<<2)|	//MH Horizontal Refresh ORDER
			(1<<3)|	//BGR RGB-BGR Order: 0: RGB=RGB; 1:RGB=BGR
			(0<<4)|	//ML Vertical Refresh Order
			(0<<5)|	//MV Row / Column Exchange
			(1<<6)|	//MX Column Address Order
			(0<<7));	//MY Row Address Order
	}
	else	//POZIOMO
	{
		LCD_write_dat_jed8(
			(0<<2)|	//MH Horizontal Refresh ORDER
			(1<<3)|	//BGR RGB-BGR Order: 0: RGB=RGB; 1:RGB=BGR
			(0<<4)|	//ML Vertical Refresh Order
			(1<<5)|	//MV Row / Column Exchange
			(1<<6)|	//MX Column Address Order
			(1<<7));	//MY Row Address Order
	}
	chOrient = orient;
}



////////////////////////////////////////////////////////////////////////////////
// Zapełnij cały ekran kolorem
// Parametry: nic
// Zwraca: nic
// Czas czyszczenia ekranu: 378ms @25MHz
////////////////////////////////////////////////////////////////////////////////
void LCD_clear(uint16_t kolor)
{
	uint32_t y;
	uint8_t x, dane[8];

	//for(y=0; y<DISP_X_SIZE * DISP_Y_SIZE; y++)
		//sBuforLCD[y] = GRAY20;
	if (!kolor)
		kolor = BLACK;

	chRysujRaz = 1;
	setColor(kolor);
	setBackColor(kolor);

	LCD_write_command8(0x2A);	//Column Address Set
	for (x=0; x<8; x++)
		dane[x] = 0;
	dane[5] = 0x01;
	dane[7] = 0xDF;		//479 = 0x1DF
	LCD_WrData(dane, 8);

	LCD_write_command8(0x2B);	//Page Address Set
	dane[5] = 0x01;
	dane[7] = 0x3F;		//319 = 0x13F
	LCD_WrData(dane, 8);

	LCD_write_command8(0x2C);	//Memory Write
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
// Rysuj prostokąt wypełniony kolorem
// Parametry:
// Zwraca: nic
// Czas rysowania pełnego ekranu: 372ms @25MHz
////////////////////////////////////////////////////////////////////////////////
void LCD_ProstokatWypelniony(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t kolor)
{
	uint16_t i, j, k;
	uint8_t n, dane[8];

	for (n=0; n<8; n++)
		dane[n] = 0;

	LCD_write_command8(0x2A);	//Column Address Set
	dane[1] = sStartX >> 8;
	dane[3] = sStartX;
	dane[5] = (sStartX + sSzerokosc - 1) >> 8;
	dane[7] = sStartX + sSzerokosc - 1;
	LCD_WrData(dane, 8);

	LCD_write_command8(0x2B);	//Page Address Set
	dane[1] = sStartY >> 8;
	dane[3] = sStartY;
	dane[5] = (sStartY +  sWysokosc - 1) >> 8;
	dane[7] =  sStartY + sWysokosc - 1;
	LCD_WrData(dane, 8);

	LCD_write_command8(0x2C);	//Memory Write
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
void drawHLine(int16_t x, int16_t y, int16_t len)
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
void drawVLine(int16_t x, int16_t y, int16_t len)
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
	uint8_t dane[4];

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

	LCD_write_command8(0x2A);	//Column Address Set
	dane[0] = x1>>8;	//Start Column H
	dane[1] = x1;		//Start Column L
	dane[2] = x2>>8;	//End Column H
	dane[3] = x2;		//End Column L
	LCD_WrData(dane, 4);

	LCD_write_command8(0x2B);	//Page Address Set
	dane[0] = y1>>8;	//Start Page H
	dane[1] = y1;		//Start Page L
	dane[2] = y2>>8;	//End Page H
	dane[3] = y2;		//End Page L
	LCD_WrData(dane, 4);

	LCD_write_command8(0x2C);	//Memory Write. Po tym poleceniu następuje transfer danych
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



#endif



