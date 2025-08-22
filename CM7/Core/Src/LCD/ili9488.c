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
#include "main.h"
#include "semafory.h"
#include "cmsis_os.h"
// Wyświetlacz pracował na 25MHz ale później zaczął śmiecić na ekranie. Próbuję na 22,2MHz - jest OK


//deklaracje zmiennych
extern SPI_HandleTypeDef hspi5;
extern uint8_t chRysujRaz;
extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
extern uint8_t chOrient;
extern uint8_t _transparent;	//flaga określająca czy mamy rysować tło czy rysujemy na istniejącym
extern struct current_font cfont;
uint8_t chKolor666[3];		//tablica kolorów RGB pierwszego planu w formacie RGB 6-6-6
uint8_t chTlo666[3];		//kolory tła w formacie RGB 6-6-6




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
	HAL_Delay(120);

	LCD_write_command8(ILI9488_RDDID);	//Read display identification information
	LCD_data_read(chBuf, 4);

	LCD_write_command8(ILI9488_GMCTRP1);
	LCD_WrData((uint8_t *)"\x00\x03\x09\x08\x16\x0A\x3F\x78\x4C\x09\x0A\x08\x16\x1A\x0F", 15);

	LCD_write_command8(ILI9488_GMCTRN1);
	LCD_WrData((uint8_t *)"\x00\x16\x19\x03\x0F\x05\x32\x45\x46\x04\x0E\x0D\x35\x37\x0F", 15);

	LCD_write_command8(ILI9488_PWCTR1);	//Power Control 1
	LCD_WrData((uint8_t *)"\x17\x15", 2);		//Vreg1out, Verg2out

	LCD_write_command8(ILI9488_PWCTR1); 	//Power Control 2
	LCD_write_dat_jed8(0x41);   //VGH,VGL

	LCD_write_command8(ILI9488_VMCTR1);   //Power Control 3
	LCD_WrData((uint8_t *)"\x00\x12\x80", 3);	//Vcom

	LCD_Orient(POZIOMO);

	LCD_write_command8(ILI9488_PIXFMT);	// Interface Pixel Format
	LCD_write_dat_jed8(0x66);	//18 bit/*

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
	LCD_WrData((uint8_t *)"\x02\x02", 2);		 //MCU, Source,Gate scan dieection

	LCD_write_command8(0xE9);      // Set Image Functio
	LCD_write_dat_jed8(0x00);    // Disable 24 bit data

	LCD_write_command8(0xF7);      // Adjust Control
	LCD_WrData((uint8_t *)"\xA9\x51\x2C\x82", 4);	// D7 stream, loose */

	LCD_write_command8(ILI9488_SLPOUT);    //Exit Sleep
	HAL_Delay(120);
	LCD_write_command8(ILI9488_DISPON);    //Display on

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
void LCD_clear(uint16_t sKolor565)
{
	uint8_t chDane[12];

	if (!sKolor565)
		sKolor565 = BLACK;

	chRysujRaz = 1;
	setColor(sKolor565);		//wykonuje konwersję z RGB565 na RGB666
	setBackColor(sKolor565);

	LCD_write_command8(ILI9488_CASET);	//Column Address Set
	for (uint8_t x=0; x<2; x++)
		chDane[x] = 0;
	chDane[2] = 0x01;
	chDane[3] = 0xDF;		//479 = 0x1DF
	LCD_WrData(chDane, 4);

	LCD_write_command8(ILI9488_PASET);	//Page Address Set
	chDane[2] = 0x01;
	chDane[3] = 0x3F;		//319 = 0x13F
	LCD_WrData(chDane, 4);

	for (uint8_t x=0; x<4; x++)
	{
		chDane[3*x + 0] = chKolor666[0];
		chDane[3*x + 1] = chKolor666[1];
		chDane[3*x + 2] = chKolor666[2];
	}
	LCD_write_command8(ILI9488_RAMWR);	//Memory Write

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
		osDelay(1);
	HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	{
		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		for(uint16_t y=0; y<320; y++)
		{
			for(uint16_t x=0; x<480/4; x++)
				HAL_SPI_Transmit(&hspi5, chDane, 12, HAL_MAX_DELAY);
		}
			UstawDekoderZewn(CS_NIC);										//LCD_CS=1
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
}


////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor rysowania jako RGB konwertując go na RGB 6-6-6
// Parametry: r, g, b - składowe RGB koloru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setColorRGB(uint8_t r, uint8_t g, uint8_t b)
{
	//fch = ((r & 0xF8) | g>>5);
	//fcl = ((g & 0x1C)<<3 | b>>3);
	chKolor666[0] = r | 0x80;	//najstarszy bit jest zawsze 1
	chKolor666[1] = g | 0x80;
	chKolor666[2] = b | 0x80;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor rysowania jako natywny dla wyświetlacza konwertując RGB 5-6-5 na RGB 6-6-6
// Parametry: color - kolor
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setColor(uint16_t color)
{
	chKolor666[0] = (uint8_t)((color % 0xF800) >> 10)| 0x80;
	chKolor666[1] = (uint8_t)((color & 0x07E0) >> 5) | 0x80;
	chKolor666[2] = (uint8_t)((color & 0x001F) << 1) | 0x80;
}



////////////////////////////////////////////////////////////////////////////////
// pobiera aktywny kolor
// Parametry: nic
// Zwraca: kolor w formacie RGB 5-6-5
////////////////////////////////////////////////////////////////////////////////
uint16_t getColor(void)
{
	return ((uint16_t)((chKolor666[0] & 0x7C) << 1) << 11) + ((uint16_t)((chKolor666[1] & 0x7E) << 1) << 5) + ((chKolor666[2] & 0x7C) << 1);
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor tła jako RGB
// Parametry: r, g, b - składowe RGB koloru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setBackColorRGB(uint8_t r, uint8_t g, uint8_t b)
{
	chTlo666[0] = r | 0x80;	//najstarszy bit jest zawsze 1
	chTlo666[1] = g | 0x80;
	chTlo666[2] = b | 0x80;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor tła jako natywny dla wyświetlacza 5R+6G+5B
// Parametry: color - kolor
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setBackColor(uint16_t color)
{
	if (color == TRANSPARENT)
		_transparent = 1;
	else
	{
		chTlo666[0] = (uint8_t)((color % 0xF800) >> 10)| 0x80;
		chTlo666[1] = (uint8_t)((color & 0x07E0) >> 5) | 0x80;
		chTlo666[2] = (uint8_t)((color & 0x001F) << 1) | 0x80;
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
	return ((uint16_t)((chTlo666[0] & 0x7C) << 1) << 11) + ((uint16_t)((chTlo666[1] & 0x7E) << 1) << 5) + ((chTlo666[2] & 0x7C) << 1);
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
	uint8_t n, dane[12];

	LCD_write_command8(ILI9488_CASET);	//Column Address Set
	dane[0] = sStartX >> 8;
	dane[1] = sStartX;
	dane[2] = (sStartX + sSzerokosc - 1) >> 8;
	dane[3] = sStartX + sSzerokosc - 1;
	LCD_WrData(dane, 4);

	LCD_write_command8(ILI9488_PASET);	//Page Address Set
	dane[0] = sStartY >> 8;
	dane[1] = sStartY;
	dane[2] = (sStartY +  sWysokosc - 1) >> 8;
	dane[3] =  sStartY + sWysokosc - 1;
	LCD_WrData(dane, 4);

	setColor(kolor);
	LCD_write_command8(ILI9488_RAMWR);	//Memory Write
	for(n=0; n<4; n++)
	{
		LCD_WrData(chKolor666, 3);
		dane[3*n + 0] = chKolor666[0];
		dane[3*n + 1] = chKolor666[1];
		dane[3*n + 2] = chKolor666[2];
	}

	for(i=0; i<sWysokosc; i++)
	{
		for(j=0; j<sSzerokosc/4; j++)
			LCD_WrData(dane, 12);

		//ponieważ wypełnianie odbywa się paczkami po 4 pixele, może zdarzyć się że nie dopełni wszystkich danych
		k = sSzerokosc - j * 4;	//policz czy jest reszta
		if (k)
		{
			for(j=0; j<k; j++)
				LCD_WrData(dane, 3);	//dopełnij reszte
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
	//int i;

	if (len < 0)
	{
		len = -len;
		x -= len;
	}
	setXY(x, y, x+len, y);

	//for (i=0; i<len+1; i++)
		//LCD_WrData(chKolor666, 3);
	LCD_WrData(chKolor666, 3 * (len + 1));
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
	//int i;

	if (len < 0)
	{
		len = -len;
		y -= len;
	}
	setXY(x, y, x, y+len);

	//for (i=0; i<len+1; i++)
		//LCD_WrData(chKolor666, 3);
	LCD_WrData(chKolor666, 3 * (len + 1));
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

	LCD_write_command8(ILI9488_CASET);	//Column Address Set
	dane[0] = x1>>8;	//Start Column H
	dane[1] = x1;		//Start Column L
	dane[2] = x2>>8;	//End Column H
	dane[3] = x2;		//End Column L
	LCD_WrData(dane, 4);

	LCD_write_command8(ILI9488_PASET);	//Page Address Set
	dane[0] = y1>>8;	//Start Page H
	dane[1] = y1;		//Start Page L
	dane[2] = y2>>8;	//End Page H
	dane[3] = y2;		//End Page L
	LCD_WrData(dane, 4);

	LCD_write_command8(ILI9488_RAMWR);	//Memory Write. Po tym poleceniu następuje transfer danych
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
	LCD_WrData(chKolor666, 3);
}



////////////////////////////////////////////////////////////////////////////////
// pisze znak na miejscu o podanych współrzędnych
// Parametry: c - znak; x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void printChar(uint8_t c, uint16_t x, uint16_t y)
{
	uint8_t i, ch;
	uint16_t j;
	uint16_t temp;
	uint16_t zz;

	if (!_transparent)
	{
		if (chOrient == POZIOMO)
		{
			setXY(x, y, x + cfont.x_size - 1, y + cfont.y_size -  1);

			temp=((c - cfont.offset) * ((cfont.x_size / 8) * cfont.y_size)) + 4;
			for(j=0; j<((cfont.x_size / 8) * cfont.y_size); j++)
			{
				ch = cfont.font[temp];
				for(i=0; i<8; i++)
				{
					if ((ch&(1<<(7-i))) != 0)
						//LCD_write_data16(fch, fcl);
						LCD_WrData(chKolor666, 3);
					else
						//LCD_write_data16(bch, bcl);
						LCD_WrData(chTlo666, 3);
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
							//LCD_write_data16(fch, fcl);
							LCD_WrData(chKolor666, 3);
						else
							//LCD_write_data16(bch, bcl);
							LCD_WrData(chTlo666, 3);
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
						//LCD_write_data16(fch, fcl);
						LCD_WrData(chKolor666, 3);
					}
				}
			}
			temp+=(cfont.x_size/8);
		}
	}
	clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// wyświetla bitmapę po jednym pikselu
// Parametry:
//  x, y - współrzędne ekranu
//  sx, sy - rozmiar bitmapy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawBitmap(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data)
{
	uint16_t col;
	uint32_t tx, ty, tc;



	if (chOrient == POZIOMO)
	{
		setXY(x, y, x+sx-1, y+sy-1);
		while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
			osDelay(1);
		HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		for (tc=0; tc<(sx*sy); tc++)
		{
			col = data[tc];
			setColor(col);
			//LCD_WrData(chKolor666, 3);
			HAL_SPI_Transmit(&hspi5, chKolor666, 3, HAL_MAX_DELAY);
		}
		UstawDekoderZewn(CS_NIC);										//LCD_CS=1
		HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
	}
	else
	{
		for (ty=0; ty<sy; ty++)
		{
			setXY(x, y+ty, x+sx-1, y+ty);
			while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
				osDelay(1);
			HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
			UstawDekoderZewn(CS_LCD);										//LCD_CS=0
			HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
			for (tx=sx-1; tx>=0; tx--)
			{
				col = data[(ty*sx)+tx];
				setColor(col);
				//LCD_WrData(chKolor666, 3);
				HAL_SPI_Transmit(&hspi5, chKolor666, 3, HAL_MAX_DELAY);
			}
			UstawDekoderZewn(CS_NIC);										//LCD_CS=1
			HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
		}
	}

	clrXY();
}
#endif



