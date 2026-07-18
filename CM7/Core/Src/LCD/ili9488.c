//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi wyświetlacza TFT 320x480 ILI9488 pracującego na magistrali SPI (max 20MHz)
// Czasy rysowania ekranu z bufora 3 bajty na piksel:
// Przy taktowaniu 83MHz/4 225ms -> 4,4 fps
// Przy taktowaniu 100MHz/4 190ms -> 5,26 fps
// Przy taktowaniu 125MHz/4 154ms -> 6,49 fps
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <LCD/ILI9488.h>
#include <Main.h>
#include <Rysuj.h>
#include "ModulySPI.h"
#include "cmsis_os.h"
#include "LCD/LCD_SPI.h"
#include "Ekran.h"
#include "Semafory.h"
#include "cmsis_os.h"
// Wyświetlacz pracował na 25MHz ale później zaczął śmiecić na ekranie. Próbuję na 22,2MHz - jest OK


//deklaracje zmiennych
extern SPI_HandleTypeDef hspi5;
extern uint8_t cRysujRaz;
extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
extern uint8_t cOrientacja;
extern uint8_t _transparent;	//flaga określająca czy mamy rysować tło czy rysujemy na istniejącym
extern struct current_font cfont;
uint8_t cKolor666[3];		//tablica kolorów RGB pierwszego planu w formacie RGB 6-6-6
uint8_t cTlo666[3];		//kolory tła w formacie RGB 6-6-6
//extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) cBuforLCD[DISP_X_SIZE * DISP_Y_SIZE * 3];	//pamięć obrazu wyświetlacza w formacie RGB888



////////////////////////////////////////////////////////////////////////////////
// Konfiguracja wyświetlacza
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLCD_ILI9488(void)
{
	extern uint8_t cPort_exp_wysylany[LICZBA_EXP_SPI_ZEWN];
	uint8_t cBłąd = BLAD_OK;

	// LCD_RESET 1 - 0 - 1
	cPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	cBłąd |= WyslijDaneExpandera(SPI_EXTIO_0, cPort_exp_wysylany[0]);
	HAL_Delay(2);

	cPort_exp_wysylany[0] &= ~EXP01_LCD_RESET;	//RES=0
	cBłąd |= WyslijDaneExpandera(SPI_EXTIO_0, cPort_exp_wysylany[0]);
	HAL_Delay(2);

	cPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	cBłąd |= WyslijDaneExpandera(SPI_EXTIO_0, cPort_exp_wysylany[0]);
	HAL_Delay(120);									//czas powrotu z resetu max 120ms

	cBłąd |= LCD_write_command8(ILI9488_GMCTRP1);
	cBłąd |= LCD_WrData((uint8_t *)"\x00\x03\x09\x08\x16\x0A\x3F\x78\x4C\x09\x0A\x08\x16\x1A\x0F", 15);

	cBłąd |= LCD_write_command8(ILI9488_GMCTRN1);
	cBłąd |= LCD_WrData((uint8_t *)"\x00\x16\x19\x03\x0F\x05\x32\x45\x46\x04\x0E\x0D\x35\x37\x0F", 15);

	cBłąd |= LCD_write_command8(ILI9488_PWCTR1);	//Power Control 1
	cBłąd |= LCD_WrData((uint8_t *)"\x17\x15", 2);		//Vreg1out, Verg2out

	cBłąd |= LCD_write_command8(ILI9488_PWCTR1); 	//Power Control 2
	cBłąd |= LCD_write_dat_jed8(0x41);   //VGH,VGL

	cBłąd |= LCD_write_command8(ILI9488_VMCTR1);   //Power Control 3
	cBłąd |= LCD_WrData((uint8_t *)"\x00\x12\x80", 3);	//Vcom

	OrientacjaEkranu(POZIOMO);

	cBłąd |= LCD_write_command8(ILI9488_PIXFMT);	// Interface Pixel Format
	cBłąd |= 	LCD_write_dat_jed8(0x66);	//18 bit/*

	cBłąd |= LCD_write_command8(0xB0);      // Interface Mode Control
	cBłąd |= LCD_write_dat_jed8(
		(0 << 7)|	//SDA_EN: 3/4 wire serial interface selection: 0=DIN and SDO pins are used, 1=DIN/SDA pin is used for 3/4 wire serial interface and SDO pin is not used.
		(0 << 3)|	//VSPL: VSYNC polarity (0 = Low level sync clock, 1 = High level sync clock)
		(0 << 2)|	//HSPL: HSYNC polarity (0 = Low level sync clock, 1 = High level sync clock)
		(0 << 1)|	//DPL: DOTCLK polarity set (0 = data fetched at the rising time, 1 = data fetched at the falling time)
		(0 << 0));	//EPL: ENABLE polarity (0 = High enable for RGB interface, 1 = Low enable for RGB interface)

	cBłąd |= LCD_write_command8(ILI9488_FRMCTR1);      //Frame rate
	cBłąd |= LCD_write_dat_pie8(0xA0);   	//60Hz
	LCD_write_dat_ost8(0x11);		//17 clocks per line

	cBłąd |= LCD_write_command8(ILI9488_INVCTR);      //Display Inversion Control
	cBłąd |= LCD_write_dat_jed8(0x02);    //2-dot

	cBłąd |= LCD_write_command8(ILI9488_DFUNCTR);      //Display Function Control  RGB/MCU Interface Control
	cBłąd |= LCD_WrData((uint8_t *)"\x02\x02", 2);		 //MCU, Source,Gate scan dieection

	cBłąd |= LCD_write_command8(0xE9);      // Set Image Functio
	cBłąd |= LCD_write_dat_jed8(0x00);    // Disable 24 bit data

	cBłąd |= LCD_write_command8(0xF7);      // Adjust Control
	cBłąd |= LCD_WrData((uint8_t *)"\xA9\x51\x2C\x82", 4);	// D7 stream, loose */

	cBłąd |= LCD_write_command8(ILI9488_SLPOUT);    //Exit Sleep
	HAL_Delay(120);									//czas powrotu z resetu max 120ms
	cBłąd |= LCD_write_command8(ILI9488_DISPON);    //Display on
	if (cBłąd == BLAD_OK)
	{
		cRysujRaz = 1;
		nZainicjowanoCM7 |= INIT_LCD480x320;
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// ustawia orientację ekranu
// Parametry: orientacja - orientacja ekranu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void OrientacjaEkranu(uint8_t orientacja)
{
	/*uint8_t chDane[8];

	LCD_write_command16(0x04);
	LCD_data_read(chDane, 8);*/


	LCD_write_command8(0x36); // Memory Access Control
	if (orientacja)	//PIONOWO
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
	cOrientacja = orientacja;
}



////////////////////////////////////////////////////////////////////////////////
// Zapełnij cały ekran kolorem
// Parametry: sKolor565 - kolor w formacie RGB 5-6-5
// Zwraca: nic
// Czas czyszczenia ekranu: 378ms @25MHz
////////////////////////////////////////////////////////////////////////////////
uint8_t WypelnijEkran(uint16_t sKolor565)
{
	uint8_t chDane[12];
	uint8_t cBłąd = BLAD_OK;
	uint8_t cLicznikProbOdblokowania = 10;

	if (!sKolor565)
		sKolor565 = CZARNY;

	cRysujRaz = 1;
	setColor(sKolor565);		//wykonuje konwersję z RGB565 na RGB666
	setBackColor(sKolor565);

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5) != BLAD_OK)
	{
		osDelay(1);
		if (cLicznikProbOdblokowania)
			cLicznikProbOdblokowania--;
		else
			return BLAD_SEMAFOR_ZAJETY;
	}
	if (HAL_HSEM_Take(HSEM_SPI5, HSEM_LCD) == BLAD_OK)
	{
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
			chDane[3*x + 0] = cKolor666[0];
			chDane[3*x + 1] = cKolor666[1];
			chDane[3*x + 2] = cKolor666[2];
		}
		LCD_write_command8(ILI9488_RAMWR);	//Memory Write

		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		for(uint16_t y=0; y<320; y++)
		{
			for(uint16_t x=0; x<480/4; x++)
				HAL_SPI_Transmit(&hspi5, chDane, 12, HAL_DELAY_SPI);
		}
		UstawDekoderZewn(CS_NIC);										//LCD_CS=1
		HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor rysowania jako RGB konwertując go na RGB 6-6-6
// Parametry: r, g, b - składowe RGB koloru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setColorRGB(uint8_t r, uint8_t g, uint8_t b)
{
	cKolor666[0] = r;
	cKolor666[1] = g;
	cKolor666[2] = b;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor rysowania jako natywny dla wyświetlacza konwertując RGB 5-6-5 na RGB 6-6-6
// Parametry: sKolor565 - kolor w formacie RGB 5-6-5
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setColor(uint16_t sKolor565)
{
	cKolor666[0] = (uint8_t)((sKolor565 & 0xF800) >> 8);
	cKolor666[1] = (uint8_t)((sKolor565 & 0x07E0) >> 3);
	cKolor666[2] = (uint8_t) (sKolor565 & 0x001F) << 3;
}



////////////////////////////////////////////////////////////////////////////////
// pobiera aktywny kolor
// Parametry: nic
// Zwraca: kolor w formacie RGB 5-6-5
////////////////////////////////////////////////////////////////////////////////
uint16_t getColor(void)
{
	return ((uint16_t)(cKolor666[0] & 0xF8)  << 8) + ((uint16_t)(cKolor666[1] & 0xFC) << 3) + ((cKolor666[2] & 0xF8) >>3);
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor tła jako RGB
// Parametry: r, g, b - składowe RGB koloru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setBackColorRGB(uint8_t r, uint8_t g, uint8_t b)
{
	cTlo666[0] = r;
	cTlo666[1] = g;
	cTlo666[2] = b;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor tła jako natywny dla wyświetlacza 5R+6G+5B
// Parametry: sKolor565 - kolor w formacie RGB 5-6-5
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setBackColor(uint16_t sKolor565)
{
	if (sKolor565 == TRANSPARENT)
		_transparent = 1;
	else
	{
		cTlo666[0] = (uint8_t)((sKolor565 & 0xF800) >> 8);
		cTlo666[1] = (uint8_t)((sKolor565 & 0x07E0) >> 3);
		cTlo666[2] = (uint8_t) (sKolor565 & 0x001F) << 3;
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
	return ((uint16_t)(cTlo666[0] & 0xF8)  << 8) + ((uint16_t)(cTlo666[1] & 0xFC) << 3) + ((cTlo666[2] & 0xF8) >>3);
}



////////////////////////////////////////////////////////////////////////////////
// Rysuj prostokąt wypełniony kolorem
// Parametry:
//  sKolor565 - kolor w formacie RGB 5-6-5
// Zwraca: kod błędu
// Czas rysowania pełnego ekranu: 372ms @25MHz
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujProstokatWypelniony(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t sKolor565)
{
	uint16_t i, j, k;
	uint8_t n, dane[12];
	uint8_t cLicznikProbOdblokowania = 10;
	uint8_t cBłąd = BLAD_OK;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5) != BLAD_OK)
	{
		osDelay(1);
		if (cLicznikProbOdblokowania)
			cLicznikProbOdblokowania--;
		else
			return BLAD_SEMAFOR_ZAJETY;
	}
	if (HAL_HSEM_Take(HSEM_SPI5, HSEM_LCD) == BLAD_OK)
	{
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

		setColor(sKolor565);
		LCD_write_command8(ILI9488_RAMWR);	//Memory Write
		for(n=0; n<4; n++)
		{
			LCD_WrData(cKolor666, 3);
			dane[3*n + 0] = cKolor666[0];
			dane[3*n + 1] = cKolor666[1];
			dane[3*n + 2] = cKolor666[2];
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
		HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje piksel w zadanym kolorze
// Parametry: x, y współrzędne punktu
// chKolor - wskaźnik na tablicę kolorów RGB666
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujPunkt(int16_t x, int16_t y, uint8_t *chKolor)
{
	uint8_t cLicznikProbOdblokowania = 10;
	uint8_t cBłąd = BLAD_OK;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5) != BLAD_OK)
	{
		osDelay(1);
		if (cLicznikProbOdblokowania)
			cLicznikProbOdblokowania--;
		else
			return BLAD_SEMAFOR_ZAJETY;
	}
	if (HAL_HSEM_Take(HSEM_SPI5, HSEM_LCD) == BLAD_OK)
	{
		setXY(x, y, x, y);

		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1

		HAL_SPI_Transmit(&hspi5, chKolor, 3, HAL_DELAY_SPI);
		UstawDekoderZewn(CS_NIC);										//LCD_CS=1
		clrXY();
		HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje poziomą linią
// Parametry: x, y współrzędne początku
// len - długośc linii
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujLiniePozioma(int16_t x, int16_t y, int16_t len)
{
	uint8_t cLicznikProbOdblokowania = 10;
	uint8_t cBłąd = BLAD_OK;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5) != BLAD_OK)
	{
		osDelay(1);
		if (cLicznikProbOdblokowania)
			cLicznikProbOdblokowania--;
		else
			return BLAD_SEMAFOR_ZAJETY;
	}
	if (HAL_HSEM_Take(HSEM_SPI5, HSEM_LCD) == BLAD_OK)
	{
		if (len < 0)
		{
			len = -len;
			x -= len;
		}
		setXY(x, y, x+len, y);

		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		for (uint16_t n=0; n<len+1; n++)
			HAL_SPI_Transmit(&hspi5, cKolor666, 3, HAL_DELAY_SPI);
		UstawDekoderZewn(CS_NIC);										//LCD_CS=1
		clrXY();
		HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje pionową linię
// Parametry: x, y współrzędne początku
// len - długość linii
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujLiniePionowa(int16_t x, int16_t y, int16_t len)
{
	uint8_t cLicznikProbOdblokowania = 10;
	uint8_t cBłąd = BLAD_OK;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5) != BLAD_OK)
	{
		osDelay(1);
		if (cLicznikProbOdblokowania)
			cLicznikProbOdblokowania--;
		else
			return BLAD_SEMAFOR_ZAJETY;
	}
	if (HAL_HSEM_Take(HSEM_SPI5, HSEM_LCD) == BLAD_OK)
	{
		if (len < 0)
		{
			len = -len;
			y -= len;
		}
		setXY(x, y, x, y+len);
		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		for (uint16_t n=0; n<len+1; n++)
			HAL_SPI_Transmit(&hspi5, cKolor666, 3, HAL_DELAY_SPI);
		UstawDekoderZewn(CS_NIC);										//LCD_CS=1
		clrXY();
		HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// rysuj linię o współrzędnych x1,y1, x2,y3 we wcześniej zdefiniowanym kolorze
// Parametry: x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujLinie(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	uint8_t cLicznikProbOdblokowania = 10;
	uint8_t cBłąd = BLAD_OK;

	if (y1==y2)
		RysujLiniePozioma(x1, y1, x2-x1);
	else if (x1==x2)
		RysujLiniePionowa(x1, y1, y2-y1);
	else
	{
		uint16_t	dx = (x2 > x1 ? x2 - x1 : x1 - x2);
		int16_t		xstep =  x2 > x1 ? 1 : -1;
		uint16_t	dy = (y2 > y1 ? y2 - y1 : y1 - y2);
		int16_t		ystep =  y2 > y1 ? 1 : -1;
		int16_t	col = x1, row = y1;

		while (HAL_HSEM_IsSemTaken(HSEM_SPI5) != BLAD_OK)
		{
			osDelay(1);
			if (cLicznikProbOdblokowania)
				cLicznikProbOdblokowania--;
			else
				return BLAD_SEMAFOR_ZAJETY;
		}
		if (HAL_HSEM_Take(HSEM_SPI5, HSEM_LCD) == BLAD_OK)
		{
			if (dx < dy)
			{
				int16_t t = - (dy >> 1);
				while (1)
				{
					setXY (col, row, col, row);
					LCD_WrData(cKolor666, 3);

					if (row == y2)
					{
						HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
						return cBłąd;
					}
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
					LCD_WrData(cKolor666, 3);
					if (col == x2)
					{
						HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
						return cBłąd;
					}
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
		HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// ustawia parametry pamięci do rysowania linii
// Funkcja używana wewnętrz obszaru chronionego semaforem. Nie ma potrzeby chronić jej osobno
// Parametry:
// x1, y1 współrzędne początku obszaru
// x2, y2 współrzędne końca obszaru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t sTemp;
	uint8_t dane[4];

	if (cOrientacja == PIONOWO)
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
// Funkcja używana wewnętrz obszaru chronionego semaforem. Nie ma potrzeby chronić jej osobno
// Parametry:nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void clrXY(void)
{
	if (cOrientacja == PIONOWO)
		setXY(0, 0, DISP_X_SIZE, DISP_Y_SIZE);
	else
		setXY(0, 0, DISP_Y_SIZE, DISP_X_SIZE);
}



////////////////////////////////////////////////////////////////////////////////
// pisze znak na miejscu o podanych współrzędnych
// Parametry: c - znak; x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujZnak(uint8_t c, uint16_t x, uint16_t y)
{
	uint8_t i, ch;
	uint16_t j;
	uint16_t temp;
	uint16_t zz;
	uint8_t cLicznikProbOdblokowania = 10;
	uint8_t cBłąd = BLAD_OK;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5) != BLAD_OK)
	{
		osDelay(1);
		if (cLicznikProbOdblokowania)
			cLicznikProbOdblokowania--;
		else
			return BLAD_SEMAFOR_ZAJETY;
	}
	if (HAL_HSEM_Take(HSEM_SPI5, HSEM_LCD) == BLAD_OK)
	{
		if (!_transparent)
		{
			if (cOrientacja == POZIOMO)
			{
				setXY(x, y, x + cfont.x_size - 1, y + cfont.y_size -  1);
				UstawDekoderZewn(CS_LCD);										//LCD_CS=0
				HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
				temp=((c - cfont.offset) * ((cfont.x_size / 8) * cfont.y_size)) + 4;
				for(j=0; j<((cfont.x_size / 8) * cfont.y_size); j++)
				{
					ch = cfont.font[temp];
					for(i=0; i<8; i++)
					{
						if ((ch&(1<<(7-i))) != 0)
							cBłąd |= HAL_SPI_Transmit(&hspi5, cKolor666, 3, HAL_DELAY_SPI);
						else
							cBłąd |= HAL_SPI_Transmit(&hspi5, cTlo666, 3, HAL_DELAY_SPI);
					}
					temp++;
				}
				UstawDekoderZewn(CS_NIC);										//LCD_CS=1
			}
			else
			{
				temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
				for(j=0;j<((cfont.x_size/8)*cfont.y_size);j+=(cfont.x_size/8))
				{
					setXY(x,y+(j/(cfont.x_size/8)),x+cfont.x_size-1,y+(j/(cfont.x_size/8)));
					UstawDekoderZewn(CS_LCD);										//LCD_CS=0
					HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
					for (zz=(cfont.x_size/8)-1; zz>=0; zz--)
					{
						ch=cfont.font[temp+zz];
						for(i=0;i<8;i++)
						{
							if((ch&(1<<i))!=0)
								cBłąd |= HAL_SPI_Transmit(&hspi5, cKolor666, 3, HAL_DELAY_SPI);
							else
								cBłąd |= HAL_SPI_Transmit(&hspi5, cTlo666, 3, HAL_DELAY_SPI);
						}
					}
					UstawDekoderZewn(CS_NIC);										//LCD_CS=1
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
							LCD_WrData(cKolor666, 3);
						}
					}
				}
				temp+=(cfont.x_size/8);
			}
		}
		clrXY();
		HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// wyświetla bitmapę po jednym pikselu
// Parametry:
//  x, y - współrzędne ekranu
//  sx, sy - rozmiar bitmapy
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujBitmape(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* obraz)
{
	uint32_t tx, ty, tc;
	uint8_t cLicznikProbOdblokowania = 10;
	uint8_t cBłąd = BLAD_OK;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5) != BLAD_OK)
	{
		osDelay(1);
		if (cLicznikProbOdblokowania)
			cLicznikProbOdblokowania--;
		else
			return BLAD_SEMAFOR_ZAJETY;
	}
	if (HAL_HSEM_Take(HSEM_SPI5, HSEM_LCD) == BLAD_OK)
	{
		if (cOrientacja == POZIOMO)
		{
			setXY(x, y, x+sx-1, y+sy-1);
			UstawDekoderZewn(CS_LCD);										//LCD_CS=0
			HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
			for (tc=0; tc<(sx*sy); tc++)
			{
				setColor(obraz[tc]);
				cBłąd |= HAL_SPI_Transmit(&hspi5, cKolor666, 3, HAL_DELAY_SPI);
			}
			UstawDekoderZewn(CS_NIC);										//LCD_CS=1
		}
		else
		{
			for (ty=0; ty<sy; ty++)
			{
				setXY(x, y+ty, x+sx-1, y+ty);
				UstawDekoderZewn(CS_LCD);										//LCD_CS=0
				HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
				for (tx=sx-1; tx>=0; tx--)
				{
					setColor(obraz[(ty*sx)+tx]);
					cBłąd |= HAL_SPI_Transmit(&hspi5, cKolor666, 3, HAL_DELAY_SPI);
				}
				UstawDekoderZewn(CS_NIC);										//LCD_CS=1
			}
		}
		clrXY();
		HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// wyświetla bitmapę po jednym pikselu z pamięci gdzie każdy z kolorów RGB zajmuje 1 bajt
// Parametry:
//  x, y - współrzędne początku ekranu (Lewy Górny Róg)
//  sx, sy - rozmiar bitmapy
// chObraz - wskaźnik na obraz
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujBitmape888(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, uint8_t* chObraz)
{
	uint16_t col;
	uint32_t tx, ty;
	uint8_t cLicznikProbOdblokowania = 10;
	uint8_t cBłąd = BLAD_OK;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5) != BLAD_OK)
	{
		osDelay(1);
		if (cLicznikProbOdblokowania)
			cLicznikProbOdblokowania--;
		else
			return BLAD_SEMAFOR_ZAJETY;
	}
	if (HAL_HSEM_Take(HSEM_SPI5, HSEM_LCD) == BLAD_OK)
	{
		if (cOrientacja == POZIOMO)
		{
			setXY(x, y, x+sx-1, y+sy-1);
			UstawDekoderZewn(CS_LCD);										//LCD_CS=0
			HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
			for (uint16_t n=0; n<sy/32; n++)
				HAL_SPI_Transmit(&hspi5, chObraz + n*32*3*sx, sx*32*3, 480*HAL_DELAY_SPI);	//w jednym transferze wyślij 32 wiersze, gdzie piksel zajmuje 3 bajty
			UstawDekoderZewn(CS_NIC);										//LCD_CS=1
		}
		else
		{
			for (ty=0; ty<sy; ty++)
			{
				setXY(x, y+ty, x+sx-1, y+ty);
				UstawDekoderZewn(CS_LCD);										//LCD_CS=0
				HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
				for (tx=sx-1; tx>=0; tx--)
				{
					col  = (uint16_t)chObraz[(ty*sx)+tx + 0] << 8;
					col |= (uint16_t)chObraz[(ty*sx)+tx + 1];
					setColor(col);
					HAL_SPI_Transmit(&hspi5, cKolor666, 3, HAL_DELAY_SPI);
				}
				UstawDekoderZewn(CS_NIC);										//LCD_CS=1
			}
		}
		clrXY();
		HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// rysuje okrąg
// Parametry:
//  x, y - współrzdne środka
//  radius - promień
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RysujOkrag(uint16_t x, uint16_t y, uint16_t promien)
{
	int16_t f = 1 - promien;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * promien;
	int16_t x1 = 0;
	int16_t y1 = promien;
	uint8_t cLicznikProbOdblokowania = 10;
	uint8_t cBłąd = BLAD_OK;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5) != BLAD_OK)
	{
		osDelay(1);
		if (cLicznikProbOdblokowania)
			cLicznikProbOdblokowania--;
		else
			return BLAD_SEMAFOR_ZAJETY;
	}
	if (HAL_HSEM_Take(HSEM_SPI5, HSEM_LCD) == BLAD_OK)
	{
		setXY(x, y + promien, x, y + promien);
		LCD_WrData(cKolor666, 3);

		setXY(x, y - promien, x, y - promien);
		LCD_WrData(cKolor666, 3);

		setXY(x + promien, y, x + promien, y);
		LCD_WrData(cKolor666, 3);

		setXY(x - promien, y, x - promien, y);
		LCD_WrData(cKolor666, 3);

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
			LCD_WrData(cKolor666, 3);

			setXY(x - x1, y + y1, x - x1, y + y1);
			LCD_WrData(cKolor666, 3);

			setXY(x + x1, y - y1, x + x1, y - y1);
			LCD_WrData(cKolor666, 3);

			setXY(x - x1, y - y1, x - x1, y - y1);
			LCD_WrData(cKolor666, 3);

			setXY(x + y1, y + x1, x + y1, y + x1);
			LCD_WrData(cKolor666, 3);

			setXY(x - y1, y + x1, x - y1, y + x1);
			LCD_WrData(cKolor666, 3);

			setXY(x + y1, y - x1, x + y1, y - x1);
			LCD_WrData(cKolor666, 3);

			setXY(x - y1, y - x1, x - y1, y - x1);
			LCD_WrData(cKolor666, 3);
		}
		clrXY();
		HAL_HSEM_Release(HSEM_SPI5, HSEM_LCD);
	}
	return cBłąd;
}



