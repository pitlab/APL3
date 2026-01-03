//////////////////////////////////////////////////////////////////////////////
//
// Rysuje demo z fraktalami na LCD 480x320
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "rysuj.h"
#include "fraktale.h"
#include "czas.h"
#include <RPi35B_480x320.h>
#include <ili9488.h>

float fZoom, fX, fY;
float fReal, fImag;
uint16_t sMnozPalety;
uint8_t chDemoMode = START_FRAKTAL;	//zacznij od rotacji palety Mandelbrota
extern uint8_t chLiczIter;		//licznik iteracji fraktala
extern uint8_t chBuforLCD[];	//rozmiar bufora zależy od typu wyświetlacza
extern char chNapis[100];
extern uint8_t MidFont[];



////////////////////////////////////////////////////////////////////////////////
// Inicjalizuj dane fraktali przed uruchomieniem
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InitFraktal(unsigned char chTyp)
{
#define ITERATION	80
#define IMG_CONSTANT	-0.1
#define REAL_CONSTANT	0.65

	switch (chTyp)
	{
	case 0:	fReal = 0.38; 	fImag = -0.1;	sMnozPalety = 0x400;	break;	//Julia Ladne palety: 0xEEE, 0xEEEE
	case 1:	fX=-0.70; 	fY=0.60;	fZoom = -0.6;	sMnozPalety = 20;	break;		//cały fraktal - rotacja palety -0.7, 0.6, -0.6,
	case 2:	fX=-0.75; 	fY=0.18;	fZoom = -0.6;	sMnozPalety = 1500;	break;		//dolina konika x=-0,75, y=0,1
	case 3:	fX= 0.30; 	fY=0.05;	fZoom = -0.6;	sMnozPalety = 2000;	break;		//dolina słonia x=0,25-0,35, y=0,05, zoom=-0,6..-40
	}
	//chMnozPalety = 43;		//8, 13, 21, 30, 34, 43, 48, 56, 61
}



////////////////////////////////////////////////////////////////////////////////
// wyświetl demo z fraktalami
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void FraktalDemo(void)
{
	switch (chDemoMode)
	{
	case 0:	FraktalTest(0);		//rysuj fraktala Julii
		if (chLiczIter > 40)
		{
			chLiczIter = 0;
			chDemoMode++;
			InitFraktal(1);
		}
		break;

	case 1:	FraktalTest(1);		//rysuj fraktala Mandelbrota - cały fraktal rotacja kolorów
		if (chLiczIter > 60)
		{
			chLiczIter = 0;
			chDemoMode++;
			InitFraktal(2);
		}
		break;

	case 2:	FraktalTest(2);		//rysuj fraktala Mandelbrota - dolina konika
		if (chLiczIter > 20)
		{
			chLiczIter = 0;
			chDemoMode++;
			InitFraktal(3);
		}
		break;

	case 3:	FraktalTest(3);		//rysuj fraktala Mandelbrota - dolina słonia
		if (chLiczIter > 20)
		{
			chLiczIter = 0;
			chDemoMode++;
			InitFraktal(2);
		}
		break;

	case 4:	chDemoMode++;	break;

	case 5:	//LCD_Test();		chMode++;
		InitFraktal(0);
		chDemoMode = 0;	break;
	}
	chLiczIter++;
}



////////////////////////////////////////////////////////////////////////////////
// zmierz czas liczenia fraktala Julii
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void FraktalTest(uint8_t chTyp)
{
	uint32_t nCzas, nCzasRysowania = 0;

	nCzas = PobierzCzasT6();
	switch (chTyp)
	{
	case 0:	GenerateJulia(DISP_X_SIZE, DISP_Y_SIZE, 135, fReal, fImag, sMnozPalety, chBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Julia: t=%.2fms, c=%.3f ", nCzas/1000.0, fImag);
		fImag -= 0.002;
		break;

			//cały fraktal - rotacja palety
	case 1: GenerateMandelbrot(fX, fY, fZoom, 30, sMnozPalety, chBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Mandelbrot: t=%.2fms z=%.1f, p=%d", nCzas/1000.0, fZoom, sMnozPalety);
		sMnozPalety += 0x1042;
		break;

			//dolina konika x=-0,75, y=0,1
	case 2: GenerateMandelbrot(fX, fY, fZoom, 30, sMnozPalety, chBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Mandelbrot: t=%.2fms z=%.1f, p=%d", nCzas/1000.0, fZoom, sMnozPalety);
		fZoom /= 0.9;
		break;

			//dolina słonia x=0,25-0,35, y=0,05, zoom=-0,6..-40
	case 3: GenerateMandelbrot(fX, fY, fZoom, 30, sMnozPalety, chBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Mandelbrot: t=%.2fms z=%.1f, p=%d", nCzas/1000.0, fZoom, sMnozPalety);
		fZoom /= 0.9;
		break;
	}
#ifdef LCD_RPI35B
	//RysujBitmape2(0, 0, DISP_X_SIZE, DISP_Y_SIZE, sBuforLCD);		//wywyła większymi paczkami - zła kolejność ale szybciej
	RysujBitmape(0, 0, DISP_X_SIZE, DISP_Y_SIZE, (uint16_t*)chBuforLCD);		//wysyła bajty parami we właściwej kolejności
	//RysujBitmape3(0, 0, DISP_X_SIZE, DISP_Y_SIZE, sBuforLCD);		//wyświetla bitmapę po 4 piksele z rotacją bajtów w wewnętrznym buforze. Dobre kolory i trochę szybciej niż po jednym pikselu
	//RysujBitmape4(0, 0, DISP_X_SIZE, DISP_Y_SIZE, sBuforLCD);		//wyświetla bitmapę po 4 piksele przez DMA z rotacją bajtów w wewnętrznym buforze - nie działa
#endif
#ifdef LCD_ILI9488
	nCzasRysowania = PobierzCzasT6();
	RysujBitmape888(0, 0, DISP_X_SIZE, DISP_Y_SIZE, chBuforLCD);
	nCzasRysowania = MinalCzas(nCzasRysowania);
#endif
	UstawCzcionke(MidFont);
	setColor(ZIELONY);
	RysujNapis(chNapis, 0, 304);

	sprintf(chNapis, "trys=%.2fms", nCzasRysowania/1000.0);
	RysujNapis(chNapis, 0, 290);
}



////////////////////////////////////////////////////////////////////////////////
// Generuj fraktal Julii
// Parametry: sSzerokosc, sWysokosc - rozmiary obrazu
// sPrzesunSzer, sPrzesunWys - przesuniecie w X i Y
// sZoom - powiększenie
// fZespolRzecz, fZespolUroj - składowe rzeczywista i urojona liczby zespolonej
// chBufor = wskaźnik na bufor obrazu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void GenerateJulia(uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t sZoom, float fZespRzecz, float fZespUroj, uint16_t sPaleta, uint8_t* chBufor)
{
	float tmp1, tmp2;
	float num_real, num_img;
	float radius;
	uint16_t i;
	uint16_t x,y;
	uint32_t nOffset;
	uint32_t nPaleta;
	uint16_t sPrzesunSzer =  sSzerokosc / 2;
	uint16_t sPrzesunWys = sWysokosc / 2;

	for (y=0; y<sWysokosc; y++)
	{
		for (x=0; x<sSzerokosc; x++)
		{
			num_real = y - sPrzesunWys;
			num_real = num_real / sZoom;
			num_img = x - sPrzesunSzer;
			num_img = num_img / sZoom;
			i=0;
			radius = 0;
			while ((i<ITERATION-1) && (radius < 2))
			{
				tmp1 = num_real * num_real;
				tmp2 = num_img * num_img;
				num_img = 2*num_real*num_img + fZespUroj;
				num_real = tmp1 - tmp2 + fZespRzecz;
				radius = tmp1 + tmp2;
				i++;
			}
			//Wynik obliczeń zamień na kolor i wstaw do bufora obrazu
			//nPaleta = (i * chPaleta) << 2;
			nPaleta = i * sPaleta;
			nOffset = 3 * (x + y * sSzerokosc);

			chBufor[nOffset + 0] = (uint8_t)((nPaleta & 0xFF0000) >> 16);	//R
			chBufor[nOffset + 1] = (uint8_t)((nPaleta & 0x00FF00) >> 8);	//G
			chBufor[nOffset + 2] = (uint8_t)((nPaleta & 0x0000FF));			//B
		}
	}
}
#define CONTROL_SIZE_Y	128


////////////////////////////////////////////////////////////////////////////////
// Generuj fraktal Mandelbrota
// Parametry:
// sMaxIteracji - maksymalna liczba iteracji algorytmu dla pixela
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void GenerateMandelbrot(float fSrodekX, float fSrodekY, float Zoom, uint16_t sMaxIteracji, uint16_t sPaleta, uint8_t* chBufor)
{
	double X_Min = fSrodekX - 1.0/Zoom;
	double X_Max = fSrodekX + 1.0/Zoom;
	double Y_Min = fSrodekY - (DISP_Y_SIZE-CONTROL_SIZE_Y) / (DISP_X_SIZE * Zoom);
	double Y_Max = fSrodekY + (DISP_Y_SIZE-CONTROL_SIZE_Y) / (DISP_X_SIZE * Zoom);
	double dx = (X_Max - X_Min) / DISP_X_SIZE;
	double dy = (Y_Max - Y_Min) / (DISP_Y_SIZE-CONTROL_SIZE_Y) ;
	double y = Y_Min;
	uint16_t j, i;
	uint32_t n, nOffset;
	uint32_t nPaleta;
	double x, Zx, Zy, Zx2, Zy2, Zxy;

	for (j=0; j<DISP_Y_SIZE; j++)
	{
		x = X_Min;
		for (i = 0; i < DISP_X_SIZE; i++)
		{
			Zx = x;
			Zy = y;
			n = 0;
			while (n < sMaxIteracji)
			{
				Zx2 = Zx * Zx;
				Zy2 = Zy * Zy;
				Zxy = 2.0 * Zx * Zy;
				Zx = Zx2 - Zy2 + x;
				Zy = Zxy + y;
				if(Zx2 + Zy2 > 16.0)
					break;
				n++;
			}
			x += dx;
			//nPaleta = (n * chPaleta) << 2;
			nPaleta = n * sPaleta;
			nOffset = 3 * (i + j*DISP_X_SIZE);
			chBufor[nOffset + 0] = (uint8_t)((nPaleta & 0xFF0000) >> 16);	//R
			chBufor[nOffset + 1] = (uint8_t)((nPaleta & 0x00FF00) >> 8);	//G
			chBufor[nOffset + 2] = (uint8_t)((nPaleta & 0x0000FF));			//B
		}
		y += dy;
	}
}


