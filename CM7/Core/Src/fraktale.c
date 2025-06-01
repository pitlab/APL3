//////////////////////////////////////////////////////////////////////////////
//
// Rysuje demo z fraktalami na LCD 480x320
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "fraktale.h"
#include "czas.h"
#include "RPi35B_480x320.h"


float fZoom, fX, fY;
float fReal, fImag;
uint8_t chMnozPalety;
uint8_t chDemoMode;
extern uint8_t chLiczIter;		//licznik iteracji fraktala
extern uint16_t sBuforLCD[DISP_X_SIZE * DISP_Y_SIZE];
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
//#define IMG_CONSTANT	0.285
//#define REAL_CONSTANT	0.005
//#define IMG_CONSTANT	-0.73
//#define REAL_CONSTANT	0.19
#define IMG_CONSTANT	-0.1
#define REAL_CONSTANT	0.65

	switch (chTyp)
	{
	case 0:	fReal = 0.38; 	fImag = -0.1;	break;	//Julia
	case 1:	fX=-0.70; 	fY=0.60;	fZoom = -0.6;	chMnozPalety = 2;	break;		//cały fraktal - rotacja palety -0.7, 0.6, -0.6,
	case 2:	fX=-0.75; 	fY=0.18;	fZoom = -0.6;	chMnozPalety = 15;	break;		//dolina konika x=-0,75, y=0,1
	case 3:	fX= 0.30; 	fY=0.05;	fZoom = -0.6;	chMnozPalety = 43;	break;		//dolina słonia x=0,25-0,35, y=0,05, zoom=-0,6..-40
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
	uint32_t nCzas;

	nCzas = PobierzCzasT6();
	switch (chTyp)
	{
	case 0:	GenerateJulia(DISP_X_SIZE, DISP_Y_SIZE, DISP_X_SIZE/2, DISP_Y_SIZE/2, 135, sBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Julia: t=%ldus, c=%.3f ", nCzas, fImag);
		fImag -= 0.002;
		break;

			//ca�y fraktal - rotacja palety
	case 1: GenerateMandelbrot(fX, fY, fZoom, 30, sBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Mandelbrot: t=%ldus z=%.1f, p=%d", nCzas, fZoom, chMnozPalety);
		chMnozPalety += 1;
		break;

			//dolina konika x=-0,75, y=0,1
	case 2: GenerateMandelbrot(fX, fY, fZoom, 30, sBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Mandelbrot: t=%ldus z=%.1f, p=%d", nCzas, fZoom, chMnozPalety);
		fZoom /= 0.9;
		break;

			//dolina s�onia x=0,25-0,35, y=0,05, zoom=-0,6..-40
	case 3: GenerateMandelbrot(fX, fY, fZoom, 30, sBuforLCD);
		nCzas = MinalCzas(nCzas);
		sprintf(chNapis, "Mandelbrot: t=%ldus z=%.1f, p=%d", nCzas, fZoom, chMnozPalety);
		fZoom /= 0.9;
		break;
	}

	//drawBitmap2(0, 0, DISP_X_SIZE, DISP_Y_SIZE, sBuforLCD);		//wywyła większymi paczkami - zła kolejność ale szybciej
	//drawBitmap(0, 0, DISP_X_SIZE, DISP_Y_SIZE, sBuforLCD);		//wysyła bajty parami we właściwej kolejności
	drawBitmap3(0, 0, DISP_X_SIZE, DISP_Y_SIZE, sBuforLCD);		//wyświetla bitmapę po 4 piksele z rotacją bajtów w wewnętrznym buforze. Dobre kolory i trochę szybciej niż po jednym pikselu
	//drawBitmap4(0, 0, DISP_X_SIZE, DISP_Y_SIZE, sBuforLCD);		//wyświetla bitmapę po 4 piksele przez DMA z rotacją bajtów w wewnętrznym buforze - nie działa
	setFont(MidFont);
	setColor(GREEN);
	print(chNapis, 0, 304);
}



////////////////////////////////////////////////////////////////////////////////
// Generuj fraktal Julii
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void GenerateJulia(unsigned short size_x, unsigned short size_y, unsigned short offset_x, unsigned short offset_y, unsigned short zoom, unsigned short * buffer)
{
	float tmp1, tmp2;
	float num_real, num_img;
	float radius;
	unsigned short i;
	unsigned short x,y;

	for (y=0; y<size_y; y++)
	{
		for (x=0; x<size_x; x++)
		{
			num_real = y - offset_y;
			num_real = num_real / zoom;
			num_img = x - offset_x;
			num_img = num_img / zoom;
			i=0;
			radius = 0;
			while ((i<ITERATION-1) && (radius < 2))
			{
				tmp1 = num_real * num_real;
				tmp2 = num_img * num_img;
				num_img = 2*num_real*num_img + fImag;
				num_real = tmp1 - tmp2 + fReal;
				radius = tmp1 + tmp2;
				i++;
			}
			/* Store the value in the buffer */
			buffer[x+y*size_x] = i*20;
		}
	}
}


#define CONTROL_SIZE_Y	128


////////////////////////////////////////////////////////////////////////////////
// Generuj fraktal Mandelbrota
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void GenerateMandelbrot(float centre_X, float centre_Y, float Zoom, unsigned short IterationMax, unsigned short * buffer)
{
	double X_Min = centre_X - 1.0/Zoom;
	double X_Max = centre_X + 1.0/Zoom;
	double Y_Min = centre_Y - (DISP_Y_SIZE-CONTROL_SIZE_Y) / (DISP_X_SIZE * Zoom);
	double Y_Max = centre_Y + (DISP_Y_SIZE-CONTROL_SIZE_Y) / (DISP_X_SIZE * Zoom);
	double dx = (X_Max - X_Min) / DISP_X_SIZE;
	double dy = (Y_Max - Y_Min) / (DISP_Y_SIZE-CONTROL_SIZE_Y) ;
	double y = Y_Min;
	unsigned short j, i;
	int n;
	double x, Zx, Zy, Zx2, Zy2, Zxy;

	for (j=0; j<DISP_Y_SIZE; j++)
	{
		x = X_Min;
		for (i = 0; i < DISP_X_SIZE; i++)
		{
			Zx = x;
			Zy = y;
			n = 0;
			while (n < IterationMax)
			{
				Zx2 = Zx * Zx;
				Zy2 = Zy * Zy;
				Zxy = 2.0 * Zx * Zy;
				Zx = Zx2 - Zy2 + x;
				Zy = Zxy + y;
				if(Zx2 + Zy2 > 16.0)
				{
					break;
				}
				n++;
			}
			x += dx;

			buffer[i+j*DISP_X_SIZE] = n*chMnozPalety;
		}
		y += dy;
	}
}


