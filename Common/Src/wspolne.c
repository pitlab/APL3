//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Funkcje wspólne dla obu rdzeni
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "Wspolne.h"
#include "math.h"

////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia obrotu wektora z układu lokalnego BSP do globalnego
// wykonując jego mnożenie przez macierze obrotu: https://pl.wikipedia.org/wiki/Macierz_obrotu
// Koleność obrotu to Z->Y->Z i obracamy o dodatnie kąty
// Parametry:
// [we] *fWektorWe - wskaźnik na tablicę float[3] z danymi do obrotu
// [wy] *fWektorWy - wskaźnik na tablicę float[3] z danymi po obrocie
// [we] *fKaty - wskaźnik na tablicę float[3] z kątami obrotu [rad]
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObrocWektorLokalny(float *fWektorWe, float *fWektorWy, float *fKat)
//void ObrocWektor(float *fWektorWe, float *fWektorWy, float *fKat)
{
	float fWektA[3], fWektB[3];	//zmienne robocze

	//obrót wokół osi Z
	fWektA[0] = *(fWektorWe+0) * cosf(*(fKat+2)) - *(fWektorWe+1) * sinf(*(fKat+2));
	fWektA[1] = *(fWektorWe+0) * sinf(*(fKat+2)) + *(fWektorWe+1) * cosf(*(fKat+2));
	fWektA[2] = *(fWektorWe+2);

	//obrót wokół osi Y
	fWektB[0] = fWektA[0] * cosf(*(fKat+1)) + fWektA[2] * sinf(*(fKat+1));
	fWektB[1] = fWektA[1];
	fWektB[2] = fWektA[2] * cosf(*(fKat+1)) - fWektA[0] * sinf(*(fKat+1));

	//obrót wokół osi X
	*(fWektorWy + 0) = fWektB[0];
	*(fWektorWy + 1) = fWektB[1] * cosf(*(fKat+0)) - fWektB[2] * sinf(*(fKat+0));
	*(fWektorWy + 2) = fWektB[1] * sinf(*(fKat+0)) + fWektB[2] * cosf(*(fKat+0));
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia obrotu wektora układu globalnego do lokalnego
// wykonując jego mnożenie przez macierze obrotu: https://pl.wikipedia.org/wiki/Macierz_obrotu
// Koleność obrotu to X->Y->Z i obracamy o ujemne kąty
// Parametry:
// [we] *fWektorWe - wskaźnik na tablicę float[3] z danymi do obrotu
// [wy] *fWektorWy - wskaźnik na tablicę float[3] z danymi po obrocie
// [we] *fKaty - wskaźnik na tablicę float[3] z kątami obrotu [rad]
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObrocWektorGlobalny(float *fWektorWe, float *fWektorWy, float *fKat)
{
	float fWektA[3], fWektB[3];	//zmienne robocze

	//obrót wokół osi X
	fWektA[0] = *(fWektorWe+0);
	fWektA[1] = *(fWektorWe+1) * cosf(-*(fKat+0)) - *(fWektorWe+2) * sinf(-*(fKat+0));
	fWektA[2] = *(fWektorWe+1) * sinf(-*(fKat+0)) + *(fWektorWe+2) * cosf(-*(fKat+0));

	//obrót wokół osi Y
	fWektB[0] = fWektA[0] * cosf(-*(fKat+1)) + fWektA[2] * sinf(-*(fKat+1));
	fWektB[1] = fWektA[1];
	fWektB[2] = fWektA[2] * cosf(-*(fKat+1)) - fWektA[0] * sinf(-*(fKat+1));

	//obrót wokół osi Z
	*(fWektorWy + 0) = fWektB[0] * cosf(-*(fKat+2)) - fWektB[1] * sinf(-*(fKat+2));
	*(fWektorWy + 1)= fWektB[0] * sinf(-*(fKat+2)) + fWektB[1] * cosf(-*(fKat+2));
	*(fWektorWy + 2)= fWektB[2];
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja zawija kąt nie pozwalając mu wyjść poza zakres +-Pi
// Parametry:
// [we] fKat - zawijany kąt obrotu [rad]
// Zwraca: kąt po zawinięciu
////////////////////////////////////////////////////////////////////////////////
float ZawińPi(float fKąt)
{
	if (fKąt > M_PI)
		fKąt -= 2 * M_PI;
	if (fKąt < -M_PI)
		fKąt += 2 * M_PI;
	return fKąt;
}
