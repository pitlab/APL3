//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Funkcje wspólne dla obu rdzeni
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "wspolne.h"
#include "math.h"

////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia obrotu wektora wykonując jego mnożenie przez macierze obrotu: https://pl.wikipedia.org/wiki/Macierz_obrotu
// Parametry:
// [we] *fWektorWe - wskaźnik na tablicę float[3] z danymi do obrotu
// [wy] *fWektorWy - wskaźnik na tablicę float[3] z danymi po obrocie
// [we] *fKaty - wskaźnik na tablicę float[3] z kątami obrotu [rad]
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObrocWektor(float *fWektorWe, float *fWektorWy, float *fKat)
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

	/*(fWektorWy + 0) = fWektA[0];
	*(fWektorWy + 1) = fWektA[1];
	*(fWektorWy + 2) = fWektA[2]; */
}
