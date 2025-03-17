//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obliczenia metodą najmniejszych kwadratów
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

#include "MetodaNajmniejszychKwadratow.h"


//struktura przechoująca punkt o współrzędnych całkowitych
typedef struct
{
	int16_t sX;
	int16_t sY;
} stPunkt16_t;

typedef struct
{
	float fA;
	float fB;
} stWspProstej_t;


////////////////////////////////////////////////////////////////////////////////
// Liczenie równania prostej metodą najmniejszych kwaratów
// Parametry: *punkty - wskaźnik na tablicę struktury punktów tworzących prostą
// sLiczbaPunktow - liczba podanych przez wskaźnik punktów
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
// https://pl.wikipedia.org/wiki/Metoda_najmniejszych_kwadrat%C3%B3w
// Dla funkcji f(x) = Ax + B funkcja minimalizacyjna to Suma(od i=1 do n) (yi - Axi - B)^2 / (delta i)^2
void MNK(stPunkt16_t* punkty, uint16_t sLiczbaPunktow)
{
	//int32_t nSumX, nSumY, nSumX2, nSumY2, nSum;

}
