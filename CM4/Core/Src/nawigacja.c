//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Procedury nawigacyjne
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

#include "nawigacja.h"



typedef struct
{
	float fX;
	float fY;
} punkt_t;


////////////////////////////////////////////////////////////////////////////////
// Przekształcenie współrzędnych geograficznych na układ prostokątny wyrażony w metrach
// Źródło: Gruszewski "Bezpilotowe aparaty latające" str. 46
// Parametry:
// [we] fDlugoscOdniesienia - długość geograficzna punktu stanowiacego punkt odniesienia [rad]
// [we] fSzerokoscOdniesienia - szerokość geograficzna punktu stanowiacego punkt odniesienia [rad]
// [we] fDlugosc - długość geograficzna punktu któego pozycję przekształcamy na metry [rad]
// [we] fSzerokosc - szerokość geograficzna punktu któego pozycję przekształcamy na metry [rad]
// Zwraca: odległość w metrach punktu (fDlugosc, fSzerokosc) wzgledem punktu (fDlugoscOdniesienia, fSzerokoscOdniesienia)
// Czas trwania: ? na 200MHz
////////////////////////////////////////////////////////////////////////////////
punkt_t WspGeoNaProstoat(float fDlugoscOdniesienia, float fSzerokoscOdniesienia, float fDlugosc, float fSzerokosc)
{
	punkt_t Punkt;

	Punkt.fX = PROMIEN_ZIEMI * cosf(fSzerokosc) * (fDlugosc - fDlugoscOdniesienia);
	Punkt.fY = PROMIEN_ZIEMI * (fSzerokosc - fSzerokoscOdniesienia);
	return Punkt;
}
