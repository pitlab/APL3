//////////////////////////////////////////////////////////////////////////////
//
// Zestaw funkcji do obliczeń szybkiej transformaty Fouriera
//
// (c) PitLab 23 maj 2019-2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "fft.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "czas.h"

stZesp_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) xXomega[FFT_MAX_ROZMIAR] = {0};		//wynik transformaty
stZesp_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) xWnk_tab[FFT_MAX_ROZMIAR] = {0};	//raz wyliczona tablica współczynników, stała dla danego rozmiaru wektora N do potęgi k
stZesp_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) stWejscie[FFT_MAX_ROZMIAR] = {0};		//zmiennna wejściow
float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fBuforPomiarow[LICZBA_WYKRESOW_FFT+1][FFT_MAX_ROZMIAR] = {0};	//bufor do zbierania danych wejściowych
float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fWynikFFT[LICZBA_WYKRESOW_FFT][FFT_MAX_ROZMIAR / 2] = {0};	//wartość sygnału wyjściowego
stZesp_t xWn2;			//wartość stała dla danego rozmiaru wektora N
stZesp_t xWnk;			//wartość stała dla danego rozmiaru wektora N do potęgi k
uint16_t sIndeksProbki;
uint32_t nCzasFFT;
stFFT_t stKonfigFFT;
extern unia_wymianyCM4_t uDaneCM4;
uint8_t chOstatniIndeksSzybkiegoIMU;


////////////////////////////////////////////////////////////////////////////////
// Inicjalizuje sprzet do pomiarów
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjujFFT(void)
{
	stKonfigFFT.chIndeksZmiennejWe = 2;
	stKonfigFFT.chRodzajOkna = 1;
	stKonfigFFT.sWykladnikPotegi = 10;
	stKonfigFFT.sLiczbaProbek = (uint16_t)powf(2, stKonfigFFT.sWykladnikPotegi);
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera nowe dane, okienkuje je i ładuje do bufora.
// Parametry: nic Funkcja bezparametrowa bo jest wywoływana w wątku głównym
// faktyczne parametry przekazywane są w strukturze stFFT wskazując na rodzaj pobieranych danych i okno
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PobierzDaneDoFFT(void)
{
	while (uDaneCM4.dane.stSzybkieIMU.chIndeksProbki != chOstatniIndeksSzybkiegoIMU)
	{
		for (uint8_t n=0; n<3; n++)
			fBuforPomiarow[0][sIndeksProbki] = uDaneCM4.dane.stSzybkieIMU.fAkcel[chOstatniIndeksSzybkiegoIMU][n];

		//fBuforPomiarow[1][sIndeksProbki] = uDaneCM4.dane.fAkcel1[1];
		//fBuforPomiarow[2][sIndeksProbki] = uDaneCM4.dane.fAkcel1[2];
		/*fBuforPomiarow[3][sIndeksProbki] = uDaneCM4.dane.fZyroKal1[0];
		fBuforPomiarow[4][sIndeksProbki] = uDaneCM4.dane.fZyroKal1[1];
		fBuforPomiarow[5][sIndeksProbki] = uDaneCM4.dane.fZyroKal1[2];*/

		fBuforPomiarow[3][sIndeksProbki] = uDaneCM4.dane.stSzybkieIMU.chIndeksProbki;	//test
		chOstatniIndeksSzybkiegoIMU++;
		chOstatniIndeksSzybkiegoIMU &= ~MASKA_BUFORA_IMU;

		sIndeksProbki++;
		if (sIndeksProbki == stKonfigFFT.sLiczbaProbek)
		{
			sIndeksProbki = 0;
			stKonfigFFT.chStatus |= FFT_NOWE_DANE;		//mamy komplet danych, można liczyć FFT
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// liczy FFT dla pary danych. Dane zostają w tym samym miejscu
// Parametry:
// xWeWyP - struktura danych wejściowych-wyjściowych parzystych
// xWeWyN - struktura danych wejściowych-wyjściowych nieparzystych
// Zwraca: dane w tych samych miejacach
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
void FFT_Motyl(stZesp_t *stWeWyP, stZesp_t *stWeWyN, stZesp_t *stWnk)
{
	stZesp_t stWeP;
	stZesp_t stWeN;
	stZesp_t stTemp;

	//przepisz parzyste dane wejściowe do zmiennej
	stWeP.Re = stWeWyP->Re;
	stWeP.Im = stWeWyP->Im;

	//przepisz nieparzste dane
	stWeN.Re = stWeWyN->Re;
	stWeN.Im = stWeWyN->Im;

	//mnóż Wnk razy nieparzyste
	stTemp = MulComplex(*stWnk, stWeN);

	//górny wynik motylka
	stWeWyP->Re = stWeP.Re + stTemp.Re;
	stWeWyP->Im = stWeP.Im + stTemp.Im;

	//dolny wynik motylka
	stWeWyN->Re = stWeP.Re - stTemp.Re;
	stWeWyN->Im = stWeP.Im - stTemp.Im;
}



////////////////////////////////////////////////////////////////////////////////
// liczy szybką transformatę fouriera
// wylicza połowę współczynników Wnk uwzlędniajac warunek szczególny dla k=1
// wejście: *xXn
// wyjscie: *xXomega
// Czas wykonania: 61,7ms @256, 133,1 ms @512
////////////////////////////////////////////////////////////////////////////////
void LiczFFT(stFFT_t *konfig, uint8_t chIndeksWyniku)
{
	uint16_t N, b, j;
	float fTemp;

	//przepisz dane do struktury wyjściowej w kolejności odwróconego bitu
	for (uint16_t k=0; k<konfig->sLiczbaProbek; k++)
	{
		//odwróć bity w słowie o rozmiarze 2^sPotega
		j = 0;
		b = k;
		for (uint16_t x=0; x<konfig->sWykladnikPotegi; x++)
		{
			j <<= 1;
			j |= b & 1;
			b >>= 1;
		}

		xXomega[j].Re = stWejscie[k].Re;
		xXomega[j].Im = stWejscie[k].Im;
	}

	for (uint16_t x=1; x<konfig->sWykladnikPotegi+1; x++)		//iteracje drzewa podziałów x^2=PROBEK_DRGANIA
	{
		N = powf(2, x);	//oblicz liczbę współczynników Wnk = N = 2^x

		//Przygotuj współczynniki Wnk dla pierwszej połowy zakresu (druga będzie identyczna ze znakiem minus)
		for (uint16_t k=0; k<(N/2); k++)
		{
			if (k==0)		//warunek szczególny
			{
				xWnk_tab[k].Re = 1;
				xWnk_tab[k].Im = 0;
			}
			else
			{
				//licz e^(-j2Pik/N)
				fTemp = -2*M_PI*k/N;
				xWnk_tab[k].Re= cosf(fTemp);
				xWnk_tab[k].Im= -sinf(fTemp);
			}
		}

		//licz operacje na motylach na danym poziomie drzewa
		for (uint16_t j=0; j<N/2; j++)		//indeks motyla w grupie
		{
			for (uint16_t k=0; k<konfig->sLiczbaProbek/N; k++)		//indeks grupy motyli
			{
				FFT_Motyl(&xXomega[k*N + j], &xXomega[k*N + j + N/2], &xWnk_tab[j]);
			}
		}
	}

	for (uint16_t n=0; n<konfig->sLiczbaProbek/2; n++)
	{
		fTemp = sqrtf(xXomega[n].Re * xXomega[n].Re + xXomega[n].Im * xXomega[n].Im);	//moduł liczby zespolonej
		if (fTemp > 0)
			fWynikFFT[chIndeksWyniku][n] = log10(fTemp);
		else
			fWynikFFT[chIndeksWyniku][n] = 0;		//nie można liczyć logarytmu z zera i liczb ujemnych
	}

	konfig->chStatus =  FFT_GOTOWY_WYNIK;	//kasuj bit nowych danych i ustaw gotowość wyniku
}



////////////////////////////////////////////////////////////////////////////////
// mnożenie liczb zespolonych
// Parametry: stWejscie1, stWejscie2 - struktury obu mnożonych liczb
// Zwraca: struktura wyjściowa liczby zespolonej
////////////////////////////////////////////////////////////////////////////////
stZesp_t MulComplex(stZesp_t stWejscie1, stZesp_t stWejscie2)
{
	stZesp_t stWynik;

	stWynik.Re = (stWejscie1.Re * stWejscie2.Re) - (stWejscie1.Im * stWejscie2.Im);
	stWynik.Im = (stWejscie1.Im * stWejscie2.Re) + (stWejscie1.Re * stWejscie2.Im);
	return stWynik;
}



////////////////////////////////////////////////////////////////////////////////
// dodawanie liczb zespolonych
// Parametry: stWejscie1, stWejscie2 - struktury obu dodawanych liczb
// Zwraca: struktura wyjściowa liczby zespolonej
////////////////////////////////////////////////////////////////////////////////
stZesp_t AddComplex(stZesp_t stWejscie1, stZesp_t stWejscie2)
{
	stZesp_t stWynik;

	stWynik.Re = stWejscie1.Re + stWejscie2.Re;
	stWynik.Im = stWejscie1.Im + stWejscie2.Im;
	return stWynik;
}



////////////////////////////////////////////////////////////////////////////////
// podnoszenie liczb zespolonych do potęgi całkowitej na podstawie wzoru de Moivre'a:
// https://www.matemaks.pl/wzor-de-moivre-a-potegowanie-liczb-zespolonych.html
// Parametry: stWejscie - struktura wejściowa liczby zespolonej
// Zwraca: struktura wyjściowa liczby zespolonej
////////////////////////////////////////////////////////////////////////////////
stZesp_t PowComplex(stZesp_t stPodstawa, float fWykładnik)
{
	stZesp_t stWynik;
	float fModuł, fFi, fTemp;

	fModuł = sqrt(stPodstawa.Re*stPodstawa.Re + stPodstawa.Im*stPodstawa.Im);	//moduł z podstawy

	//oblicz kąt fi  https://pl.wikipedia.org/wiki/Liczby_zespolone
	if (stPodstawa.Im >= 0)
		fFi = acos(stPodstawa.Re / fModuł);
	else
		fFi = -1 * acos(stPodstawa.Re / fModuł);

	fTemp  = pow(fModuł, fWykładnik);	//moduł do potęgi

	stWynik.Re = fTemp * cosf(fWykładnik * fFi);
	stWynik.Im = fTemp * sinf(fWykładnik * fFi);
	return stWynik;
}



////////////////////////////////////////////////////////////////////////////////
// podnoszenie liczby e do potęgi zespolonej
// korzystając ze wzoru: e^ix = cos(x) + isin(x) patrz  https://pl.wikipedia.org/wiki/Liczby_zespolone
// Parametry: stWejscie - struktura wejściowa liczby zespolonej
// Zwraca: struktura wyjściowa liczby zespolonej
////////////////////////////////////////////////////////////////////////////////
stZesp_t PowEComplex(stZesp_t stWejscie)
{
	stZesp_t stWynik;

	stWynik.Re = cos(stWejscie.Im);
	stWynik.Im = sin(stWejscie.Im);
	return stWynik;
}



