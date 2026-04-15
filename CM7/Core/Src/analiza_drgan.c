//////////////////////////////////////////////////////////////////////////////
//
// Wykonuje analizę drgań konstrukcji BSP zwiększając krokami obroty wybranych silników napędu.
// Dla każdego kroku wykonywany jest pomiar FFT dla akcelerometrów i żyroskopów a wyniki gromadzone są w wodospadach.
// Wybór aktywnych silników i zakres ich wysterowania jest ustawiany w aplikacji
//
// (c) PitLab 31 marzec 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <analiza_drgan.h>
#include "fft.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "czas.h"


extern float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fWynikFFT[LICZBA_TESTOW_FFT+1][LICZBA_ZMIENNYCH_FFT][FFT_MAX_ROZMIAR / 2];	//wartość sygnału wyjściowego
extern stFFT_t stKonfigFFT;
extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
extern uint8_t chRysujRaz;


////////////////////////////////////////////////////////////////////////////////
// Inicjalizuje sprzet do pomiarów
// Parametry:
//  *stKonfigFFT - wskaźnik na strukturę zawierającą konfigurację FFT, a konkretnie silniki do uruchomienia i górny zakres pracy
//  *chTrybPracy - wskaźnik na tryb pracy wyświetlania danych na LCD
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t RozpocznijAnalizęDrgań(stFFT_t *stKonfigFFT, uint8_t *chTrybPracy)
{
	uint8_t chBłąd = BLAD_OK;

	//wejdź do trybu FFT akcelerometrów, wyświetl wykres
	*chTrybPracy = TP_POMIARY_ANALIZA_DRGAN;
	chRysujRaz = 1;

	//przekaż do CM4 etap testu, informacje który silnik ma pracować i górną granicę wysterwania. CM4 wyliczy sobie z tego wysterowanie dla wszystkich etapów
	stKonfigFFT->chIndeksTestu = 0;
	uDaneCM7.dane.uRozne.U8[0] = 0;	//bieżące etap badania: 0..LICZBA_TESTOW_FFT
	uDaneCM7.dane.uRozne.U8[1] = stKonfigFFT->chAktywnSilniki;
	uDaneCM7.dane.uRozne.U8[2] = stKonfigFFT->chMaxWysterowanie;
	uDaneCM7.dane.chWykonajPolecenie = POL7_WYSTERUJ_SILNIKI_AD;
	return chBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Nadzoruje proces analizy drgań. Zwiększa wysterowanie, czeka na wykonanie FFT
// Parametry:
//  *stKonfigFFT - wskaźnik na strukturę zawierającą konfigurację FFT, a konkretnie silniki do uruchomienia i górny zakres pracy
//  *chTrybPracy - wskaźnik na tryb pracy wyświetlania danych na LCD
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KrokAnalizyDrgań(stFFT_t *stKonfigFFT, uint8_t *chTrybPracy)
{
	uint8_t chBłąd = BLAD_OK;

	if (stKonfigFFT->chIndeksTestu < LICZBA_TESTOW_FFT)
	{
		uDaneCM7.dane.uRozne.U8[0] = stKonfigFFT->chIndeksTestu;	//bieżące etap badania: 0..LICZBA_TESTOW_FFT
		uDaneCM7.dane.chWykonajPolecenie = POL7_WYSTERUJ_SILNIKI_AD;
	}
	else
	if (stKonfigFFT->chIndeksTestu == LICZBA_TESTOW_FFT)
	{
		uDaneCM7.dane.uRozne.U8[0] = 0;	//bieżące etap badania: 0..LICZBA_TESTOW_FFT
		uDaneCM7.dane.uRozne.U8[1] = 0;	//aktywne siliki
		uDaneCM7.dane.chWykonajPolecenie = POL7_PRZYWROC_NAPED;	//przywróć funkcję napędu dla silników po analizie FFT rezonansu ramy
		*chTrybPracy = TP_MENU_GLOWNE;
	}
	return chBłąd;
}
