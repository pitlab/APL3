//////////////////////////////////////////////////////////////////////////////
//
// Wykonuje analizę drgań konstrukcji BSP zwiększając krokami obroty wybranych silników napędu.
// Dla każdego kroku wykonywany jest pomiar FFT dla akcelerometrów i żyroskopów a wyniki gromadzone są w wodospadach.
// Wybór aktywnych silników i zakres ich wysterowania jest ustawiany w aplikacji
//
// (c) PitLab 31 marzec 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "analiza_drgań.h"
#include "fft.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "czas.h"


extern float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fWynikFFT[LICZBA_TESTOW_FFT][LICZBA_ZMIENNYCH_FFT][FFT_MAX_ROZMIAR / 2] = {0};	//wartość sygnału wyjściowego
extern stFFT_t stKonfigFFT;
extern unia_wymianyCM4_t uDaneCM4;



////////////////////////////////////////////////////////////////////////////////
// Inicjalizuje sprzet do pomiarów
// Parametry:
//  *stKonfigFFT - wskaźnik na strukturę zawierającą konfigurację FFT, a konkretnie silniki do uruchomienia i górny zakres pracy
//  *chTrybPracy - wskaźnik na tryb pracy wyświetlania danych na LCD
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t RozpocznijAnalizęDrgań(stFFT_t *stKonfigFFT, uint8_t *chTrybPracy)
{
	//wejdź do trybu FFT akcelerometrów, wyświetl wykres
	*chTrybPracy = TP_POMIARY_FFT_ACC;
	chRysujRaz = 1;

	//przekaź do CM4 wysterowanie wszystkich 8 silników a w ostatnim bajcie informacje który silnik ma pracować
	stKonfigFFT->chIndeksTestu = 0;
	for (uint8_t n=0; n<8; n++)
		uDaneCM4.dane.uRozne.U16[n] = 0;

	uDaneCM4.dane.uRozne.U8[16] = stKonfigFFT->chAktywnSilniki;

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
	*chTrybPracy = TP_POMIARY_FFT_ZYR;
	chRysujRaz = 1;

}
