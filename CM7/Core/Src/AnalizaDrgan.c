//////////////////////////////////////////////////////////////////////////////
//
// Wykonuje analizę drgań konstrukcji BSP zwiększając krokami obroty wybranych silników napędu.
// Dla każdego kroku wykonywany jest pomiar FFT dla akcelerometrów i żyroskopów a wyniki gromadzone są
// w  pamięci DRAM i przezntowane w formie wodospadu
// Wybór aktywnych silników i zakres ich wysterowania jest ustawiany w aplikacji
// Dodatkowo dodana obsługa sprawzenia kolejności silników, która wykorzystuje ten sam mechanizm serowania silnikami w CM4
//
// (c) PitLab 31 marzec 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <Czas.h>
#include <FFT.h>
#include "AnalizaDrgan.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "KonfigFram.h"
#include "cmsis_os2.h"
#include "LCD.h"


extern float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fWynikFFT[LICZBA_TESTOW_FFT+1][LICZBA_ZMIENNYCH_FFT][FFT_MAX_ROZMIAR / 2];	//wartość sygnału wyjściowego
extern stFFT_t stKonfigFFT;
extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
extern uint8_t chRysujRaz;
stIdentyfikacjaSilnikow_t stIdentSiln;


////////////////////////////////////////////////////////////////////////////////
// Inicjalizuje sprzet do pomiarów
// Parametry:
//  *stKonfigFFT - wskaźnik na strukturę zawierającą konfigurację FFT, a konkretnie silniki do uruchomienia i górny zakres pracy
//  *chTrybPracy - wskaźnik na tryb pracy wyświetlania danych na LCD
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t RozpocznijAnalizęDrgań(stFFT_t *stKonfigFFT, uint8_t *chTrybPracy)
{
	uint8_t cBłąd = BLAD_OK;

	//wejdź do trybu FFT akcelerometrów, wyświetl wykres
	*chTrybPracy = TP_POMIARY_ANALIZA_DRGAN;
	chRysujRaz = 1;

	//przekaż do CM4 etap testu, informacje który silnik ma pracować i górną granicę wysterwania. CM4 wyliczy sobie z tego wysterowanie dla wszystkich etapów
	stKonfigFFT->chIndeksTestu = 0;
	//uDaneCM7.dane.uRozne.U8[0] = 0;	//bieżące etap badania: 0..LICZBA_TESTOW_FFT
	uDaneCM7.dane.sAdres = stKonfigFFT->chIndeksTestu;	//bieżące etap badania: 0..LICZBA_TESTOW_FFT
	uDaneCM7.dane.uRozne.U8[1] = stKonfigFFT->chAktywnSilniki;
	uDaneCM7.dane.uRozne.U8[2] = stKonfigFFT->chMaxWysterowanie;
	uDaneCM7.dane.chWykonajPolecenie = POL7_WYSTERUJ_SILNIKI_AD;
	return cBłąd;
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
	uint8_t cBłąd = BLAD_OK;

	if (stKonfigFFT->chIndeksTestu < LICZBA_TESTOW_FFT)
	{
		uDaneCM7.dane.sAdres = stKonfigFFT->chIndeksTestu;	//bieżące etap badania: 0..LICZBA_TESTOW_FFT
		uDaneCM7.dane.chWykonajPolecenie = POL7_WYSTERUJ_SILNIKI_AD;
	}
	else
	if (stKonfigFFT->chIndeksTestu >= LICZBA_TESTOW_FFT)
	{
		uDaneCM7.dane.uRozne.U8[0] = 0;	//bieżące etap badania: 0..LICZBA_TESTOW_FFT
		uDaneCM7.dane.uRozne.U8[1] = 0;	//aktywne siliki
		uDaneCM7.dane.chWykonajPolecenie = POL7_PRZYWROC_NAPED;	//przywróć funkcję napędu dla silników po analizie FFT rezonansu ramy
		*chTrybPracy = TP_MENU_GLOWNE;
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjalizuje sprzet do pomiarów
// Parametry:
//  *stKonfigFFT - wskaźnik na strukturę zawierającą konfigurację FFT, a konkretnie silniki do uruchomienia i górny zakres pracy
//  *chTrybPracy - wskaźnik na tryb pracy wyświetlania danych na LCD
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t RozpocznijIdentyfikacjęSilników(stIdentyfikacjaSilnikow_t *stIdentSiln, uint8_t *chTrybPracy)
{
	float fSkladowaPrzechylenia[KANALY_MIKSERA];
	float fSkladowaPochylenia[KANALY_MIKSERA];
	uint8_t cBłąd = BLAD_OK;

	*chTrybPracy = TP_PROCES_IDENT_SILN;
	chRysujRaz = 1;

	//odczytaj nastawy miksera aby wiedziec ile jest silników i jak są ułożone
	//silniki zawsze są ułożone zgodnie z tarczą zegara. Pierwszy silnik jest w okolicy godziny pierwszej, kolejne rosną w stronę ruchu wskazówek.
	//Jeżeli składowa przechylenia jest zerowa, to znaczy że silnik jest z przodu lub tyłu i nie bierze udziału w przechylaniu. Jeżeli skladowa
	//ma wartość maksymalną, to silnik jest pod kątem prostym na prawo lub lewo.
	cBłąd = CzytajFramFloat(FAU_MIX_PRZECH, KANALY_MIKSERA, fSkladowaPrzechylenia);
	cBłąd = CzytajFramFloat(FAU_MIX_POCHYL, KANALY_MIKSERA, fSkladowaPochylenia);

	//policz skonfigurowane silniki
	stIdentSiln->cLiczbaSilnikow = 0;
	for (uint8_t n=0; n<KANALY_MIKSERA; n++)
	{
		if ((fSkladowaPrzechylenia[n] > 1.0f) || (fSkladowaPrzechylenia[n] < -1.0f) || (fSkladowaPochylenia[n] > 1.0f) || (fSkladowaPochylenia[n] < -1.0f))
			stIdentSiln->cLiczbaSilnikow++;
		stIdentSiln->fSkładowaPrzechylenia[n] = fSkladowaPrzechylenia[n];
		stIdentSiln->fSkładowaPochylenia[n] = fSkladowaPochylenia[n];
	}

	cBłąd = CzytajFramChar(FAU_RC_WY_IDENT, 4, &uDaneCM4.dane.uRozne.U8[0]);
	stIdentSiln->nCzasPoprzedniegoEtapu = PobierzCzasT6() - (uint32_t)stIdentSiln->sCzasIdent * 1000;//Pobierz ujemny czas aby proces rozpoczał się od razu
	stIdentSiln->cNumerEtapu = 0;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje identyfikację silników wysterowując po kolei każdy z nich do określonej wartosci przez określony czas
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t IdentyfikacjaSilników(stIdentyfikacjaSilnikow_t *stIdentSiln)
{
	uint8_t cBłąd = BLAD_OK;




	return cBłąd;
}
