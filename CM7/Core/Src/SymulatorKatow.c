//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Sumulator kątów orientacji
// Obraca o zadany kat syntetyczne wektory: magnetyczny i przyspieszenia ziemskiego.
// Ich składowe XYZ są używane jako substytut pomiaru magnetometru i akcelerometru nie obarczony błędami pomiaru
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <SymulatorKatow.h>
#include "Wspolne.h"
#include <stdlib.h>
#include <LCD/ILI9488.h>
#include "Ekran.h"
#include <Rysuj.h>
#include <Czas.h>

//stSym_t stSym;	//zmienna struktury symulatora
extern uint8_t cRysujRaz;
extern char cNapis[100];
extern unia_wymianyCM7_t uDaneCM7;
uint32_t nPoprzedniCzas;


////////////////////////////////////////////////////////////////////////////////
// Inicjuje wektory początkowe ustawiając symulowany BSP poziomo skierowany na północ geograficzną
// Parametry: *stSym - wskaźnik na strukturę danych symulacji
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujSymulatorKątów(stSym_t *stSym)
{
	float fKatObrotuMagn[3];
	uint8_t cBłąd = BLAD_OK;

	for (uint8_t n=0; n<3; n++)
	{
		stSym->fAkcelerometr[n] = 0.0f;
		stSym->fMagnetometr[n] = 0.0f;
		stSym->fPrędkośćKątowa[n] = 0.1f;
	}

	stSym->fAkcelerometr[2] = AKCEL1G;			//oś Z akcelerometru wszkazuje przyspieszenie ziemskie
	stSym->fMagnetometr[0] = NOMINALNE_MAGN;	//oś X magnetometru wskazuje północ magnetyczną jak na równiku

	fKatObrotuMagn[0] = 0.0f;
	fKatObrotuMagn[1] = -INKLINACJA_MAG;		//inklinacja magnetyczna w radianach
	fKatObrotuMagn[2] = DEKLINACJA_MAG;			//deklinacja magnetyczna w radianach
	ObrocWektor(stSym->fMagnetometr, stSym->fMagnetometr, fKatObrotuMagn);

	//włącz symulację
	stSym->cFlagi = SYM_WLACZONA | SYM_OBRACAJ_X;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Właściwa symulacja obracajaca obona wektorami
// Parametry: *stSym - wskaźnik na strukturę danych symulacji
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t SymulujKątyOrientacji(stSym_t *stSym)
{
	uint8_t cBłąd = BLAD_OK;
	float fPrzyrostKąta[3];
	uint32_t nCzas, nCzasBieżący = PobierzCzasT6();

	nCzas = MinalCzas2(nPoprzedniCzas, nCzasBieżący);
	nPoprzedniCzas = nCzasBieżący;

	//całkuj prędkość kątową
	for (uint8_t n=0; n<3; n++)
	{
		fPrzyrostKąta[n] = (nCzas * stSym->fPrędkośćKątowa[n]) / 1000000;
		stSym->fCałkaKataObrotu[n] += fPrzyrostKąta[n];
		if (stSym->fCałkaKataObrotu[n] > 2*M_PI)
			stSym->fCałkaKataObrotu[n] -= 2*M_PI;
		if (stSym->fCałkaKataObrotu[n] < -2*M_PI)
			stSym->fCałkaKataObrotu[n] += 2*M_PI;
	}

	//obróć oba wektory o bieżący przyrost kąta
	ObrocWektor(stSym->fAkcelerometr, stSym->fAkcelerometr, fPrzyrostKąta);
	ObrocWektor(stSym->fMagnetometr, stSym->fMagnetometr, fPrzyrostKąta);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Obsługa okna dialogowego symulacji
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t SymulacjaKątówOrientacji(stSym_t *stSym)
{
	uint8_t cBłąd;

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		sprintf(cNapis, "Symulacja k%ct%cw orientacji", ą, ó);
		BelkaTytulu(cNapis);
		setColor(SZARY60);
		sprintf(cNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		RysujNapis(cNapis, CENTER, 300);
		InicjujSymulatorKątów(stSym);
	}
	cBłąd = SymulujKątyOrientacji(stSym);

	setColor(BIALY);
	for (uint8_t n=0; n<3; n++)
	{
		sprintf(cNapis, "Akcel %c = %.4f ", 'X'+n, stSym->fAkcelerometr[n]);
		RysujNapis(cNapis, 0, 30 + n * 20);
		sprintf(cNapis, "Magn %c = %.4f ", 'X'+n, stSym->fMagnetometr[n]);
		RysujNapis(cNapis, 200, 30 + n * 20);
	}
	return cBłąd;
}
