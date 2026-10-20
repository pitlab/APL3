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

stSym_t stSym;	//zmienna struktury symulatora


////////////////////////////////////////////////////////////////////////////////
// Inicjuje wektory początkowe ustawiając symulowany BSP poziomo skierowany na północ geograficzną
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujSymulatorKątów(void)
{
	float fKatObrotuMagn[3];
	uint8_t cBłąd = BLAD_OK;

	for (uint8_t n=0; n<3; n++)
	{
		stSym.fAkcelerometr[n] = 0.0f;
		stSym.fMagnetometr[n] = 0.0f;
		stSym.fPrędkośćKątowa[n] = 0.0f;
	}

	stSym.fAkcelerometr[2] = AKCEL1G;	//oś Z akcelerometru wszkazuje przyspieszenie ziemskie
	stSym.fMagnetometr[0] = NOMINALNE_MAGN;	//oś X magnetometru wskazuje północ magnetyczną jak na równiku

	fKatObrotuMagn[0] = 0.0f;
	fKatObrotuMagn[1] = -INKLINACJA_MAG;		//inklinacja magnetyczna w radianach
	fKatObrotuMagn[2] = DEKLINACJA_MAG;			//deklinacja magnetyczna w radianach
	ObrocWektor(stSym.fMagnetometr, stSym.fMagnetometr, fKatObrotuMagn);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjuje wektory początkowe ustawiając symulowany BSP poziomo skierowany na północ geograficzną
// Parametry: nDeltaCzasu - czas w mikrosekundach jaki upłynął od ostatniej symulacji
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t SymulujKątyOrientacji(uint32_t nDeltaCzasu)
{
	uint8_t cBłąd = BLAD_OK;
	float fPrzyrostKata[3];

	//całkuj prędkość kątową
	for (uint8_t n=0; n<3; n++)
	{
		fPrzyrostKata[n] = (nDeltaCzasu / 1000000) * stSym.fPrędkośćKątowa[n];
		stSym.fCałkaKataObrotu[n] += fPrzyrostKata[n];
		if (stSym.fCałkaKataObrotu[n] > 2*M_PI)
			stSym.fCałkaKataObrotu[n] -= 2*M_PI;
		if (stSym.fCałkaKataObrotu[n] < -2*M_PI)
			stSym.fCałkaKataObrotu[n] += 2*M_PI;
	}

	//obróć oba wektory o bieżący przyrost kąta
	ObrocWektor(stSym.fAkcelerometr, stSym.fAkcelerometr, fPrzyrostKata);
	ObrocWektor(stSym.fMagnetometr, stSym.fMagnetometr, fPrzyrostKata);
	return cBłąd;
}
