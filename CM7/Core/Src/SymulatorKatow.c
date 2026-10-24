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
#include "Dotyk.h"
#include <Audio.h>
#include "SampleAudio.h"

//stSym_t stSym;	//zmienna struktury symulatora
extern uint8_t cRysujRaz;
extern char cNapis[100];
extern unia_wymianyCM7_t uDaneCM7;
uint32_t nPoprzedniCzas;
extern stStatusDotyku_t stStatusDotyku;

////////////////////////////////////////////////////////////////////////////////
// Inicjuje wektory początkowe ustawiając symulowany BSP poziomo skierowany na północ geograficzną
// Parametry: *stSym - wskaźnik na strukturę danych symulacji
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujSymulatorKątów(stSym_t *stSym)
{
	uint8_t cBłąd = BLAD_OK;

	InicjujKąty(stSym);
	for (uint8_t n=0; n<3; n++)
		stSym->fPrędkośćKątowa[n] = 0.0f;

	//włącz symulację
	stSym->cFlagi = SYM_WLACZONA | SYM_OBRACAJ_X;
	stSym->fPrędkośćKątowa[0] = 0.1f;

	//wyślij polecenie zerowania kątów całkowanych z żyroskopu
	uDaneCM7.dane.cWykonajPolecenie = POL7_ZERUJ_KATY_ZYRO;		//zeruje kąty całkowane z prędkosci katowych żyroskopów
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjuje wektory początkowe ustawiając symulowany BSP poziomo skierowany na północ geograficzną
// Parametry: *stSym - wskaźnik na strukturę danych symulacji
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujKąty(stSym_t *stSym)
{
	float fKatObrotuMagn[3];
	uint8_t cBłąd = BLAD_OK;

	for (uint8_t n=0; n<3; n++)
	{
		stSym->fAkcelerometr[n] = 0.0f;
		stSym->fMagnetometr[n] = 0.0f;
		stSym->fCałkaKataObrotu[n] = 0.0f;
	}
	stSym->fAkcelerometr[2] = AKCEL1G;			//oś Z akcelerometru wszkazuje przyspieszenie ziemskie
	stSym->fMagnetometr[0] = NOMINALNE_MAGN;	//oś X magnetometru wskazuje północ magnetyczną jak na równiku

	fKatObrotuMagn[0] = 0.0f;
	fKatObrotuMagn[1] = -INKLINACJA_MAG;		//inklinacja magnetyczna w radianach
	fKatObrotuMagn[2] = DEKLINACJA_MAG;			//deklinacja magnetyczna w radianach
	ObrocWektorLokalny(stSym->fMagnetometr, stSym->fMagnetometr, fKatObrotuMagn);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Właściwa symulacja obracająca oboma wektorami
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
		fPrzyrostKąta[n] = (nCzas * stSym->fPrędkośćKątowa[n]) / -1000000;	//obrót wymaga ujemnej prędkości kątowej
		stSym->fCałkaKataObrotu[n] += fPrzyrostKąta[n];
		if (stSym->fCałkaKataObrotu[n] > M_PI)
			stSym->fCałkaKataObrotu[n] -= 2*M_PI;
		if (stSym->fCałkaKataObrotu[n] < -M_PI)
			stSym->fCałkaKataObrotu[n] += 2*M_PI;
	}

	//obróć oba wektory o bieżący przyrost kąta
	ObrocWektorGlobalny(stSym->fAkcelerometr, stSym->fAkcelerometr, fPrzyrostKąta);
	ObrocWektorGlobalny(stSym->fMagnetometr, stSym->fMagnetometr, fPrzyrostKąta);
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

		//Narysuj przyciski
		RysujLiniePozioma(0, 200, DISP_X_SIZE);
		RysujLiniePozioma(0, 300, DISP_X_SIZE);
		RysujLiniePionowa(160, 200, 100);
		RysujLiniePionowa(320, 200, 100);
		for (uint8_t n=0; n<3; n++)
		{
			if (stSym->fPrędkośćKątowa[n] > MIN_PREDKOSC_KATOWA)
				setColor(ZOLTY);
			else
				setColor(SZARY60);

			sprintf(cNapis, "Obr%ct wok%c%c %c", ó, ó, ł, 'X'+n);
			RysujNapis(cNapis, 30 + n*160, 250);
		}
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

	if(stStatusDotyku.cFlagi & DOTYK_DOTKNIETO)
	{
		//sprawdź współrzędne dotyku Y z lekkim zapasem +10
		if (stStatusDotyku.sY > 190)		//naciśnieto na przyciski w dole ekranu
		{
			InicjujKąty(stSym);		//nową symulację zaczynaj od ustalonego stanu
			for (uint8_t n=0; n<3; n++)
			{
				if ((stStatusDotyku.sX > 160 * (n+0)) && (stStatusDotyku.sX < 160 * (n+1)))	//naciśnięto przycisk obrotu wokół osi[n]
				{
					if (stSym->fPrędkośćKątowa[n] > MIN_PREDKOSC_KATOWA)
					{
						stSym->fPrędkośćKątowa[n] = 0.0f;
						setColor(SZARY60);
					}
					else
					{
						stSym->fPrędkośćKątowa[n] = 0.1f;
						setColor(ZOLTY);
					}
					sprintf(cNapis, "Obr%ct wok%c%c %c", ó, ó, ł, 'X'+n);
					RysujNapis(cNapis, 30 + n*160, 250);
				}
			}
		}
		else	//naciśnieto poza przyciskami, więc wyjdź
		{
			cBłąd = BLAD_GOTOWE;
			stSym->cFlagi = 0;	//wyłącz symulację
		}
		DodajProbkeDoMalejKolejki(PGA_PRZYCISK, ROZM_MALEJ_KOLEJKI_KOMUNIK);		//odtwórz komunikat audio przycisku
		stStatusDotyku.cFlagi &= ~DOTYK_DOTKNIETO;	//wyczyść flagę dotknięcia
	}
	return cBłąd;
}
