//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi pomiaru czasu z rozdzielczością 1 us
//
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "Czas.h"


extern TIM_HandleTypeDef htim7;



////////////////////////////////////////////////////////////////////////////////
// Pobiera stan licznika pracującego na 200MHz/200
// Parametry: brak
// Zwraca: stan licznika w mikrosekundach
////////////////////////////////////////////////////////////////////////////////
uint32_t PobierzCzasT7(void)
{
	extern volatile uint16_t sCzasH;
	return htim7.Instance->CNT + ((uint32_t)sCzasH<<16);
}



////////////////////////////////////////////////////////////////////////////////
// Liczy upływ czasu mierzony timerem T7 z rozdzieczością 1us
// Wersja z wewnetrznym czasem bieżącym
// Parametry: nPoczatek - licznik czasu na na początku pomiaru
// Zwraca: ilość czasu w mikrosekundach jaki upłynął do podanego czasu początkowego
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzasT7(uint32_t nPoczatek)
{
	uint32_t nCzasAkt;

	nCzasAkt = PobierzCzasT7();
	return MinalCzas2T7(nPoczatek, nCzasAkt);
}



////////////////////////////////////////////////////////////////////////////////
// Liczy upływ czasu mierzony timerem T7 z rozdzieczością 1us
// Wersja z zewnetrznym czasem bieżącym
// Parametry:
// [we] nPoczatek - licznik czasu na na początku pomiaru
// [we] nKoniec - licznik czasu na na końcu pomiaru
// Zwraca: ilość czasu w mikrosekundach jaki upłynął do podanego czasu początkowego
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas2T7(uint32_t nPoczatek, uint32_t nKoniec)
{
	uint32_t nCzas;

	if (nKoniec >= nPoczatek)
		nCzas = nKoniec - nPoczatek;
	else
		nCzas = 0xFFFFFFFF - nPoczatek + nKoniec;
	return nCzas;
}


