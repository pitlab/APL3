//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi pętli głównej programu autopilota na rdzeniu CM4
//
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

#include "petla_glowna.h"
#include "main.h"
#include "fram.h"



////////////////////////////////////////////////////////////////////////////////
// Pętla główna programu autopilota
// Parametry: brak
// Zwraca: nic, program nigdy z niej nie wychodzi
////////////////////////////////////////////////////////////////////////////////
void PetlaGlowna(void)
{
	uint8_t chDane[1024];
	uint16_t n;
	uint32_t nCzas;

	CzytajIdFRAM(chDane);

	nCzas = HAL_GetTick();
	CzytajBuforFRAM(0x1000, chDane, 1024);
	nCzas = MinalCzas(nCzas);

	WyslijDaneExpandera(0xAA);

	for (n=0; n<1024; n++)
		chDane[n] = (uint16_t)(n & 0xFF);

	nCzas = HAL_GetTick();
	ZapiszBuforFRAM(0x1000, chDane, 1024);
	nCzas = MinalCzas(nCzas);

	WyslijDaneExpandera(0x55);
}



////////////////////////////////////////////////////////////////////////////////
// Liczy upływ czasu
// Parametry: nStart - licznik czasu na na początku pomiaru
// Zwraca: ilość czasu w milisekundach jaki upłynął do podanego czasu startu
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas(uint32_t nStart)
{
	uint32_t nCzas, nCzasAkt;

	nCzasAkt = HAL_GetTick();
	if (nCzasAkt >= nStart)
		nCzas = nCzasAkt - nStart;
	else
		nCzas = 0xFFFFFFFF - nStart + nCzasAkt;
	return nCzas;
}
