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
#include "moduly_wew.h"
#include "wymiana_CM4.h"

extern unia_wymianyCM4 uDaneCM4;
uint16_t sGenerator;



////////////////////////////////////////////////////////////////////////////////
// Pętla główna programu autopilota
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PetlaGlowna(void)
{
	uint8_t chDane[1024];
	uint16_t n;
	uint32_t nCzas;
	float fTemp;

	CzytajIdFRAM(chDane);
	ZapiszFRAM(0x2000, 0x55);
	chDane[0] = CzytajFRAM(0x2000);

	nCzas = HAL_GetTick();
	CzytajBuforFRAM(0x1000, chDane, 16);
	nCzas = MinalCzas(nCzas);

	WyslijDaneExpandera(0xAA);

	for (n=0; n<1024; n++)
		chDane[n] = (uint16_t)(n & 0xFF);

	nCzas = HAL_GetTick();
	ZapiszBuforFRAM(0x1000, chDane, 16);
	nCzas = MinalCzas(nCzas);

	WyslijDaneExpandera(0x55);

	//generuj testowe dane
	fTemp = (float)sGenerator/100;
	for (n=0; n<3; n++)
	{
		uDaneCM4.dane.fAkcel1[n] = 1 + n + fTemp;
		uDaneCM4.dane.fAkcel2[n] = 2 + n + fTemp;
		uDaneCM4.dane.fZyro1[n] = 3 + n + fTemp;
		uDaneCM4.dane.fZyro2[n] = 4 + n + fTemp;
		uDaneCM4.dane.fMagnet1[n] = 5 + n + fTemp;
		uDaneCM4.dane.fMagnet1[n] = 6 + n + fTemp;
	}
	sGenerator++;

	//wymień dane między rdzeniami
	PobierzDaneWymiany_CM7();
	UstawDaneWymiany_CM4();
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
