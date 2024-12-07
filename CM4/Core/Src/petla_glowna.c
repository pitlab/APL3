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

extern TIM_HandleTypeDef htim7;
extern volatile unia_wymianyCM4 uDaneCM4;
extern volatile unia_wymianyCM7 uDaneCM7;
uint16_t sGenerator;
uint16_t sCzasOstatniegoOdcinka;	//przechowuje czas uruchomienia ostatniego odcinka
uint8_t chNrOdcinkaCzasu;
uint16_t sCzasOdcinka[LICZBA_ODCINKOW_CZASU];		//zmierzony czas obsługo odcinka
uint16_t sMaxCzasOdcinka[LICZBA_ODCINKOW_CZASU];	//maksymalna wartość czasu odcinka
uint16_t sCzasJalowy;
uint8_t chBledyPetliGlownej = ERR_OK;
/*float fAkcel1[3], fAkcel2[3];
float fZyro1[3], fZyro2[3];
float fMagn1[3], fMagn2[3];
uint16_t sSerwa[16];*/



////////////////////////////////////////////////////////////////////////////////
// Pętla główna programu autopilota
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PetlaGlowna(void)
{
	uint8_t chDane[16];
	uint16_t n;
	uint32_t nCzas;
	float fTemp;


	switch (chNrOdcinkaCzasu)
	{
	case 0:		//obsługa modułu w gnieździe 1
		//generuj testowe dane
		fTemp = (float)sGenerator/100;
		sGenerator++;
		for (n=0; n<3; n++)
		{
			uDaneCM4.dane.fAkcel1[n] = 1 + n + fTemp;
			uDaneCM4.dane.fAkcel2[n] = 2 + n + fTemp;
			uDaneCM4.dane.fZyro1[n] = 3 + n + fTemp;
			uDaneCM4.dane.fZyro2[n] = 4 + n + fTemp;
			uDaneCM4.dane.fMagn1[n] = 5 + n + fTemp;
			uDaneCM4.dane.fMagn2[n] = 6 + n + fTemp;
		}
		chBledyPetliGlownej |= UstawDekoderModulow(ADR_MOD1);
		break;

	case 1:		//obsługa modułu w gnieździe 2
		for (n=0; n<16; n++)
		{
			uDaneCM4.dane.sSerwa[n] += n;
			if (uDaneCM4.dane.sSerwa[n] > 2000)
				uDaneCM4.dane.sSerwa[n] = 0;
		}
		chBledyPetliGlownej |= UstawDekoderModulow(ADR_MOD2);
		break;

	case 2:		//obsługa modułu w gnieździe 3
		fTemp = (float)sGenerator/100;
		uDaneCM4.dane.fWysokosc[0] = 20 + fTemp;
		uDaneCM4.dane.fWysokosc[1] = 10 + fTemp;
		chBledyPetliGlownej |= UstawDekoderModulow(ADR_MOD3);
		break;

	case 3:		//obsługa modułu w gnieździe 4
		chBledyPetliGlownej |= UstawDekoderModulow(ADR_MOD4);
		break;

	case 8:
		chBledyPetliGlownej |= WyslijDaneExpandera(0x55);
		break;

	case 9:
		chBledyPetliGlownej |= WyslijDaneExpandera(0xAA);
		break;

	case 10:
		CzytajIdFRAM(chDane);
		ZapiszFRAM(0x2000, 0x55);
		chDane[0] = CzytajFRAM(0x2000);

		nCzas = PobierzCzasT7();
		CzytajBuforFRAM(0x1000, chDane, 16);
		nCzas = MinalCzas(nCzas);

		for (n=0; n<16; n++)
			chDane[n] = (uint16_t)(n & 0xFF);

		nCzas = PobierzCzasT7();
		ZapiszBuforFRAM(0x1000, chDane, 16);
		nCzas = MinalCzas(nCzas);
		break;

	case 11:
		chBledyPetliGlownej |= UstawDekoderModulow(ADR_NIC);
		break;

	case 12:		break;

	case 15:	//wymień dane między rdzeniami
		uDaneCM4.dane.chBledyPetliGlownej = chBledyPetliGlownej;
		chBledyPetliGlownej  = PobierzDaneWymiany_CM7();
		chBledyPetliGlownej |= UstawDaneWymiany_CM4();
		break;

	default:	break;
	}

	//pomiar czasu zajetego w każdym odcinku
	sCzasOdcinka[chNrOdcinkaCzasu] = MinalCzas(sCzasOstatniegoOdcinka);
	if (sCzasOdcinka[chNrOdcinkaCzasu] > sMaxCzasOdcinka[chNrOdcinkaCzasu])   //przechwyć wartość maksymalną
		sMaxCzasOdcinka[chNrOdcinkaCzasu] = sCzasOdcinka[chNrOdcinkaCzasu];

	chNrOdcinkaCzasu++;
	if (chNrOdcinkaCzasu == LICZBA_ODCINKOW_CZASU)
	{
		chNrOdcinkaCzasu = 0;
	}


	//nadwyżkę czasu odcinka wytrać w jałowej petli
	do
	{
		sCzasJalowy = PobierzCzasT7() - sCzasOstatniegoOdcinka;
	}
	while (sCzasJalowy < CZAS_ODCINKA);

	sCzasOstatniegoOdcinka = PobierzCzasT7();
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera stan licznika pracującego na 200MHz/200
// Parametry: brak
// Zwraca: stan licznika w mikrosekundach
////////////////////////////////////////////////////////////////////////////////
uint16_t PobierzCzasT7(void)
{
	return htim7.Instance->CNT;
}

////////////////////////////////////////////////////////////////////////////////
// Liczy upływ czasu mierzony timerem T7 z rozdzieczością 1us
// Parametry: sPoczatek - licznik czasu na na początku pomiaru
// Zwraca: ilość czasu w mikrosekundach jaki upłynął do podanego czasu początkowego
////////////////////////////////////////////////////////////////////////////////
uint16_t MinalCzas(uint16_t sPoczatek)
{
	uint16_t sCzas, sCzasAkt;

	sCzasAkt = PobierzCzasT7();
	if (sCzasAkt >= sPoczatek)
		sCzas = sCzasAkt - sPoczatek;
	else
		sCzas = 0xFFFF - sPoczatek + sCzasAkt;
	return sCzas;
}


