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
#include "GNSS.h"
#include "nmea.h"

extern TIM_HandleTypeDef htim7;
extern volatile unia_wymianyCM4_t uDaneCM4;
extern volatile unia_wymianyCM7_t uDaneCM7;
uint16_t sGenerator;
uint32_t nCzasOstatniegoOdcinka;	//przechowuje czas uruchomienia ostatniego odcinka
uint8_t chNrOdcinkaCzasu;
uint32_t nCzasOdcinka[LICZBA_ODCINKOW_CZASU];		//zmierzony czas obsługo odcinka
uint32_t nMaxCzasOdcinka[LICZBA_ODCINKOW_CZASU];	//maksymalna wartość czasu odcinka
uint32_t nCzasJalowy;
uint8_t chBledyPetliGlownej = ERR_OK;
uint8_t chStanIOwy, chStanIOwe;	//stan wejść IO modułów wewnetrznych
extern uint8_t chBuforAnalizyGNSS[ROZMIAR_BUF_ANA_GNSS];
extern volatile uint8_t chWskNapBaGNSS, chWskOprBaGNSS;
//uint32_t nZainicjowanoCM4[2] = {0, 0};		//flagi inicjalizacji sprzętu

uint8_t chGeneratorNapisow;

////////////////////////////////////////////////////////////////////////////////
// Pętla główna programu autopilota
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PetlaGlowna(void)
{
	uint8_t chErr;//, chDane[16];
	uint16_t n;
	//uint32_t nCzas;
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

	case 4:		//obsługa GNSS na UART8
		while (chWskNapBaGNSS != chWskOprBaGNSS)
		{
			chErr = DekodujNMEA(chBuforAnalizyGNSS[chWskOprBaGNSS]);	//analizuj dane z GNSS
			chWskOprBaGNSS++;
			chWskOprBaGNSS &= MASKA_ROZM_BUF_ANA_GNSS;
			chStanIOwy ^= 0x80;		//Zielona LED
		}
		if ((uDaneCM4.dane.nZainicjowano & INIT_GNSS_GOTOWY) == 0)
			InicjujGNSS();		//gdy nie jest zainicjowany to przeprowadź odbiornik przez kolejne etapy inicjalizacji
		break;

	case 8:

		chBledyPetliGlownej |= WyslijDaneExpandera(chStanIOwy);
		break;

	case 9:
		//chBledyPetliGlownej |= WyslijDaneExpandera(chStanIOwy);
		break;

	/*case 10:
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
		break;*/

	case 11:
		chBledyPetliGlownej |= UstawDekoderModulow(ADR_NIC);
		break;

	case 12:	//test przekazywania napisów
		chGeneratorNapisow++;
		if (!chGeneratorNapisow)
		{
			const uint8_t chNapis[] = "CM4 pracuje\n\r";
			for (uint8_t n=0; n<sizeof(chNapis); n++)
			  uDaneCM4.dane.chNapis[n] = chNapis[n];
		}
		break;

	case 15:	//wymień dane między rdzeniami
		uDaneCM4.dane.chBledyPetliGlownej = chBledyPetliGlownej;
		chBledyPetliGlownej  = PobierzDaneWymiany_CM7();
		chErr = UstawDaneWymiany_CM4();
		if (chErr == ERR_SEMAFOR_ZAJETY)
			chStanIOwy &= ~0x40;	//zaświeć czerwoną LED
		else
			chStanIOwy |= 0x40;		//zgaś czerwoną LED
		chBledyPetliGlownej |= chErr;
		//chStanIOwy ^= 0x80;		//Zielona LED
		break;



	default:	break;
	}

	//pomiar czasu zajętego w każdym odcinku
	nCzasOdcinka[chNrOdcinkaCzasu] = MinalCzas(nCzasOstatniegoOdcinka);
	if (nCzasOdcinka[chNrOdcinkaCzasu] > nMaxCzasOdcinka[chNrOdcinkaCzasu])   //przechwyć wartość maksymalną
		nMaxCzasOdcinka[chNrOdcinkaCzasu] = nCzasOdcinka[chNrOdcinkaCzasu];

	chNrOdcinkaCzasu++;
	if (chNrOdcinkaCzasu == LICZBA_ODCINKOW_CZASU)
	{
		chNrOdcinkaCzasu = 0;
	}


	//nadwyżkę czasu odcinka wytrać w jałowej petli
	do
	{
		nCzasJalowy = PobierzCzasT7() - nCzasOstatniegoOdcinka;
	}
	while (nCzasJalowy < CZAS_ODCINKA);

	nCzasOstatniegoOdcinka = PobierzCzasT7();
}



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
// Parametry: sPoczatek - licznik czasu na na początku pomiaru
// Zwraca: ilość czasu w mikrosekundach jaki upłynął do podanego czasu początkowego
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas(uint32_t nPoczatek)
{
	uint32_t nCzas, nCzasAkt;

	nCzasAkt = PobierzCzasT7();
	if (nCzasAkt >= nPoczatek)
		nCzas = nCzasAkt - nPoczatek;
	else
		nCzas = 0xFFFFFFFF - nPoczatek + nCzasAkt;
	return nCzas;
}


