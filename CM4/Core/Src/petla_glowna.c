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
#include "HMC5883.h"
#include "jedn_inercyjna.h"
#include <stdio.h>
#include "MS5611.h"

extern TIM_HandleTypeDef htim7;
extern volatile unia_wymianyCM4_t uDaneCM4;
extern volatile unia_wymianyCM7_t uDaneCM7;
uint16_t sGenerator;
uint32_t nCzasOstatniegoOdcinka;	//przechowuje czas uruchomienia ostatniego odcinka
uint8_t chNrOdcinkaCzasu;
uint32_t nCzasOdcinka[LICZBA_ODCINKOW_CZASU];		//zmierzony czas obsługo odcinka
uint32_t nMaxCzasOdcinka[LICZBA_ODCINKOW_CZASU];	//maksymalna wartość czasu odcinka
uint32_t nCzasJalowy;
uint8_t chErrPG = ERR_OK;		//błąd petli głównej
uint8_t chStanIOwy, chStanIOwe;	//stan wejść IO modułów wewnetrznych
extern uint8_t chBuforAnalizyGNSS[ROZMIAR_BUF_ANA_GNSS];
extern volatile uint8_t chWskNapBaGNSS, chWskOprBaGNSS;
uint16_t chTimeoutGNSS;		//licznik timeoutu odbierania danych z modułu GNSS. Po timeoucie inicjalizuje moduł.
uint8_t chEtapOperacjiI2C;
uint8_t chGeneratorNapisow, chLicznikKomunikatow;
extern I2C_HandleTypeDef hi2c3;

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
		chErrPG |= UstawDekoderModulow(ADR_MOD1);
		break;

	case 1:		//obsługa modułu w gnieździe 2
		chErrPG |= UstawDekoderModulow(ADR_MOD2);

		chErrPG |= WyslijDaneExpandera(chStanIOwy & ~MIO22);	//ustaw adres A2 = 0
		ObslugaMS5611();

		chErrPG |= WyslijDaneExpandera(chStanIOwy | MIO22);		//ustaw adres A2 = 1
		break;

	case 2:		//obsługa modułu w gnieździe 3
		fTemp = (float)sGenerator/100;
		uDaneCM4.dane.fWysokosc[0] = 20 + fTemp;
		uDaneCM4.dane.fWysokosc[1] = 10 + fTemp;
		chErrPG |= UstawDekoderModulow(ADR_MOD3);
		break;

	case 3:		//obsługa modułu w gnieździe 4
		for (n=0; n<16; n++)
		{
			uDaneCM4.dane.sSerwa[n] += n;
			if (uDaneCM4.dane.sSerwa[n] > 2000)
				uDaneCM4.dane.sSerwa[n] = 0;
		}
		chErrPG |= UstawDekoderModulow(ADR_MOD4);
		break;

	case 4:		//obsługa GNSS na UART8
		while (chWskNapBaGNSS != chWskOprBaGNSS)
		{
			chErr = DekodujNMEA(chBuforAnalizyGNSS[chWskOprBaGNSS]);	//analizuj dane z GNSS
			chWskOprBaGNSS++;
			chWskOprBaGNSS &= MASKA_ROZM_BUF_ANA_GNSS;
			chStanIOwy ^= 0x80;		//Zielona LED
			chTimeoutGNSS = TIMEOUT_GNSS;
		}
		if ((uDaneCM4.dane.nZainicjowano & INIT_GNSS_GOTOWY) == 0)
		{
			InicjujGNSS();		//gdy nie jest zainicjowany to przeprowadź odbiornik przez kolejne etapy inicjalizacji
		}
		if (chTimeoutGNSS)
			chTimeoutGNSS--;
		else
		{
			chTimeoutGNSS = TIMEOUT_GNSS;
			uDaneCM4.dane.nZainicjowano &= ~MASKA_INIT_GNSS;	//wyczyść wszystkie bity użuwane przez GNSS
			uDaneCM4.dane.stGnss1.chFix = 0;
		}
		break;

	case 5: chErrPG |= RozdzielniaOperacjiI2C();	break;

	case 8:

		chErrPG |= WyslijDaneExpandera(chStanIOwy);
		break;

	case 9:	chErrPG |= ObliczeniaJednostkiInercujnej();	break;

	/*case 10:
		CzytajIdFRAM(chDane);
		ZapiszFRAM(0x2000, 0x55);
		chDane[0] = CzytajFRAM(0x2000);

		nCzas = PobierzCzas();
		CzytajBuforFRAM(0x1000, chDane, 16);
		nCzas = MinalCzas(nCzas);

		for (n=0; n<16; n++)
			chDane[n] = (uint16_t)(n & 0xFF);

		nCzas = PobierzCzas();
		ZapiszBuforFRAM(0x1000, chDane, 16);
		nCzas = MinalCzas(nCzas);
		break;*/

	case 11:
		chErrPG |= UstawDekoderModulow(ADR_NIC);
		break;

	case 12:	//test przekazywania napisów
		chGeneratorNapisow++;
		if (!chGeneratorNapisow)
		{
			char chNapis[20];
			sprintf(chNapis, "CM4 pracuje %d\n\r", chLicznikKomunikatow);
			for (uint8_t n=0; n<sizeof(chNapis); n++)
			  uDaneCM4.dane.chNapis[n] = chNapis[n];
			chLicznikKomunikatow++;
		}
		break;

	case 15:	//wymień dane między rdzeniami
		uDaneCM4.dane.chErrPetliGlownej = chErrPG;
		chErrPG  = PobierzDaneWymiany_CM7();
		chErr = UstawDaneWymiany_CM4();
		if (chErr == ERR_SEMAFOR_ZAJETY)
			chStanIOwy &= ~0x40;	//zaświeć czerwoną LED
		else
			chStanIOwy |= 0x40;		//zgaś czerwoną LED
		chErrPG |= chErr;
		//chStanIOwy ^= 0x80;		//Zielona LED
		break;

	case 16:
		uint8_t chVTG1[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x79 }; // NMEA GxVTG /10
		uint8_t chVTG2[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47 }; // NMEA GxVTG
		uint8_t chGSA[16]  = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 }; // NMEA GxGSA /0
		uint8_t chGST[16]  = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x54 }; // NMEA GxGST /0
		uint8_t chRATE[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x00, 0x00, 0x79, 0x10};	//pomiar 10 Hz wzgl. UTC
		uint8_t chDAT[10] =  {0xB5, 0x62, 0x06, 0x06, 0x02, 0x00, 0x00, 0x00, 0x0E, 0x4A};
		SumaKontrolnaUBX(chVTG1);
		SumaKontrolnaUBX(chVTG2);
		SumaKontrolnaUBX(chGSA);
		SumaKontrolnaUBX(chGST);
		SumaKontrolnaUBX(chRATE);
		SumaKontrolnaUBX(chDAT);
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
		nCzasJalowy = PobierzCzas() - nCzasOstatniegoOdcinka;
	}
	while (nCzasJalowy < CZAS_ODCINKA);

	nCzasOstatniegoOdcinka = PobierzCzas();
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera stan licznika pracującego na 200MHz/200
// Parametry: brak
// Zwraca: stan licznika w mikrosekundach
////////////////////////////////////////////////////////////////////////////////
uint32_t PobierzCzas(void)
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

	nCzasAkt = PobierzCzas();
	if (nCzasAkt >= nPoczatek)
		nCzas = nCzasAkt - nPoczatek;
	else
		nCzas = 0xFFFFFFFF - nPoczatek + nCzasAkt;
	return nCzas;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja dzieli złożone operacje I2C na sekwencje dające sie realizować w tle, dzięki czemu nie blokuje czasu procesora oczekując na zakończenie
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RozdzielniaOperacjiI2C(void)
{
	uint8_t chErr = ERR_OK;

	switch(chEtapOperacjiI2C)
	{
	case 0:	chErr = StartujPomiarMagHMC();	break;
	case 1:	break;
	case 2:	chErr = StartujOdczytMagHMC();		break;
	case 3:	chErr = CzytajMagnetometrHMC();		break;
	}
	chEtapOperacjiI2C++;
	chEtapOperacjiI2C &= 0x03;
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Callback nadawania I2C
// Parametry: hi2c - wskaźnik na interfejs I2C
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}



////////////////////////////////////////////////////////////////////////////////
// Callback odbioru I2C
// Parametry: hi2c - wskaźnik na interfejs I2C
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	extern uint8_t chDaneMagHMC[6];
	if (hi2c->Instance == I2C3)
	{
		if ((chDaneMagHMC[0] || chDaneMagHMC[1]) && (chDaneMagHMC[2] || chDaneMagHMC[3]) && (chDaneMagHMC[4] || chDaneMagHMC[5]))
		{
			uDaneCM4.dane.fMagn3[0] = (int16_t)(0x100 * chDaneMagHMC[0] + chDaneMagHMC[1]);
			uDaneCM4.dane.fMagn3[1] = (int16_t)(0x100 * (int16_t)chDaneMagHMC[2] + chDaneMagHMC[3]);
			uDaneCM4.dane.fMagn3[2] = (int16_t)(0x100 * (int16_t)chDaneMagHMC[4] + chDaneMagHMC[5]);
		}
	}
}
