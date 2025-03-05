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
#include "modul_IiP.h"
#include "MS4525.h"
#include "ND130.h"
#include "konfig_fram.h"
#include "MS5611.h"	//testowo

extern TIM_HandleTypeDef htim7;
extern volatile unia_wymianyCM4_t uDaneCM4;
extern volatile unia_wymianyCM7_t uDaneCM7;
uint16_t sGenerator;
uint32_t nCzasOstatniegoOdcinka;	//przechowuje czas uruchomienia ostatniego odcinka
uint32_t nCzasPoprzedniegoObiegu[4];	//czas poprzedniego obiegu pętli głównej dla 4 modułów wewnetrznych
uint32_t ndT[4];						//czas jaki upłynął od poprzeniego obiegu pętli dla 4 modułów wewnetrznych
uint32_t nCzasBiezacy;
uint8_t chNrOdcinkaCzasu;
uint32_t nCzasOdcinka[LICZBA_ODCINKOW_CZASU];		//zmierzony czas obsługo odcinka
uint32_t nMaxCzasOdcinka[LICZBA_ODCINKOW_CZASU];	//maksymalna wartość czasu odcinka
uint32_t nCzasJalowy;

uint8_t chErrPG = ERR_OK;		//błąd petli głównej
uint8_t chStanIOwy, chStanIOwe;	//stan wejść IO modułów wewnetrznych
extern uint8_t chBuforAnalizyGNSS[ROZMIAR_BUF_ANA_GNSS];
extern volatile uint8_t chWskNapBaGNSS, chWskOprBaGNSS;
uint16_t chTimeoutGNSS;		//licznik timeoutu odbierania danych z modułu GNSS. Po timeoucie inicjalizuje moduł.
static uint8_t chEtapOperacjiI2C;
uint8_t chGeneratorNapisow, chLicznikKomunikatow;
extern I2C_HandleTypeDef hi2c3;
uint8_t chCzujnikOdczytywanyNaI2CExt;	//identyfikator czujnika odczytywanego na zewntrznym I2C. Potrzebny do tego aby powiązać odczytane dane z rodzajem obróbki

float fTempTest;
uint32_t TestWej;
uint16_t sLicznik;
////////////////////////////////////////////////////////////////////////////////
// Pętla główna programu autopilota
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PetlaGlowna(void)
{
	uint8_t chErr;

	//licz czas obiegu petli dla modułów wewnętrznych które mogą wymagać czasu do całkowania swoich wartosci
	if (chNrOdcinkaCzasu < 4)
	{
		nCzasBiezacy = PobierzCzas();
		ndT[chNrOdcinkaCzasu] = MinalCzas2(nCzasPoprzedniegoObiegu[chNrOdcinkaCzasu], nCzasBiezacy);	//licz czas od ostatniego obiegu pętli
		nCzasPoprzedniegoObiegu[chNrOdcinkaCzasu] = nCzasBiezacy;
	}

	switch (chNrOdcinkaCzasu)
	{
	case 0:		//obsługa modułu w gnieździe 1
		chErrPG |= UstawDekoderModulow(ADR_MOD1);

		break;

	case 1:		//obsługa modułu w gnieździe 2
		chErr = ObslugaModuluI2P(ADR_MOD2);
		if (chErr)
			chStanIOwy &= ~MIO41;	//zaświeć czerwoną LED
		else
			chStanIOwy |= MIO41;	//zgaś czerwoną LED
		chErrPG |= PobierzDaneExpandera(&chStanIOwe);
		break;

	case 2:		//obsługa modułu w gnieździe 3
		//chErrPG |= ObslugaModuluIiP(ADR_MOD3);
		//chErrPG |= UstawDekoderModulow(ADR_MOD3);
		int32_t ndT;
		sLicznik++;
		TestWej = sLicznik * 0x100;
		fTempTest = MS5611_LiczTemperature(TestWej, &ndT);
		break;

	case 3:		//obsługa modułu w gnieździe 4
		for (uint16_t n=0; n<16; n++)
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
			chErrPG |= DekodujNMEA(chBuforAnalizyGNSS[chWskOprBaGNSS]);	//analizuj dane z GNSS
			chWskOprBaGNSS++;
			chWskOprBaGNSS &= MASKA_ROZM_BUF_ANA_GNSS;
			chStanIOwy ^= MIO42;		//Zielona LED
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
			uDaneCM4.dane.nZainicjowano &= ~MASKA_INIT_GNSS;	//wyczyść wszystkie bity używane przez GNSS
			uDaneCM4.dane.stGnss1.chFix = 0;
		}
		break;

	case 5: chErrPG |= RozdzielniaOperacjiI2C();			break;
	case 6:	ObliczeniaJednostkiInercujnej(2);	break;
	case 8: chErrPG |= WyslijDaneExpandera(chStanIOwy); 	break;

	case 9:
		/*FramDataWriteFloat(FAH_ZYRO1_TEMP_ZIM, 10.11);
		FramDataWriteFloat(FAH_ZYRO2_TEMP_ZIM, 10.12);
		FramDataWriteFloat(FAH_ZYRO1_TEMP_POK, 25.13);
		FramDataWriteFloat(FAH_ZYRO2_TEMP_POK, 25.14);
		FramDataWriteFloat(FAH_ZYRO1_TEMP_GOR, 40.15);
		FramDataWriteFloat(FAH_ZYRO2_TEMP_GOR, 40.16);*/
		break;

	case 10:
		//InicjujModulI2P();
		//static uint8_t chFramBuf1[24];
		//CzytajBuforFRAM(FAH_ZYRO_TEMP, chFramBuf1, 24);
		/*static uint8_t chFramBuf2[36];
		CzytajBuforFRAM(FAH_MOD_ZYRO1, chFramBuf2, 36);
		static uint8_t chFramBuf3[36];
		CzytajBuforFRAM(FAH_MOD_ZYRO2, chFramBuf3, 36);*/
		break;

	case 11:
		/*static float fTemp1[3], fTemp2[3];
		for (uint16_t n=0; n<3; n++)
		{
			fTemp1[n] = FramDataReadFloat(FAH_ZYRO1_TEMP_ZIM+(4*n));	//temepratury żyroskopu 1
			fTemp2[n] = FramDataReadFloat(FAH_ZYRO2_TEMP_ZIM+(4*n));	//temepratury żyroskopu 2
		}*/
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
		chErrPG |= UstawDaneWymiany_CM4();
		//

		//wykonaj polecenie przekazane z CM7
		switch(uDaneCM7.dane.chWykonajPolecenie)
		{
		case POL_NIC:	break;		//polecenie neutralne
		case POL_KALIBRUJ_ZYRO_ZIM:	RozpocznijKalibracje(POL_KALIBRUJ_ZYRO_ZIM);	break;	//uruchom kalibrację żyroskopów na zimno 10°C
		case POL_KALIBRUJ_ZYRO_POK:	RozpocznijKalibracje(POL_KALIBRUJ_ZYRO_POK);	break;	//uruchom kalibrację żyroskopów w temperaturze pokojowej 25°C
		case POL_KALIBRUJ_ZYRO_GOR:	RozpocznijKalibracje(POL_KALIBRUJ_ZYRO_GOR);	break;	//uruchom kalibrację żyroskopów na gorąco 40°C

		case POL_KALIBRUJ_MAGN1:	//uruchom kalibrację magnetometru 1
		case POL_KALIBRUJ_MAGN2:	//uruchom kalibrację magnetometru 2
		case POL_KALIBRUJ_MAGN3:	//uruchom kalibrację magnetometru 3
		case POL_CZYSC_BLEDY:		uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_OK;	break;	//nadpisz poprzednio zwrócony błąd
    	}
		uDaneCM7.dane.chWykonajPolecenie = POL_NIC;
		break;

	case 16:

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
		__WFI();	//uśpij kontroler w oczekwianiu na przerwanie od czegokolwiek np timera 7 mierzącego czas
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
// Wersja z wewnetrznym czasem bieżącym
// Parametry: sPoczatek - licznik czasu na na początku pomiaru
// Zwraca: ilość czasu w mikrosekundach jaki upłynął do podanego czasu początkowego
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas(uint32_t nPoczatek)
{
	uint32_t nCzasAkt;

	nCzasAkt = PobierzCzas();
	return MinalCzas2(nPoczatek, nCzasAkt);
}

////////////////////////////////////////////////////////////////////////////////
// Liczy upływ czasu mierzony timerem T7 z rozdzieczością 1us
// Wersja z zewnetrznym czasem bieżącym
// Parametry: sPoczatek - licznik czasu na na początku pomiaru
// Zwraca: ilość czasu w mikrosekundach jaki upłynął do podanego czasu początkowego
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas2(uint32_t nPoczatek, uint32_t nCzasAkt)
{
	uint32_t nCzas;

	if (nCzasAkt >= nPoczatek)
		nCzas = nCzasAkt - nPoczatek;
	else
		nCzas = 0xFFFFFFFF - nPoczatek + nCzasAkt;
	return nCzas;
}


////////////////////////////////////////////////////////////////////////////////
// Funkcja dzieli złożone operacje I2C na sekwencje dające sie realizować w tle, dzięki czemu nie blokuje czasu procesora oczekując na zakończenie
// Parametry: brak
// Zwraca: kod błędu operacji I2C
////////////////////////////////////////////////////////////////////////////////
uint8_t RozdzielniaOperacjiI2C(void)
{
	uint8_t chErr = ERR_OK;
	printf("I2C");

	//operacje na zewnętrznej magistrali I2C3
	switch(chEtapOperacjiI2C)
	{
	case 0:	chErr = StartujPomiarMagHMC();		break;
	case 1: chErr = ObslugaMS4525();			break;
	case 2:	chErr = StartujOdczytMagHMC();		break;
	case 3:	chErr = CzytajMagnetometrHMC();		break;
	default: break;
	}

	//operacje na wewnętrznej magistrali I2C4
	switch(chEtapOperacjiI2C)
	{
	case 0:	chErr = ObslugaIIS2MDC();			break;
	case 1:	chErr = ObslugaMMC3416x();			break;
	case 2: chErr = ObslugaIIS2MDC();			break;
	case 3:	chErr = ObslugaMMC3416x();			break;
	default: break;
	}

	/*switch(chEtapOperacjiI2C)
	{
	case 0:	chErr = StartujPomiarMMC3416x();	break;
	case 1:	chErr = StartujOdczytIIS2MDC();		break;
	case 2: chErr = CzytajIIS2MDC();			break;
	case 3:	chErr = StartujOdczytMMC3416x();	break;

	case 4:	chErr = CzytajMMC3416x();			break;
	case 5:	chErr = StartujOdczytIIS2MDC();		break;
	case 6: chErr = CzytajIIS2MDC();			break;
	case 7:
	default: break;
	}*/

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
	extern uint8_t chDaneMagMMC[6];
	extern uint8_t chDaneMagIIS[8];
	extern uint8_t chDaneMS4525[5];
	extern uint8_t chOdczytywanyMagnetometr;	//zmienna wskazuje który magnetometr jest odczytywany: MAG_MMC lub MAG_IIS
	extern int16_t sPomiarMMCH[3], sPomiarMMCL[3];	//wyniki pomiarów dla dodatniego i ujemnego namagnesowania czujnika
	extern uint8_t chSekwencjaPomiaruMMC;
	if (hi2c->Instance == I2C3)	//magistrala I2C modułów zewnętrznych
	{
		if (chCzujnikOdczytywanyNaI2CExt == MAG_HMC)	//magnetometr HMC5883
		//if ((chDaneMagHMC[0] || chDaneMagHMC[1]) && (chDaneMagHMC[2] || chDaneMagHMC[3]) && (chDaneMagHMC[4] || chDaneMagHMC[5]))
		{
			uDaneCM4.dane.sMagne3[0] = (int16_t)(chDaneMagHMC[0] * 0x100 + chDaneMagHMC[1]);
			uDaneCM4.dane.sMagne3[1] = (int16_t)(chDaneMagHMC[2] * 0x100 + chDaneMagHMC[3]);
			uDaneCM4.dane.sMagne3[2] = (int16_t)(chDaneMagHMC[4] * 0x100 + chDaneMagHMC[5]);
		}
		else
		if (chCzujnikOdczytywanyNaI2CExt == CISN_ROZN_MS2545)	//ciśnienie różnicowe czujnika MS2545DO
		{
			uDaneCM4.dane.fCisnRozn[1] = CisnienieMS2545(chDaneMS4525);
			uDaneCM4.dane.fPredkosc[1] = PredkoscRurkiPrantla(uDaneCM4.dane.fCisnRozn[1], 101315.f);	//dla ciśnienia standardowego. Docelowo zamienić na cisnienie zmierzone
		}
		else
		if (chCzujnikOdczytywanyNaI2CExt == CISN_TEMP_MS2545)	//ciśnienie różnicowe i temperatura czujnika MS2545DO
		{
			uDaneCM4.dane.fTemper[6] = TemperaturaMS2545(chDaneMS4525);
			uDaneCM4.dane.fCisnRozn[1] = (15 * uDaneCM4.dane.fCisnRozn[1] + CisnienieMS2545(chDaneMS4525)) / 16;
			uDaneCM4.dane.fPredkosc[1] = PredkoscRurkiPrantla(uDaneCM4.dane.fCisnRozn[1], 101315.f);	//dla ciśnienia standardowego. Docelowo zamienić na cisnienie zmierzone
		}
	}

	if (hi2c->Instance == I2C4)		//magistrala I2C modułów wewnętrznych
	{
		if (chOdczytywanyMagnetometr == MAG_IIS)
		{
			if ((chDaneMagIIS[0] || chDaneMagIIS[1]) && (chDaneMagIIS[2] || chDaneMagIIS[3]) && (chDaneMagIIS[4] || chDaneMagIIS[5]))
			{
				//	sCisnienie = (int16_t)chBufND130[0] * 0x100 + chBufND130[1];

				uDaneCM4.dane.sMagne1[0] = (int16_t)chDaneMagIIS[1] * 0x100 + chDaneMagIIS[0];
				uDaneCM4.dane.sMagne1[1] = (int16_t)chDaneMagIIS[3] * 0x100 + chDaneMagIIS[2];
				uDaneCM4.dane.sMagne1[2] = (int16_t)chDaneMagIIS[5] * 0x100 + chDaneMagIIS[4];
				uDaneCM4.dane.fTemper[4] = ((int16_t)chDaneMagIIS[7] * 0x100 + chDaneMagIIS[6]) / 8;	//The nominal sensitivity is 8 LSB/°C.
			}
		}
		else
		if (chOdczytywanyMagnetometr == MAG_MMC)
		{
			if ((chDaneMagMMC[0] || chDaneMagMMC[1]) && (chDaneMagMMC[2] || chDaneMagMMC[3]) && (chDaneMagMMC[4] || chDaneMagMMC[5]))
			{
				if (chSekwencjaPomiaruMMC < SPMMC3416_REFIL_RESET)	//poniżej SPMMC3416_REFIL_RESET będzie obsługa dla SET, powyżej dla RESET
				{
					sPomiarMMCH[0] = (chDaneMagMMC[1] * 0x100 + chDaneMagMMC[0]) - 32768;	//czujnik zwraca liczby unsigned z 32768 dla zera
					sPomiarMMCH[1] = (chDaneMagMMC[3] * 0x100 + chDaneMagMMC[2]) - 32768;
					sPomiarMMCH[2] = (chDaneMagMMC[5] * 0x100 + chDaneMagMMC[4]) - 32768;
				}
				else
				{
					sPomiarMMCL[0] = (chDaneMagMMC[1] * 0x100 + chDaneMagMMC[0]) - 32768;
					sPomiarMMCL[1] = (chDaneMagMMC[3] * 0x100 + chDaneMagMMC[2]) - 32768;
					sPomiarMMCL[2] = (chDaneMagMMC[5] * 0x100 + chDaneMagMMC[4]) - 32768;
				}
				for (uint8_t n=0; n<3; n++)
					uDaneCM4.dane.sMagne2[n] = (sPomiarMMCH[n] - sPomiarMMCL[n]) / 2;
			}
		}
	}
}
