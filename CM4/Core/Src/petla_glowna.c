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
uint32_t nCzasPoprzedniegoObiegu[4];	//czas [us] poprzedniego obiegu pętli głównej dla 4 modułów wewnetrznych
uint32_t ndT[4];						//czas [us] jaki upłynął od poprzeniego obiegu pętli dla 4 modułów wewnetrznych
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
volatile uint8_t chCzujnikOdczytywanyNaI2CExt;	//identyfikator czujnika obsługiwanego na zewntrznej magistrali I2C. Potrzebny do tego aby powiązać odczytane dane z rodzajem obróbki
volatile uint8_t chCzujnikOdczytywanyNaI2CInt;	//identyfikator czujnika obsługiwanego na wewnętrznej magistrali I2C: MAG_MMC lub MAG_IIS
uint8_t chNoweDaneI2C;	//zestaw flag informujący o pojawieniu sie nowych danych odebranych na magistrali I2C
extern uint16_t sLicznikCzasuKalibracji;

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
		ObrotWektora(ADR_MOD3);
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

	case 5:
		uDaneCM4.dane.chNowyPomiar = 0;	//unieważnij wszystkie poprzednie pomiary. Flagi nowych pomiarów zostaną ustawnine w funkcji ObslugaCzujnikowI2C()
		if (chNoweDaneI2C)
			ObslugaCzujnikowI2C(&chNoweDaneI2C);	//jeżeli odebrano nowe dane z czujników na obu magistralach I2C to je obrób
		chErrPG |= RozdzielniaOperacjiI2C();
		break;

	case 6:	JednostkaInercyjna1Trygonometria(ADR_MOD2);	break;	//dane do IMU1
	case 7:	JednostkaInercyjna4Kwaterniony(ADR_MOD2);	break;	//dane do IMU2
	case 8:		break;
	case 9:		break;
	case 10:	break;
	case 11: chErrPG |= WyslijDaneExpandera(chStanIOwy); 	break;


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
		uDaneCM4.dane.chOdpowiedzNaPolecenie = uDaneCM7.dane.chWykonajPolecenie;	//domyślnie odeślij to co przyszło aby potwierdzić otrzymanie
		switch(uDaneCM7.dane.chWykonajPolecenie)
		{
		case POL_NIC:	break;		//polecenie neutralne
		case POL_KALIBRUJ_ZYRO_ZIM:	RozpocznijKalibracjeZeraZyroskopu(POL_KALIBRUJ_ZYRO_ZIM);	break;		//uruchom kalibrację żyroskopów na zimno 10°C
		case POL_KALIBRUJ_ZYRO_POK:	RozpocznijKalibracjeZeraZyroskopu(POL_KALIBRUJ_ZYRO_POK);	break;		//uruchom kalibrację żyroskopów w temperaturze pokojowej 25°C
		case POL_KALIBRUJ_ZYRO_GOR:	RozpocznijKalibracjeZeraZyroskopu(POL_KALIBRUJ_ZYRO_GOR);	break;		//uruchom kalibrację żyroskopów na gorąco 40°C

		case POL_KALIBRUJ_ZYRO_WZMP:		//uruchom kalibrację wzmocnienia żyroskopów P
		case POL_KALIBRUJ_ZYRO_WZMQ:		//uruchom kalibrację wzmocnienia żyroskopów Q
		case POL_KALIBRUJ_ZYRO_WZMR:		//uruchom kalibrację wzmocnienia żyroskopów R
		case POL_ZERUJ_CALKE_ZYRO:	KalibracjaWzmocnieniaZyro(uDaneCM7.dane.chWykonajPolecenie);	break;	//zeruje całkę prędkosci katowej żyroskopów przed kalibracją wzmocnienia

		case POL_CZYTAJ_WZM_ZYROP:	//odczytaj wzmocnienia żyroskopów P
			uDaneCM4.dane.fRozne[2] = FramDataReadFloat(FAH_ZYRO1P_WZMOC);
			uDaneCM4.dane.fRozne[3] = FramDataReadFloat(FAH_ZYRO2P_WZMOC);
			break;

		case POL_CZYTAJ_WZM_ZYROQ:	//odczytaj wzmocnienia żyroskopów Q
			uDaneCM4.dane.fRozne[2] = FramDataReadFloat(FAH_ZYRO1Q_WZMOC);
			uDaneCM4.dane.fRozne[3] = FramDataReadFloat(FAH_ZYRO2Q_WZMOC);
			break;

		case POL_CZYTAJ_WZM_ZYROR:	//odczytaj wzmocnienia żyroskopów R
			uDaneCM4.dane.fRozne[2] = FramDataReadFloat(FAH_ZYRO1R_WZMOC);
			uDaneCM4.dane.fRozne[3] = FramDataReadFloat(FAH_ZYRO2R_WZMOC);
			break;

		case POL_SPRAWDZ_MAGN1:
		case POL_KAL_ZERO_MAGN1:	ZnajdzEkstremaMagnetometru((float*)uDaneCM4.dane.fMagne1);	break;	//uruchom kalibrację zera magnetometru 1
		case POL_SPRAWDZ_MAGN2:
		case POL_KAL_ZERO_MAGN2:	ZnajdzEkstremaMagnetometru((float*)uDaneCM4.dane.fMagne2);	break;	//uruchom kalibrację zera magnetometru 2
		case POL_SPRAWDZ_MAGN3:
		case POL_KAL_ZERO_MAGN3:	ZnajdzEkstremaMagnetometru((float*)uDaneCM4.dane.fMagne3);	break;	//uruchom kalibrację zera magnetometru 3

		case POL_ZAPISZ_KONF_MAGN1:	ZapiszKonfiguracjeMagnetometru(MAG1);	break;
		case POL_ZAPISZ_KONF_MAGN2:	ZapiszKonfiguracjeMagnetometru(MAG2);	break;
		case POL_ZAPISZ_KONF_MAGN3:	ZapiszKonfiguracjeMagnetometru(MAG3);	break;
		case POL_ZERUJ_EKSTREMA:	ZerujEkstremaMagnetometru();	break;

		case POL_INICJUJ_USREDN:	KalibrujCisnienie(0, 0, 0, CZAS_KALIBRACJI, 0xFF);	break;	//inicjalizacja
		case POL_ZERUJ_LICZNIK:
			sLicznikCzasuKalibracji = 0;
			break;
		case POL_USREDNIJ_CISN1:
			uDaneCM4.dane.chOdpowiedzNaPolecenie = KalibrujCisnienie(uDaneCM4.dane.fCisnie[0], uDaneCM4.dane.fCisnie[1], uDaneCM4.dane.fTemper[TEMP_BARO1], sLicznikCzasuKalibracji, 0);
			if (sLicznikCzasuKalibracji <= CZAS_KALIBRACJI)
				uDaneCM4.dane.sPostepProcesu = sLicznikCzasuKalibracji++;
			break;
		case POL_USREDNIJ_CISN2:
			uDaneCM4.dane.chOdpowiedzNaPolecenie = KalibrujCisnienie(uDaneCM4.dane.fCisnie[0], uDaneCM4.dane.fCisnie[1], uDaneCM4.dane.fTemper[TEMP_BARO1], sLicznikCzasuKalibracji, 1);
			if (sLicznikCzasuKalibracji <= CZAS_KALIBRACJI)
				uDaneCM4.dane.sPostepProcesu = sLicznikCzasuKalibracji++;
			break;
		case POL_CZYSC_BLEDY:		uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_OK;	break;	//nadpisz poprzednio zwrócony błąd
    	}
		break;

	case 16:	break;
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
		__WFI();	//uśpij kontroler w oczekwianiu na przerwanie od czegokolwiek np. timera 7 mierzącego czas
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
// nCzasAkt - czas aktualny jako zmienna
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
	if (hi2c->Instance == I2C3)	//magistrala I2C modułów zewnętrznych
	{
		if (chCzujnikOdczytywanyNaI2CExt == MAG_HMC)	//magnetometr HMC5883
			chNoweDaneI2C |= MAG_HMC;
		else
		if (chCzujnikOdczytywanyNaI2CExt == CISN_ROZN_MS2545)	//ciśnienie różnicowe czujnika MS2545DO
			chNoweDaneI2C |= CISN_ROZN_MS2545;
		else
		if (chCzujnikOdczytywanyNaI2CExt == CISN_TEMP_MS2545)	//ciśnienie różnicowe i temperatura czujnika MS2545DO
			chNoweDaneI2C |= CISN_TEMP_MS2545;
	}

	if (hi2c->Instance == I2C4)		//magistrala I2C modułów wewnętrznych
	{
		if (chCzujnikOdczytywanyNaI2CInt == MAG_IIS)
			chNoweDaneI2C |= MAG_IIS;
		else
		if (chCzujnikOdczytywanyNaI2CInt == MAG_MMC)
			chNoweDaneI2C |= MAG_MMC;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Obsługuje obróbkę danych odczytanych z czujników na I2C
// Parametry: chCzujniki - wskaźnik na zmienną z polami bitowymi inforującymi o danych z czujników
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaCzujnikowI2C(uint8_t *chCzujniki)
{
	extern uint8_t chDaneMagHMC[6];
	extern uint8_t chDaneMagMMC[6];
	extern uint8_t chDaneMagIIS[8];
	extern uint8_t chDaneMS4525[5];
	extern int16_t sPomiarMMCH[3], sPomiarMMCL[3];	//wyniki pomiarów dla dodatniego i ujemnego namagnesowania czujnika
	extern uint8_t chSekwencjaPomiaruMMC;
	extern float fPrzesMagn1[3], fSkaloMagn1[3];
	extern float fPrzesMagn2[3], fSkaloMagn2[3];
	extern float fPrzesMagn3[3], fSkaloMagn3[3];
	uint8_t chErr = ERR_OK;
	int16_t sZeZnakiem;	//zmiena robocza do konwersji dnych 8-bitowych bez znaku na liczbę 16-bitową ze znakiem

	if (*chCzujniki & MAG_HMC)
	{
		//Uwaga! Czujnik HMS5883L ma osie w nietypowej kolejności XZY, więc konwersję trzeba zrobić ręcznie poza pętlą
		sZeZnakiem = (int16_t)chDaneMagHMC[0] * 0x100 + chDaneMagHMC[1];	//oś X
		if ((uDaneCM7.dane.chWykonajPolecenie == POL_KAL_ZERO_MAGN3) ||  (uDaneCM7.dane.chWykonajPolecenie == POL_ZERUJ_EKSTREMA))
			uDaneCM4.dane.fMagne3[0] = (float)sZeZnakiem * CZULOSC_HMC5883;			//dane surowe podczas kalibracji magnetometru wyrażone w Teslach
		else
			uDaneCM4.dane.fMagne3[0] = ((float)sZeZnakiem * CZULOSC_HMC5883 - fPrzesMagn3[0]) * fSkaloMagn3[0];	//dane skalibrowane

		sZeZnakiem = (int16_t)chDaneMagHMC[4] * 0x100 + chDaneMagHMC[5];	//oś Y
		if ((uDaneCM7.dane.chWykonajPolecenie == POL_KAL_ZERO_MAGN3) ||  (uDaneCM7.dane.chWykonajPolecenie == POL_ZERUJ_EKSTREMA))
			uDaneCM4.dane.fMagne3[1] = (float)sZeZnakiem * CZULOSC_HMC5883;			//dane surowe podczas kalibracji magnetometru
		else
			uDaneCM4.dane.fMagne3[1] = ((float)sZeZnakiem * CZULOSC_HMC5883 - fPrzesMagn3[1]) * fSkaloMagn3[1];	//dane skalibrowane

		sZeZnakiem = (int16_t)chDaneMagHMC[2] * 0x100 + chDaneMagHMC[3];	//oś Z
		if ((uDaneCM7.dane.chWykonajPolecenie == POL_KAL_ZERO_MAGN3) ||  (uDaneCM7.dane.chWykonajPolecenie == POL_ZERUJ_EKSTREMA))
			uDaneCM4.dane.fMagne3[2] = (float)sZeZnakiem * CZULOSC_HMC5883;			//dane surowe podczas kalibracji magnetometru
		else
			uDaneCM4.dane.fMagne3[2] = ((float)sZeZnakiem * CZULOSC_HMC5883 - fPrzesMagn3[2]) * fSkaloMagn3[2];	//dane skalibrowane
		*chCzujniki &= ~MAG_HMC;	//dane obsłużone
		uDaneCM4.dane.chNowyPomiar |= NP_MAG3;	//jest nowy pomiar
	}

	if (*chCzujniki & CISN_ROZN_MS2545)
	{
		uDaneCM4.dane.fCisnRozn[1] = CisnienieMS2545(chDaneMS4525);
		uDaneCM4.dane.fPredkosc[1] = PredkoscRurkiPrantla(uDaneCM4.dane.fCisnRozn[1], 101315.f);	//dla ciśnienia standardowego. Docelowo zamienić na cisnienie zmierzone
		*chCzujniki &= ~CISN_ROZN_MS2545;	//dane obsłużone
		uDaneCM4.dane.chNowyPomiar |= NP_EXT_IAS;	//jest nowy pomiar
	}

	if (*chCzujniki & CISN_TEMP_MS2545)
	{
		uDaneCM4.dane.fTemper[6] = TemperaturaMS2545(chDaneMS4525);
		uDaneCM4.dane.fCisnRozn[1] = (15 * uDaneCM4.dane.fCisnRozn[1] + CisnienieMS2545(chDaneMS4525)) / 16;
		uDaneCM4.dane.fPredkosc[1] = PredkoscRurkiPrantla(uDaneCM4.dane.fCisnRozn[1], 101315.f);	//dla ciśnienia standardowego. Docelowo zamienić na cisnienie zmierzone
		*chCzujniki &= ~CISN_TEMP_MS2545;	//dane obsłużone
	}

	if (*chCzujniki & MAG_IIS)
	{
		for (uint8_t n=0; n<3; n++)
		{
			sZeZnakiem = (int16_t)chDaneMagIIS[2*n+1] * 0x100 + chDaneMagIIS[2*n];
			if ((uDaneCM7.dane.chWykonajPolecenie == POL_KAL_ZERO_MAGN1) || (uDaneCM7.dane.chWykonajPolecenie == POL_ZERUJ_EKSTREMA))
				uDaneCM4.dane.fMagne1[n] = (float)sZeZnakiem * CZULOSC_IIS2MDC;			//dane surowe podczas kalibracji magnetometru
			else
				uDaneCM4.dane.fMagne1[n] = ((float)sZeZnakiem * CZULOSC_IIS2MDC - fPrzesMagn1[n]) * fSkaloMagn1[n];	//dane skalibrowane
		}
		uDaneCM4.dane.fTemper[4] = ((int16_t)chDaneMagIIS[7] * 0x100 + chDaneMagIIS[6]) / 8;	//The nominal sensitivity is 8 LSB/°C.
		//if ((chDaneMagIIS[0] || chDaneMagIIS[1]) && (chDaneMagIIS[2] || chDaneMagIIS[3]) && (chDaneMagIIS[4] || chDaneMagIIS[5]))
		*chCzujniki &= ~MAG_IIS;	//dane obsłużone
		uDaneCM4.dane.chNowyPomiar |= NP_MAG1;	//jest nowy pomiar
	}

	if (*chCzujniki & MAG_MMC)
	{
		//if ((chDaneMagMMC[0] || chDaneMagMMC[1]) && (chDaneMagMMC[2] || chDaneMagMMC[3]) && (chDaneMagMMC[4] || chDaneMagMMC[5]))
		for (uint8_t n=0; n<3; n++)
		{
			if (chSekwencjaPomiaruMMC < SPMMC3416_REFIL_RESET)	//poniżej SPMMC3416_REFIL_RESET będzie obsługa dla SET, powyżej dla RESET
					sPomiarMMCH[n] = ((int16_t)chDaneMagMMC[2*n+1] * 0x100 + chDaneMagMMC[2*n]) - 32768;	//czujnik zwraca liczby unsigned z 32768 dla zera
			else
					sPomiarMMCL[n] = ((int16_t)chDaneMagMMC[2*n+1] * 0x100 + chDaneMagMMC[2*n]) - 32768;

			if ((uDaneCM7.dane.chWykonajPolecenie == POL_KAL_ZERO_MAGN2) || (uDaneCM7.dane.chWykonajPolecenie == POL_ZERUJ_EKSTREMA))
				uDaneCM4.dane.fMagne2[n] = (float)(sPomiarMMCH[n] - sPomiarMMCL[n]) * CZULOSC_MMC34160 / 2;	//dane surowe podczas kalibracji magnetometru
			else
				uDaneCM4.dane.fMagne2[n] = ((float)(sPomiarMMCH[n] - sPomiarMMCL[n]) * CZULOSC_MMC34160 / 2 - fPrzesMagn2[n]) * fSkaloMagn2[n];	//dane skalibrowane;
		}
		*chCzujniki &= ~MAG_MMC;	//dane obsłużone
		uDaneCM4.dane.chNowyPomiar |= NP_MAG2;	//jest nowy pomiar
	}
	return chErr;
}


