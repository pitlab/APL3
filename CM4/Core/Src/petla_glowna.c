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
#include "pid.h"
#include "WeWyRC.h"
#include "adc.h"
#include "dshot.h"
#include "mikser.h"

extern TIM_HandleTypeDef htim7;
extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
uint16_t sGenerator;
uint32_t nCzasOstatniegoOdcinka;	//przechowuje czas uruchomienia ostatniego odcinka
uint32_t nCzasPoprzedniegoObiegu;	//czas [us] poprzedniego obiegu pętli głównej
uint32_t ndT;						//czas [us] jaki upłynął od poprzeniego obiegu pętli
uint32_t nCzasBiezacy;
uint8_t chNrOdcinkaCzasu;
uint32_t nCzasOdcinka[LICZBA_ODCINKOW_CZASU];		//zmierzony czas obsługo odcinka
uint32_t nMaxCzasOdcinka[LICZBA_ODCINKOW_CZASU];	//maksymalna wartość czasu odcinka
uint32_t nCzasJalowy;

uint8_t chErrPG = BLAD_OK;		//błąd petli głównej
uint8_t chStanIOwy, chStanIOwe;	//stan wejść IO modułów wewnetrznych
extern uint8_t chBuforAnalizyGNSS[ROZMIAR_BUF_ANA_GNSS];
extern volatile uint8_t chWskNapBaGNSS, chWskOprBaGNSS;
uint16_t chTimeoutGNSS;		//licznik timeoutu odbierania danych z modułu GNSS. Po timeoucie inicjalizuje moduł.
static uint8_t chEtapOperacjiI2C;
uint8_t chGeneratorNapisow, chLicznikKomunikatow;
extern I2C_HandleTypeDef hi2c3;
volatile uint8_t chCzujnikOdczytywanyNaI2CExt;	//identyfikator czujnika obsługiwanego na zewntrznej magistrali I2C. Potrzebny do tego aby powiązać odczytane dane z rodzajem obróbki
volatile uint8_t chCzujnikOdczytywanyNaI2CInt;	//identyfikator czujnika obsługiwanego na wewnętrznej magistrali I2C: MAG_MMC lub MAG_IIS
volatile uint8_t chCzujnikZapisywanyNaI2CInt;
uint8_t chNoweDaneI2C;	//zestaw flag informujący o pojawieniu sie nowych danych odebranych na magistrali I2C
extern uint16_t sLicznikCzasuKalibracji;
uint8_t chPoprzedniRodzajPomiaru;	//okresla czy poprzedni pomiar magnetometrem MMC był ze zmianą przemagnesowania czy bez
float fPoleCzujnkaMMC[3];
extern stRC_t stRC;					//struktura przechowująca dane odbiorników RC
extern stKonfPID_t stKonfigPID[LICZBA_PID];	//struktura przechowująca dane dotyczące konfiguracji regulatora PID
extern stMikser_t stMikser[KANALY_MIKSERA];	//struktura zmiennych miksera

////////////////////////////////////////////////////////////////////////////////
// Pętla główna programu autopilota
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PetlaGlowna(void)
{
	//Ponieważ dekoder modułów  steruje zarówno linią CS modułu oraz przełącza multipleksery kanałów przetwornika A/C
	// więc równolegle z pierwszymi 8 odcinkami pętli głównej wykonaj pomiary analogowe
	if (chNrOdcinkaCzasu < 8)
	{
		chErrPG |= UstawDekoderModulow(chNrOdcinkaCzasu);
		PomiarADC(chNrOdcinkaCzasu);
	}

	switch (chNrOdcinkaCzasu)
	{
	case ADR_MOD1:		//obsługa modułu w gnieździe 1
		break;

	case ADR_MOD2:		//obsługa modułu w gnieździe 2
		uint8_t chErr = ObslugaModuluI2P(ADR_MOD2, &chStanIOwy);
		if (chErr)
			chStanIOwy &= ~MIO40;	//zaświeć czerwoną LED
		else
			chStanIOwy |= MIO40;	//zgaś czerwoną LED
		break;

	case ADR_MOD3:		//obsługa modułu w gnieździe 3
		//chErrPG |= ObslugaModuluIiP(ADR_MOD3);
		TestyObrotu(ADR_MOD3);
		break;

	case ADR_MOD4:		//obsługa modułu w gnieździe 4
		for (uint16_t n=8; n<KANALY_SERW; n++)
		{
			uDaneCM4.dane.sSerwo[n] += n;
			if (uDaneCM4.dane.sSerwo[n] > 2000)
				uDaneCM4.dane.sSerwo[n] = 0;
		}
		break;

	case 4:
		break;

	case 5:		//obsługa GNSS na UART8
		while (chWskNapBaGNSS != chWskOprBaGNSS)
		{
			chErrPG |= DekodujNMEA(chBuforAnalizyGNSS[chWskOprBaGNSS]);	//analizuj dane z GNSS
			chWskOprBaGNSS++;
			chWskOprBaGNSS &= MASKA_ROZM_BUF_ANA_GNSS;
			//chStanIOwy ^= MIO41;		//Zielona LED
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

	case 6:
		uDaneCM4.dane.chNowyPomiar = 0;	//unieważnij wszystkie poprzednie pomiary. Flagi nowych pomiarów zostaną ustawnine w funkcji ObslugaCzujnikowI2C()
		if (chNoweDaneI2C)
			ObslugaCzujnikowI2C(&chNoweDaneI2C);	//jeżeli odebrano nowe dane z czujników na obu magistralach I2C to je obrób
		chErrPG |= RozdzielniaOperacjiI2C();
		break;

	case 7:	JednostkaInercyjnaTrygonometria(ndT);	break;	//dane do IMU1
	case 8:	JednostkaInercyjnaKwaterniony(ndT, (float*)uDaneCM4.dane.fZyroKal2, (float*)uDaneCM4.dane.fAkcel2, (float*)uDaneCM4.dane.fMagne2);	break;	//dane do IMU2
	case 9:	chErrPG |= DywersyfikacjaOdbiornikowRC(&stRC, &uDaneCM4.dane);	break;
	case 10:	break;
	case 11:
		//chErrPG |= PobierzDaneExpandera(&chStanIOwe);		//wszystkie porty ustawione na wyjściowe, nie ma co pobierać
		chErrPG |= WyslijDaneExpandera(chStanIOwy); 	break;

	case 12:
		break;

	case 15:	//wymień dane między rdzeniami
		uDaneCM4.dane.chErrPetliGlownej = chErrPG;
		chErrPG  = UstawDaneWymiany_CM4();
		chErrPG |= PobierzDaneWymiany_CM7();
		WykonajPolecenieCM7();		//wykonaj polecenie przekazane z CM7
		break;

	case 16:	//pozwól na testowe uruchomienie inicjalizacji
		if (chBuforAnalizyGNSS[0] == 0xFF)
		{
			//InicjujWyjsciaRC();
			//InicjujWyjsciaRC();
			//InicjujModulI2P();
			//chBuforAnalizyGNSS[0] = 0;
			InicjujPID();
		}
		break;

	case 17:	StabilizacjaPID(ndT, &uDaneCM4.dane, stKonfigPID);	break;
	case 18:	LiczMikser(stMikser, &uDaneCM4.dane, stKonfigPID);	break;
	case 19:	AktualizujWyjsciaRC(&uDaneCM4.dane);	break;
	default:	break;
	}



	nCzasBiezacy = PobierzCzas();
	ndT = MinalCzas2(nCzasPoprzedniegoObiegu, nCzasBiezacy);	//licz czas od ostatniego obiegu pętli
	nCzasPoprzedniegoObiegu = nCzasBiezacy;

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
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);	//kanał serw 2 skonfigurowany jako IO
	do
	{
		__WFI();	//uśpij kontroler w oczekwianiu na przerwanie od czegokolwiek np. timera 7 mierzącego czas
		nCzasJalowy = PobierzCzas() - nCzasOstatniegoOdcinka;
	}
	while (nCzasJalowy < CZAS_ODCINKA);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);	//kanał serw 2 skonfigurowany jako IO

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
// Parametry: nPoczatek - licznik czasu na na początku pomiaru
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
// Parametry:
// [we] nPoczatek - licznik czasu na na początku pomiaru
// [we] nKoniec - licznik czasu na na końcu pomiaru
// Zwraca: ilość czasu w mikrosekundach jaki upłynął do podanego czasu początkowego
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas2(uint32_t nPoczatek, uint32_t nKoniec)
{
	uint32_t nCzas;

	if (nKoniec >= nPoczatek)
		nCzas = nKoniec - nPoczatek;
	else
		nCzas = 0xFFFFFFFF - nPoczatek + nKoniec;
	return nCzas;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje polecenie przekazane z rdzenia CM7
// Parametry: brak
// Zwraca: nic
// Czas wykonania: 680ns (dla POL_NIC)
////////////////////////////////////////////////////////////////////////////////
void WykonajPolecenieCM7(void)
{
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
		uDaneCM4.dane.uRozne.f32[2] = CzytajFramFloat(FAH_ZYRO1P_WZMOC);
		uDaneCM4.dane.uRozne.f32[3] = CzytajFramFloat(FAH_ZYRO2P_WZMOC);
		break;

	case POL_CZYTAJ_WZM_ZYROQ:	//odczytaj wzmocnienia żyroskopów Q
		uDaneCM4.dane.uRozne.f32[2] = CzytajFramFloat(FAH_ZYRO1Q_WZMOC);
		uDaneCM4.dane.uRozne.f32[3] = CzytajFramFloat(FAH_ZYRO2Q_WZMOC);
		break;

	case POL_CZYTAJ_WZM_ZYROR:	//odczytaj wzmocnienia żyroskopów R
		uDaneCM4.dane.uRozne.f32[2] = CzytajFramFloat(FAH_ZYRO1R_WZMOC);
		uDaneCM4.dane.uRozne.f32[3] = CzytajFramFloat(FAH_ZYRO2R_WZMOC);
		break;

	case POL_KAL_ZERO_MAGN1:	ZnajdzEkstremaMagnetometru((float*)uDaneCM4.dane.fMagne1);	break;	//uruchom kalibrację zera magnetometru 1
	case POL_KAL_ZERO_MAGN2:	ZnajdzEkstremaMagnetometru((float*)uDaneCM4.dane.fMagne2);	break;	//uruchom kalibrację zera magnetometru 2
	case POL_KAL_ZERO_MAGN3:	ZnajdzEkstremaMagnetometru((float*)uDaneCM4.dane.fMagne3);	break;	//uruchom kalibrację zera magnetometru 3

	case POL_ZAPISZ_KONF_MAGN1:	ZapiszKalibracjeMagnetometru(MAG1);	break;
	case POL_ZAPISZ_KONF_MAGN2:	ZapiszKalibracjeMagnetometru(MAG2);	break;
	case POL_ZAPISZ_KONF_MAGN3:	ZapiszKalibracjeMagnetometru(MAG3);	break;

	case POL_POBIERZ_KONF_MAGN1: PobierzKalibracjeMagnetometru(MAG1);	break;
	case POL_POBIERZ_KONF_MAGN2: PobierzKalibracjeMagnetometru(MAG2);	break;
	case POL_POBIERZ_KONF_MAGN3: PobierzKalibracjeMagnetometru(MAG3);	break;

	case POL_ZERUJ_EKSTREMA:
		if (ZerujEkstremaMagnetometru())
		{
			uDaneCM4.dane.chOdpowiedzNaPolecenie = POL_NIC;	//jeżeli dane nie są jeszcze wyzerowane to zwróć inną odpowiedź niż numer polecenia
			//chStanIOwy &= ~MIO40;	//zaświeć czerwoną LED
			//chStanIOwy |= MIO41;	//zgaś zieloną LED
		}
		else
		{
			//chStanIOwy |= MIO40;	//zgaś czerwoną LED
			//chStanIOwy &= ~MIO41;	//zaświeć zieloną LED
		}
		break;

	case POL_INICJUJ_USREDN:	KalibrujCisnienie(0, 0, 0, CZAS_KALIBRACJI, 0xFF);	break;	//inicjalizacja
	case POL_ZERUJ_LICZNIK:
		sLicznikCzasuKalibracji = 0;
		break;

	case POL_USREDNIJ_CISN1:
		uDaneCM4.dane.chOdpowiedzNaPolecenie = KalibrujCisnienie(uDaneCM4.dane.fCisnieBzw[0], uDaneCM4.dane.fCisnieBzw[1], uDaneCM4.dane.fTemper[TEMP_BARO1], sLicznikCzasuKalibracji, 0);
		if (sLicznikCzasuKalibracji <= CZAS_KALIBRACJI)
			uDaneCM4.dane.sPostepProcesu = sLicznikCzasuKalibracji++;
		break;

	case POL_USREDNIJ_CISN2:
		uDaneCM4.dane.chOdpowiedzNaPolecenie = KalibrujCisnienie(uDaneCM4.dane.fCisnieBzw[0], uDaneCM4.dane.fCisnieBzw[1], uDaneCM4.dane.fTemper[TEMP_BARO1], sLicznikCzasuKalibracji, 1);
		if (sLicznikCzasuKalibracji <= CZAS_KALIBRACJI)
			uDaneCM4.dane.sPostepProcesu = sLicznikCzasuKalibracji++;
		break;

	case POL_CZYTAJ_FRAM_U8:
		if (uDaneCM7.dane.chRozmiar > 4*ROZMIAR_ROZNE)
			uDaneCM7.dane.chRozmiar = 4*ROZMIAR_ROZNE;
		CzytajBuforFRAM(uDaneCM7.dane.sAdres, uDaneCM4.dane.uRozne.U8, uDaneCM7.dane.chRozmiar);
		uDaneCM4.dane.chRozmiar = uDaneCM7.dane.chRozmiar;	//odeślij skorygowany rozmiar
		uDaneCM4.dane.sAdres = uDaneCM7.dane.sAdres;		//odeślij adres jako potwierdzenie odczytu
		break;

	case POL_CZYTAJ_FRAM_FLOAT:			//odczytaj i wyślij zawartość FRAM spod podanego adresu
		if (uDaneCM7.dane.chRozmiar > ROZMIAR_ROZNE)
			uDaneCM7.dane.chRozmiar = ROZMIAR_ROZNE;
		for (uint16_t n=0; n<uDaneCM7.dane.chRozmiar; n++)
			uDaneCM4.dane.uRozne.f32[n] = CzytajFramFloat(uDaneCM7.dane.sAdres + n*4);
		uDaneCM4.dane.chRozmiar = uDaneCM7.dane.chRozmiar;	//odeślij skorygowany rozmiar
		uDaneCM4.dane.sAdres = uDaneCM7.dane.sAdres;		//odeślij adres jako potwierdzenie odczytu
		break;

	case POL_ZAPISZ_FRAM_U8:	//zapisz dane uint8_t pod podany adres
		if (uDaneCM7.dane.chRozmiar > 4*ROZMIAR_ROZNE)
			uDaneCM7.dane.chRozmiar = 4*ROZMIAR_ROZNE;
		ZapiszBuforFRAM(uDaneCM7.dane.sAdres, uDaneCM7.dane.uRozne.U8, uDaneCM7.dane.chRozmiar);
		uDaneCM4.dane.chRozmiar = uDaneCM7.dane.chRozmiar;	//odeślij skorygowany rozmiar
		uDaneCM4.dane.sAdres = uDaneCM7.dane.sAdres;		//odeślij adres jako potwierdzenie zapisu
		break;

	case POL_ZAPISZ_FRAM_FLOAT:			//zapisz przekazane dane typu float do FRAM pod podany adres
		if (uDaneCM7.dane.chRozmiar > ROZMIAR_ROZNE)
			uDaneCM7.dane.chRozmiar = ROZMIAR_ROZNE;
		for (uint16_t n=0; n<uDaneCM7.dane.chRozmiar; n++)
			ZapiszFramFloat(uDaneCM7.dane.sAdres + n*4, uDaneCM7.dane.uRozne.f32[n]);
		uDaneCM4.dane.chRozmiar = uDaneCM7.dane.chRozmiar;	//odeślij skorygowany rozmiar
		uDaneCM4.dane.sAdres = uDaneCM7.dane.sAdres;		//odeślij adres jako potwierdzenie zapisu
		break;

	case POL_KASUJ_DRYFT_ZYRO:
		for (uint16_t n=0; n<3; n++)
		{
			uDaneCM4.dane.fKatZyro1[n] = uDaneCM4.dane.fKatAkcel1[n];
			uDaneCM4.dane.fKatZyro2[n] = uDaneCM4.dane.fKatAkcel2[n];
		}
		break;

	case POL_REKONFIG_SERWA_RC:
		InicjujWejsciaRC();
		InicjujWyjsciaRC();
		break;

	case POL_CZYSC_BLEDY:		uDaneCM4.dane.chOdpowiedzNaPolecenie = BLAD_OK;	break;	//nadpisz poprzednio zwrócony błąd

	}
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja dzieli złożone operacje I2C na sekwencje dające sie realizować w tle, dzięki czemu nie blokuje czasu procesora oczekując na zakończenie
// Parametry: brak
// Zwraca: kod błędu operacji I2C
////////////////////////////////////////////////////////////////////////////////
uint8_t RozdzielniaOperacjiI2C(void)
{
	uint8_t chErr = BLAD_OK;
	//printf("I2C");
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);	//kanał serw 2 skonfigurowany jako IO
	//operacje na zewnętrznej magistrali I2C3
	switch(chEtapOperacjiI2C)
	{
	case 0:
	case 2: chErr = ObslugaMS4525();		break;
	case 1:
	case 3:	chErr = ObslugaHMC5883();		break;
	default: break;
	}

	//operacje na wewnętrznej magistrali I2C4
	switch(chEtapOperacjiI2C)
	{
	case 0:
	case 2:	chErr = ObslugaIIS2MDC();		break;
	case 1:
	case 3:	chErr = ObslugaMMC3416x();		break;
	default: break;
	}

	chEtapOperacjiI2C++;
	chEtapOperacjiI2C &= 0x03;
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Callback nadawania I2C. Automatycznie uruchamia drugą część operacji dzielonych
// Parametry: hi2c - wskaźnik na interfejs I2C
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C4)		//magistrala I2C modułów wewnętrznych
	{
		if (chCzujnikZapisywanyNaI2CInt == MAG_IIS_STATUS)	//po zapisie wykonaj operację odczytu
			MagIIS_CzytajStatus();

		if (chCzujnikZapisywanyNaI2CInt == MAG_IIS)	//po zapisie wykonaj operację odczytu
			MagIIS_CzytajDane();

		if (chCzujnikZapisywanyNaI2CInt == MAG_MMC_STATUS)	//po zapisie wykonaj operację odczytu
			MagMMC_CzytajStatus();

		if (chCzujnikZapisywanyNaI2CInt == MAG_MMC)	//po zapisie wykonaj operację odczytu
			MagMMC_CzytajDane();


		chCzujnikZapisywanyNaI2CInt = 0;
	}


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

		chCzujnikOdczytywanyNaI2CExt = 0;
	}

	if (hi2c->Instance == I2C4)		//magistrala I2C modułów wewnętrznych
	{
		if (chCzujnikOdczytywanyNaI2CInt == MAG_IIS)
			chNoweDaneI2C |= MAG_IIS;
		else
		if (chCzujnikOdczytywanyNaI2CInt == MAG_MMC)
			chNoweDaneI2C |= MAG_MMC;

		chCzujnikOdczytywanyNaI2CInt = 0;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Obsługuje obróbkę danych odczytanych z czujników na I2C
// Parametry: chCzujniki - wskaźnik na zmienną z polami bitowymi inforującymi o danych z czujników
// Zwraca: kod błędu
// Czas wykonania: 8/16us
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaCzujnikowI2C(uint8_t *chCzujniki)
{
	extern uint8_t chDaneMagHMC[6];
	extern uint8_t chDaneMagMMC[6];
	extern uint8_t chDaneMagIIS[6];
	extern uint8_t chDaneMS4525[5];
	extern int16_t sPomiarMMCH[3], sPomiarMMCL[3];	//wyniki pomiarów dla dodatniego i ujemnego namagnesowania czujnika
	extern uint8_t chRodzajPomiaruMMC;
	extern float fPrzesMagn1[3], fSkaloMagn1[3];
	extern float fPrzesMagn2[3], fSkaloMagn2[3];
	extern float fPrzesMagn3[3], fSkaloMagn3[3];
	uint8_t chErr = BLAD_OK;
	int16_t sZeZnakiem;	//zmiena robocza do konwersji dnych 8-bitowych bez znaku na liczbę 16-bitową ze znakiem
	float fZeZnakiem;
	const int8_t chZnakIIS[3] = {1, -1, -1};	//magnetometr 1: odwrotnie jest oś Z, ale żeby ją odwrócić, trzeba zmienić też znak w osi X lub Y. Zmieniam w Y: OK
	//const int8_t chZnakMMC[3] = {-1, 1, -1};	//magnetometr 2: odwrotnie jest oś X, ale żeby ją odwrócić, trzeba zmienić też znak w osi Y lub Z. Zmieniam w Z - zle
	const int8_t chZnakMMC[3] = {-1, -1, 1};		//magnetometr 2: odwrotnie jest oś X, ale żeby ją odwrócić, trzeba zmienić też znak w osi Y lub Z. Zmieniam w Z
	const int8_t chZnakHMC[3] = {1, -1, 1};	//magnetometr 3: jest OK

	if (*chCzujniki & MAG_HMC)
	{
		//Uwaga! Czujnik HMS5883L ma osie w nietypowej kolejności XZY, więc konwersję trzeba wykonać ręcznie poza pętlą
		sZeZnakiem = ((int16_t)chDaneMagHMC[0] * 0x100 + chDaneMagHMC[1]) * chZnakHMC[0];	//oś X
		if ((uDaneCM7.dane.chWykonajPolecenie == POL_KAL_ZERO_MAGN3) ||  (uDaneCM7.dane.chWykonajPolecenie == POL_ZERUJ_EKSTREMA))
			uDaneCM4.dane.fMagne3[0] = (float)sZeZnakiem * CZULOSC_HMC5883;			//dane surowe podczas kalibracji magnetometru wyrażone w Teslach
		else
			uDaneCM4.dane.fMagne3[0] = ((float)sZeZnakiem * CZULOSC_HMC5883 - fPrzesMagn3[0]) * fSkaloMagn3[0];	//dane skalibrowane

		sZeZnakiem = ((int16_t)chDaneMagHMC[4] * 0x100 + chDaneMagHMC[5]) * chZnakHMC[1];	//oś Y
		if ((uDaneCM7.dane.chWykonajPolecenie == POL_KAL_ZERO_MAGN3) ||  (uDaneCM7.dane.chWykonajPolecenie == POL_ZERUJ_EKSTREMA))
			uDaneCM4.dane.fMagne3[1] = (float)sZeZnakiem * CZULOSC_HMC5883;			//dane surowe podczas kalibracji magnetometru
		else
			uDaneCM4.dane.fMagne3[1] = ((float)sZeZnakiem * CZULOSC_HMC5883 - fPrzesMagn3[1]) * fSkaloMagn3[1];	//dane skalibrowane

		sZeZnakiem = ((int16_t)chDaneMagHMC[2] * 0x100 + chDaneMagHMC[3]) * chZnakHMC[2];	//oś Z
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
		uDaneCM4.dane.chNowyPomiar |= NP_EXT_IAS;	//jest nowy pomiar
	}

	if (*chCzujniki & MAG_IIS)
	{
		for (uint8_t n=0; n<3; n++)
		{
			sZeZnakiem = ((int16_t)chDaneMagIIS[2*n+1] * 0x100 + chDaneMagIIS[2*n]) * chZnakIIS[n];
			if ((uDaneCM7.dane.chWykonajPolecenie == POL_KAL_ZERO_MAGN1) || (uDaneCM7.dane.chWykonajPolecenie == POL_ZERUJ_EKSTREMA))
				uDaneCM4.dane.fMagne1[n] = (float)sZeZnakiem * CZULOSC_IIS2MDC;			//dane surowe podczas kalibracji magnetometru
			else
				uDaneCM4.dane.fMagne1[n] = ((float)sZeZnakiem * CZULOSC_IIS2MDC - fPrzesMagn1[n]) * fSkaloMagn1[n];	//dane skalibrowane
				//uDaneCM4.dane.fMagne1[n] = (uDaneCM4.dane.fMagne1[n] + ((float)sZeZnakiem * CZULOSC_IIS2MDC - fPrzesMagn1[n]) * fSkaloMagn1[n]) / 2;	//filtruj pomiar bo jest mocno zaszumiony a jest wystarczajaco szybki żeby filtracja nie przesuwała istotnie fazy
		}
		*chCzujniki &= ~MAG_IIS;	//dane obsłużone
		uDaneCM4.dane.chNowyPomiar |= NP_MAG1;	//jest nowy pomiar
	}

	if (*chCzujniki & MAG_MMC)
	{
		for (uint8_t n=0; n<3; n++)
		{
			if (chRodzajPomiaruMMC < SPMMC3416_REFIL_RESET)	//poniżej SPMMC3416_REFIL_RESET będzie obsługa dla SET, powyżej dla RESET
				sPomiarMMCH[n] = (((int16_t)chDaneMagMMC[2*n+1] * 0x100 + chDaneMagMMC[2*n]) - 32768) * chZnakMMC[n];	//czujnik zwraca liczby unsigned z 32768 dla zera
			else
				sPomiarMMCL[n] = (((int16_t)chDaneMagMMC[2*n+1] * 0x100 + chDaneMagMMC[2*n]) - 32768) * chZnakMMC[n];

			//dla zmiany przemagnesowania czyli świeżych pomiarów H+ i H- policz nateżenie pola ziemi bezpośrednio odejmując je od siebie
			if (chRodzajPomiaruMMC != chPoprzedniRodzajPomiaru)
			{
				float fBiezacePoleCzujnika;

				fZeZnakiem = (float)(sPomiarMMCH[n] - sPomiarMMCL[n]) / 2;
				if (chRodzajPomiaruMMC < SPMMC3416_REFIL_RESET)
					fBiezacePoleCzujnika = fZeZnakiem - sPomiarMMCH[n];	//licz pole namagnesowania czujnika
				else
					fBiezacePoleCzujnika = -fZeZnakiem - sPomiarMMCL[n];
				fPoleCzujnkaMMC[n] = (7 * fPoleCzujnkaMMC[n] + fBiezacePoleCzujnika) / 8;	//filtruj obliczenia pola magnesowania czujnika
			}
			else	//dla ciagłego pomiaru z jednym typem magnesowania, natężenie pola ziemi licz odejmując obliczone wcześniej pole czujnika
			{
				if (chRodzajPomiaruMMC < SPMMC3416_REFIL_RESET)
					fZeZnakiem = sPomiarMMCH[n] + fPoleCzujnkaMMC[n];
				else
					fZeZnakiem = (sPomiarMMCL[n] + fPoleCzujnkaMMC[n]) * -1;
			}

			if ((uDaneCM7.dane.chWykonajPolecenie == POL_KAL_ZERO_MAGN2) || (uDaneCM7.dane.chWykonajPolecenie == POL_ZERUJ_EKSTREMA))
				uDaneCM4.dane.fMagne2[n] = fZeZnakiem * CZULOSC_MMC34160;	//dane surowe podczas kalibracji magnetometru
			else
				uDaneCM4.dane.fMagne2[n] = (fZeZnakiem * CZULOSC_MMC34160 - fPrzesMagn2[n]) * fSkaloMagn2[n];	//dane skalibrowane;
		}
		chPoprzedniRodzajPomiaru = chRodzajPomiaruMMC;
		*chCzujniki &= ~MAG_MMC;	//dane obsłużone
		uDaneCM4.dane.chNowyPomiar |= NP_MAG2;	//jest nowy pomiar
	}
	return chErr;
}


