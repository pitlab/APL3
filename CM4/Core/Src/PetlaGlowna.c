//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi pętli głównej programu autopilota na rdzeniu CM4
//
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "PetlaGlowna.h"
#include "Czas.h"
#include <Adc.h>
#include <DShot.h>
#include <Fram.h>
#include <GNSS.h>
#include <KonfigFram.h>
#include <Main.h>
#include <Mikser.h>
#include <Nmea.h>
#include "ModulyWew.h"
#include "WymianaCM4.h"
#include "HMC5883.h"
#include "JednostkaInercyjna.h"
#include <stdio.h>
#include <SBus.h>
#include <WS281x.h>
#include "Modul_I2P.h"
#include "MS4525.h"
#include "ND130.h"
#include "RegulatorPID.h"
#include "WeWyRC.h"
#include "KontrolerLotu.h"
#include "SampleAudio.h"
#include "Crossfire.h"
#include <Kalman.h>

extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
uint16_t sGenerator;
uint32_t nCzasOstatniegoOdcinka;	//przechowuje czas uruchomienia ostatniego odcinka
uint32_t nCzasPoprzedniegoObiegu;	//czas [us] poprzedniego obiegu pętli głównej
uint32_t ndT;						//czas [us] jaki upłynął od poprzeniego obiegu pętli
uint32_t nCzasBiezacy;
uint8_t cNrOdcinkaCzasu;
uint32_t nCzasOdcinka[LICZBA_ODCINKOW_CZASU + 1];		//zmierzony czas obsługi odcinka. Na ostatniej pozycji jest czas jałowy
uint32_t nMaxCzasOdcinka[LICZBA_ODCINKOW_CZASU];	//maksymalna wartość czasu odcinka
uint8_t cBłądPG = BLAD_OK;		//błąd petli głównej
uint8_t cStanIOwy, cStanIOwe;	//stan wejść IO modułów wewnetrznych
extern uint8_t cBuforAnalizyGNSS[ROZMIAR_BUF_ANA_GNSS];
extern volatile uint8_t cWskNapBufAnaGNSS, cWskOprBufAnaGNSS;		//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych GNSS
uint16_t sTimeoutGNSS;		//licznik timeoutu odbierania danych z modułu GNSS. Po timeoucie inicjalizuje moduł.
static uint8_t cEtapOperacjiI2C;
uint8_t cGeneratorNapisow, cLicznikKomunikatow;
extern I2C_HandleTypeDef hi2c3;
volatile uint8_t cCzujnikOdczytywanyNaI2CExt;	//identyfikator czujnika obsługiwanego na zewntrznej magistrali I2C. Potrzebny do tego aby powiązać odczytane dane z rodzajem obróbki
volatile uint8_t cCzujnikOdczytywanyNaI2CInt;	//identyfikator czujnika obsługiwanego na wewnętrznej magistrali I2C: MAG_MMC lub MAG_IIS
volatile uint8_t cCzujnikZapisywanyNaI2CInt;
uint8_t cNoweDaneI2C;	//zestaw flag informujący o pojawieniu sie nowych danych odebranych na magistrali I2C
extern uint16_t sLicznikCzasuKalibracji;
uint8_t cPoprzedniRodzajPomiaru;	//okresla czy poprzedni pomiar magnetometrem MMC był ze zmianą przemagnesowania czy bez
float fPoleCzujnkaMMC[3];
//extern stRC2_t stRC;		//struktura przechowująca dane odbiorników RC
extern stRC_t stRC1, stRC2;	//struktura danych odbiorników RC1 i RC2
extern stKonfPID_t stKonfigPID[LICZBA_PID];	//struktura przechowująca dane dotyczące konfiguracji regulatora PID
extern stMikser_t stMikser[KANALY_MIKSERA];	//struktura zmiennych miksera
extern float fSkalaWartosciZadanejAkro[LICZBA_DRAZKOW];	//wartość zadana dla pełnego wychylenia drążka aparatury w trybie AKRO
extern float fSkalaWartosciZadanejStab[LICZBA_DRAZKOW];	//wartość zadana dla pełnego wychylenia drążka aparatury w trybie STAB
extern uint16_t sWysterowanieMin;		//wartość wysterowania regulatorów dla uzyskania obrotów minimalnych w trakcie lotu
extern uint16_t sWysterowanieZawisu;	//wartość wysterowania regulatorów dla uzyskania obrotów pozwalajacych na zawis
extern uint16_t sWysterowanieMax;		//wartość wysterowania regulatorów dla uzyskania obrotów maksymalnych
extern uint8_t cTrybRegulacji[LICZBA_REG_PARAM];	//rodzaj regulacji dla 4 podstawowych parametrów sterowanych z aparatury
extern uint8_t cFunkcjaWyjscRC[KANALY_WYJSC_RC];		//funkcje przypisane do kanałów wyjściowych: silnik, LED, retransmisja wejścia RC
extern uint8_t cFunkcjaSilnika[KANALY_MIKSERA];		//funkcje przypisane do silników: normalna praca lub analiza FFT rezonansu drgań ramy
extern uint16_t sTS_CAL1, sTS_CAL2;	//wspólczynniki kalibracji czujnika temperatury odczytywane w CM7 i przekazywane poleceniem
extern uint8_t cWykonanoPomiarADC;	//pole bitowe wykonania pomiarów bit0 = ADC2, bit1 = ADC3
uint8_t cBityPozwoleniaNaPomiarADC;	//pole bitowe informujące który pomiar można wykonać w danym obiegu pętli
extern uint16_t sWysterowanieIdentSiln;	//wysterowanie silników podczas procesu identfikacji


////////////////////////////////////////////////////////////////////////////////
// Pętla główna programu autopilota
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PetlaGlowna(void)
{
	uint32_t nCzasStartuADC;

	//przykłady machania pinami IO
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);	//kanał serw 1 skonfigurowany jako IO
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);	//kanał serw 2 skonfigurowany jako IO
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);	//kanał serw 4 skonfigurowany jako IO
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);	//kanał serw 5 skonfigurowany jako IO
	//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);	//kanał serw 7 skonfigurowany jako IO



	//Ponieważ dekoder modułów  steruje zarówno linią CS modułu oraz przełącza multipleksery kanałów przetwornika A/C
	// więc równolegle z pierwszymi 8 odcinkami pętli głównej wykonaj pomiary analogowe
	if (cNrOdcinkaCzasu < LICZBA_POMIAROW_ADC3)
	{
		nCzasStartuADC = PobierzCzasT7();
		cBłądPG |= ObsługaDekoderaiADC(cNrOdcinkaCzasu, cBityPozwoleniaNaPomiarADC);	//zarządza rozpoczęciem pomiaru ADC i pobraniem wyników, przełacza dekoder modułów
		nCzasOdcinka[20] = MinalCzasT7(nCzasStartuADC);		//czas konwersji ADC
	}

	switch (cNrOdcinkaCzasu)
	{
	case ADR_MOD1:		//obsługa modułu w gnieździe 1
		uDaneCM4.dane.uRozne.f32[11] += 0.5f;
		//do; while (( chWykonanoPomiarADC != WYKONANO_POMIAR_ADC2) || (chWykonanoPomiarADC != WYKONANO_POMIAR_ADC3));	//czekaj na wykonanie pomiarów ADC
		break;

	case 11:	//moduł jest obsługiwany na 2 slotach aby szybciej dostarczać dane dla filtrów  i FFT

	case ADR_MOD2:		//obsługa modułu w gnieździe 2
		cBłądPG = ObslugaModuluI2P(ADR_MOD2, &cStanIOwy);
		if (cBłądPG)
			cStanIOwy &= ~MIO40;	//zaświeć czerwoną LED
		else
			cStanIOwy |= MIO40;	//zgaś czerwoną LED
		break;

	case ADR_MOD3:		//obsługa modułu w gnieździe 3
		//do; while (( chWykonanoPomiarADC != WYKONANO_POMIAR_ADC2) || (chWykonanoPomiarADC != WYKONANO_POMIAR_ADC3));	//czekaj na wykonanie pomiarów ADC
		break;

	case ADR_MOD4:		//obsługa modułu w gnieździe 4
		//do; while (( chWykonanoPomiarADC != WYKONANO_POMIAR_ADC2) || (chWykonanoPomiarADC != WYKONANO_POMIAR_ADC3));	//czekaj na wykonanie pomiarów ADC
		break;

	case 4:		//obsługa GNSS na UART8
		while (cWskNapBufAnaGNSS != cWskOprBufAnaGNSS)
		{
			cBłądPG |= DekodujNMEA(cBuforAnalizyGNSS[cWskOprBufAnaGNSS]);	//analizuj dane z GNSS
			cWskOprBufAnaGNSS++;
			cWskOprBufAnaGNSS &= MASKA_ROZM_BUF_ANA_GNSS;
			sTimeoutGNSS = TIMEOUT_GNSS;
		}
		if ((uDaneCM4.dane.nZainicjowano & INIT_GNSS_GOTOWY) == 0)
		{
			InicjujGNSS();		//gdy nie jest zainicjowany to przeprowadź odbiornik przez kolejne etapy inicjalizacji
		}
		if (sTimeoutGNSS)
			sTimeoutGNSS--;
		else
		{
			sTimeoutGNSS = TIMEOUT_GNSS;
			uDaneCM4.dane.nZainicjowano &= ~MASKA_INIT_GNSS;	//wyczyść wszystkie bity używane przez GNSS
			uDaneCM4.dane.stGnss1.chFix = 0;
		}
		break;

	case 5:
		uDaneCM4.dane.cNowyPomiar = 0;	//unieważnij wszystkie poprzednie pomiary. Flagi nowych pomiarów zostaną ustawnine w funkcji ObslugaCzujnikowI2C()
		if (cNoweDaneI2C)
			ObslugaCzujnikowI2C(&cNoweDaneI2C);	//jeżeli odebrano nowe dane z czujników na obu magistralach I2C to je obrób
		cBłądPG |= RozdzielniaOperacjiI2C();
		break;

	case 6:	//przepisz czujniki do struktury BSP - finalnie ma to zrobić filtr Kalmana
		FiltrDanychIMUiWysokosci(&uDaneCM4.dane);
		break;

	case 7:
		nCzasBiezacy = PobierzCzasT7();
		ndT = MinalCzas2T7(nCzasPoprzedniegoObiegu, nCzasBiezacy);	//licz czas od ostatniego obiegu pętli
		uDaneCM4.dane.ndT = ndT;
		nCzasPoprzedniegoObiegu = nCzasBiezacy;
		JednostkaInercyjnaTrygonometria(ndT);	break;	//dane do IMU1

	case 8:	JednostkaInercyjnaKwaterniony(ndT, (float*)uDaneCM4.dane.fZyroKal2, (float*)uDaneCM4.dane.fAkcel2, (float*)uDaneCM4.dane.fMagne2);	break;	//dane do IMU2

	case 9:	//obsługa odbiorników RC
		cBłądPG |= ObsługaRamkiSBus();
		cBłądPG |= ObsługaRamkiCrossfire();
		cBłądPG |= DywersyfikacjaOdbiornikowRC(&stRC1, &stRC2, uDaneCM7.dane.cWyborOdbiornikaRC, &uDaneCM4.dane);	//scalenie obu kanałów w jedne dane dane odbiornika RC
		cBłądPG |= AnalizujSygnalRC(&uDaneCM4.dane, &uDaneCM7.dane);
		break;

	case 10:
		//cBłądPG |= PobierzDaneExpandera(&chStanIOwe);		//wszystkie porty ustawione na wyjściowe, nie ma co pobierać
		cBłądPG |= WyslijDaneExpandera(cStanIOwy); 	break;

	case 12:	//wymień dane między rdzeniami
		uDaneCM4.dane.cBladPetliGlownej = cBłądPG;
		cBłądPG  = UstawDaneWymiany_CM4();
		cBłądPG |= PobierzDaneWymiany_CM7();
		break;

	case 13:
		WykonajPolecenieCM7();		//wykonaj polecenie przekazane z CM7
		break;

	case 15:	//pozwól na testowe uruchomienie inicjalizacji
		if (cBuforAnalizyGNSS[0] == 0xFF)
		{
			InicjujMikser();
			//InicjujWyjsciaRC();
			//InicjujModulI2P();
			//chBuforAnalizyGNSS[0] = 0;
			//InicjujPID();
		}
		break;

	case 16:
		/*if (cDzielnikAktualizacjiLED)
		{
			cDzielnikAktualizacjiLED--;

		}
		else
		{
			cDzielnikAktualizacjiLED = DZIELNIK_AKTUALIZACJI_LED;
			AktualizujKolorLedWs821x();
		}*/
		break;

	case 17:	cBłądPG |= KontrolerLotu(cTrybRegulacji, ndT, &uDaneCM4.dane, stKonfigPID);	break;
	case 18:	LiczMikser(stMikser, &uDaneCM4.dane, stKonfigPID);	break;
	case 19:	AktualizujWyjsciaRC(&uDaneCM4.dane);	break;
	default:	break;
	}



	//pomiar czasu zajętego w każdym odcinku
	nCzasOdcinka[cNrOdcinkaCzasu] = MinalCzasT7(nCzasOstatniegoOdcinka);
	if (nCzasOdcinka[cNrOdcinkaCzasu] > nMaxCzasOdcinka[cNrOdcinkaCzasu])   //przechwyć wartość maksymalną
		nMaxCzasOdcinka[cNrOdcinkaCzasu] = nCzasOdcinka[cNrOdcinkaCzasu];
	nCzasOstatniegoOdcinka = PobierzCzasT7();

	cNrOdcinkaCzasu++;
	if (cNrOdcinkaCzasu == LICZBA_ODCINKOW_CZASU)
	{
		cNrOdcinkaCzasu = 0;
		cBityPozwoleniaNaPomiarADC <<= 1;
		if (cBityPozwoleniaNaPomiarADC == 0)
			cBityPozwoleniaNaPomiarADC = 1;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje polecenie przekazane z rdzenia CM7
// Parametry: brak
// Zwraca: nic
// Czas wykonania: 680ns (dla POL_NIC)
////////////////////////////////////////////////////////////////////////////////
void WykonajPolecenieCM7(void)
{
	if ((uDaneCM7.dane.cWykonajPolecenie != uDaneCM4.dane.cPotwierdzenieWykonania) | (uDaneCM7.dane.sAdres != uDaneCM4.dane.sAdres))
	{
		uDaneCM4.dane.cPotwierdzenieWykonania = uDaneCM7.dane.cWykonajPolecenie;	//domyślnie odeślij to co przyszło aby potwierdzić wykonanie
		uDaneCM4.dane.sAdres = uDaneCM7.dane.sAdres;	//potwierdź wykonanie operacji na tym adresie aby można było wykonać kolejną operację
		switch(uDaneCM7.dane.cWykonajPolecenie)
		{
		case POL7_NIC:	break;		//polecenie neutralne
		case POL7_KALIBRUJ_ZYRO_ZIM:	RozpocznijKalibracjeZeraZyroskopu(POL7_KALIBRUJ_ZYRO_ZIM);	break;		//uruchom kalibrację żyroskopów na zimno 10°C
		case POL7_KALIBRUJ_ZYRO_POK:	RozpocznijKalibracjeZeraZyroskopu(POL7_KALIBRUJ_ZYRO_POK);	break;		//uruchom kalibrację żyroskopów w temperaturze pokojowej 25°C
		case POL7_KALIBRUJ_ZYRO_GOR:	RozpocznijKalibracjeZeraZyroskopu(POL7_KALIBRUJ_ZYRO_GOR);	break;		//uruchom kalibrację żyroskopów na gorąco 40°C

		case POL7_KALIBRUJ_ZYRO_WZMP:		//uruchom kalibrację wzmocnienia żyroskopów P
		case POL7_KALIBRUJ_ZYRO_WZMQ:		//uruchom kalibrację wzmocnienia żyroskopów Q
		case POL7_KALIBRUJ_ZYRO_WZMR:		//uruchom kalibrację wzmocnienia żyroskopów R
		case POL7_ZERUJ_CALKE_ZYRO:	KalibracjaWzmocnieniaZyro(uDaneCM7.dane.cWykonajPolecenie);	break;	//zeruje całkę prędkosci katowej żyroskopów przed kalibracją wzmocnienia

		case POL7_CZYTAJ_WZM_ZYROP:	//odczytaj wzmocnienia żyroskopów P
			uDaneCM4.dane.uRozne.f32[2] = CzytajFramFloat(FAH_ZYRO1P_WZMOC);
			uDaneCM4.dane.uRozne.f32[3] = CzytajFramFloat(FAH_ZYRO2P_WZMOC);
			break;

		case POL7_CZYTAJ_WZM_ZYROQ:	//odczytaj wzmocnienia żyroskopów Q
			uDaneCM4.dane.uRozne.f32[2] = CzytajFramFloat(FAH_ZYRO1Q_WZMOC);
			uDaneCM4.dane.uRozne.f32[3] = CzytajFramFloat(FAH_ZYRO2Q_WZMOC);
			break;

		case POL7_CZYTAJ_WZM_ZYROR:	//odczytaj wzmocnienia żyroskopów R
			uDaneCM4.dane.uRozne.f32[2] = CzytajFramFloat(FAH_ZYRO1R_WZMOC);
			uDaneCM4.dane.uRozne.f32[3] = CzytajFramFloat(FAH_ZYRO2R_WZMOC);
			break;

		case POL7_KAL_ZERO_MAGN1:	ZnajdzEkstremaMagnetometru((float*)uDaneCM4.dane.fMagne1);	break;	//uruchom kalibrację zera magnetometru 1
		case POL7_KAL_ZERO_MAGN2:	ZnajdzEkstremaMagnetometru((float*)uDaneCM4.dane.fMagne2);	break;	//uruchom kalibrację zera magnetometru 2
		case POL7_KAL_ZERO_MAGN3:	ZnajdzEkstremaMagnetometru((float*)uDaneCM4.dane.fMagne3);	break;	//uruchom kalibrację zera magnetometru 3

		case POL7_ZAPISZ_KONF_MAGN1:	ZapiszKalibracjeMagnetometru(MAG1);	break;
		case POL7_ZAPISZ_KONF_MAGN2:	ZapiszKalibracjeMagnetometru(MAG2);	break;
		case POL7_ZAPISZ_KONF_MAGN3:	ZapiszKalibracjeMagnetometru(MAG3);	break;

		case POL7_POBIERZ_KONF_MAGN1: PobierzKalibracjeMagnetometru(MAG1);	break;
		case POL7_POBIERZ_KONF_MAGN2: PobierzKalibracjeMagnetometru(MAG2);	break;
		case POL7_POBIERZ_KONF_MAGN3: PobierzKalibracjeMagnetometru(MAG3);	break;

		case POL7_ZERUJ_EKSTREMA:
			if (ZerujEkstremaMagnetometru())
			{
				//uDaneCM4.dane.chOdpowiedzNaPolecenie = POL7_NIC;	//jeżeli dane nie są jeszcze wyzerowane to zwróć inną odpowiedź niż numer polecenia
				uDaneCM4.dane.cPotwierdzenieWykonania = POL7_NIC;	//jeżeli dane nie są jeszcze wyzerowane to zwróć inną odpowiedź niż numer polecenia
				//cStanIOwy &= ~MIO40;	//zaświeć czerwoną LED
				//cStanIOwy |= MIO41;	//zgaś zieloną LED
			}
			else
			{
				//cStanIOwy |= MIO40;	//zgaś czerwoną LED
				//cStanIOwy &= ~MIO41;	//zaświeć zieloną LED
			}
			break;

		case POL7_INICJUJ_USREDN:	KalibrujCisnienie(0, 0, 0, CZAS_KALIBRACJI, 0xFF);	break;	//inicjalizacja
		case POL7_ZERUJ_LICZNIK:
			sLicznikCzasuKalibracji = 0;
			break;

		case POL7_USREDNIJ_CISN1:
			//uDaneCM4.dane.chOdpowiedzNaPolecenie = KalibrujCisnienie(uDaneCM4.dane.fCisnieBzw[0], uDaneCM4.dane.fCisnieBzw[1], uDaneCM4.dane.fTemper[TEMP_BARO1], sLicznikCzasuKalibracji, 0);
			uDaneCM4.dane.uRozne.U8[0] = KalibrujCisnienie(uDaneCM4.dane.fCisnieBzw[0], uDaneCM4.dane.fCisnieBzw[1], uDaneCM4.dane.fTemper[TEMP_BARO1], sLicznikCzasuKalibracji, 0);
			if (sLicznikCzasuKalibracji <= CZAS_KALIBRACJI)
				uDaneCM4.dane.sPostepProcesu = sLicznikCzasuKalibracji++;
			break;

		case POL7_USREDNIJ_CISN2:
			//uDaneCM4.dane.chOdpowiedzNaPolecenie = KalibrujCisnienie(uDaneCM4.dane.fCisnieBzw[0], uDaneCM4.dane.fCisnieBzw[1], uDaneCM4.dane.fTemper[TEMP_BARO1], sLicznikCzasuKalibracji, 1);
			uDaneCM4.dane.uRozne.U8[0] = KalibrujCisnienie(uDaneCM4.dane.fCisnieBzw[0], uDaneCM4.dane.fCisnieBzw[1], uDaneCM4.dane.fTemper[TEMP_BARO1], sLicznikCzasuKalibracji, 1);
			if (sLicznikCzasuKalibracji <= CZAS_KALIBRACJI)
				uDaneCM4.dane.sPostepProcesu = sLicznikCzasuKalibracji++;
			break;

		case POL7_CZYTAJ_FRAM_U8:
			if (uDaneCM7.dane.cRozmiar > ROZMIAR_ROZNE_CHAR)
				uDaneCM7.dane.cRozmiar = ROZMIAR_ROZNE_CHAR;
			CzytajBuforFRAM(uDaneCM7.dane.sAdres, uDaneCM4.dane.uRozne.U8, (uint16_t)uDaneCM7.dane.cRozmiar);
			uDaneCM4.dane.cRozmiar = uDaneCM7.dane.cRozmiar;	//odeślij skorygowany rozmiar
			break;

		case POL7_CZYTAJ_FRAM_FLOAT:			//odczytaj i wyślij zawartość FRAM spod podanego adresu
			if (uDaneCM7.dane.cRozmiar > ROZMIAR_ROZNE_FLOAT)
				uDaneCM7.dane.cRozmiar = ROZMIAR_ROZNE_FLOAT;
			for (uint16_t n=0; n<uDaneCM7.dane.cRozmiar; n++)
				uDaneCM4.dane.uRozne.f32[n] = CzytajFramFloat(uDaneCM7.dane.sAdres + n*4);
			uDaneCM4.dane.cRozmiar = uDaneCM7.dane.cRozmiar;	//odeślij skorygowany rozmiar
			break;

		case POL7_ZAPISZ_FRAM_U8:	//zapisz dane uint8_t pod podany adres
			if (uDaneCM7.dane.cRozmiar > ROZMIAR_ROZNE_CHAR)
				uDaneCM7.dane.cRozmiar = ROZMIAR_ROZNE_CHAR;
			ZapiszBuforFRAM(uDaneCM7.dane.sAdres, uDaneCM7.dane.uRozne.U8, (uint16_t)uDaneCM7.dane.cRozmiar);
			uDaneCM4.dane.cRozmiar = uDaneCM7.dane.cRozmiar;	//odeślij skorygowany rozmiar
			break;

		case POL7_ZAPISZ_FRAM_FLOAT:			//zapisz przekazane dane typu float do FRAM pod podany adres
			if (uDaneCM7.dane.cRozmiar > ROZMIAR_ROZNE_FLOAT)
				uDaneCM7.dane.cRozmiar = ROZMIAR_ROZNE_FLOAT;
			for (uint16_t n=0; n<uDaneCM7.dane.cRozmiar; n++)
				ZapiszFramFloat(uDaneCM7.dane.sAdres + n*4, uDaneCM7.dane.uRozne.f32[n]);
			uDaneCM4.dane.cRozmiar = uDaneCM7.dane.cRozmiar;	//odeślij skorygowany rozmiar
			break;

		case POL7_KASUJ_DRYFT_ZYRO:
			for (uint16_t n=0; n<3; n++)
			{
				uDaneCM4.dane.fKatZyro1[n] = uDaneCM4.dane.fKatAkcel1[n];
				uDaneCM4.dane.fKatZyro2[n] = uDaneCM4.dane.fKatAkcel2[n];
			}
			break;

		/*case POL7_REKONFIG_SERWA_RC:
			InicjujWejsciaRC();
			InicjujWyjsciaRC();
			break;*/

		case POL7_CZYSC_BLEDY:	uDaneCM4.dane.uRozne.U8[ODPOWIEDZ_U8] = BLAD_OK;	break;	//nadpisz poprzednio zwrócony błąd
		case POL7_ZBIERAJ_EKSTREMA_RC:	RozpocznijZbieranieEkstremowWejscRC();	break;	//rozpoczyna zbieranie ekstremalnych wartości kanałów obu odbiorników RC
		case POL7_ZAPISZ_EKSTREMA_RC:	ZapiszEkstremaWejscRC();	break;	//kończy zbieranie ekstremalnych wartości kanałów obu odbiorników RC i zapisuje wyniki do FRAM

		case POL7_ZAPISZ_KONFIG_PID:
	#ifdef TESTY
			assert(uDaneCM7.dane.cRozmiar == ROZMIAR_REG_PID / sizeof(float));
	#endif
			if (uDaneCM7.dane.cRozmiar > ROZMIAR_ROZNE_FLOAT)
				uDaneCM7.dane.cRozmiar = ROZMIAR_ROZNE_FLOAT;
			uint8_t chIndeksRegulatora = (uint8_t)uDaneCM7.dane.sAdres;
			uint16_t sAdresFram = FA_USER_PID +  chIndeksRegulatora * ROZMIAR_REG_PID;

			//na początek zapisz 10 floatów i  wstaw je do pamięci
			for (uint16_t n=0; n<uDaneCM7.dane.cRozmiar; n++)
				ZapiszFramFloat(sAdresFram + n*4, uDaneCM7.dane.uRozne.f32[n]);

			stKonfigPID[chIndeksRegulatora].fWzmP = uDaneCM7.dane.uRozne.f32[0];
			stKonfigPID[chIndeksRegulatora].fWzmI = uDaneCM7.dane.uRozne.f32[1];
			stKonfigPID[chIndeksRegulatora].fWzmD = uDaneCM7.dane.uRozne.f32[2];
			stKonfigPID[chIndeksRegulatora].fOgrCalki = uDaneCM7.dane.uRozne.f32[3];
			stKonfigPID[chIndeksRegulatora].fMinWyj = uDaneCM7.dane.uRozne.f32[4];
			stKonfigPID[chIndeksRegulatora].fMaxWyj = uDaneCM7.dane.uRozne.f32[5];
			stKonfigPID[chIndeksRegulatora].fSkalaWartZadanej = uDaneCM7.dane.uRozne.f32[6];
			stKonfigPID[chIndeksRegulatora].fPrzesunWartZadanej = uDaneCM7.dane.uRozne.f32[7];
			stKonfigPID[chIndeksRegulatora].fWzmWyprz = uDaneCM7.dane.uRozne.f32[8];
			//ostatni float[9] zawiera flagi i nastawy 3 filtrów
			stKonfigPID[chIndeksRegulatora].cFlagi = uDaneCM7.dane.uRozne.U8[9*4+0];
			stKonfigPID[chIndeksRegulatora].cPodstFiltraD = uDaneCM7.dane.uRozne.U8[9*4+1];
			stKonfigPID[chIndeksRegulatora].cPodstFiltraWZad = uDaneCM7.dane.uRozne.U8[9*4+2];
			stKonfigPID[chIndeksRegulatora].cPodstFiltraWej = uDaneCM7.dane.uRozne.U8[9*4+3];
			break;

		case POL7_ZAPISZ_PWM_NAPEDU:
	#ifdef TESTY
			assert(uDaneCM7.dane.cRozmiar == LICZBA_DANYCH_NAPEDU);	//zapisz 3 słowa 16-bitowe
	#endif
			for (uint16_t n=0; n<uDaneCM7.dane.cRozmiar; n++)
				ZapiszFramU16(FAU_RC_WY_MIN + n*2, uDaneCM7.dane.uRozne.U16[n]);
			sWysterowanieMin = uDaneCM7.dane.uRozne.U16[0];
			sWysterowanieMax = uDaneCM7.dane.uRozne.U16[1];
			sWysterowanieZawisu = uDaneCM7.dane.uRozne.U16[2];
			break;

		case POL7_ZAPISZ_TRYB_REG:	//rodzaj regulacji dla 4 podstawowych parametrów sterowanych z aparatury
	#ifdef TESTY
			assert(uDaneCM7.dane.cRozmiar == LICZBA_REG_PARAM);
	#endif
			ZapiszBuforFRAM(FAU_TRYB_REG, uDaneCM7.dane.uRozne.U8, uDaneCM7.dane.cRozmiar);
			for (uint16_t n=0; n<uDaneCM7.dane.cRozmiar; n++)
				cTrybRegulacji[n] = uDaneCM7.dane.uRozne.U8[n];
			break;

		case POL7_RESETUJ_CM4:
			//uDaneCM4.dane.sAdres = uDaneCM7.dane.sAdres;		//odeślij adres jako potwierdzenie zapisu
			//SCB->AIRCR |= SCB_AIRCR_SYSRESETREQ_Msk;

		   /* __disable_irq();

			// Deinit peryferiów
			HAL_DeInit();

			// Wyłącz przerwania NVIC
			for (int i = 0; i < 8; i++)
				NVIC->ICER[i] = 0xFFFFFFFF;

			// Ustaw MSP na wartość z wektora resetu
			uint32_t reset_sp = *(uint32_t *)FLASH_BANK2_BASE;
			uint32_t reset_pc = *(uint32_t *)(FLASH_BANK2_BASE + 4);

			__set_MSP(reset_sp);

			// Skok do Reset_Handler
			((void (*)(void))reset_pc)();*/
			break;

		case POL7_PRZELADUJ_WSKAZN_LED: 		InicjujKoloryWS281x();	break;

		case POL7_WYSTERUJ_SILNIKI_AD:
			for (uint16_t n=0; n<KANALY_MIKSERA; n++)
			{
				if (uDaneCM7.dane.uRozne.U8[1] & (1 << n))	//czy jest ustawiony bit aktywacji silnika w teście
					cFunkcjaSilnika[n] = FSIL_AN_DRGAN;	//podmień źródło danych do silnika. Po zakonczeniu będzie potrzebne przeładowanie konfiguracji
				else
					cFunkcjaSilnika[n] = FSIL_ZATRZYMANY;	//pozostałe silniki zatrzymaj
			}
			break;

		case POL7_PRZYWROC_NAPED:		//przywróć funkcję napędu dla silników po analizie FFT rezonansu ramy
			for (uint16_t n=0; n<KANALY_MIKSERA; n++)
				cFunkcjaSilnika[n] = FSIL_NAPED;
			break;

		case POL7_PRZELADUJ_PID:	InicjujPID();	break;	//ponownie załaduj konfigurację PID aby odświeżyć ustawienia po zmianie konfiguracji
		case POL7_CZYTAJ_KALIBR_TEMP:
			sTS_CAL1 = uDaneCM7.dane.uRozne.U16[0];
			sTS_CAL2 = uDaneCM7.dane.uRozne.U16[1];	//współczynniki kalibracji czujnika temperatury odczytywane w CM7 i przekazywane poleceniem
			break;

		case POL7_CZYTAJ_CZAS_PETLI_GL:	//odczytuje czas wykonania fragmentów petli głównej
			assert(ROZMIAR_ROZNE_SHORT >= LICZBA_ODCINKOW_CZASU+1);	//zatrzymaj gdyby bufor Różne danych był mniejszy niż liczba odcinków
			for (uint8_t n=0; n<LICZBA_ODCINKOW_CZASU+1; n++)
				//uDaneCM4.dane.uRozne.U16[n] = nCzasOdcinka[n];	//przepisz czasy wprost
				uDaneCM4.dane.uRozne.U16[n] = (nCzasOdcinka[n] + uDaneCM4.dane.uRozne.U16[n] * 3) / 4;	//przepisz czasy z filtrowaniem
			break;

		case POL7_REKONFIG_WEJSCIA_RC:	InicjujWejsciaRC();	break;	//wykonuje ponowną konfigurację wejść RC po zmianie konfiguracji we FRAM
		case POL7_REKONFIG_WYJSCIA_RC:	InicjujWyjsciaRC();	break;	//wykonuje ponowną konfigurację wyjść RC po zmianie konfiguracji we FRAM

		case POL7_URUCHOM_INDENT_SILN:
			sWysterowanieIdentSiln = uDaneCM4.dane.uRozne.U16[0];	//wysterowanie silników podczas procesu identyfikacji
			for (uint16_t n=0; n<KANALY_MIKSERA; n++)
			{
				if (uDaneCM7.dane.sAdres & (1 << n))	//czy jest ustawiony bit aktywacji silnika
					cFunkcjaSilnika[n] = FSIL_IDENTYFIKACJA;	//podmień źródło danych do silnika. Po zakonńczeniu będzie potrzebne przeładowanie konfiguracji
				else
					cFunkcjaSilnika[n] = FSIL_ZATRZYMANY;	//pozostałe silniki zatrzymaj
			}
			break;

		}	//switch
	}
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja dzieli złożone operacje I2C na sekwencje dające sie realizować w tle, dzięki czemu nie blokuje czasu procesora oczekując na zakończenie
// Parametry: brak
// Zwraca: kod błędu operacji I2C
////////////////////////////////////////////////////////////////////////////////
uint8_t RozdzielniaOperacjiI2C(void)
{
	uint8_t cBłąd = BLAD_OK;

	//operacje na zewnętrznej magistrali I2C3
	switch(cEtapOperacjiI2C)
	{
	case 0:
	case 2: cBłąd = ObslugaMS4525();		break;
	case 1:
	case 3:	cBłąd = ObslugaHMC5883();		break;
	default: break;
	}

	//operacje na wewnętrznej magistrali I2C4
	switch(cEtapOperacjiI2C)
	{
	case 0:
	case 2:	cBłąd = ObslugaIIS2MDC();		break;
	case 1:
	//case 3:	cBłąd = ObslugaMMC3416x();		break;	//na egzemplarzu 2 jest problem blokowania się przerwania obsługi tego magnetometru
	default: break;
	}

	cEtapOperacjiI2C++;
	cEtapOperacjiI2C &= 0x03;
	return cBłąd;
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
		if (cCzujnikZapisywanyNaI2CInt == MAG_IIS_STATUS)	//po zapisie wykonaj operację odczytu
			MagIIS_CzytajStatus();

		if (cCzujnikZapisywanyNaI2CInt == MAG_IIS)	//po zapisie wykonaj operację odczytu
			MagIIS_CzytajDane();

		if (cCzujnikZapisywanyNaI2CInt == MAG_MMC_STATUS)	//po zapisie wykonaj operację odczytu
			MagMMC_CzytajStatus();

		if (cCzujnikZapisywanyNaI2CInt == MAG_MMC)	//po zapisie wykonaj operację odczytu
			MagMMC_CzytajDane();


		cCzujnikZapisywanyNaI2CInt = 0;
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
		if (cCzujnikOdczytywanyNaI2CExt == MAG_HMC)	//magnetometr HMC5883
			cNoweDaneI2C |= MAG_HMC;
		else
		if (cCzujnikOdczytywanyNaI2CExt == CISN_ROZN_MS2545)	//ciśnienie różnicowe czujnika MS2545DO
			cNoweDaneI2C |= CISN_ROZN_MS2545;
		else
		if (cCzujnikOdczytywanyNaI2CExt == CISN_TEMP_MS2545)	//ciśnienie różnicowe i temperatura czujnika MS2545DO
			cNoweDaneI2C |= CISN_TEMP_MS2545;

		cCzujnikOdczytywanyNaI2CExt = 0;
	}

	if (hi2c->Instance == I2C4)		//magistrala I2C modułów wewnętrznych
	{
		if (cCzujnikOdczytywanyNaI2CInt == MAG_IIS)
			cNoweDaneI2C |= MAG_IIS;
		else
		if (cCzujnikOdczytywanyNaI2CInt == MAG_MMC)
			cNoweDaneI2C |= MAG_MMC;

		cCzujnikOdczytywanyNaI2CInt = 0;
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
	extern uint8_t cDaneMagHMC[6];
	extern uint8_t cDaneMagMMC[6];
	extern uint8_t cDaneMagIIS[6];
	extern uint8_t cDaneMS4525[5];
	extern int16_t sPomiarMMCH[3], sPomiarMMCL[3];	//wyniki pomiarów dla dodatniego i ujemnego namagnesowania czujnika
	extern uint8_t cRodzajPomiaruMMC;
	extern float fPrzesMagn1[3], fSkaloMagn1[3];
	extern float fPrzesMagn2[3], fSkaloMagn2[3];
	extern float fPrzesMagn3[3], fSkaloMagn3[3];
	uint8_t cBłąd = BLAD_OK;
	int16_t sZeZnakiem;	//zmiena robocza do konwersji dnych 8-bitowych bez znaku na liczbę 16-bitową ze znakiem
	float fZeZnakiem;
	const int8_t cZnakIIS[3] = {1, -1, -1};	//magnetometr 1: odwrotnie jest oś Z, ale żeby ją odwrócić, trzeba zmienić też znak w osi X lub Y. Zmieniam w Y: OK
	//const int8_t cZnakMMC[3] = {-1, 1, -1};	//magnetometr 2: odwrotnie jest oś X, ale żeby ją odwrócić, trzeba zmienić też znak w osi Y lub Z. Zmieniam w Z - zle
	const int8_t cZnakMMC[3] = {-1, -1, 1};		//magnetometr 2: odwrotnie jest oś X, ale żeby ją odwrócić, trzeba zmienić też znak w osi Y lub Z. Zmieniam w Z
	const int8_t cZnakHMC[3] = {1, -1, 1};	//magnetometr 3: jest OK

	if (*chCzujniki & MAG_HMC)
	{
		//Uwaga! Czujnik HMS5883L ma osie w nietypowej kolejności XZY, więc konwersję trzeba wykonać ręcznie poza pętlą
		sZeZnakiem = ((int16_t)cDaneMagHMC[0] * 0x100 + cDaneMagHMC[1]) * cZnakHMC[0];	//oś X
		if ((uDaneCM7.dane.cWykonajPolecenie == POL7_KAL_ZERO_MAGN3) ||  (uDaneCM7.dane.cWykonajPolecenie == POL7_ZERUJ_EKSTREMA))
			uDaneCM4.dane.fMagne3[0] = (float)sZeZnakiem * CZULOSC_HMC5883;			//dane surowe podczas kalibracji magnetometru wyrażone w Teslach
		else
			uDaneCM4.dane.fMagne3[0] = ((float)sZeZnakiem * CZULOSC_HMC5883 - fPrzesMagn3[0]) * fSkaloMagn3[0];	//dane skalibrowane

		sZeZnakiem = ((int16_t)cDaneMagHMC[4] * 0x100 + cDaneMagHMC[5]) * cZnakHMC[1];	//oś Y
		if ((uDaneCM7.dane.cWykonajPolecenie == POL7_KAL_ZERO_MAGN3) ||  (uDaneCM7.dane.cWykonajPolecenie == POL7_ZERUJ_EKSTREMA))
			uDaneCM4.dane.fMagne3[1] = (float)sZeZnakiem * CZULOSC_HMC5883;			//dane surowe podczas kalibracji magnetometru
		else
			uDaneCM4.dane.fMagne3[1] = ((float)sZeZnakiem * CZULOSC_HMC5883 - fPrzesMagn3[1]) * fSkaloMagn3[1];	//dane skalibrowane

		sZeZnakiem = ((int16_t)cDaneMagHMC[2] * 0x100 + cDaneMagHMC[3]) * cZnakHMC[2];	//oś Z
		if ((uDaneCM7.dane.cWykonajPolecenie == POL7_KAL_ZERO_MAGN3) ||  (uDaneCM7.dane.cWykonajPolecenie == POL7_ZERUJ_EKSTREMA))
			uDaneCM4.dane.fMagne3[2] = (float)sZeZnakiem * CZULOSC_HMC5883;			//dane surowe podczas kalibracji magnetometru
		else
			uDaneCM4.dane.fMagne3[2] = ((float)sZeZnakiem * CZULOSC_HMC5883 - fPrzesMagn3[2]) * fSkaloMagn3[2];	//dane skalibrowane
		*chCzujniki &= ~MAG_HMC;	//dane obsłużone
		uDaneCM4.dane.cNowyPomiar |= NP_MAG3;	//jest nowy pomiar
	}

	if (*chCzujniki & CISN_ROZN_MS2545)
	{
		uDaneCM4.dane.fCisnRozn[1] = CisnienieMS2545(cDaneMS4525);
		uDaneCM4.dane.fPredkosc[1] = PredkoscRurkiPrantla(uDaneCM4.dane.fCisnRozn[1], 101315.f);	//dla ciśnienia standardowego. Docelowo zamienić na cisnienie zmierzone
		*chCzujniki &= ~CISN_ROZN_MS2545;	//dane obsłużone
		uDaneCM4.dane.cNowyPomiar |= NP_EXT_IAS;	//jest nowy pomiar
	}

	if (*chCzujniki & CISN_TEMP_MS2545)
	{
		uDaneCM4.dane.fTemper[6] = TemperaturaMS2545(cDaneMS4525);
		uDaneCM4.dane.fCisnRozn[1] = (15 * uDaneCM4.dane.fCisnRozn[1] + CisnienieMS2545(cDaneMS4525)) / 16;
		uDaneCM4.dane.fPredkosc[1] = PredkoscRurkiPrantla(uDaneCM4.dane.fCisnRozn[1], 101315.f);	//dla ciśnienia standardowego. Docelowo zamienić na cisnienie zmierzone
		*chCzujniki &= ~CISN_TEMP_MS2545;	//dane obsłużone
		uDaneCM4.dane.cNowyPomiar |= NP_EXT_IAS;	//jest nowy pomiar
	}

	if (*chCzujniki & MAG_IIS)
	{
		for (uint8_t n=0; n<3; n++)
		{
			sZeZnakiem = ((int16_t)cDaneMagIIS[2*n+1] * 0x100 + cDaneMagIIS[2*n]) * cZnakIIS[n];
			if ((uDaneCM7.dane.cWykonajPolecenie == POL7_KAL_ZERO_MAGN1) || (uDaneCM7.dane.cWykonajPolecenie == POL7_ZERUJ_EKSTREMA))
				uDaneCM4.dane.fMagne1[n] = (float)sZeZnakiem * CZULOSC_IIS2MDC;			//dane surowe podczas kalibracji magnetometru
			else
				uDaneCM4.dane.fMagne1[n] = ((float)sZeZnakiem * CZULOSC_IIS2MDC - fPrzesMagn1[n]) * fSkaloMagn1[n];	//dane skalibrowane
				//uDaneCM4.dane.fMagne1[n] = (uDaneCM4.dane.fMagne1[n] + ((float)sZeZnakiem * CZULOSC_IIS2MDC - fPrzesMagn1[n]) * fSkaloMagn1[n]) / 2;	//filtruj pomiar bo jest mocno zaszumiony a jest wystarczajaco szybki żeby filtracja nie przesuwała istotnie fazy
		}
		*chCzujniki &= ~MAG_IIS;	//dane obsłużone
		uDaneCM4.dane.cNowyPomiar |= NP_MAG1;	//jest nowy pomiar
	}

	if (*chCzujniki & MAG_MMC)
	{
		for (uint8_t n=0; n<3; n++)
		{
			if (cRodzajPomiaruMMC < SPMMC3416_REFIL_RESET)	//poniżej SPMMC3416_REFIL_RESET będzie obsługa dla SET, powyżej dla RESET
				sPomiarMMCH[n] = (((int16_t)cDaneMagMMC[2*n+1] * 0x100 + cDaneMagMMC[2*n]) - 32768) * cZnakMMC[n];	//czujnik zwraca liczby unsigned z 32768 dla zera
			else
				sPomiarMMCL[n] = (((int16_t)cDaneMagMMC[2*n+1] * 0x100 + cDaneMagMMC[2*n]) - 32768) * cZnakMMC[n];

			//dla zmiany przemagnesowania czyli świeżych pomiarów H+ i H- policz nateżenie pola ziemi bezpośrednio odejmując je od siebie
			if (cRodzajPomiaruMMC != cPoprzedniRodzajPomiaru)
			{
				float fBiezacePoleCzujnika;

				fZeZnakiem = (float)(sPomiarMMCH[n] - sPomiarMMCL[n]) / 2;
				if (cRodzajPomiaruMMC < SPMMC3416_REFIL_RESET)
					fBiezacePoleCzujnika = fZeZnakiem - sPomiarMMCH[n];	//licz pole namagnesowania czujnika
				else
					fBiezacePoleCzujnika = -fZeZnakiem - sPomiarMMCL[n];
				fPoleCzujnkaMMC[n] = (7 * fPoleCzujnkaMMC[n] + fBiezacePoleCzujnika) / 8;	//filtruj obliczenia pola magnesowania czujnika
			}
			else	//dla ciagłego pomiaru z jednym typem magnesowania, natężenie pola ziemi licz odejmując obliczone wcześniej pole czujnika
			{
				if (cRodzajPomiaruMMC < SPMMC3416_REFIL_RESET)
					fZeZnakiem = sPomiarMMCH[n] + fPoleCzujnkaMMC[n];
				else
					fZeZnakiem = (sPomiarMMCL[n] + fPoleCzujnkaMMC[n]) * -1;
			}

			if ((uDaneCM7.dane.cWykonajPolecenie == POL7_KAL_ZERO_MAGN2) || (uDaneCM7.dane.cWykonajPolecenie == POL7_ZERUJ_EKSTREMA))
				uDaneCM4.dane.fMagne2[n] = fZeZnakiem * CZULOSC_MMC34160;	//dane surowe podczas kalibracji magnetometru
			else
				uDaneCM4.dane.fMagne2[n] = (fZeZnakiem * CZULOSC_MMC34160 - fPrzesMagn2[n]) * fSkaloMagn2[n];	//dane skalibrowane;
		}
		cPoprzedniRodzajPomiaru = cRodzajPomiaruMMC;
		*chCzujniki &= ~MAG_MMC;	//dane obsłużone
		uDaneCM4.dane.cNowyPomiar |= NP_MAG2;	//jest nowy pomiar
	}
	return cBłąd;
}


