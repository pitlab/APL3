//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa modułu I2P (IiP - Inercyjny i Pneumatyczny)
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "modul_IiP.h"
#include "main.h"
#include "moduly_wew.h"
#include "BMP581.h"
#include "MS5611.h"
#include "ICM42688.h"
#include "LSM6DSV.h"
#include "ND130.h"
#include "fram.h"
#include "wymiana_CM4.h"
#include "konfig_fram.h"

extern SPI_HandleTypeDef hspi2;
extern uint8_t chStanIOwy, chStanIOwe;	//stan wejść IO modułów wewnetrznych
volatile uint8_t chOdczytywanyMagnetometr;	//zmienna wskazuje który magnetometr jest odczytywany: MAG_MMC lub MAG_IIS
uint16_t sLicznikCzasuKalibracjiZyro;
extern volatile unia_wymianyCM4_t uDaneCM4;
float fOffsetZyro1[3], fOffsetZyro2[3];
double dSumaZyro1[3], dSumaZyro2[3];
float fSumaCisnRozn[2];	//do kalibracji czujników ciśnienia różnicowego
WspRownProstej3_t stWspKalOffsetuZyro1;		//współczynniki równania prostych do estymacji offsetu
WspRownProstej3_t stWspKalOffsetuZyro2;		//współczynniki równania prostych do estymacji offsetu
WspRownProstej1_t stWspKalOffsetuCzujnRozn1;
WspRownProstej1_t stWspKalOffsetuCzujnRozn2;


////////////////////////////////////////////////////////////////////////////////
// wykonuje czynności pomiarowe dla ukłądów znajdujących się na module
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujModulI2P(void)
{
	uint8_t chErr = ERR_OK;
	float fTemp1[3], fTemp2[3];	//temperatury na zimno, pokojowo i gorąco dla obu żyroskopów
	float fOffsetZyro1Z, fOffsetZyro2Z;		//offsety na zimno
	float fOffsetZyro1P, fOffsetZyro2P;		//offsety w temp pokojowej
	float fOffsetZyro1G, fOffsetZyro2G;		//offsety na gorąco
	float fOffsetCisnRoz1Z, fOffsetCisnRoz1P, fOffsetCisnRoz1G;	//offsety czujnika ciśnienia różnicowego 1
	float fOffsetCisnRoz2Z, fOffsetCisnRoz2P, fOffsetCisnRoz2G;	//offsety czujnika ciśnienia różnicowego 2

	//odczytaj tempertury kalibracji: 0=zimna, 1=pokojowa, 2=gorąca
	for (uint16_t n=0; n<3; n++)
	{
		fTemp1[n] = FramDataReadFloat(FAH_ZYRO1_TEMP_ZIM+(4*n));	//temepratury żyroskopu 1
		fTemp2[n] = FramDataReadFloat(FAH_ZYRO2_TEMP_ZIM+(4*n));	//temepratury żyroskopu 2
	}
	stWspKalOffsetuZyro1.fTempPok = fTemp1[POK];
	stWspKalOffsetuZyro2.fTempPok = fTemp2[POK];
	stWspKalOffsetuCzujnRozn1.fTempPok = fTemp1[POK];

	//odczytaj kalibrację żyroskopów i licz charakterystykę
	for (uint16_t n=0; n<3; n++)
	{
		chErr += FramDataReadFloatValid(FAH_ZYRO1_X_PRZ_ZIM+(4*n), &fOffsetZyro1Z, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset żyroskopu1 na zimno
		chErr += FramDataReadFloatValid(FAH_ZYRO2_X_PRZ_ZIM+(4*n), &fOffsetZyro2Z, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset żyroskopu2 na zimno

		chErr += FramDataReadFloatValid(FAH_ZYRO1_X_PRZ_POK+(4*n), &fOffsetZyro1P, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset żyroskopu1 w temp pokojowej
		chErr += FramDataReadFloatValid(FAH_ZYRO2_X_PRZ_POK+(4*n), &fOffsetZyro2P, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset żyroskopu1 w temp pokojowej

		chErr += FramDataReadFloatValid(FAH_ZYRO1_X_PRZ_GOR+(4*n), &fOffsetZyro1G, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset żyroskopu1 na gorąco
		chErr += FramDataReadFloatValid(FAH_ZYRO2_X_PRZ_GOR+(4*n), &fOffsetZyro2G, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset żyroskopu2 na gorąco

		ObliczRownanieFunkcjiTemperatury(fOffsetZyro1Z, fOffsetZyro1P, fTemp1[ZIM], fTemp1[POK], &stWspKalOffsetuZyro1.fAzim[n], &stWspKalOffsetuZyro1.fBzim[n]);	//Żyro 1 na zimno
		ObliczRownanieFunkcjiTemperatury(fOffsetZyro1P, fOffsetZyro1G, fTemp1[POK], fTemp1[GOR], &stWspKalOffsetuZyro1.fAgor[n], &stWspKalOffsetuZyro1.fBgor[n]);	//Żyro 1 na gorąco
		ObliczRownanieFunkcjiTemperatury(fOffsetZyro2Z, fOffsetZyro2P, fTemp2[ZIM], fTemp2[POK], &stWspKalOffsetuZyro2.fAzim[n], &stWspKalOffsetuZyro2.fBzim[n]);	//Żyro 2 na zimno
		ObliczRownanieFunkcjiTemperatury(fOffsetZyro2P, fOffsetZyro2G, fTemp2[POK], fTemp2[GOR], &stWspKalOffsetuZyro2.fAgor[n], &stWspKalOffsetuZyro2.fBgor[n]);	//Żyro 2 na gorąco
	}

	//odczytaj kalibrację czujników ciśnienia różnicowego i licz charakterystykę. Używana jest temperatura żyroskopu 1
	chErr += FramDataReadFloatValid(FAH_CISN_ROZN1_ZIM, &fOffsetCisnRoz1Z, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 1 na zimno
	chErr += FramDataReadFloatValid(FAH_CISN_ROZN1_POK, &fOffsetCisnRoz1P, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 1 w temp pokojowej
	chErr += FramDataReadFloatValid(FAH_CISN_ROZN1_GOR, &fOffsetCisnRoz1G, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 1 na gorąco
	ObliczRownanieFunkcjiTemperatury(fOffsetCisnRoz1Z, fOffsetCisnRoz1P, fTemp1[ZIM], fTemp1[POK], &stWspKalOffsetuCzujnRozn1.fAzim, &stWspKalOffsetuCzujnRozn1.fBzim);	//czujnik cisnienia różnicowego 1 na zimno
	ObliczRownanieFunkcjiTemperatury(fOffsetCisnRoz1P, fOffsetCisnRoz1G, fTemp1[POK], fTemp1[GOR], &stWspKalOffsetuCzujnRozn1.fAgor, &stWspKalOffsetuCzujnRozn1.fBgor);	//czujnik cisnienia różnicowego 1 na gorąco

	chErr += FramDataReadFloatValid(FAH_CISN_ROZN2_ZIM, &fOffsetCisnRoz2Z, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 2 na zimno
	chErr += FramDataReadFloatValid(FAH_CISN_ROZN2_POK, &fOffsetCisnRoz2P, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 2 w temp pokojowej
	chErr += FramDataReadFloatValid(FAH_CISN_ROZN2_GOR, &fOffsetCisnRoz2G, MIN_OFFSET, MAX_OFFSET, DEF_OFFSET, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 2 na gorąco
	ObliczRownanieFunkcjiTemperatury(fOffsetCisnRoz2Z, fOffsetCisnRoz2P, fTemp1[ZIM], fTemp1[POK], &stWspKalOffsetuCzujnRozn1.fAzim, &stWspKalOffsetuCzujnRozn1.fBzim);	//czujnik cisnienia różnicowego 2 na zimno
	ObliczRownanieFunkcjiTemperatury(fOffsetCisnRoz2P, fOffsetCisnRoz2G, fTemp1[POK], fTemp1[GOR], &stWspKalOffsetuCzujnRozn1.fAgor, &stWspKalOffsetuCzujnRozn1.fBgor);	//czujnik cisnienia różnicowego 2 na gorąco
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// wykonuje czynności pomiarowe dla układów znajdujących się na module
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaModuluI2P(uint8_t gniazdo)
{
	uint8_t chErr;
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 40MHz a układy mogą pracować z prędkością max 10MHz, przy każdym dostępie przestaw dzielnik zegara na 4
	nZastanaKonfiguracja_SPI_CFG1 = hspi2.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi2.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;		//maska preskalera
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_4;
	//hspi2.Instance->CFG2 |= SPI_POLARITY_HIGH | SPI_PHASE_2EDGE;	//testowo ustaw mode 3

	//ustaw adres A2 = 0 zrobiony z linii Ix2 modułu
	switch (gniazdo)
	{
	case ADR_MOD1: 	chStanIOwy &= ~MIO12;	break;
	case ADR_MOD2:	chStanIOwy &= ~MIO22;	break;
	case ADR_MOD3:	chStanIOwy &= ~MIO32;	break;
	case ADR_MOD4:	chStanIOwy &= ~MIO42;	break;
	}

	chErr = WyslijDaneExpandera(chStanIOwy);
	chErr |= UstawDekoderModulow(gniazdo);				//ustaw adres dekodera modułów, ponieważ użycie expandera przestawia adres
	UstawAdresNaModule(ADR_MIIP_MS5611);				//ustaw adres A0..1
	chErr |= ObslugaMS5611();

	UstawAdresNaModule(ADR_MIIP_BMP581);				//ustaw adres A0..1
	chErr |= ObslugaBMP581();

	UstawAdresNaModule(ADR_MIIP_ICM42688);				//ustaw adres A0..1
	chErr |= ObslugaICM42688();

	UstawAdresNaModule(ADR_MIIP_LSM6DSV);				//ustaw adres A0..1
	chErr |= ObslugaLSM6DSV();

	if (uDaneCM4.dane.nZainicjowano & (INIT_TRWA_KALZ_ZYRO | INIT_TRWA_KALP_ZYRO | INIT_TRWA_KALG_ZYRO))
		KalibrujZyroskopy();

	//ustaw adres A2 = 1 zrobiony z linii Ix2 modułu
	switch (gniazdo)
	{
	case ADR_MOD1: 	chStanIOwy |= MIO12;	break;
	case ADR_MOD2:	chStanIOwy |= MIO22;	break;
	case ADR_MOD3:	chStanIOwy |= MIO32;	break;
	case ADR_MOD4:	chStanIOwy |= MIO42;	break;
	}
	chErr |= WyslijDaneExpandera(chStanIOwy);
	chErr |= UstawDekoderModulow(gniazdo);				//ustaw adres dekodera modułów, ponieważ użycie expandera przestawia adres
	UstawAdresNaModule(ADR_MIIP_GRZALKA);				//ustaw adres A0..1

	//grzałkę włącza się przez CS=0
	//HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0


	// Układ ND130 pracujacy na magistrali SPI ma okres zegara 6us co odpowiada częstotliwości 166kHz
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_256;	//przestaw zegar na 40MHz / 256 = 156kHz
	UstawAdresNaModule(ADR_MIIP_ND130);				//ustaw adres A0..1
	chErr |= ObslugaND130();
	if (chErr == ERR_TIMEOUT)
	{
		UstawAdresNaModule(ADR_MIIP_RES_ND130);	//wejście resetujące ND130
		HAL_Delay(1);	//resetuj układ
	}

	//hspi2.Instance->CFG2 &= ~(SPI_POLARITY_HIGH | SPI_PHASE_2EDGE);
	hspi2.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróc poprzednie nastawy
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza równanie prostej linearyzującej zmianę offsetu żyroskopu w funkcji tempertury
// Parametry:
// [we] fOffset1, fOffset2 - wartości offsetu żyroskopów
// [we] fTemp1, fTemp1 - temperatury dla offsetów [K]
// [wy] *fA, *fB - wspóczynniki równania prostej offsetu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObliczRownanieFunkcjiTemperatury(float fOffset1, float fOffset2, float fTemp1, float fTemp2, float *fA, float *fB)
{
	//Uwaga! We wzorze liczenia dryftu temperatura jest w mianowniku, więc skala nie może być w stopniach Celsjusza gdyż wystąpi dzielenie przez zero,
	//a wcześniej dzielenie przez bardzo małą wartość, co powoduje że charakterystyka strzela do nieskończonosci.
	//Temperaturę trzeba przeliczyć na Kelviny
	//fTemp1 += KELVIN;
	//fTemp2 += KELVIN;

	//układ równań prostych w postaci kierunkowej:
	//offset1 = A * temperatura1 + B
	//offset2 = A * temperatura2 + B
	//po odjeciu stronami: offset1 - offset2 =  temperatura1 * A - temperatura2 * A
	//A = (offset1 - offset2) / (temperatura1 - temperatura2)
	*fA = (fOffset1 - fOffset2) / (fTemp1 - fTemp2);

	//B wyznaczamy z pierwszego równania: B = offset1 - A * temperatura1
	*fB = fOffset1 - (*fA * fTemp1);
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza wartość offsetu żyroskopu dla danej temperatury na podstawie równania prostej linearyzującej zmianę offsetu żyroskopu w funkcji temperatury
// Parametry:
// [we] stWsp struktura z wartościami współczynników równania prostych
// [we] fTemp - temperatura żyroskopu [K]
// [wy] *fOffset[3] - obliczone wartości offsetu dla wszystkich osi
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObliczOffsetTemperaturowy(WspRownProstej3_t stWsp, float fTemp, float *fOffset)
{
	if (fTemp > stWsp.fTempPok)
	{
		for (uint8_t n=0; n<3; n++)
			//*(fOffset + n) = stWsp.fAgor[n] * (fTemp + KELVIN) + stWsp.fBgor[n];
			*(fOffset + n) = stWsp.fAgor[n] * fTemp + stWsp.fBgor[n];
	}
	else
	{
		for (uint8_t n=0; n<3; n++)
			//*(fOffset + n) = stWsp.fAzim[n] * (fTemp + KELVIN) + stWsp.fBzim[n];
			*(fOffset + n) = stWsp.fAzim[n] * fTemp + stWsp.fBzim[n];
	}
}



////////////////////////////////////////////////////////////////////////////////
// Liczy wysokość barometryczną na podstawie ciśnienia i temperatury
// Parametry:
//  fP - ciśnienie na mierzonej wysokości [Pa]	jednostka ciśnienia jest dowolna, byle taka sama dla obu ciśnień
//  fP0 - ciśnienie na poziome odniesienia [Pa]
//  fTemp - temperatura [K)
// Zwraca: obliczoną wysokość
////////////////////////////////////////////////////////////////////////////////
float WysokoscBarometryczna(float fP, float fP0, float fTemp)
{
	//funkcja bazuje na wzorze barometrycznym: https://pl.wikipedia.org/wiki/Wz%C3%B3r_barometryczny
	//P = P0 * e^(-(u*g*h)/(R*T))	/:P0
	//P/P0 = e^(-(u*g*h)/(R*T))		/ln
	//ln(P/P0) = -(u*g*h)/(R*T)		/*R*T
	//ln(P/P0) * R*T = -u*g*h		/:-u*g
	//h = ln(P/P0) * R*T / (-u*g)

	return logf(fP/fP0) * STALA_GAZOWA_R * fTemp / (-1 * MASA_MOLOWA_POWIETRZA * PRZYSPIESZENIE_ZIEMSKIE);
}



////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna kalibrację żyroskopów
// Parametry: chRodzajKalib - rodzaj kalibracji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RozpocznijKalibracje(uint8_t chRodzajKalib)
{
	float fTemperatura;

	//sprawdź czy kalibracja już trwa jeżeli tak, to nie zaczynaj kolejnej przed zakończeniem obecnej
	if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KALP_ZYRO)
		return ERR_OK;

	fTemperatura = (uDaneCM4.dane.fTemper[TEMP_IMU1] + uDaneCM4.dane.fTemper[TEMP_IMU2]) / 2;

	switch (chRodzajKalib)
	{
	case POL_KALIBRUJ_ZYRO_ZIM:
		if (fTemperatura < TEMP_KAL_ZIMNO - TEMP_KAL_ODCHYLKA)
		{
			uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZA_ZIMNO;
			return ERR_ZA_ZIMNO;
		}
		else
		if (fTemperatura > TEMP_KAL_ZIMNO + TEMP_KAL_ODCHYLKA)
		{
			uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZA_CIEPLO;
			return ERR_ZA_CIEPLO;
		}
		else
		{
			uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KALZ_ZYRO ;	//uruchom kalibrację żyroskopów na zimno w +10°C
			uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_OK;
		}
		break;

	case POL_KALIBRUJ_ZYRO_POK:
		if (fTemperatura < TEMP_KAL_POKOJ - TEMP_KAL_ODCHYLKA)
		{
			uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZA_ZIMNO;
			return ERR_ZA_ZIMNO;
		}
		else
		if (fTemperatura > TEMP_KAL_POKOJ + TEMP_KAL_ODCHYLKA)
		{
			uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZA_CIEPLO;
			return ERR_ZA_CIEPLO;
		}
		else
		{
			uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KALP_ZYRO;		//uruchom kalibrację żyroskopów w temperaturze pokojowej 25°C
			uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_OK;
		}
		break;

	case POL_KALIBRUJ_ZYRO_GOR:
		if (fTemperatura < TEMP_KAL_GORAC - TEMP_KAL_ODCHYLKA)
		{
			uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZA_ZIMNO;
			return ERR_ZA_ZIMNO;
		}
		else
		if (fTemperatura > TEMP_KAL_GORAC + TEMP_KAL_ODCHYLKA)
		{
			uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZA_CIEPLO;
			return ERR_ZA_CIEPLO;
		}
		else
		{
			uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KALG_ZYRO;		//uruchom kalibrację żyroskopów na gorąco 40°C
			uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_OK;
		}
		break;

	default: return ERR_ZLE_POLECENIE;
	}

	for (uint8_t n=0; n<3; n++)
	{
		dSumaZyro1[n] = 0.0;
		dSumaZyro2[n] = 0.0;
	}
	fSumaCisnRozn[0] = 0.0;
	uDaneCM4.dane.sPostepProcesu = CZAS_KALIBRACJI_ZYROSKOPU;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje kalibrację żyroskopów i zapisuje wynik we FRAM
// Parametry: sCzasKalibracji - czas liczny w kwantach obiegu pętli głównej
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibrujZyroskopy(void)
{
	if (uDaneCM4.dane.sPostepProcesu)
		uDaneCM4.dane.sPostepProcesu--;

	if (uDaneCM4.dane.nZainicjowano & (INIT_TRWA_KALZ_ZYRO | INIT_TRWA_KALP_ZYRO | INIT_TRWA_KALG_ZYRO))	//jeżeli trwa którakolwiek z kalibracji
	{
		for (uint8_t n=0; n<3; n++)
		{
			dSumaZyro1[n] += uDaneCM4.dane.fZyroSur1[n];
			dSumaZyro2[n] += uDaneCM4.dane.fZyroSur2[n];
		}
		fSumaCisnRozn[0] += uDaneCM4.dane.fCisnRozn[0];

		if (uDaneCM4.dane.sPostepProcesu == 0)
		{
			for (uint8_t n=0; n<3; n++)
			{
				fOffsetZyro1[n] = dSumaZyro1[n] / CZAS_KALIBRACJI_ZYROSKOPU;
				fOffsetZyro2[n] = dSumaZyro2[n] / CZAS_KALIBRACJI_ZYROSKOPU;

				if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KALZ_ZYRO)
				{
					FramDataWriteFloat(FAH_ZYRO1_X_PRZ_ZIM+(4*n), fOffsetZyro1[n]);	//zapisz do FRAM jako liczbę float zajmującą 4 bajty
					FramDataWriteFloat(FAH_ZYRO2_X_PRZ_ZIM+(4*n), fOffsetZyro2[n]);
				}
				else
				if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KALP_ZYRO)
				{
					FramDataWriteFloat(FAH_ZYRO1_X_PRZ_POK+(4*n), fOffsetZyro1[n]);	//zapisz do FRAM jako liczbę float zajmującą 4 bajty
					FramDataWriteFloat(FAH_ZYRO2_X_PRZ_POK+(4*n), fOffsetZyro2[n]);
				}
				else
				if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KALG_ZYRO)
				{
					FramDataWriteFloat(FAH_ZYRO1_X_PRZ_GOR+(4*n), fOffsetZyro1[n]);	//zapisz do FRAM jako liczbę float zajmującą 4 bajty
					FramDataWriteFloat(FAH_ZYRO2_X_PRZ_GOR+(4*n), fOffsetZyro2[n]);
				}
			}

			//zapisz temperatury i dane czujnika różnicowego
			if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KALZ_ZYRO)
			{
				FramDataWriteFloat(FAH_ZYRO1_TEMP_ZIM, uDaneCM4.dane.fTemper[TEMP_IMU1]);
				FramDataWriteFloat(FAH_ZYRO2_TEMP_ZIM, uDaneCM4.dane.fTemper[TEMP_IMU2]);
				FramDataWriteFloat(FAH_CISN_ROZN1_ZIM, fSumaCisnRozn[0] / CZAS_KALIBRACJI_ZYROSKOPU);		//4F korekcja czujnika ciśnienia różnicowego 1 na zimno
			}
			else
			if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KALP_ZYRO)
			{
				FramDataWriteFloat(FAH_ZYRO1_TEMP_POK, uDaneCM4.dane.fTemper[TEMP_IMU1]);
				FramDataWriteFloat(FAH_ZYRO2_TEMP_POK, uDaneCM4.dane.fTemper[TEMP_IMU2]);
				FramDataWriteFloat(FAH_CISN_ROZN1_POK, fSumaCisnRozn[0] / CZAS_KALIBRACJI_ZYROSKOPU);	//4F korekcja czujnika ciśnienia różnicowego 1 w 25°C
			}
			else
			if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KALG_ZYRO)
			{
				FramDataWriteFloat(FAH_ZYRO1_TEMP_GOR, uDaneCM4.dane.fTemper[TEMP_IMU1]);
				FramDataWriteFloat(FAH_ZYRO2_TEMP_GOR, uDaneCM4.dane.fTemper[TEMP_IMU2]);
				FramDataWriteFloat(FAH_CISN_ROZN1_GOR, fSumaCisnRozn[0] / CZAS_KALIBRACJI_ZYROSKOPU);	//4F korekcja czujnika ciśnienia różnicowego 1 na gorąco
			}
			uDaneCM4.dane.nZainicjowano &= ~(INIT_TRWA_KALZ_ZYRO | INIT_TRWA_KALP_ZYRO | INIT_TRWA_KALG_ZYRO);	//wyłącz kalibrację
			InicjujModulI2P();	//przelicz współczynniki
		}
	}
	return ERR_OK;
}
