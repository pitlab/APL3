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

struct _RownanieProstej stRownanieOffsetuZyro1Zimno[3];		//równanie prostej offsetu dla temepratur < 25°
struct _RownanieProstej stRownanieOffsetuZyro1Goraco[3];	//równanie prostej offsetu dla temepratur > 25°
struct _RownanieProstej stRownanieOffsetuZyro2Zimno[3];		//równanie prostej offsetu dla temepratur < 25°
struct _RownanieProstej stRownanieOffsetuZyro2Goraco[3];	//równanie prostej offsetu dla temepratur > 25°

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

	//odczytaj kalibrację żyroskopów
	for (uint16_t n=0; n<3; n++)
	{

		fOffsetZyro1Z = FramDataReadFloat(FAH_ZYRO1_X_PRZ_ZIM+(4*n));	//offset żyroskopu1 na zimno
		fOffsetZyro2Z = FramDataReadFloat(FAH_ZYRO2_X_PRZ_ZIM+(4*n));	//offset żyroskopu2 na zimno

		fOffsetZyro1P = FramDataReadFloat(FAH_ZYRO1_X_PRZ_POK+(4*n));	//offset żyroskopu1 w temp pokojowej
		fOffsetZyro2P = FramDataReadFloat(FAH_ZYRO2_X_PRZ_POK+(4*n));	//offset żyroskopu2 w temp pokojowej

		fOffsetZyro1G = FramDataReadFloat(FAH_ZYRO1_X_PRZ_GOR+(4*n));	//offset żyroskopu1 na gorąco
		fOffsetZyro2G = FramDataReadFloat(FAH_ZYRO2_X_PRZ_GOR+(4*n));	//offset żyroskopu2 na gorąco

		fTemp1[n] = FramDataReadFloat(FAH_ZYRO1_TEMP_ZIM+(4*n));	//temepratury żyroskopu 1
		fTemp2[n] = FramDataReadFloat(FAH_ZYRO2_TEMP_ZIM+(4*n));	//temepratury żyroskopu 2

		ObliczRownanieFunkcjiTemperaturowyZyro(fOffsetZyro1Z, fOffsetZyro1P, fTemp1[0], fTemp1[1], &stRownanieOffsetuZyro1Zimno[n].fA, &stRownanieOffsetuZyro1Zimno[n].fB);	//Żyro 1 na zimno
		ObliczRownanieFunkcjiTemperaturowyZyro(fOffsetZyro1P, fOffsetZyro1G, fTemp1[1], fTemp1[2], &stRownanieOffsetuZyro1Goraco[n].fA, &stRownanieOffsetuZyro1Goraco[n].fB);	//Żyro 1 na gorąco
		ObliczRownanieFunkcjiTemperaturowyZyro(fOffsetZyro2Z, fOffsetZyro2P, fTemp2[0], fTemp2[1], &stRownanieOffsetuZyro2Zimno[n].fA, &stRownanieOffsetuZyro2Zimno[n].fB);	//Żyro 2 na zimno
		ObliczRownanieFunkcjiTemperaturowyZyro(fOffsetZyro2P, fOffsetZyro2G, fTemp2[1], fTemp2[2], &stRownanieOffsetuZyro2Goraco[n].fA, &stRownanieOffsetuZyro2Goraco[n].fB);	//Żyro 2 na gorąco
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// wykonuje czynności pomiarowe dla ukłądów znajdujących się na module
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
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_8;
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
		UstawAdresNaModule(ADR_MIIP_RES_ND130);
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
// [we] fTemp1, fTemp1 - temperatury dla offsetów
// [wy] *fA, *fB - wspóczynniki równania prostej offsetu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObliczRownanieFunkcjiTemperaturowyZyro(float fOffset1, float fOffset2, float fTemp1, float fTemp2, float *fA, float *fB)
{
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
// Liczy wysokość baromatryczną na podstawie ciśnienia i temperatury
// Parametry:
//  fP - ciśnienie na mierzonej wysokości [Pa]	jednostka ciśnienia jest dowolna, byle taka sama dla obu ciśnień
//  fP0 - ciśnienie na poziome odniesienia [Pa]
//  fTemp - temperatura [°C)
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

	return logf(fP/fP0) * STALA_GAZOWA_R * (fTemp + KELWIN) / (-1 * MASA_MOLOWA_POWIETRZA * PRZYSPIESZENIE_ZIEMSKIE);
}



////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna kalibrację żyroskopów
// Parametry: chRodzajKalib - rodzaj kalibracji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RozpocznijKalibracjeZyro(uint8_t chRodzajKalib)
{
	switch (chRodzajKalib)
	{
	case POL_KALIBRUJ_ZYRO_ZIM:	uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KALZ_ZYRO ;	break;	//uruchom kalibrację żyroskopów na zimno 10°C
	case POL_KALIBRUJ_ZYRO_POK: uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KALP_ZYRO;		break;	//uruchom kalibrację żyroskopów w temperaturze pokojowej 25°C
	case POL_KALIBRUJ_ZYRO_GOR:	uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KALG_ZYRO;		break;	//uruchom kalibrację żyroskopów na gorąco 40°C
	default: return ERR_ZLE_POLECENIE;
	}

	for (uint8_t n=0; n<3; n++)
	{
		dSumaZyro1[n] = 0.0;
		dSumaZyro2[n] = 0.0;
	}
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

				if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KALP_ZYRO)
				{
					FramDataWriteFloat(FAH_ZYRO1_X_PRZ_POK+(4*n), fOffsetZyro1[n]);	//zapisz do FRAM jako liczbę float zajmującą 4 bajty
					FramDataWriteFloat(FAH_ZYRO2_X_PRZ_POK+(4*n), fOffsetZyro2[n]);
				}

				if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KALG_ZYRO)
				{
					FramDataWriteFloat(FAH_ZYRO1_X_PRZ_GOR+(4*n), fOffsetZyro1[n]);	//zapisz do FRAM jako liczbę float zajmującą 4 bajty
					FramDataWriteFloat(FAH_ZYRO2_X_PRZ_GOR+(4*n), fOffsetZyro2[n]);
				}
			}
			uDaneCM4.dane.nZainicjowano &= ~(INIT_TRWA_KALZ_ZYRO | INIT_TRWA_KALP_ZYRO | INIT_TRWA_KALG_ZYRO);	//wyłącz kalibrację
		}
	}
	return ERR_OK;
}
