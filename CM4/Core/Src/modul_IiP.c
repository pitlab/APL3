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
uint16_t sLicznikCzasuKalibracji;
extern volatile unia_wymianyCM4_t uDaneCM4;
float fSkaloZyro1[3], fSkaloZyro2[3];
float fSkaloCisn[LICZBA_CZUJ_CISN];	//skalowanie odpowiedzi czujników ciśnienia
double dSuma1[3], dSuma2[3];		//do uśredniania pomiarów podczas kalibracji czujników
float fSumaCisn[LICZBA_CZUJ_CISN];	//do kalibracji czujników ciśnienia
WspRownProstej3_t stWspKalTempZyro1;		//współczynniki równania prostych do estymacji przesuniecia zera żyroskopu
WspRownProstej3_t stWspKalTempZyro2;		//współczynniki równania prostych do estymacji przesuniecia zera żyroskopu
WspRownProstej1_t stWspKalTempCzujnRozn1;
WspRownProstej1_t stWspKalTempCzujnRozn2;
float fKatMagnetometru1, fKatMagnetometru2, fKatMagnetometru3;	//kąt odchylenia z magnetometru
magn_t stMagn;
extern float fPrzesMagn1[3], fSkaloMagn1[3];
extern float fPrzesMagn2[3], fSkaloMagn2[3];
extern float fPrzesMagn3[3], fSkaloMagn3[3];
float fWzmocRegTermostatu = WZMOCNIENIE_REGULATORA_TERMOSTATU;
static uint8_t chLicznikOkresuPWMTermostatu;
static uint8_t chWypelnieniePWM;
float fTemeraturaTermostatu;

////////////////////////////////////////////////////////////////////////////////
// wykonuje czynności pomiarowe dla ukłądów znajdujących się na module
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujModulI2P(void)
{
	uint8_t chErr = BLAD_OK;
	uint8_t chLiczbaPowtorzen = 3;
	float fTemp1[3], fTemp2[3];	//temperatury na zimno, pokojowo i gorąco dla obu żyroskopów i czujników ciśnienia
	float fOffsetZyro1Z, fOffsetZyro2Z;		//offsety na zimno
	float fOffsetZyro1P, fOffsetZyro2P;		//offsety w temp pokojowej
	float fOffsetZyro1G, fOffsetZyro2G;		//offsety na gorąco
	float fOffsetCisnRoz1Z, fOffsetCisnRoz1P, fOffsetCisnRoz1G;	//offsety czujnika ciśnienia różnicowego 1
	float fOffsetCisnRoz2Z, fOffsetCisnRoz2P, fOffsetCisnRoz2G;	//offsety czujnika ciśnienia różnicowego 2

	do
	{
		//odczytaj tempertury kalibracji: 0=zimna, 1=pokojowa, 2=gorąca
		for (uint16_t n=0; n<3; n++)
		{
			fTemp1[n] = CzytajFramFloat(FAH_ZYRO1_TEMP_ZIM+(4*n));	//temepratury żyroskopu 1 [K]
			fTemp2[n] = CzytajFramFloat(FAH_ZYRO2_TEMP_ZIM+(4*n));	//temepratury żyroskopu 2 [K]
		}
		stWspKalTempZyro1.fTempPok = fTemp1[POK];
		stWspKalTempZyro2.fTempPok = fTemp2[POK];
		stWspKalTempCzujnRozn1.fTempPok = fTemp1[POK];

		//odczytaj kalibrację żyroskopów i licz charakterystykę
		for (uint16_t n=0; n<3; n++)
		{
			chErr |= CzytajFramZWalidacja(FAH_ZYRO1_X_PRZ_ZIM+(4*n), &fOffsetZyro1Z, VMIN_PRZES_ZYRO, VMAX_PRZES_ZYRO, VDEF_PRZES_ZYRO, ERR_ZLA_KONFIG);		//offset żyroskopu1 na zimno
			chErr |= CzytajFramZWalidacja(FAH_ZYRO2_X_PRZ_ZIM+(4*n), &fOffsetZyro2Z, VMIN_PRZES_ZYRO, VMAX_PRZES_ZYRO, VDEF_PRZES_ZYRO, ERR_ZLA_KONFIG);		//offset żyroskopu2 na zimno

			chErr |= CzytajFramZWalidacja(FAH_ZYRO1_X_PRZ_POK+(4*n), &fOffsetZyro1P, VMIN_PRZES_ZYRO, VMAX_PRZES_ZYRO, VDEF_PRZES_ZYRO, ERR_ZLA_KONFIG);		//offset żyroskopu1 w temp pokojowej
			chErr |= CzytajFramZWalidacja(FAH_ZYRO2_X_PRZ_POK+(4*n), &fOffsetZyro2P, VMIN_PRZES_ZYRO, VMAX_PRZES_ZYRO, VDEF_PRZES_ZYRO, ERR_ZLA_KONFIG);		//offset żyroskopu1 w temp pokojowej

			chErr |= CzytajFramZWalidacja(FAH_ZYRO1_X_PRZ_GOR+(4*n), &fOffsetZyro1G, VMIN_PRZES_ZYRO, VMAX_PRZES_ZYRO, VDEF_PRZES_ZYRO, ERR_ZLA_KONFIG);		//offset żyroskopu1 na gorąco
			chErr |= CzytajFramZWalidacja(FAH_ZYRO2_X_PRZ_GOR+(4*n), &fOffsetZyro2G, VMIN_PRZES_ZYRO, VMAX_PRZES_ZYRO, VDEF_PRZES_ZYRO, ERR_ZLA_KONFIG);		//offset żyroskopu2 na gorąco

			chErr |= ObliczRownanieFunkcjiTemperatury(fOffsetZyro1Z, fOffsetZyro1P, fTemp1[ZIM], fTemp1[POK], &stWspKalTempZyro1.fAzim[n], &stWspKalTempZyro1.fBzim[n]);	//Żyro 1 na zimno
			chErr |= ObliczRownanieFunkcjiTemperatury(fOffsetZyro1P, fOffsetZyro1G, fTemp1[POK], fTemp1[GOR], &stWspKalTempZyro1.fAgor[n], &stWspKalTempZyro1.fBgor[n]);	//Żyro 1 na gorąco
			chErr |= ObliczRownanieFunkcjiTemperatury(fOffsetZyro2Z, fOffsetZyro2P, fTemp2[ZIM], fTemp2[POK], &stWspKalTempZyro2.fAzim[n], &stWspKalTempZyro2.fBzim[n]);	//Żyro 2 na zimno
			chErr |= ObliczRownanieFunkcjiTemperatury(fOffsetZyro2P, fOffsetZyro2G, fTemp2[POK], fTemp2[GOR], &stWspKalTempZyro2.fAgor[n], &stWspKalTempZyro2.fBgor[n]);	//Żyro 2 na gorąco

			chErr |= CzytajFramZWalidacja(FAH_ZYRO1P_WZMOC+(4*n), &fSkaloZyro1[n], VMIN_SKALO_ZYRO, VMAX_SKALO_ZYRO, VDEF_SKALO_ZYRO, ERR_ZLA_KONFIG);	//wzmocnienie osi żyroskopu 1
			chErr |= CzytajFramZWalidacja(FAH_ZYRO2P_WZMOC+(4*n), &fSkaloZyro2[n], VMIN_SKALO_ZYRO, VMAX_SKALO_ZYRO, VDEF_SKALO_ZYRO, ERR_ZLA_KONFIG);	//wzmocnienie osi żyroskopu 2

			//konfiguracja magnetometrów przesunieta do czujników
			/*chErr |= CzytajFramZWalidacja(FAH_MAGN1_PRZESX + 4*n, &fPrzesMagn1[n], VMIN_PRZES_MAGN, VMAX_PRZES_MAGN, VDEF_PRZES_MAGN, ERR_ZLA_KONFIG);
			chErr |= CzytajFramZWalidacja(FAH_MAGN2_PRZESX + 4*n, &fPrzesMagn2[n], VMIN_PRZES_MAGN, VMAX_PRZES_MAGN, VDEF_PRZES_MAGN, ERR_ZLA_KONFIG);
			//chErr |= CzytajFramZWalidacja(FAH_MAGN3_PRZESX + 4*n, &fPrzesMagn3[n], VMIN_PRZES_MAGN, VMAX_PRZES_MAGN, VDEF_PRZES_MAGN, ERR_ZLA_KONFIG);
			chErr |= CzytajFramZWalidacja(FAH_MAGN1_SKALOX + 4*n, &fSkaloMagn1[n], VMIN_SKALO_MAGN, VMAX_SKALO_MAGN, VDEF_SKALO_MAGN, ERR_ZLA_KONFIG);
			chErr |= CzytajFramZWalidacja(FAH_MAGN2_SKALOX + 4*n, &fSkaloMagn2[n], VMIN_SKALO_MAGN, VMAX_SKALO_MAGN, VDEF_SKALO_MAGN, ERR_ZLA_KONFIG);
			//chErr |= CzytajFramZWalidacja(FAH_MAGN3_SKALOX + 4*n, &fSkaloMagn3[n], VMIN_SKALO_MAGN, VMAX_SKALO_MAGN, VDEF_SKALO_MAGN, ERR_ZLA_KONFIG);*/
		}

		//odczytaj kalibrację czujników ciśnienia różnicowego i licz charakterystykę. Używana jest temperatura żyroskopu 1
		chErr |= CzytajFramZWalidacja(FAH_CISN_ROZN1_ZIM, &fOffsetCisnRoz1Z, VMIN_PRZES_PDIF, VMAX_PRZES_PDIF, VDEF_PRZES_PDIF, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 1 na zimno
		chErr |= CzytajFramZWalidacja(FAH_CISN_ROZN1_POK, &fOffsetCisnRoz1P, VMIN_PRZES_PDIF, VMAX_PRZES_PDIF, VDEF_PRZES_PDIF, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 1 w temp pokojowej
		chErr |= CzytajFramZWalidacja(FAH_CISN_ROZN1_GOR, &fOffsetCisnRoz1G, VMIN_PRZES_PDIF, VMAX_PRZES_PDIF, VDEF_PRZES_PDIF, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 1 na gorąco
		chErr |= ObliczRownanieFunkcjiTemperatury(fOffsetCisnRoz1Z, fOffsetCisnRoz1P, fTemp1[ZIM], fTemp1[POK], &stWspKalTempCzujnRozn1.fAzim, &stWspKalTempCzujnRozn1.fBzim);	//czujnik cisnienia różnicowego 1 na zimno
		chErr |= ObliczRownanieFunkcjiTemperatury(fOffsetCisnRoz1P, fOffsetCisnRoz1G, fTemp1[POK], fTemp1[GOR], &stWspKalTempCzujnRozn1.fAgor, &stWspKalTempCzujnRozn1.fBgor);	//czujnik cisnienia różnicowego 1 na gorąco

		chErr |= CzytajFramZWalidacja(FAH_CISN_ROZN2_ZIM, &fOffsetCisnRoz2Z, VMIN_PRZES_PDIF, VMAX_PRZES_PDIF, VDEF_PRZES_PDIF, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 2 na zimno
		chErr |= CzytajFramZWalidacja(FAH_CISN_ROZN2_POK, &fOffsetCisnRoz2P, VMIN_PRZES_PDIF, VMAX_PRZES_PDIF, VDEF_PRZES_PDIF, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 2 w temp pokojowej
		chErr |= CzytajFramZWalidacja(FAH_CISN_ROZN2_GOR, &fOffsetCisnRoz2G, VMIN_PRZES_PDIF, VMAX_PRZES_PDIF, VDEF_PRZES_PDIF, ERR_ZLA_KONFIG);		//offset czujnika różnicowego 2 na gorąco
		chErr |= ObliczRownanieFunkcjiTemperatury(fOffsetCisnRoz2Z, fOffsetCisnRoz2P, fTemp1[ZIM], fTemp1[POK], &stWspKalTempCzujnRozn2.fAzim, &stWspKalTempCzujnRozn2.fBzim);	//czujnik cisnienia różnicowego 2 na zimno
		chErr |= ObliczRownanieFunkcjiTemperatury(fOffsetCisnRoz2P, fOffsetCisnRoz2G, fTemp1[POK], fTemp1[GOR], &stWspKalTempCzujnRozn2.fAgor, &stWspKalTempCzujnRozn2.fBgor);	//czujnik cisnienia różnicowego 2 na gorąco

		chErr |= CzytajFramZWalidacja(FAH_SKALO_CISN_BEZWZGL1, &fSkaloCisn[0], VMIN_SKALO_PABS, VMAX_SKALO_PABS, VDEF_SKALO_PAB, ERR_ZLA_KONFIG);		//skalowanie wartości  czujnika ciśnienia bezwzględnego
		chErr |= CzytajFramZWalidacja(FAH_SKALO_CISN_BEZWZGL2, &fSkaloCisn[1], VMIN_SKALO_PABS, VMAX_SKALO_PABS, VDEF_SKALO_PAB, ERR_ZLA_KONFIG);		//skalowanie wartości  czujnika ciśnienia bezwzględnego
	}
	while (chErr && --chLiczbaPowtorzen);	//w przypadku jakiegokolwiek błędu powtórz całość
	return chErr;
}


////////////////////////////////////////////////////////////////////////////////
// wykonuje czynności pomiarowe dla układów znajdujących się na module
// Parametry:
// [we] chGniazdo - numer gniazda, w którym siedzi moduł
// [we-wy] *pchStanIOwy - wskaźnik na stan linii IO na modułach
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaModuluI2P(uint8_t gniazdo, uint8_t* pchStanIOwy)
{
	uint8_t chErr;
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 40MHz a układy mogą pracować z prędkością max 10MHz, przy każdym dostępie przestaw dzielnik zegara na 4
	nZastanaKonfiguracja_SPI_CFG1 = hspi2.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi2.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;		//maska preskalera
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_4;
	//hspi2.Instance->CFG2 |= SPI_POLARITY_HIGH | SPI_PHASE_2EDGE;	//testowo ustaw mode 3

	//ustaw adres A2 = 0 zrobiony z linii MOD_IOx1 modułu
	switch (gniazdo)
	{
	case ADR_MOD1: 	*pchStanIOwy &= ~MIO11;	break;
	case ADR_MOD2:	*pchStanIOwy &= ~MIO21;	break;
	case ADR_MOD3:	*pchStanIOwy &= ~MIO31;	break;
	case ADR_MOD4:	*pchStanIOwy &= ~MIO41;	break;
	}

	chErr = WyslijDaneExpandera(*pchStanIOwy);
	chErr |= UstawDekoderModulow(gniazdo);				//ustaw adres dekodera modułów, ponieważ użycie expandera przestawia adres
	chErr |= UstawAdresNaModule(ADR_MIIP_MS5611);				//ustaw adres na module A0..1
	chErr |= ObslugaMS5611();

	chErr |= UstawAdresNaModule(ADR_MIIP_BMP581);				//ustaw adres na module A0..1
	chErr |= ObslugaBMP581();

	chErr |= UstawAdresNaModule(ADR_MIIP_ICM42688);				//ustaw adres na module A0..1
	chErr |= ObslugaICM42688();

	chErr |= UstawAdresNaModule(ADR_MIIP_LSM6DSV);				//ustaw adres na module A0..1
	chErr |= ObslugaLSM6DSV();

	if (uDaneCM4.dane.nZainicjowano & (INIT_TRWA_KAL_ZYRO_ZIM | INIT_TRWA_KAL_ZYRO_POK | INIT_TRWA_KAL_ZYRO_GOR))
		KalibrujZeroZyroskopu();

	//termostatuj moduł uśrednioną temperaturą obu czujników IMU a jezeli nie ma obu to chociaż jednego
	uint8_t chLiczbaTermometrow = 0;
	float fTemeratura = 0.0f;

	for (uint8_t n=TEMP_IMU1; n<=TEMP_IMU2; n++)
	{
		if ((uDaneCM4.dane.fTemper[n] > TEMPERATURA_MIN_OK) && (uDaneCM4.dane.fTemper[n] < TEMPERATURA_MAX_OK))
		{
			fTemeratura += uDaneCM4.dane.fTemper[n];
			chLiczbaTermometrow++;
		}
	}
	if (chLiczbaTermometrow > 1)
		fTemeratura /= chLiczbaTermometrow;

	fTemeraturaTermostatu = (3*fTemeraturaTermostatu + fTemeratura)/4;
	if (fTemeraturaTermostatu > 0.0f)
		Termostat(gniazdo, pchStanIOwy, fTemeraturaTermostatu);	//termostat zwraca stan linii grzałki ale sam jej nie ustawia

	//ustaw adres A2 = 1 zrobiony z linii Ix1 modułu
	switch (gniazdo)
	{
	case ADR_MOD1: 	*pchStanIOwy |= MIO11;	break;
	case ADR_MOD2:	*pchStanIOwy |= MIO21;	break;
	case ADR_MOD3:	*pchStanIOwy |= MIO31;	break;
	case ADR_MOD4:	*pchStanIOwy |= MIO41;	break;
	}
	chErr = WyslijDaneExpandera(*pchStanIOwy);	//ustaw stan linii A2 i grzałki

	// Układ ND130 pracujacy na magistrali SPI ma okres zegara 6us co odpowiada częstotliwości 166kHz
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_256;	//przestaw zegar na 40MHz / 256 = 156kHz
	UstawAdresNaModule(ADR_MIIP_ND130);				//ustaw adres A0..1
	chErr |= ObslugaND130();
	if (chErr == BLAD_TIMEOUT)
	{
		UstawAdresNaModule(ADR_MIIP_RES_ND130);	//wejście resetujące ND130
		HAL_Delay(1);	//resetuj układ
	}

	hspi2.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróc poprzednie nastawy
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza równanie prostej linearyzującej zmianę przesunięcia zera żyroskopu w funkcji temperatury
// Parametry:
// [we] fPrzes1, fPrzes2 - wartości przesunięcia żyroskopów
// [we] fTemp1, fTemp2 - temperatury [K]
// [wy] *fA, *fB - wspóczynniki równania prostej
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObliczRownanieFunkcjiTemperatury(float fPrzes1, float fPrzes2, float fTemp1, float fTemp2, float *fA, float *fB)
{
	uint8_t chErr = BLAD_OK;
	float fdTemp, fdPrzes;
	//Uwaga! We wzorze liczenia dryftu temperatura jest w mianowniku, więc skala nie może być w stopniach Celsjusza gdyż wystąpi dzielenie przez zero,
	//a wcześniej dzielenie przez bardzo małą wartość, co powoduje że charakterystyka strzela do nieskończonosci.
	//Temperaturę trzeba przeliczyć na Kelviny
	//fTemp1 += KELVIN;
	//fTemp2 += KELVIN;

	//układ równań prostych w postaci kierunkowej:
	//fPrzes1 = A * temperatura1 + B
	//fPrzes2 = A * temperatura2 + B
	//po odjeciu stronami: fPrzes1 - fPrzes2 =  temperatura1 * A - temperatura2 * A
	//A = (fPrzes1 - fPrzes2) / (temperatura1 - temperatura2)
	fdTemp = fTemp1 - fTemp2;
	fdPrzes = fPrzes1 - fPrzes2;

	if ((fdTemp == 0.0f) || (isnan(fdTemp)) || (isnan(fdPrzes)))
	{
		*fA = 0.0f;
		chErr = ERR_ZLE_DANE;
	}
	else
		//*fA = (fPrzes1 - fPrzes2) / (fTemp1 - fTemp2);
		*fA = fdPrzes / fdTemp;

	//B wyznaczamy z pierwszego równania: B = fPrzes1 - A * temperatura1
	if ((fTemp1 == 0.0f) || (isnan(fTemp1)))
	{
		*fB = 0.0f;
		chErr = ERR_ZLE_DANE;
	}
	else
		*fB = fPrzes1 - (*fA * fTemp1);

	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza wartość offsetu dla czujnika 3-osiowego dla danej temperatury na podstawie równania prostej linearyzującej zmianę offsetu żyroskopu w funkcji temperatury
// Parametry:
// [we] stWsp struktura z wartościami współczynników równania prostych
// [we] fTemp - temperatura żyroskopu [K]
// [wy] *fOffset[3] - obliczone wartości offsetu dla wszystkich osi
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObliczWspTemperaturowy3(WspRownProstej3_t stWsp, float fTemp, float *fPrzesZera)
{
	if (fTemp > stWsp.fTempPok)
	{
		for (uint8_t n=0; n<3; n++)
			*(fPrzesZera + n) = stWsp.fAgor[n] * fTemp + stWsp.fBgor[n];
	}
	else
	{
		for (uint8_t n=0; n<3; n++)
			*(fPrzesZera + n) = stWsp.fAzim[n] * fTemp + stWsp.fBzim[n];
	}
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza wartość offsetu dla czujnika 1 wymiarowego dla danej temperatury na podstawie równania prostej linearyzującej zmianę offsetu żyroskopu w funkcji temperatury
// Parametry:
// [we] stWsp struktura z wartościami współczynników równania prostych
// [we] fTemp - temperatura żyroskopu [K]
// Zwraca: obliczone wartości offsetu
////////////////////////////////////////////////////////////////////////////////
float ObliczWspTemperaturowy1(WspRownProstej1_t stWsp, float fTemp)
{
	float fPrzesZera;

	if (fTemp > stWsp.fTempPok)
		fPrzesZera = stWsp.fAgor * fTemp + stWsp.fBgor;
	else
		fPrzesZera = stWsp.fAzim * fTemp + stWsp.fBzim;
	return fPrzesZera;
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
// Liczy ciśnienie barometryczne na podstawie wysokości i temperatury: wzór barometryczny: https://pl.wikipedia.org/wiki/Wz%C3%B3r_barometryczny
// Parametry:
//  fWysokosc - wysokość [m]
//  fP0 - ciśnienie na poziome odniesienia [Pa]
//  fTemp - temperatura [K)
// Zwraca: obliczoną wysokość
////////////////////////////////////////////////////////////////////////////////
float CisnienieBarometryczne(float fWysokosc, float fP0, float fTemp)
{
	//P = P0 * e^(-(u*g*h)/(R*T))
	return fP0 * expf(-1 * MASA_MOLOWA_POWIETRZA * PRZYSPIESZENIE_ZIEMSKIE * fWysokosc / (STALA_GAZOWA_R * fTemp));
}



////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna kalibrację żyroskopów i czujników ciśnienia różnicowego
// Parametry: chRodzajKalib - rodzaj kalibracji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RozpocznijKalibracjeZeraZyroskopu(uint8_t chRodzajKalib)
{
	float fTemperatura;

	//sprawdź czy kalibracja już trwa jeżeli tak, to nie zaczynaj kolejnej przed zakończeniem obecnej
	if (uDaneCM4.dane.nZainicjowano & (INIT_TRWA_KAL_ZYRO_ZIM | INIT_TRWA_KAL_ZYRO_POK | INIT_TRWA_KAL_ZYRO_GOR))
		return BLAD_OK;

	fTemperatura = (uDaneCM4.dane.fTemper[TEMP_IMU1] + uDaneCM4.dane.fTemper[TEMP_IMU2]) / 2;
	uDaneCM4.dane.chOdpowiedzNaPolecenie = BLAD_OK;

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
			uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KAL_ZYRO_ZIM ;	//uruchom kalibrację żyroskopów na zimno w +10°C
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
			uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KAL_ZYRO_POK;		//uruchom kalibrację żyroskopów w temperaturze pokojowej 25°C
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
			uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KAL_ZYRO_GOR;		//uruchom kalibrację żyroskopów na gorąco 40°C
		break;

	default: return ERR_ZLE_POLECENIE;
	}

	for (uint8_t n=0; n<3; n++)
	{
		dSuma1[n] = 0.0;
		dSuma2[n] = 0.0;
	}
	fSumaCisn[0] = 0.0;
	uDaneCM4.dane.sPostepProcesu = CZAS_KALIBRACJI;
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj kalibrację wzmocnienia żyroskopów  i zapisz współczynnik kalibracji
// Parametry: chRodzajKalib - rodzaj kalibracji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibracjaWzmocnieniaZyro(uint8_t chRodzajKalib)
{
	//uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KAL_WZM_ZYRO;	//trwa kalibracja, w tym czasie wyłącz zawijanie katów do +-Pi
	switch (chRodzajKalib)
	{
	case POL_KALIBRUJ_ZYRO_WZMP:	//kalibruj wzmocnienia żyroskopów P
		if ((uDaneCM4.dane.nZainicjowano & INIT_WYK_KAL_WZM_ZYRO) != INIT_WYK_KAL_WZM_ZYRO)
		{
			fSkaloZyro1[0] = OBR_KAL_WZM * 2 * M_PI  / fabsf(uDaneCM4.dane.fKatZyro1[0]);
			if ((fSkaloZyro1[0] > VMIN_SKALO_ZYRO) && (fSkaloZyro1[0] < VMAX_SKALO_ZYRO))
			{
				ZapiszFramFloat(FAH_ZYRO1P_WZMOC, fSkaloZyro1[0]);
				uDaneCM4.dane.nZainicjowano |= INIT_WYK_KAL_WZM_ZYRO;
				uDaneCM4.dane.uRozne.f32[0] = fSkaloZyro1[0];	//przekaż wartości kalibracji
			}
			else
			{
				uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZLE_OBLICZENIA;
				return BLAD_OK;
			}

			fSkaloZyro2[0] = OBR_KAL_WZM * 2 * M_PI  / fabsf(uDaneCM4.dane.fKatZyro2[0]);
			if ((fSkaloZyro2[0] > VMIN_SKALO_ZYRO) && (fSkaloZyro2[0] < VMAX_SKALO_ZYRO))
			{
				ZapiszFramFloat(FAH_ZYRO2P_WZMOC, fSkaloZyro2[0]);
				uDaneCM4.dane.nZainicjowano |= INIT_WYK_KAL_WZM_ZYRO;
				uDaneCM4.dane.uRozne.f32[1] = fSkaloZyro2[0];
			}
			else
			{
				uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZLE_OBLICZENIA;
				return BLAD_OK;
			}
			uDaneCM4.dane.uRozne.f32[2] = CzytajFramFloat(FAH_ZYRO1P_WZMOC);	//przekaż wartości bieżącej kalibracji
			uDaneCM4.dane.uRozne.f32[3] = CzytajFramFloat(FAH_ZYRO2P_WZMOC);
		}
		break;


	case POL_KALIBRUJ_ZYRO_WZMQ:	//kalibruj wzmocnienia żyroskopów Q
		if ((uDaneCM4.dane.nZainicjowano & INIT_WYK_KAL_WZM_ZYRO) != INIT_WYK_KAL_WZM_ZYRO)
		{
			fSkaloZyro1[1] = OBR_KAL_WZM * 2 * M_PI  / fabsf(uDaneCM4.dane.fKatZyro1[1]);
			if ((fSkaloZyro1[1] > VMIN_SKALO_ZYRO) && (fSkaloZyro1[1] < VMAX_SKALO_ZYRO))
			{
				ZapiszFramFloat(FAH_ZYRO1Q_WZMOC, fSkaloZyro1[1]);
				uDaneCM4.dane.nZainicjowano |= INIT_WYK_KAL_WZM_ZYRO;
				uDaneCM4.dane.uRozne.f32[0] = fSkaloZyro1[1];	//przekaż wartości kalibracji
			}
			else
			{
				uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZLE_OBLICZENIA;
				return BLAD_OK;
			}

			fSkaloZyro2[1] = OBR_KAL_WZM * 2 * M_PI  / fabsf(uDaneCM4.dane.fKatZyro2[1]);
			if ((fSkaloZyro2[1] > VMIN_SKALO_ZYRO) && (fSkaloZyro2[1] < VMAX_SKALO_ZYRO))
			{
				ZapiszFramFloat(FAH_ZYRO2Q_WZMOC, fSkaloZyro2[1]);
				uDaneCM4.dane.nZainicjowano |= INIT_WYK_KAL_WZM_ZYRO;
				uDaneCM4.dane.uRozne.f32[1] = fSkaloZyro2[1];
			}
			else
			{
				uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZLE_OBLICZENIA;
				return BLAD_OK;
			}
		}
		uDaneCM4.dane.uRozne.f32[2] = CzytajFramFloat(FAH_ZYRO1Q_WZMOC);	//przekaż wartości bieżącej kalibracji
		uDaneCM4.dane.uRozne.f32[3] = CzytajFramFloat(FAH_ZYRO2Q_WZMOC);
		break;

	case POL_KALIBRUJ_ZYRO_WZMR:	//kalibruj wzmocnienia żyroskopów R
		if ((uDaneCM4.dane.nZainicjowano & INIT_WYK_KAL_WZM_ZYRO) != INIT_WYK_KAL_WZM_ZYRO)
		{
			fSkaloZyro1[2] = OBR_KAL_WZM * 2 * M_PI  / fabsf(uDaneCM4.dane.fKatZyro1[2]);
			if ((fSkaloZyro1[2] > VMIN_SKALO_ZYRO) && (fSkaloZyro1[2] < VMAX_SKALO_ZYRO))
			{
				ZapiszFramFloat(FAH_ZYRO1R_WZMOC, fSkaloZyro1[2]);
				uDaneCM4.dane.nZainicjowano |= INIT_WYK_KAL_WZM_ZYRO;
				uDaneCM4.dane.uRozne.f32[0] = fSkaloZyro1[2];	//przekaż wartości kalibracji
			}
			else
			{
				uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZLE_OBLICZENIA;
				return BLAD_OK;
			}

			fSkaloZyro2[2] = OBR_KAL_WZM * 2 * M_PI  / fabsf(uDaneCM4.dane.fKatZyro2[2]);
			if ((fSkaloZyro2[2] > VMIN_SKALO_ZYRO) && (fSkaloZyro2[2] < VMAX_SKALO_ZYRO))
			{
				ZapiszFramFloat(FAH_ZYRO2R_WZMOC, fSkaloZyro2[2]);
				uDaneCM4.dane.nZainicjowano |= INIT_WYK_KAL_WZM_ZYRO;
				uDaneCM4.dane.uRozne.f32[1] = fSkaloZyro2[2];
			}
			else
			{
				uDaneCM4.dane.chOdpowiedzNaPolecenie = ERR_ZLE_OBLICZENIA;
				return BLAD_OK;
			}
		}
		uDaneCM4.dane.uRozne.f32[2] = CzytajFramFloat(FAH_ZYRO1R_WZMOC);	//przekaż wartości bieżącej kalibracji
		uDaneCM4.dane.uRozne.f32[3] = CzytajFramFloat(FAH_ZYRO2R_WZMOC);
		break;

	case POL_ZERUJ_CALKE_ZYRO:		//zeruje całkę prędkosci katowej żyroskopów przed kalibracją wzmocnienia
		for (uint16_t n=0; n<3; n++)
		{
			uDaneCM4.dane.fKatZyro1[n] = 0.0f;
			uDaneCM4.dane.fKatZyro2[n] = 0.0f;
		}
		uDaneCM4.dane.uRozne.f32[0] = 0.0f;
		uDaneCM4.dane.uRozne.f32[1] = 0.0f;
		uDaneCM4.dane.nZainicjowano &= ~INIT_WYK_KAL_WZM_ZYRO;	//nie jest zainicjowane
		break;

	default: return ERR_ZLE_POLECENIE;
	}

	uDaneCM4.dane.chOdpowiedzNaPolecenie = chRodzajKalib;	//zwróć potwierdzenie że znajduje się w danym etapie
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje kalibrację przesunięcia zera żyroskopów i czujnika różnicowego, nastepnie zapisuje wynik we FRAM
// Parametry: sCzasKalibracji - czas liczny w kwantach obiegu pętli głównej
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibrujZeroZyroskopu(void)
{
	if (uDaneCM4.dane.sPostepProcesu)
		uDaneCM4.dane.sPostepProcesu--;

	if (uDaneCM4.dane.nZainicjowano & (INIT_TRWA_KAL_ZYRO_ZIM | INIT_TRWA_KAL_ZYRO_POK | INIT_TRWA_KAL_ZYRO_GOR))	//jeżeli trwa którakolwiek z kalibracji
	{
		for (uint8_t n=0; n<3; n++)
		{
			dSuma1[n] += uDaneCM4.dane.fZyroSur1[n];
			dSuma2[n] += uDaneCM4.dane.fZyroSur2[n];
		}
		fSumaCisn[0] += uDaneCM4.dane.fCisnRozn[0];

		if (uDaneCM4.dane.sPostepProcesu == 0)
		{
			for (uint8_t n=0; n<3; n++)
			{
				if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KAL_ZYRO_ZIM)
				{
					ZapiszFramFloat(FAH_ZYRO1_X_PRZ_ZIM+(4*n), dSuma1[n] / CZAS_KALIBRACJI);	//zapisz do FRAM jako liczbę float zajmującą 4 bajty
					ZapiszFramFloat(FAH_ZYRO2_X_PRZ_ZIM+(4*n), dSuma2[n] / CZAS_KALIBRACJI);
				}
				else
				if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KAL_ZYRO_POK)
				{
					ZapiszFramFloat(FAH_ZYRO1_X_PRZ_POK+(4*n), dSuma1[n] / CZAS_KALIBRACJI);	//zapisz do FRAM jako liczbę float zajmującą 4 bajty
					ZapiszFramFloat(FAH_ZYRO2_X_PRZ_POK+(4*n), dSuma2[n] / CZAS_KALIBRACJI);
				}
				else
				if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KAL_ZYRO_GOR)
				{
					ZapiszFramFloat(FAH_ZYRO1_X_PRZ_GOR+(4*n), dSuma1[n] / CZAS_KALIBRACJI);	//zapisz do FRAM jako liczbę float zajmującą 4 bajty
					ZapiszFramFloat(FAH_ZYRO2_X_PRZ_GOR+(4*n), dSuma2[n] / CZAS_KALIBRACJI);
				}
			}

			//zapisz temperatury i dane czujnika różnicowego
			if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KAL_ZYRO_ZIM)
			{
				ZapiszFramFloat(FAH_ZYRO1_TEMP_ZIM, uDaneCM4.dane.fTemper[TEMP_IMU1]);
				ZapiszFramFloat(FAH_ZYRO2_TEMP_ZIM, uDaneCM4.dane.fTemper[TEMP_IMU2]);
				ZapiszFramFloat(FAH_CISN_ROZN1_ZIM, fSumaCisn[0] / CZAS_KALIBRACJI);		//4F korekcja czujnika ciśnienia różnicowego 1 na zimno
			}
			else
			if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KAL_ZYRO_POK)
			{
				ZapiszFramFloat(FAH_ZYRO1_TEMP_POK, uDaneCM4.dane.fTemper[TEMP_IMU1]);
				ZapiszFramFloat(FAH_ZYRO2_TEMP_POK, uDaneCM4.dane.fTemper[TEMP_IMU2]);
				ZapiszFramFloat(FAH_CISN_ROZN1_POK, fSumaCisn[0] / CZAS_KALIBRACJI);	//4F korekcja czujnika ciśnienia różnicowego 1 w 25°C
			}
			else
			if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KAL_ZYRO_GOR)
			{
				ZapiszFramFloat(FAH_ZYRO1_TEMP_GOR, uDaneCM4.dane.fTemper[TEMP_IMU1]);
				ZapiszFramFloat(FAH_ZYRO2_TEMP_GOR, uDaneCM4.dane.fTemper[TEMP_IMU2]);
				ZapiszFramFloat(FAH_CISN_ROZN1_GOR, fSumaCisn[0] / CZAS_KALIBRACJI);	//4F korekcja czujnika ciśnienia różnicowego 1 na gorąco
			}
			uDaneCM4.dane.nZainicjowano &= ~(INIT_TRWA_KAL_ZYRO_ZIM | INIT_TRWA_KAL_ZYRO_POK | INIT_TRWA_KAL_ZYRO_GOR);	//wyłącz kalibrację
			InicjujModulI2P();	//przelicz współczynniki
		}
	}
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje kalibrację skalowania ciśnienia czujników ciśnienia bezwzglednego odniesionych do wzorca wysokości.
// Uśrednia początkowe i końcowe ciśnienie oraz temperaturę  w 2etapowym procesie kalibracji
// Po zsumowaniu n=CZAS_KALIBRACJI pomiarów w zmiennych dSuma1[3] i dSuma2[3]
// Kolejność pomiaru ciśnień jest dowolna: jako P0 biorę ciśnienie większe a różnica jest liczbą bezwzgledną
// Parametry:
// fCisnienie1, fCisnienie2 - ciśnienia do uśredniania
// fTemp - temperatura do uśredniania
// chPrzebieg - wskazuje czy uśredniamy ciśnienie początku (0) czy końca (1) kalibracji
// sLicznik - czas liczny w kwantach obiegu pętli głównej
// uDaneCM4.dane.uRozne.f32[0] - pierwsze uśrednione ciśnienie czujnika 1
// uDaneCM4.dane.uRozne.f32[1] - pierwsze uśrednione ciśnienie czujnika 2
// uDaneCM4.dane.uRozne.f32[2] - drugie uśrednione ciśnienie czujnika 1
// uDaneCM4.dane.uRozne.f32[3] - drugie uśrednione ciśnienie czujnika 2
// uDaneCM4.dane.uRozne.f32[4] - współczynnik skalowania czujnika 1
// uDaneCM4.dane.uRozne.f32[5] - współczynnik skalowania czujnika 2
// Zwraca: kod błędu ERR_DONE gdy gotowe, BLAD_OK w trakcie pracy
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibrujCisnienie(float fCisnienie1, float fCisnienie2, float fTemp, uint16_t sLicznik, uint8_t chPrzebieg)
{
	uint8_t chErr = BLAD_OK;
	float fSredCisn1[LICZBA_CZUJ_CISN], fSredCisn2[LICZBA_CZUJ_CISN];

	//wstępne inicjowanie procesu
	if (chPrzebieg == 0xFF)
	{
		for (uint8_t n=0; n<4; n++)
			uDaneCM4.dane.uRozne.f32[n] = 0;	//wyczyść ewentualne wcześniejsze dane

		for (uint8_t n=0; n<LICZBA_CZUJ_CISN; n++)
			uDaneCM4.dane.uRozne.f32[n+4] = fSkaloCisn[n];	//ustaw bieżący współczynnik skalowania

		for (uint8_t n=0; n<3; n++)
		{
			dSuma1[n] = 0.0;		//inicjuj zmienne do przechowywania sumy ciśnień i temperatury
			dSuma2[n] = 0.0;
		}
	}

	//dopóki proces nie jest skończony to sumuj pomiary w zmiennej po podwójnej precyzji
	if (sLicznik < CZAS_KALIBRACJI)
	{
		if (chPrzebieg)
		{
			dSuma2[0] += (double)fCisnienie1;
			dSuma2[1] += (double)fCisnienie2;
			dSuma2[2] += (double)fTemp;			//temperatura [K]
			for (uint8_t n=0; n<LICZBA_CZUJ_CISN; n++)
				uDaneCM4.dane.uRozne.f32[n+2] = (float)(dSuma2[n] / (sLicznik + 1));
		}
		else
		{
			dSuma1[0] += (double)fCisnienie1;
			dSuma1[1] += (double)fCisnienie2;
			dSuma1[2] += (double)fTemp;
			for (uint8_t n=0; n<LICZBA_CZUJ_CISN; n++)
				uDaneCM4.dane.uRozne.f32[n+0] = (float)(dSuma1[n] / (sLicznik + 1));
		}
	}
	else
		chErr = ERR_GOTOWE;		//jeżeli zebrano tyle co trzeba to przestań sumować i zwróć kod zakończenia

	//za drugim przebiegiem gdy są już uśrednione oba ciśnienia można policzyć finalne współczynniki skalowania kalibracji
	if ((sLicznik == CZAS_KALIBRACJI - 1) && chPrzebieg)
	{
		float fCisnWzorcowe[LICZBA_CZUJ_CISN];	//teoretyczne ciśnienie P1 na wysokości 27m względem obecnego P0
		float fdCisn, fdWzorc;	//różnica ciśnień rzeczywistych i wzorcowych
		float fRoboczySkaloCisn[LICZBA_CZUJ_CISN];

		fTemp = (float)((dSuma1[2] + dSuma2[2]) / ( 2 *CZAS_KALIBRACJI));		//średnia temperatura z obu pomiarów

		for (uint8_t n=0; n<LICZBA_CZUJ_CISN; n++)
		{
			fSredCisn1[n] = (float)(dSuma1[n] / CZAS_KALIBRACJI);
			fSredCisn2[n] = (float)(dSuma2[n] / CZAS_KALIBRACJI);

			//jako ciśnienie P0 weź to które jest wyższe
			if (fSredCisn1[n] > fSredCisn2[n])
			{
				fCisnWzorcowe[n] = CisnienieBarometryczne(WYSOKOSC10PIETER, fSredCisn1[n], fTemp);
				fRoboczySkaloCisn[n] = fCisnWzorcowe[n] / fSredCisn2[n];
			}
			else
			{
				fCisnWzorcowe[n] = CisnienieBarometryczne(WYSOKOSC10PIETER, fSredCisn2[n], fTemp);
				fRoboczySkaloCisn[n] = fCisnWzorcowe[n] / fSredCisn1[n];
			}

			//sprawdź poprawność współczynnika aby nie zapisywać nieprawdziwych danych
			fdCisn = fabs(fSredCisn2[n] - fSredCisn1[n]);	//bezwzględna różnica ciśnień rzeczywistych
			fdWzorc = CisnienieBarometryczne(WYSOKOSC10PIETER, 100000, fTemp) - CisnienieBarometryczne(WYSOKOSC10PIETER, 100000 - fdCisn, fTemp);//Wzorcowa różnica ciśnień dla 27m i P0=1000 hPa

			if ((fdCisn > (0.75f * fdWzorc)) && (fdCisn < (1.25f * fdWzorc)))	//kryteruim poprawnosci to +/- 25%
			{
				 fSkaloCisn[n] = fRoboczySkaloCisn[n];										//przepisz roboczy współczynnik do globalnego tylko gdy jest poprawny
				 ZapiszFramFloat(FAH_SKALO_CISN_BEZWZGL1 + n*4, fSkaloCisn[n]);			//zapisz do FRAM
			}
			else
				chErr = ERR_ZLE_OBLICZENIA;
			uDaneCM4.dane.uRozne.f32[n+4] = fSkaloCisn[n];	//odeślij ewentualnie zmodyfikowany współczynnik globalny
		}
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Znajduje o odsyła do CM7  ekstrema wskazań magnetometru na potrzeby kalibracji przesunięcia
// Parametry: *sMag - wskaźnik na dane z 3 osi magnetometru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ZnajdzEkstremaMagnetometru(float *fMag)
{
	for (uint16_t n=0; n<3; n++)
	{
		if (*(fMag+n) < stMagn.fMin[n])
			stMagn.fMin[n] = *(fMag+n);

		if (*(fMag+n) > stMagn.fMax[n])
			stMagn.fMax[n] = *(fMag+n);

		uDaneCM4.dane.uRozne.f32[2*n+0] = stMagn.fMin[n];
		uDaneCM4.dane.uRozne.f32[2*n+1] = stMagn.fMax[n];
	}
}



////////////////////////////////////////////////////////////////////////////////
// Zeruje znalezione wcześniej ekstrema magnetometru tak aby nowy pomiar rozpoczął się od czytej sytuacji
// Parametry: nic
// Zwraca: BLAD_OK
////////////////////////////////////////////////////////////////////////////////
uint8_t ZerujEkstremaMagnetometru(void)
{
	uint8_t chErr = BLAD_OK;

	for (uint16_t n=0; n<3; n++)
	{
		stMagn.fMin[n] = 0.0f;
		stMagn.fMax[n] = 0.0f;
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Liczy i zapisuje przesunięcie zera magnetometru wg wzoru: (max + min) / 2
// Tak uzyskane przesuniecie należy odejmować od bieżących wskazań magnetometru.
// Oraz liczy skalowanie  długości wektora wg  wzoru (2 * DLUGOSC_WEKTORA) / (max - min)
// skalowanie należy mnożyć przez surowy odczyt po przesunięciu zera.
// Parametry: chMagn - indeks układu magnetometru
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszKalibracjeMagnetometru(uint8_t chMagn)
{
	uint8_t chErr = BLAD_OK;

	for (uint16_t n=0; n<3; n++)
	{
		switch (chMagn)
		{
		case MAG1:
			fPrzesMagn1[n] = (stMagn.fMax[n] + stMagn.fMin[n]) / 2;
			ZapiszFramFloat(FAH_MAGN1_PRZESX + 4*n, fPrzesMagn1[n]);
			fSkaloMagn1[n] = (2 * NOMINALNE_MAGN) / (stMagn.fMax[n] - stMagn.fMin[n]);
			ZapiszFramFloat(FAH_MAGN1_SKALOX + 4*n, fSkaloMagn1[n]);
			break;

		case MAG2:
			fPrzesMagn2[n] = (stMagn.fMax[n] + stMagn.fMin[n]) / 2;
			ZapiszFramFloat(FAH_MAGN2_PRZESX + 4*n, fPrzesMagn2[n]);
			fSkaloMagn2[n] = (2 * NOMINALNE_MAGN) / (stMagn.fMax[n] - stMagn.fMin[n]);
			ZapiszFramFloat(FAH_MAGN2_SKALOX + 4*n, fSkaloMagn2[n]);
			break;

		case MAG3:
			fPrzesMagn3[n] = (stMagn.fMax[n] + stMagn.fMin[n]) / 2;
			ZapiszFramFloat(FAH_MAGN3_PRZESX + 4*n, fPrzesMagn3[n]);
			fSkaloMagn3[n] = (2 * NOMINALNE_MAGN) / (stMagn.fMax[n] - stMagn.fMin[n]);
			ZapiszFramFloat(FAH_MAGN3_SKALOX + 4*n, fSkaloMagn3[n]);
			break;

		default: chErr = ERR_ZLE_DANE; 	break;
		}
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Znajduje o odsyła do CM7  wyniki kalibracji magnetometru
// Parametry: chMagn - indeks układu magnetometru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PobierzKalibracjeMagnetometru(uint8_t chMagn)
{
	switch (chMagn)
	{
	case MAG1:
		for (uint16_t n=0; n<3; n++)
		{
			uDaneCM4.dane.uRozne.f32[2*n+0] = fPrzesMagn1[n];
			uDaneCM4.dane.uRozne.f32[2*n+1] = fSkaloMagn1[n];
		}
		break;

	case MAG2:
		for (uint16_t n=0; n<3; n++)
		{
			uDaneCM4.dane.uRozne.f32[2*n+0] = fPrzesMagn2[n];
			uDaneCM4.dane.uRozne.f32[2*n+1] = fSkaloMagn2[n];
		}
		break;

	case MAG3:
		for (uint16_t n=0; n<3; n++)
		{
			uDaneCM4.dane.uRozne.f32[2*n+0] = fPrzesMagn3[n];
			uDaneCM4.dane.uRozne.f32[2*n+1] = fSkaloMagn3[n];
		}
		break;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje akcję termostatowania modułu sterując grzałką na MOD_IOx0
// Regulator P steruje wypełnieniem PWM o okresie 100 obiegów pętli
// Parametry:
// [we] chGniazdo - numer gniazda, w którym siedzi moduł
// [we-wy] *pchStanIOwy - wskaźnik na stan linii IO na modułach
// [we] fTemeratura - bieżąca temperatura modułu
// Zwraca: stan linii IO sterującej grzałką
////////////////////////////////////////////////////////////////////////////////
void Termostat(uint8_t chGniazdo, uint8_t* pchStanIOwy, float fTemeratura)
{
	if (chLicznikOkresuPWMTermostatu >= OKRES_PWM_TERMOSTATU)	//czy jest początek okresu PWM
	{
		chLicznikOkresuPWMTermostatu = 0;
		float fWypelnieniePWM = fWzmocRegTermostatu * (TEMPERATURA_ZADANA_TERMOSTATU - fTemeratura);

		if (fWypelnieniePWM > OKRES_PWM_TERMOSTATU)
			fWypelnieniePWM = OKRES_PWM_TERMOSTATU;
		if (fWypelnieniePWM < 0)	//nie ma akcji chłodzenia
			fWypelnieniePWM = 0;
		chWypelnieniePWM = (int8_t)fWypelnieniePWM;
	}

	if (chLicznikOkresuPWMTermostatu < chWypelnieniePWM)
	{
		//włącz grzałkę na linii Ix0 modułu
		switch (chGniazdo)
		{
		case ADR_MOD1: 	*pchStanIOwy |= MIO10;	break;
		case ADR_MOD2:	*pchStanIOwy |= MIO20;	break;
		case ADR_MOD3:	*pchStanIOwy |= MIO30;	break;
		case ADR_MOD4:	*pchStanIOwy |= MIO40;	break;
		}
	}
	else
	{
		//wyłącz grzałkę na linii Ix0 modułu
		switch (chGniazdo)
		{
		case ADR_MOD1: 	*pchStanIOwy &= ~MIO10;	break;
		case ADR_MOD2:	*pchStanIOwy &= ~MIO20;	break;
		case ADR_MOD3:	*pchStanIOwy &= ~MIO30;	break;
		case ADR_MOD4:	*pchStanIOwy &= ~MIO40;	break;
		}
	}
	chLicznikOkresuPWMTermostatu++;
}
