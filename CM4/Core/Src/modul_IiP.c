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



////////////////////////////////////////////////////////////////////////////////
// wykonuje czynności pomiarowe dla ukłądów znajdujących się na module
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujModulI2P(void)
{
	uint8_t chErr = ERR_OK;

	//odczytaj kalibrację żyroskopów
	for (uint16_t n=0; n<3; n++)
	{
		fOffsetZyro1[n] = FramDataReadFloat(FAH_ZYRO1_X_PRZES+(4*n));	//zapisz przesunięcie do FRAM jako liczbę float zajmującą 4 bajty
		fOffsetZyro2[n] = FramDataReadFloat(FAH_ZYRO2_X_PRZES+(4*n));	//zapisz przesunięcie do FRAM
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

	if ((uDaneCM4.dane.nZainicjowano & INIT_TRWA_KAL_ZYRO1) || (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KAL_ZYRO2))
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
// Parametry: chBityZyro - pole bitowe określające który z żyroskopów ma być kalibrowany
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RozpocznijKalibracjeZyro(uint8_t chBityZyro)
{
	if (chBityZyro & 0x01)
	{
		uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KAL_ZYRO1;	//włącz kalibrację
		for (uint8_t n=0; n<3; n++)
			fOffsetZyro1[n] = uDaneCM4.dane.fZyroSur1[n];
	}

	if (chBityZyro & 0x02)
	{
		uDaneCM4.dane.nZainicjowano |= INIT_TRWA_KAL_ZYRO2;	//włącz kalibrację
		for (uint8_t n=0; n<3; n++)
			fOffsetZyro2[n] = uDaneCM4.dane.fZyroSur2[n];
	}

	uDaneCM4.dane.sPostepProcesu = CZAS_KALIBRACJI_ZYROSKOPU;
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

	if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KAL_ZYRO1)
	{
		for (uint8_t n=0; n<3; n++)
			fOffsetZyro1[n] = (127 * fOffsetZyro1[n] + uDaneCM4.dane.fZyroSur1[n]) / 128;

		if (uDaneCM4.dane.sPostepProcesu == 0)
		{
			uDaneCM4.dane.nZainicjowano &= ~INIT_TRWA_KAL_ZYRO1;	//wyłącz kalibrację
			for (uint8_t n=0; n<3; n++)
				FramDataWriteFloat(FAH_ZYRO1_X_PRZES+(4*n), fOffsetZyro1[n]);	//zapisz przesunięcie do FRAM jako liczbę float zajmującą 4 bajty
		}
	}

	if (uDaneCM4.dane.nZainicjowano & INIT_TRWA_KAL_ZYRO2)
	{
		for (uint8_t n=0; n<3; n++)
			fOffsetZyro2[n] = (127 * fOffsetZyro2[n] + uDaneCM4.dane.fZyroSur2[n]) / 128;
		if (uDaneCM4.dane.sPostepProcesu == 0)
		{
			uDaneCM4.dane.nZainicjowano &= ~INIT_TRWA_KAL_ZYRO2;	//wyłącz kalibrację
			for (uint8_t n=0; n<3; n++)
				FramDataWriteFloat(FAH_ZYRO2_X_PRZES+(4*n), fOffsetZyro2[n]);	//zapisz przesunięcie do FRAM
		}
	}
	return ERR_OK;
}
