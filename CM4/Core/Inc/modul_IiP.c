//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa modułu IiP (Inercyjny i Pneumatyczny)
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "modul_IiP.h"
#include "main.h"
#include "moduly_wew.h"



extern SPI_HandleTypeDef hspi2;
extern uint8_t chStanIOwy, chStanIOwe;	//stan wejść IO modułów wewnetrznych

volatile uint8_t chOdczytywanyMagnetometr;	//zmienna wskazuje który magnetometr jest odczytywany: MAG_MMC lub MAG_IIS


////////////////////////////////////////////////////////////////////////////////
// wykonuje czynności pomiarowe dla ukłądów znajdujących się na module
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaModuluIiP(uint8_t modul)
{
	uint8_t chErr;
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 80MHz a układ może pracować z prędkością max 10MHz, przy każdym dostępie przestaw dzielnik zegara na 8
	nZastanaKonfiguracja_SPI_CFG1 = hspi2.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi2.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;		//maska preskalera
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_32;

	//ustaw adres A2 = 0 zrobiony z linii Ix2 modułu
	switch (modul)
	{
	case ADR_MOD1: 	chStanIOwy &= ~MIO12;	break;
	case ADR_MOD2:	chStanIOwy &= ~MIO22;	break;
	case ADR_MOD3:	chStanIOwy &= ~MIO32;	break;
	case ADR_MOD4:	chStanIOwy &= ~MIO42;	break;
	}

	chErr = WyslijDaneExpandera(chStanIOwy);
	chErr |= UstawDekoderModulow(modul);				//ustaw adres dekodera modułów, ponieważ użycie expandera przestawia adres
	UstawAdresNaModule(ADR_MIIP_MS5611);				//ustaw adres A0..1
	chErr |= ObslugaMS5611();

	UstawAdresNaModule(ADR_MIIP_BMP581);				//ustaw adres A0..1
	chErr |= ObslugaBMP581();

	UstawAdresNaModule(ADR_MIIP_ICM42688);				//ustaw adres A0..1
	chErr |= ObslugaICM42688();

	UstawAdresNaModule(ADR_MIIP_LSM6DSV);				//ustaw adres A0..1
	chErr |= ObslugaLSM6DSV();

	//ustaw adres A2 = 1 zrobiony z linii Ix2 modułu
	switch (modul)
	{
	case ADR_MOD1: 	chStanIOwy |= MIO12;	break;
	case ADR_MOD2:	chStanIOwy |= MIO22;	break;
	case ADR_MOD3:	chStanIOwy |= MIO32;	break;
	case ADR_MOD4:	chStanIOwy |= MIO42;	break;
	}
	chErr |= WyslijDaneExpandera(chStanIOwy);
	chErr |= UstawDekoderModulow(modul);				//ustaw adres dekodera modułów, ponieważ użycie expandera przestawia adres
	UstawAdresNaModule(ADR_MIIP_GRZALKA);				//ustaw adres A0..1

	//grzałkę włącza się przez CS=0
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0

	hspi2.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróc poprzednie nastawy
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Liczy wysokość baromatryczną na podstawie ciśnienia i temperatury
// Parametry:
//  fP0 - ciśnienie na poziome odniesienia [kPa]
//  fP1 - ciśnienie na mierzonej wysokości [kPa]
//  fTemp - temperatura [°C)
// Zwraca: obliczoną wysokość
////////////////////////////////////////////////////////////////////////////////
float LiczWysokoscBaro(float fP0, float fP1, float fTemp)
{


	return 0.0;
}
