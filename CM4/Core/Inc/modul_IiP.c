//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa modułu IiP (Inercyjny i Pneumatyczny)
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "modul_IiP.h"
#include "MS5611.h"
#include "moduly_wew.h"



extern uint8_t chStanIOwy, chStanIOwe;	//stan wejść IO modułów wewnetrznych
////////////////////////////////////////////////////////////////////////////////
// wykonuje czynności pomiarowe dla ukłądów znajdujących się na module
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaModuluIiP(uint8_t modul)
{
	uint8_t chErr;

	chErr = WyslijDaneExpandera(chStanIOwy & ~MIO22);	//ustaw adres A2 = 0 zrobiony z linii IO2 modułu
	chErr |= UstawDekoderModulow(modul);				//ustaw adres dekodera modułów, ponieważ użycie expandera przestawia adres
	UstawAdresNaModule(ADR_MIIP_MS5611);				//ustaw adres A0..1
	ObslugaMS5611();

	//UstawAdresNaModule(ADR_MIIP_LSM6DSV);				//ustaw adres A0..1
	//chErr |= WyslijDaneExpandera(chStanIOwy | MIO22);	//ustaw adres A2 = 1
	return chErr;
}
