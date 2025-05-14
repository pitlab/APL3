//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł osługi telemetrii
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "telemetria.h"
#include "wymiana_CM7.h"

// Dane telemetryczne są wysyłane w zbiorczej ramce. Na początku ramki znajdują się słowa identyfikujące rodzaj przesyłanych danych gdzie kolejne bity określają rodzaj zmiennej
// Każda zmienna może mieć zdefiniowany inny okres wysyłania będący wielokrotnością 2ms (50Hz)


ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM4")))	chBuforTelemetrii[2][ROZMIAR_RAMKI_UART]);	//ramki telemetryczne: przygotowywana i wysyłana
uint8_t chOkresTelem[LICZBA_ZMIENNYCH_TELEMETRYCZNYCH];	//zmienna definiujaca okres wysyłania telemetrii
uint8_t chLicznikTelem[LICZBA_ZMIENNYCH_TELEMETRYCZNYCH];

extern unia_wymianyCM4_t uDaneCM4;

////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjalizuje zmienne używane do obsługi telemetrii
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjalizacjaTelemetrii(void)
{

}




///////////////////////////////////////////////////////////////////////////////
// Funkcja obsługuje przygotownie i wysyłkę zmiennych telemetrycznych na zewnatrz
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObslugaTelemetrii(void)
{
	uint64_t lIdnetyfikatorZmiennej, lListaZmiennych = 0;
	float fZmiennaTelem;

	for(uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
	{
		chLicznikTelem[n]--;
		if (chLicznikTelem[n] == 0)
		{
			chLicznikTelem[n] = chOkresTelem[n];
			lIdnetyfikatorZmiennej = 0x01 << n;
			lListaZmiennych |= lIdnetyfikatorZmiennej;
			fZmiennaTelem = PobierzZmiennaTele(lIdnetyfikatorZmiennej);
			WstawDoRamkiTele(fZmiennaTelem);
		}
	}
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja wstawia do bieżącego bufora telemetrii liczbę do wysłania
// Parametry: fDane - liczba do wysłania
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
float PobierzZmiennaTele(uint64_t lZmienna)
{
	float fZmiennaTelem;

	switch (lZmienna)
	{
	case TELEM1_AKCEL1X:	fZmiennaTelem = uDaneCM4.dane.fAkcel1[0];		break;
	case TELEM1_AKCEL1Y:	fZmiennaTelem = uDaneCM4.dane.fAkcel1[1];		break;
	case TELEM1_AKCEL1Z:	fZmiennaTelem = uDaneCM4.dane.fAkcel1[2];		break;
	case TELEM1_AKCEL2X:	fZmiennaTelem = uDaneCM4.dane.fAkcel2[0];		break;
	case TELEM1_AKCEL2Y:	fZmiennaTelem = uDaneCM4.dane.fAkcel2[1];		break;
	case TELEM1_AKCEL2Z:	fZmiennaTelem = uDaneCM4.dane.fAkcel2[2];		break;
	case TELEM1_ZYRO1P:		fZmiennaTelem = uDaneCM4.dane.fKatZyro1[0];		break;
	case TELEM1_ZYRO1Q:		fZmiennaTelem = uDaneCM4.dane.fKatZyro1[1];		break;
	case TELEM1_ZYRO1R:		fZmiennaTelem = uDaneCM4.dane.fKatZyro1[2];		break;
	case TELEM1_ZYRO2P:		fZmiennaTelem = uDaneCM4.dane.fKatZyro2[0];		break;
	case TELEM1_ZYRO2Q:		fZmiennaTelem = uDaneCM4.dane.fKatZyro2[1];		break;
	case TELEM1_ZYRO2R:		fZmiennaTelem = uDaneCM4.dane.fKatZyro2[2];		break;
	case TELEM1_MAGNE1X:	fZmiennaTelem = uDaneCM4.dane.fMagne1[0];		break;
	case TELEM1_MAGNE1Y:	fZmiennaTelem = uDaneCM4.dane.fMagne1[1];		break;
	case TELEM1_MAGNE1Z:	fZmiennaTelem = uDaneCM4.dane.fMagne1[2];		break;
	case TELEM1_MAGNE2X:	fZmiennaTelem = uDaneCM4.dane.fMagne2[0];		break;
	case TELEM1_MAGNE2Y:	fZmiennaTelem = uDaneCM4.dane.fMagne2[1];		break;
	case TELEM1_MAGNE2Z:	fZmiennaTelem = uDaneCM4.dane.fMagne2[2];		break;
	case TELEM1_MAGNE3X:	fZmiennaTelem = uDaneCM4.dane.fMagne3[0];		break;
	case TELEM1_MAGNE3Y:	fZmiennaTelem = uDaneCM4.dane.fMagne3[1];		break;
	case TELEM1_MAGNE3Z:	fZmiennaTelem = uDaneCM4.dane.fMagne3[2];		break;
	case TELEM1_KAT_IMU1X:	fZmiennaTelem = uDaneCM4.dane.fKatIMU1[0];		break;
	case TELEM1_KAT_IMU1Y:	fZmiennaTelem = uDaneCM4.dane.fKatIMU1[1];		break;
	case TELEM1_KAT_IMU1Z:	fZmiennaTelem = uDaneCM4.dane.fKatIMU1[2];		break;
	case TELEM1_KAT_IMU2X:	fZmiennaTelem = uDaneCM4.dane.fKatIMU2[0];		break;
	case TELEM1_KAT_IMU2Y:	fZmiennaTelem = uDaneCM4.dane.fKatIMU2[1];		break;
	case TELEM1_KAT_IMU2Z:	fZmiennaTelem = uDaneCM4.dane.fKatIMU2[2];		break;
	case TELEM1_KAT_ZYRO1X:	fZmiennaTelem = uDaneCM4.dane.fKatZyro1[0];		break;
	case TELEM1_KAT_ZYRO1Y:	fZmiennaTelem = uDaneCM4.dane.fKatZyro1[1];		break;
	case TELEM1_KAT_ZYRO1Z:	fZmiennaTelem = uDaneCM4.dane.fKatZyro1[2];		break;

	case TELEM2_CISBEZW1:	fZmiennaTelem = uDaneCM4.dane.fCisnie[0];		break;
	case TELEM2_CISBEZW2:	fZmiennaTelem = uDaneCM4.dane.fCisnie[1];		break;
	case TELEM2_WYSOKOSC1:	fZmiennaTelem = uDaneCM4.dane.fWysoko[0];		break;
	case TELEM2_WYSOKOSC2:	fZmiennaTelem = uDaneCM4.dane.fWysoko[1];		break;
	case TELEM2_CISROZN1:	fZmiennaTelem = uDaneCM4.dane.fCisnRozn[0];		break;
	case TELEM2_CISROZN2:	fZmiennaTelem = uDaneCM4.dane.fCisnRozn[1];		break;
	case TELEM2_PREDIAS1:	fZmiennaTelem = uDaneCM4.dane.fPredkosc[0];		break;
	case TELEM2_PREDIAS2:	fZmiennaTelem = uDaneCM4.dane.fPredkosc[1];		break;
	case TELEM2_TEMPCIS1:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_BARO1];		break;
	case TELEM2_TEMPCIS2:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_BARO2];		break;
	case TELEM2_TEMPIMU1:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_IMU1];		break;
	case TELEM2_TEMPIMU2:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_IMU1];		break;
	case TELEM2_TEMPCISR1:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_CISR1];		break;
	case TELEM2_TEMPCISR2:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_CISR2];		break;
	}
	return fZmiennaTelem;
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja wstawia do bieżącego bufora telemetrii liczbę do wysłania
// Parametry: fDane - liczba do wysłania
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WstawDoRamkiTele(float fDane)
{

}
