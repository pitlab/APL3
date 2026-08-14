//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł filtra łączącego dane z wielu czujników
//
// (c) PitLab 2026
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <Kalman.h>



float fDeklinacjaMagnetyczna = DEKLINACJA_MAG;	//https://www.magnetic-declination.com/



////////////////////////////////////////////////////////////////////////////////
// Funkcja zaślepka właściwego filtra Kalmana
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t FiltrDanychIMUiWysokosci(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	for (uint8_t n=0; n<3; n++)
	{
		dane->stBSP.fKatIMU[n] = (dane->fKatIMU1[n] + dane->fKatIMU2[n]) / 2;
		dane->stBSP.fAkcel[n] = (dane->fAkcel1[n] + dane->fAkcel2[n]) / 2;
		dane->stBSP.fZyro[n] = (dane->fZyroKal1[n] + dane->fZyroKal2[n]) / 2;
		dane->stBSP.fMagne[n] = dane->fMagne1[n];
	}

	if (dane->stGnss1.fWysokoscMSL)
		dane->stBSP.fWysokoscMSL = (4 * dane->stGnss1.fWysokoscMSL + 12 * dane->fWysokoMSL[0]) / 16;
	else
		dane->stBSP.fWysokoscMSL = dane->fWysokoMSL[0];
	dane->stBSP.fWysokoscAGL = dane->fWysokoAGL[0];

	dane->stBSP.fPredkoscN = dane->stGnss1.fPredkoscN;
	dane->stBSP.fPredkoscE = dane->stGnss1.fPredkoscE;
	dane->stBSP.fPredkoscD = (7 * dane->fWariometr[0] + dane->fWariometr[1]) / 8;
	dane->stBSP.dDlugoscGeo = dane->stGnss1.dDlugoscGeo;
	dane->stBSP.dSzerokoscGeo = dane->stGnss1.dSzerokoscGeo;

	if (dane->stGnss1.fKurs != 0.0)
		dane->stBSP.fKursGeo = (dane->stGnss1.fKurs + ((dane->fKatIMU1[2] + dane->fKatIMU2[2]) / 2) + fDeklinacjaMagnetyczna) / 2;
	else
		dane->stBSP.fKursGeo = ((dane->fKatIMU1[2] + dane->fKatIMU2[2]) / 2) + fDeklinacjaMagnetyczna;

	return cBłąd;
}


////////////////////////////////////////////////////////////////////////////////
// Funkcja liczy liniowy filtr Kalmana dla wysokości, jej pierwszej pochodnej - prędkości pionowej
// oraz drugiej pochodnej - przyspieszenia w pionie.
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KalmanWysokośćPrędkośćPrzyspieszenieZ(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	float32_t fx[3];	//wektor stanu: 0=wysokość, 1=prędkość, 2=przyspieszenie
	float32_t fz[3];	//wektor pomiarów: 0=wysokość, 2=przyspieszenie
	float32_t fR[3][3];	//macierz kowariancji pomiarów

	fz[0] = dane->fWysokoAGL[0];
	fz[2] = dane->fAkcel1[2];
	fR[0][0] = 1;

	return cBłąd;
}
