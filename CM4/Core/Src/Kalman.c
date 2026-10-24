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

	/*for (uint8_t n=0; n<3; n++)
	{
		//dane->stBSP.fKatIMU[n] = (dane->fKatIMU1[n] + dane->fKatIMU2[n]) / 2;	//to robi filtr Kalmana kątów
		dane->stBSP.fAkcel[n] = (dane->fAkcel1[n] + dane->fAkcel2[n]) / 2;
		dane->stBSP.fZyro[n] = (dane->fZyroKal1[n] + dane->fZyroKal2[n]) / 2;
		dane->stBSP.fMagne[n] = dane->fMagne1[n];
	}*/

	dane->stBSP.fPrzyspieszenie[0] = dane->fAkcel1[0] - AKCEL1G * sinf(dane->fKątIMU1[0]) * sinf(dane->fKątIMU1[1]);
	dane->stBSP.fPrzyspieszenie[1] = dane->fAkcel1[1] - AKCEL1G * sinf(dane->fKątIMU1[0]) * sinf(dane->fKątIMU1[1]);
	dane->stBSP.fPrzyspieszenie[2] = dane->fAkcel1[2] - AKCEL1G * cosf(dane->fKątIMU1[0]) * cosf(dane->fKątIMU1[1]);

	/*if (dane->stGnss1.fWysokoscMSL)
		dane->stBSP.fWysokoscMSL = (4 * dane->stGnss1.fWysokoscMSL + 12 * dane->fWysokoMSL[0]) / 16;
	else
		dane->stBSP.fWysokoscMSL = dane->fWysokoMSL[0];
	dane->stBSP.fWysokoscAGL = dane->fWysokoAGL[0];*/

	dane->stBSP.fPredkoscN = dane->stGnss[0].fPredkoscN;
	dane->stBSP.fPredkoscE = dane->stGnss[0].fPredkoscE;
	//dane->stBSP.fPredkoscD = (7 * dane->fWariometr[0] + dane->fWariometr[1]) / 8;
	dane->stBSP.dDlugoscGeo = dane->stGnss[0].dDlugoscGeo;
	dane->stBSP.dSzerokoscGeo = dane->stGnss[0].dSzerokoscGeo;

	if (dane->stGnss[0].fKurs != 0.0)
		dane->stBSP.fKursGeo = (dane->stGnss[0].fKurs + ((dane->fKątIMU1[2] + dane->fKątIMU2[2]) / 2) + fDeklinacjaMagnetyczna) / 2;
	else
		dane->stBSP.fKursGeo = ((dane->fKątIMU1[2] + dane->fKątIMU2[2]) / 2) + fDeklinacjaMagnetyczna;

	return cBłąd;
}


