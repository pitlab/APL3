//////////////////////////////////////////////////////////////////////////////
//
// Obliczenia związane układem geodezyjnym PL-2000
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "PL-2000.h"

////////////////////////////////////////////////////////////////////////////////
// Konwersja zw współrzędnych elipsoidalnych GNSS na współrzedne płaskie PL-2000
// Parametry:
// [we] *stGnss - wskaźnik na dane z GNSS
// [wy] *stBSP - wskaźnik na dane BSP
// Zwraca: kod błędu
// czas wykonania: 584us
////////////////////////////////////////////////////////////////////////////////
uint8_t ZamieńGNSSnaPL_2000d(stGnss_t *stGnss, stBSP_t *stBSP)
{
	uint8_t cBłąd = BLAD_OK;
	uint8_t cNumerStrefy;	//w Polsce znajdują się 4 strefy o szerokości 3°: 5 dla 15°, 6 dla 18°, 7 dla 21° i 8 dla 24°
	double dRóżnicaSzerokościL;	//różnica szerokosci geograficznej od szerokosci strefy
	double dDługośćLukuPołudnika;
	double dPromieńKrzywiznyElipsoidyN;
	double dTanFi = tan(stGnss->dDlugoscGeo);
	double dTanFi2 = pow(dTanFi, 2);
	double dTanFi4 = pow(dTanFi, 4);
	double dSinFi = sin(stGnss->dDlugoscGeo);
	double dCosFi = cos(stGnss->dDlugoscGeo);
	double dCosFi3 = pow(dCosFi, 3);
	double dCosFi5 = pow(dCosFi, 5);
	double dCosFi7 = pow(dCosFi, 7);
	double dEtaKwadrat = MIMOSROD_GRS80_EP2 * pow(dCosFi, 2);

	//wybierz strefę. Przelicz szerokość na stopnie i podziel przez szerokość strefy
	cNumerStrefy = (uint8_t)(stGnss->dSzerokoscGeo * RAD2DEG / SZEROKOSC_STREFY_PL2000);
	dRóżnicaSzerokościL = stGnss->dSzerokoscGeo - (cNumerStrefy * SZEROKOSC_STREFY_PL2000 * DEG2RAD);	//wynik w radianach

	dDługośćLukuPołudnika = POLOS_WIELKA_ELIPSOIDY_GRS80 * (WSP_A0 * stGnss->dDlugoscGeo - WSP_A2 * sin(2 * stGnss->dDlugoscGeo) + WSP_A4 * sin(4 * stGnss->dDlugoscGeo) - WSP_A6 * sin(6 * stGnss->dDlugoscGeo) + WSP_A8 * sin(8 * stGnss->dDlugoscGeo));
	dPromieńKrzywiznyElipsoidyN = POLOS_WIELKA_ELIPSOIDY_GRS80 / sqrt(1 - MIMOSROD_GRS80_E2 * pow(sin(stGnss->dDlugoscGeo), 2));

	//obliczenie surowych współrzędnych X i Y
	double dX = dDługośćLukuPołudnika + (dPromieńKrzywiznyElipsoidyN * dSinFi * dCosFi / 2) * pow(dRóżnicaSzerokościL, 2) + (dPromieńKrzywiznyElipsoidyN * dSinFi * dCosFi3 / 24) *
			(5.0 - dTanFi2 + 9 * dEtaKwadrat + 4 * pow(dEtaKwadrat, 2)) * pow(dRóżnicaSzerokościL, 4) +
			dPromieńKrzywiznyElipsoidyN * dSinFi * dCosFi5 / 720 * (61.0 - 58 * dTanFi2 + dTanFi4 + 270 * dEtaKwadrat - 330 * dTanFi2 * dEtaKwadrat) * pow(dRóżnicaSzerokościL, 6);

	double dY = dPromieńKrzywiznyElipsoidyN * dRóżnicaSzerokościL * dCosFi + dPromieńKrzywiznyElipsoidyN * dCosFi3 / 6 * (1.0 - dTanFi2 + dEtaKwadrat) * pow(dRóżnicaSzerokościL, 3) +
			dPromieńKrzywiznyElipsoidyN * dCosFi5 / 120 * (5.0 - 18 * dTanFi2 + dTanFi4 + 14 * dEtaKwadrat - 58 * dTanFi2 * dEtaKwadrat) * pow(dRóżnicaSzerokościL, 5) +
			dPromieńKrzywiznyElipsoidyN * dCosFi7 / 5040 * (61.0 - 479 * dTanFi2 + 179 * dTanFi4 - pow(dTanFi, 6)) * pow(dRóżnicaSzerokościL, 7);

	//finalne współrzędne X i Y układu PL-2000
	stBSP->dXPółn = SKALA_ODWZOROWANIA_M0 * dX;
	stBSP->dYWsch = SKALA_ODWZOROWANIA_M0 * dY;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Konwersja zw współrzędnych elipsoidalnych GNSS na współrzedne płaskie PL-2000. Wersja pracujaca na liczbach float
// Parametry:
// [we] *stGnss - wskaźnik na dane z GNSS
// [wy] *stBSP - wskaźnik na dane BSP
// Zwraca: kod błędu
// czas wykonania: 83us
////////////////////////////////////////////////////////////////////////////////
uint8_t ZamieńGNSSnaPL_2000f(stGnss_t *stGnss, stBSP_t *stBSP)
{
	uint8_t cBłąd = BLAD_OK;
	uint8_t cNumerStrefy;	//w Polsce znajdują się 4 strefy o szerokości 3°: 5 dla 15°, 6 dla 18°, 7 dla 21° i 8 dla 24°
	float dRóżnicaSzerokościL;	//różnica szerokosci geograficznej od szerokosci strefy
	float dDługośćLukuPołudnika;
	float dPromieńKrzywiznyElipsoidyN;
	float dTanFi = tanf(stGnss->dDlugoscGeo);
	float dTanFi2 = powf(dTanFi, 2);
	float dTanFi4 = powf(dTanFi, 4);
	float dSinFi = sinf(stGnss->dDlugoscGeo);
	float dCosFi = cosf(stGnss->dDlugoscGeo);
	float dCosFi3 = powf(dCosFi, 3);
	float dCosFi5 = powf(dCosFi, 5);
	float dCosFi7 = powf(dCosFi, 7);
	float dEtaKwadrat = MIMOSROD_GRS80_EP2 * powf(dCosFi, 2);

	//wybierz strefę. Przelicz szerokość na stopnie i podziel przez szerokość strefy
	cNumerStrefy = (uint8_t)(stGnss->dSzerokoscGeo * RAD2DEG / SZEROKOSC_STREFY_PL2000);
	dRóżnicaSzerokościL = stGnss->dSzerokoscGeo - (cNumerStrefy * SZEROKOSC_STREFY_PL2000 * DEG2RAD);	//wynik w radianach

	dDługośćLukuPołudnika = POLOS_WIELKA_ELIPSOIDY_GRS80 * (WSP_A0 * stGnss->dDlugoscGeo - WSP_A2 * sinf(2 * stGnss->dDlugoscGeo) + WSP_A4 * sinf(4 * stGnss->dDlugoscGeo) - WSP_A6 * sinf(6 * stGnss->dDlugoscGeo) + WSP_A8 * sinf(8 * stGnss->dDlugoscGeo));
	dPromieńKrzywiznyElipsoidyN = POLOS_WIELKA_ELIPSOIDY_GRS80 / sqrtf(1 - MIMOSROD_GRS80_E2 * powf(sinf(stGnss->dDlugoscGeo), 2));

	//obliczenie surowych współrzędnych X i Y
	float dX = dDługośćLukuPołudnika + (dPromieńKrzywiznyElipsoidyN * dSinFi * dCosFi / 2) * powf(dRóżnicaSzerokościL, 2) + (dPromieńKrzywiznyElipsoidyN * dSinFi * dCosFi3 / 24) *
			(5.0 - dTanFi2 + 9 * dEtaKwadrat + 4 * powf(dEtaKwadrat, 2)) * powf(dRóżnicaSzerokościL, 4) +
			dPromieńKrzywiznyElipsoidyN * dSinFi * dCosFi5 / 720 * (61.0 - 58 * dTanFi2 + dTanFi4 + 270 * dEtaKwadrat - 330 * dTanFi2 * dEtaKwadrat) * powf(dRóżnicaSzerokościL, 6);

	float dY = dPromieńKrzywiznyElipsoidyN * dRóżnicaSzerokościL * dCosFi + dPromieńKrzywiznyElipsoidyN * dCosFi3 / 6 * (1.0 - dTanFi2 + dEtaKwadrat) * powf(dRóżnicaSzerokościL, 3) +
			dPromieńKrzywiznyElipsoidyN * dCosFi5 / 120 * (5.0 - 18 * dTanFi2 + dTanFi4 + 14 * dEtaKwadrat - 58 * dTanFi2 * dEtaKwadrat) * powf(dRóżnicaSzerokościL, 5) +
			dPromieńKrzywiznyElipsoidyN * dCosFi7 / 5040 * (61.0 - 479 * dTanFi2 + 179 * dTanFi4 - powf(dTanFi, 6)) * powf(dRóżnicaSzerokościL, 7);

	//finalne współrzędne X i Y układu PL-2000
	stBSP->fXPółn = SKALA_ODWZOROWANIA_M0 * dX;
	stBSP->fYWsch = SKALA_ODWZOROWANIA_M0 * dY;
	return cBłąd;
}
