//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Filtr Kalmana o 7-elementowym wektorze stanu zawierajacym: 4 skłądowe kwaternionu opisującego orientację oraz 3 błędów żyroskopów
// Filtr aktualizowany jest prędkościami kątowymi z żyroskopów, wektorem przyspieszenia z akcelerometru oraz wektorem magnetycznym z magnetometru
//
// (c) PitLab 2026
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <KalmanKatow7X9Z.h>
#include "kwaterniony.h"


//Wynik mnożenia macierzy ma rozmiar wierszy drugiej x kolumn pierwszej

//zmienne z przedrostkiem f oznaczaja macierze lub wektory na liczbach float
static float32_t fX[KKSTAN];					//wektor stanu: 0..3=kwaternion orientacji, 4..6= błędy prędkości kątowych
static float32_t fYa[KKPOMR];				//wektor innowacji akcelerometru
static float32_t fYm[KKPOMR];				//wektor innowacji magnetometru
static float32_t fF[KKSTAN][KKSTAN];			//macierz przejścia wektora stanu
static float32_t fP[KKSTAN][KKSTAN];			//macierz kowariancji predykcji
static float32_t fRa[KKPOMR][KKPOMR];			//macierz kowariancji pomiaru wektora przyspieszenia
static float32_t fRm[KKPOMR][KKPOMR];			//macierz kowariancji pomiaru wektora magnetycznego
static float32_t fG[KKSTAN][KKPOMR];			//macierz sterująca
static float32_t fQ[KKSTAN][KKSTAN];			//macierz szumu procesu
static float32_t fI[KKSTAN][KKSTAN];			//macierz jednostkowa
static float32_t fHa[KKPOMR][KKSTAN];			//macierz obserwacji przyspieszenia
static float32_t fHm[KKPOMR][KKSTAN];			//macierz obserwacji wektora magnetycznego
static float32_t fKa[KKSTAN][KKPOMR];			//macierz wzmocnień Kalmana dla IMU
static float32_t fKm[KKSTAN][KKPOMR];			//macierz wzmocnień Kalmana dla magnetometru

//macierze robocze do przechowywania wyników pośrednich
static float32_t fPHa[KKSTAN][KKPOMR];		//macierz [Stan x Pomiar] na wyniki pośrednie
static float32_t fPHm[KKSTAN][KKPOMR];		//macierz [Stan x Pomiar] na wyniki pośrednie
static float32_t fTempSPA[KKSTAN][KKPOMR];	//macierz [Stan x Pomiar] na wyniki pośrednie A
static float32_t fTempPSA[KKPOMR][KKSTAN];	//macierz [Pomiar x Stan] na wyniki pośrednie A
static float32_t fTempPPA[KKPOMR][KKPOMR];	//macierz [Pomiar x Pomiar] na wyniki pośrednie A
static float32_t fTempPPB[KKPOMR][KKPOMR];	//macierz [Pomiar x Pomiar] na wyniki pośrednie B
static float32_t fTempSSA[KKSTAN][KKSTAN];	//macierz [Stan x Stan] na wyniki pośrednie A
static float32_t fTempSSB[KKSTAN][KKSTAN];	//macierz [Stan x Stan] na wyniki pośrednie B
static float32_t fTempSSC[KKSTAN][KKSTAN];	//macierz [Stan x Stan] na wyniki pośrednie C
static float32_t fTempS1A[KKSTAN];			//wektor [Stan] na wyniki pośrednie A
static float32_t fTempS1B[KKSTAN];			//wektor [Stan] na wyniki pośrednie B

//zmienne z przedrostkiem m oznaczają macierze (lub wektory) w formacie biblioteki ARM DSP
static arm_matrix_instance_f32 mX   = {KKSTAN, 1, fX};					//wektor stanu
static arm_matrix_instance_f32 mYa  = {KKPOMR, 1, fYa};					//wektor innowacji akcelerometru
static arm_matrix_instance_f32 mYm  = {KKPOMR, 1, fYm};					//wektor innowacji magnetometru
static arm_matrix_instance_f32 mF   = {KKSTAN, KKSTAN, &fF[0][0]};		//macierz przejścia wektora stanu
static arm_matrix_instance_f32 mP   = {KKSTAN, KKSTAN, &fP[0][0]};		//macierz kowariancji predykcji
static arm_matrix_instance_f32 mRa  = {KKPOMR, KKPOMR, &fRa[0][0]};		//macierz kowariancji pomiaru przyspieszenia
static arm_matrix_instance_f32 mRm  = {KKPOMR, KKPOMR, &fRm[0][0]};		//macierz kowariancji pomiaru magnetometru
static arm_matrix_instance_f32 mG   = {KKSTAN, KKPOMR, &fG[0][0]};		//macierz sterująca
static arm_matrix_instance_f32 mQ   = {KKSTAN, KKSTAN, &fQ[0][0]};		//macierz szumu procesu
static arm_matrix_instance_f32 mI   = {KKSTAN, KKSTAN, &fI[0][0]};		//macierz jednostkowa
static arm_matrix_instance_f32 mHa  = {KKPOMR, KKSTAN, &fHa[0][0]};		//macierz obserwacji wektora przyspieszenia
static arm_matrix_instance_f32 mHm  = {KKPOMR, KKSTAN, &fHm[0][0]};		//macierz obserwacji wektora magnetycznego
static arm_matrix_instance_f32 mKa  = {KKSTAN, KKPOMR, &fKa[0][0]};		//macierz wzmocnień Kalmana czujnika IMU
static arm_matrix_instance_f32 mKm  = {KKSTAN, KKPOMR, &fKm[0][0]};		//macierz wzmocnień Kalmana czujnika magnetometru

static arm_matrix_instance_f32 mPHa = {KKSTAN, KKPOMR, &fPHa[0][0]};		//macierz SxPc na iloczyn P*Ha
static arm_matrix_instance_f32 mPHm = {KKSTAN, KKPOMR, &fPHm[0][0]};		//macierz SxPa na iloczyn P*Hm
static arm_matrix_instance_f32 mTempSPA  = {KKSTAN, KKPOMR, &fTempSPA[0][0]};
static arm_matrix_instance_f32 mTempPSA  = {KKPOMR, KKSTAN, &fTempPSA[0][0]};
static arm_matrix_instance_f32 mTempPPA = {KKPOMR, KKPOMR, &fTempPPA[0][0]};		//macierz Pc x Pc na wyniki pośrednie A
static arm_matrix_instance_f32 mTempPPB = {KKPOMR, KKPOMR, &fTempPPB[0][0]};		//macierz Pc x Pc na wyniki pośrednie B
static arm_matrix_instance_f32 mTempSSA  = {KKSTAN, KKSTAN, &fTempSSA[0][0]};		//macierz S x S na wyniki pośrednie A
static arm_matrix_instance_f32 mTempSSB  = {KKSTAN, KKSTAN, &fTempSSB[0][0]};		//macierz S x S na wyniki pośrednie B
static arm_matrix_instance_f32 mTempSSC  = {KKSTAN, KKSTAN, &fTempSSC[0][0]};		//macierz S x S na wyniki pośrednie C
static arm_matrix_instance_f32 mTempS1A  = {KKSTAN, 1, fTempS1A};				//macierz Sx1 na wyniki pośrednie A
static arm_matrix_instance_f32 mTempS1B  = {KKSTAN, 1, fTempS1B};				//macierz Sx1 na wyniki pośrednie B

float fRefMagX;	//układ odniesienia pola magnetycznego
float fRefMagY;
float fRefMagZ;
float fWariancjaSzumuŻyro, fWariancjaProcesuŻyro;
float fNormaOmega;	//długość wektora prędkości kątowej


////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje rozszerzony filtr Kalmana dla kwaternionu kątów orientacji i błędów żyroskopów
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFiltrKalmanaKątów7X9Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	dane->nZainicjowano |= INIT_KALMAN_KATOW;

	//inicjuj pierwszy pomiar i wektor stanu
	KatyNaKwaternion(dane->fKatAkcel1, fX);		//przelicz kąty obliczone trygonometrią na kwaternion początkowy
	//KwaternionNaKąty(fX, fYa);		//test konwersji kąt - kwaternion - kąt

	fX[4] = 0.0f;	//błąd pomiaru prędkosci kątowej P
	fX[5] = 0.0f;	//błąd pomiaru prędkosci kątowej Q
	fX[6] = 0.0f;	//błąd pomiaru prędkosci kątowej R
	arm_mat_init_f32(&mX, KKSTAN, 1, fX);
	arm_mat_init_f32(&mYa, KKPOMR, 1, fYa);

	memset(fF, 0, sizeof(float) * KKSTAN * KKSTAN);	//wypełnij zerami macierz przejścia
	arm_mat_init_f32(&mF, KKSTAN, KKSTAN, &fF[0][0]);

	//początkowa wariancja predykcji
	memset(fP, 0, sizeof(float) * KKSTAN * KKSTAN);
	fP[0][0] = WARIANCJA_PROCESU_ZYRO;
	fP[1][1] = WARIANCJA_PROCESU_ZYRO;
	fP[2][2] = WARIANCJA_PROCESU_ZYRO;
	fP[3][3] = WARIANCJA_PROCESU_ZYRO;
	fP[4][4] = 1.0e-2;
	fP[5][5] = 1.0e-2;
	fP[6][6] = 1.0e-2;
	arm_mat_init_f32(&mP, KKSTAN, KKSTAN, &fP[0][0]);

	//inicjalizacja macierzy wariancji procesu
	memset(fQ, 0, sizeof(float) * KKSTAN * KKSTAN);	//wypełnij zerami
	arm_mat_init_f32(&mQ, KKSTAN, KKSTAN, &fQ[0][0]);

	//inicjalizacja macierzy wariancji pomiaru akcelerometrem
	memset(fRa, 0, sizeof(float) * KKPOMR * KKPOMR);
	fRa[0][0] = WARIANCJA_SZUMU_ACEL;
	fRa[1][1] = WARIANCJA_SZUMU_ACEL;
	fRa[2][2] = WARIANCJA_SZUMU_ACEL;
	arm_mat_init_f32(&mRa, KKPOMR, KKPOMR, &fRa[0][0]);

	memset(fRm, 0, sizeof(float) * KKPOMR * KKPOMR);
	fRm[0][0] = WARIANCJA_SZUMU_MAGN;
	fRm[1][1] = WARIANCJA_SZUMU_MAGN;
	fRm[2][2] = WARIANCJA_SZUMU_MAGN;
	arm_mat_init_f32(&mRm, KKPOMR, KKPOMR, &fRm[0][0]);

	//inicjalizacja macierzy sterowania
	memset(fG, 0, sizeof(float) * KKSTAN * KKPOMR);	//wypełnij zerami
	arm_mat_init_f32(&mG, KKSTAN, KKPOMR, &fG[0][0]);

	//inicjalziacja macierzy jednostkowej
	memset(fI, 0, sizeof(float) * KKSTAN * KKSTAN);	//wypełnij zerami
	for (uint8_t n=0; n<KKSTAN; n++)
		fI[n][n] = 1.0f;
	arm_mat_init_f32(&mI, KKSTAN, KKSTAN, &fI[0][0]);

	memset(fHa, 0, sizeof(float) * KKPOMR * KKSTAN);
	arm_mat_init_f32(&mHa, KKPOMR, KKSTAN, &fHa[0][0]);	//macierz obserwacji wektora przyspieszenia

	arm_mat_init_f32(&mKa, KKSTAN, KKPOMR, &fKa[0][0]);	//macierz wzmocnień Kalmana

	memset(fPHa, 0, sizeof(float) * KKSTAN * KKPOMR);
	arm_mat_init_f32(&mPHa, KKSTAN, KKPOMR, &fPHa[0][0]);

	memset(fTempPPA, 0, sizeof(float) * KKPOMR * KKPOMR);
	arm_mat_init_f32(&mTempPPA, KKPOMR, KKPOMR, &fTempPPA[0][0]);

	memset(fTempPPB, 0, sizeof(float) * KKPOMR * KKPOMR);
	arm_mat_init_f32(&mTempPPB, KKPOMR, KKPOMR, &fTempPPB[0][0]);

	memset(fTempSPA, 0, sizeof(float) * KKSTAN * KKPOMR);
	arm_mat_init_f32(&mTempSPA, KKSTAN, KKPOMR, &fTempSPA[0][0]);

	memset(fTempPSA, 0, sizeof(float) * KKPOMR * KKSTAN);
	arm_mat_init_f32(&mTempPSA, KKPOMR, KKSTAN, &fTempPSA[0][0]);

	memset(fTempSSA, 0, sizeof(float) * KKSTAN * KKSTAN);
	arm_mat_init_f32(&mTempSSA, KKSTAN, KKSTAN, &fTempSSA[0][0]);

	memset(fTempSSB, 0, sizeof(float) * KKSTAN * KKSTAN);
	arm_mat_init_f32(&mTempSSB, KKSTAN, KKSTAN, &fTempSSB[0][0]);

	memset(fTempSSC, 0, sizeof(float) * KKSTAN * KKSTAN);
	arm_mat_init_f32(&mTempSSC, KKSTAN, KKSTAN, &fTempSSC[0][0]);

	memset(fTempS1A, 0, sizeof(float) * KKSTAN * 1);
	arm_mat_init_f32(&mTempS1A, KKSTAN, 1, &fTempS1A[0]);
	memset(fTempS1B, 0, sizeof(float) * KKSTAN * 1);
	arm_mat_init_f32(&mTempS1B, KKSTAN, 1, &fTempS1B[0]);

	//układ odniesienia pola magnetycznego
	fRefMagX = cosf(INKLINACJA_MAG);
	fRefMagY = 0.0f;
	fRefMagZ = sinf(INKLINACJA_MAG);

	//Udostępnij stałe jako zmienne by móc dostrajać filtr bez rekompilacji
	fWariancjaProcesuŻyro = WARIANCJA_PROCESU_ZYRO;
	fWariancjaSzumuŻyro = WARIANCJA_SZUMU_ZYRO;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja estymuje nowe wartości wektora stanu ze etapu (n) na (n+1)
// x(n+1) = f(x, u) = w. Funkcja f buduje i normalizuje kwaternion na podstawie prędkosci kątowych żyroskopu
// oraz wykonuje predykcję kowariancji (niepewności) nowej wartości:
// P(n+1) = F * P(n) * F^T + Q
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PredykcjaFiltraKalmanaKątów7X9Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;
	float32_t fDeltaCzasu = (float32_t)dane->ndT / 1e6;
	float fOmega[3];	//rzeczywista prędkość kątowa

	//odrzuć predykcje gdy czas obiegu pętli znacznie odbiega od normy
	if ((fDeltaCzasu < 5e-6) || (fDeltaCzasu > 0.1f))
		return BLAD_ZLE_DANE;

	//1. Predykcja kwaternionu zawierajacego estymatę nowej wartości kątów orientacji
	//1.1 Obliczenie prędkości kątowej omega = omega_pomiaru - bład_dryftu_żyro
	for (uint8_t n=0; n<3; n++)
	{
		if ((isnan(fX[4+n])) || (fabs(fX[4+n]) > 2*M_PI))		//błąd dryftu żyroskopu musi być poprawną liczbą mniejszą od 2 Pi
			fOmega[n] = dane->fZyroKal1[n];		//jeżeli błąd jest niewłaściwą liczbą  to go nie używaj aby nie eskalować błędów numerycznych
		else
			//fOmega[n] = dane->fZyroSur1[n] - fX[4+n];
			fOmega[n] = dane->fZyroKal1[n] - fX[4+n];
	}

	//długość wektora prędkosci kątowej
	fNormaOmega = sqrtf(fOmega[0] * fOmega[0] + fOmega[1] * fOmega[1] + fOmega[2] * fOmega[2]);

	//sprawdź czy kwaternion jest zerem
	if (fX[0] == 0)
	{
		if ((fX[1] == 0) && (fX[2] == 0) && (fX[3] == 0))
			fX[0] = 1.0f; //napraw kwaternion
	}

	//jeżeli macierz P jest NaN to ją inicjuj
	if (isnan(fP[0][0]))
		memset(fP, 0, sizeof(float) * KKSTAN * KKSTAN);

	//Macierz F przejścia wektora stanu: F = I + 0.5 * Omega * dt - wymaga zasilenie starym kwaternionem, więc wypełniam ją jeszcze przed predykcją
	fF[0][0] =  1.0f;
	fF[0][1] = -0.5f * fOmega[0] * fDeltaCzasu;
	fF[0][2] = -0.5f * fOmega[1] * fDeltaCzasu;
	fF[0][3] = -0.5f * fOmega[2] * fDeltaCzasu;

	fF[1][0] =  0.5f * fOmega[0] * fDeltaCzasu;
	fF[1][1] =  1.0f;
	fF[1][2] =  0.5f * fOmega[2] * fDeltaCzasu;
	fF[1][3] = -0.5f * fOmega[1] * fDeltaCzasu;

	fF[2][0] =  0.5f * fOmega[1] * fDeltaCzasu;
	fF[2][1] = -0.5f * fOmega[2] * fDeltaCzasu;
	fF[2][2] =  1.0f;
	fF[2][3] =  0.5f * fOmega[0] * fDeltaCzasu;

	fF[3][0] =  0.5f * fOmega[2] * fDeltaCzasu;
	fF[3][1] =  0.5f * fOmega[1] * fDeltaCzasu;
	fF[3][2] = -0.5f * fOmega[0] * fDeltaCzasu;
	fF[3][3] =  1.0f;

	 //Pochodne q względem biasu gyro
	fF[0][4] =  0.5f * fX[1] * fDeltaCzasu;
	fF[0][5] =  0.5f * fX[2] * fDeltaCzasu;
	fF[0][6] =  0.5f * fX[3] * fDeltaCzasu;

	fF[1][4] = -0.5f * fX[0] * fDeltaCzasu;
	fF[1][5] =  0.5f * fX[3] * fDeltaCzasu;
	fF[1][6] = -0.5f * fX[2] * fDeltaCzasu;

	fF[2][4] = -0.5f * fX[3] * fDeltaCzasu;
	fF[2][5] = -0.5f * fX[0] * fDeltaCzasu;
	fF[2][6] =  0.5f * fX[1] * fDeltaCzasu;

	fF[3][4] =  0.5f * fX[2] * fDeltaCzasu;
	fF[3][5] = -0.5f * fX[1] * fDeltaCzasu;
	fF[3][6] = -0.5f * fX[0] * fDeltaCzasu;

	//Błąd jest random walk:
	fF[4][4] = 1.0f;
	fF[5][5] = 1.0f;
	fF[6][6] = 1.0f;

	//Predykcja kwaternionu
	/*fTempS1A[0] = fX[0] + fDeltaCzasu * 0.5f * (-fX[1] * fOmega[0] - fX[2] * fOmega[1] - fX[3] * fOmega[2]);	//kwaternion w:
	fTempS1A[1] = fX[1] + fDeltaCzasu * 0.5f * ( fX[0] * fOmega[0] + fX[2] * fOmega[2] - fX[3] * fOmega[1]);	//kwaternion x: qx = q0 * omega_x + qy * omega_z - qz * omega_y
	fTempS1A[2] = fX[2] + fDeltaCzasu * 0.5f * ( fX[0] * fOmega[1] - fX[1] * fOmega[2] + fX[3] * fOmega[0]);	//kwaternion y: qy = q0 * omega_y + qz * omega_x + qx * omega_z
	fTempS1A[3] = fX[3] + fDeltaCzasu * 0.5f * ( fX[0] * fOmega[2] + fX[1] * fOmega[1] - fX[2] * fOmega[0]);	//kwaternion z: qz = q0 * omega_z + qx * omega_y + qy * omega_x */

	fTempS1A[0] = fX[0] + fDeltaCzasu * 0.5f * (-fX[1] * fOmega[0] - fX[2] * fOmega[1] - fX[3] * fOmega[2]);	//kwaternion w:
	fTempS1A[1] = fX[1] + fDeltaCzasu * 0.5f * ( fX[0] * fOmega[0] + fX[2] * fOmega[2] - fX[3] * fOmega[1]);	//kwaternion x: qx = q0 * omega_x + qy * omega_z - qz * omega_y
	fTempS1A[2] = fX[2] + fDeltaCzasu * 0.5f * ( fX[0] * fOmega[1] + fX[3] * fOmega[0] - fX[1] * fOmega[2]);	//kwaternion y: qy = q0 * omega_y + qz * omega_x + qx * omega_z
	fTempS1A[3] = fX[3] + fDeltaCzasu * 0.5f * ( fX[0] * fOmega[2] + fX[1] * fOmega[1] - fX[2] * fOmega[0]);	//kwaternion z: qz = q0 * omega_z + qx * omega_y + qy * omega_x
	NormalizujWektor(fTempS1A, fTempS1B, KWATER);

	//przepisz znormalizowany kwaternion ponownie do wektora stanu X
	for (uint8_t n=0; n<KWATER; n++)
		fX[n] = fTempS1B[n];

	//Przepisz wynik predykcji do zmiennych wynikowych
	for (uint8_t n=0; n<KKSTAN; n++)
		dane->stKalmanKąta.fX[n] = fX[n];

	//oblicz kąty orientacji z kwaternionu
	dane->stBSP.fKatIMU[0] = -atan2f(2.0f * (fX[0] * fX[1] + fX[2] * fX[3]), 1 - (2.0f * (fX[1] * fX[1] + fX[2] * fX[2])));
	dane->stBSP.fKatIMU[1] = -asinf (2.0f * (fX[0] * fX[2] - fX[1] * fX[3]));
	dane->stBSP.fKatIMU[2] =  atan2f(2.0f * (fX[0] * fX[3] + fX[1] * fX[2]), 1 - (2.0f * (fX[2] * fX[2] + fX[3] * fX[3])));

	//przygotuj prędkość kątową z usunietym biasem
	for (uint8_t n=0; n<3; n++)
		dane->stBSP.fZyro[n] = dane->fZyroKal1[n] - fX[n+4];

	//2 Oblicz predykcję kowariancji (niepewności) nowej wartości:  P(n+1) = F * P(n) * F^T + Q
	//Obliczenie szumu procesu Q składajacego się z szumu kwaterniony na który wpływa żyroskop i szumu błędu żyroskopów
	//Najpierw szum procesu dla żyroskopu wpływającego na kwaternion: Qzyro = Gg * Qg * Gg^T
	memset(fTempPPA, 0, sizeof(float) * KKPOMR * KKPOMR);	//wypełnij zerami macierz roboczą Qg

	/*fTempPPA[0][0] = fDeltaCzasu * WARIANCJA_SZUMU_ZYRO;
	fTempPPA[1][1] = fDeltaCzasu * WARIANCJA_SZUMU_ZYRO;
	fTempPPA[2][2] = fDeltaCzasu * WARIANCJA_SZUMU_ZYRO;*/
	fTempPPA[0][0] = fDeltaCzasu * fWariancjaSzumuŻyro;
	fTempPPA[1][1] = fDeltaCzasu * fWariancjaSzumuŻyro;
	fTempPPA[2][2] = fDeltaCzasu * fWariancjaSzumuŻyro;

	//wpływ szumu żyroskopu na kwaternion
	fG[0][0] = -0.5f * fX[1];
	fG[0][1] = -0.5f * fX[2];
	fG[0][2] = -0.5f * fX[3];

	fG[1][0] =  0.5f * fX[0];
	fG[1][1] = -0.5f * fX[3];
	fG[1][2] =  0.5f * fX[2];

	fG[2][0] =  0.5f * fX[3];
	fG[2][1] =  0.5f * fX[0];
	fG[2][2] = -0.5f * fX[1];

	fG[3][0] = -0.5f * fX[2];
	fG[3][1] =  0.5f * fX[1];
	fG[3][2] =  0.5f * fX[0];

	//Pierwsze mnożenie Gg * Qg		 [Stan x Żyro] * [Żyro x Żyro] = [Stan x Żyro]
	cBłąd |= arm_mat_mult_f32(&mG, &mTempPPA, &mTempSPA);

	//transpozycja macierzy Gg [Stan x Żyro] => [Żyro x Stan]
	cBłąd |= arm_mat_trans_f32(&mG, &mTempPSA);

	//finalne mnożenie (Gg * Qg) * (Gg^T)  [Stan x Żyro] * [Stan x Żyro] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSPA, &mTempPSA, &mQ);

	//teraz szum procesu dla błędu żyro: wariancja dryftu błędu żyro * dt
	/*fQ[4][4] = fDeltaCzasu * WARIANCJA_PROCESU_ZYRO;
	fQ[5][5] = fDeltaCzasu * WARIANCJA_PROCESU_ZYRO;
	fQ[6][6] = fDeltaCzasu * WARIANCJA_PROCESU_ZYRO;*/
	fQ[4][4] = fDeltaCzasu * fWariancjaProcesuŻyro;
	fQ[5][5] = fDeltaCzasu * fWariancjaProcesuŻyro;
	fQ[6][6] = fDeltaCzasu * fWariancjaProcesuŻyro;

	//2.1 Mnożenie   F * P(n) 		[Stan x Stan] * [Stan x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mF, &mP, &mTempSSA);

	//2.2 Transpozycja F
	cBłąd |= arm_mat_trans_f32(&mF, &mTempSSB);

	//2.3 Mnożenie (F*P(n)) * (F^T)
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSB, &mTempSSC);

	//2.4 Dodaj macierz szumu Q procesu do iloczynu (F * P(n)) * (F^T) -> P
	cBłąd |= arm_mat_add_f32(&mQ, &mTempSSC, &mP);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru magnetometrem
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *fAkcel - wskaźnik na dane akcelerometru
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaAkcelerometremFiltraKalmanaKątów7X9Z(float *fAkcel)
{
	uint8_t cBłąd = BLAD_OK;
	float fModelAcc[KKPOMR];
	float fAccNormalizowany[KKPOMR];

	//sprawdź czy kwaternion jest zerem
	if (fX[0] == 0)
	{
		if ((fX[1] == 0) && (fX[2] == 0) && (fX[3] == 0))
			fX[0] = 1.0f; //napraw kwaternion
	}

	//model pomiaru akcelerometru: Ya(n) = Za - Ha(x) = Za - R(q)^T * [0 0 g]
	fModelAcc[0] = 2.0f * (fX[1] * fX[3] - fX[0] * fX[2]);
	fModelAcc[1] = 2.0f * (fX[0] * fX[1] + fX[2] * fX[3]);
	fModelAcc[2] = (fX[0] * fX[0] - fX[1] * fX[1] - fX[2] * fX[2] + fX[3] * fX[3]);

	//Normalizuj akcelerometr i od razu licz innowację
	float fNormaAcc = sqrtf(*(fAkcel+0) * *(fAkcel+0) + *(fAkcel+1) * *(fAkcel+1) + *(fAkcel+2) * *(fAkcel+2));
	for (uint8_t n=0; n<KKPOMR; n++)
	{
		fAccNormalizowany[n] = *(fAkcel+n) / fNormaAcc;
		fYa[n] = fAccNormalizowany[n] - fModelAcc[n];		//Innowacja = znormalizowany pomiar - model
	}

	//Macierz obserwacji Ha  [STAN x Pomiar]
	fHa[0][0] = -2.0f * fX[2];		// ax
	fHa[0][1] =  2.0f * fX[3];
	fHa[0][2] = -2.0f * fX[0];
	fHa[0][3] =  2.0f * fX[1];
	fHa[1][0] =  2.0f * fX[1];		// ay
	fHa[1][1] =  2.0f * fX[0];
	fHa[1][2] =  2.0f * fX[3];
	fHa[1][3] =  2.0f * fX[2];
	fHa[2][0] =  2.0f * fX[0];		// az
	fHa[2][1] = -2.0f * fX[1];
	fHa[2][2] = -2.0f * fX[2];
	fHa[2][3] =  2.0f * fX[3];

	//liczę współczynnik wzmocnienia Kalmana: mK,

	//najpierw transponowane H -> mTempSPc	 [Pomiar x Stan] -> [Stan x Pomiar]
	cBłąd |= arm_mat_trans_f32(&mHa, &mTempSPA);

	// P(n-1) * (H^T) -> mPHa				[Stan x Stan] * [Stan x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mP, &mTempSPA, &mPHa);

	//H * (P(n-1)*H^T) -> fTempPPcA			[Pomiar x Stan] * [Stan x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mHa, &mPHa, &mTempPPA);

	//(H*P(n-1)*H^T) + R(n) -> fTempPPcB	[Pomiar x Pomiar] + [Pomiar x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_add_f32(&mTempPPA, &mRa, &mTempPPB);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> fTempPPcA	[Pomiar x Pomiar] -> [Pomiar x Pomiar]
	cBłąd |= arm_mat_inverse_f32(&mTempPPB, &mTempPPA);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mPHa, &mTempPPA, &mKa);

	//dla odpowiednio dużych prędkosci kątowych nie licz biasu aby błędy nie przesiąkały do niego
	if (fNormaOmega > 0.1f)
	{
		for (uint8_t i = 4; i < KKSTAN; i++)
		{
			for (uint8_t j = 0; j < KKPOMR; j++)
				fKa[i][j] = 0.0f;
		}
	}

	//Estymata_x(n) = Estymata_x(n-1) + K(n) * y(n)
	//mnożenie przez K: K(n) *  y(n)		[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mKa, &mYa, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//normalizacja kwaternionu
	NormalizujWektor(fTempS1B, fTempS1A, KWATER);

	//przepisanie estymaty kwaternionu do wektora stanu
	for (uint8_t n=0; n<KWATER; n++)
		fX[n] = fTempS1A[n];

	//przepisanie estymaty błędu do wektora stanu
	for (uint8_t n=KWATER; n<(KWATER + KKPOMR); n++)
		fX[n] = fTempS1B[n];

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempSSA	 [Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mKa, &mHa, &mTempSSA);

	//odejmowanie (I - K(n) * H) -> mTempSSB
	cBłąd |= arm_mat_sub_f32(&mI, &mTempSSA, &mTempSSB);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempSSA 	[Stan x Stan] * [Stan x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSSB, &mP, &mTempSSA);

	//transpozycja: (I-K(n)*H)^T-> mTempSSC
	cBłąd |= arm_mat_trans_f32(&mTempSSB, &mTempSSC);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSC, &mTempSSB);

	//mnożenie 	K(n) * R(n)  -> mTempSPc  	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mKa, &mRa, &mTempSPA);

	//transpozycja K(n)^T -> mTempM24  		[Stan x Pomiar] -> [Pomiar x Stan]
	cBłąd |= arm_mat_trans_f32(&mKa, &mTempPSA);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T) 	 	[Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSPA, &mTempPSA, &mTempSSA);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempSSB, &mTempSSA, &mP);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru wektora magnetycznego
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *fMagne - wskaźnik składowe magnetometru
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaMagnetometremFiltraKalmanaKątów7X9Z(float *fMagne)
{
	uint8_t cBłąd = BLAD_OK;
	float fModelMag[KKPOMR];
	float fMagNormalizowany[KKPOMR];

	//model magnetyczny
	//hx = mx(1 − 2qy^2​ − 2qz^2​) + my ​2(qx​qy ​+ qw​qz​) + mz​ 2(qx​qz ​− qw​qy​)
	fModelMag[0] = fRefMagX * (1.0f - (2 * fX[2] * fX[2]) - (2 * fX[3] * fX[3])) + fRefMagY * 2 * (fX[1] * fX[2] + fX[0] * fX[3]) + fRefMagZ * 2 * (fX[1] * fX[3] - fX[0] * fX[2]);

	//hy ​ =mx ​2(qx​qy ​− qw​qz​) + my​(1 − 2qx^2 ​− 2qz^2​) + mz​2(qy​qz ​+ qw​qx​)
	fModelMag[1] = fRefMagX * 2 * (fX[1] * fX[2] - fX[0] * fX[3]) + fRefMagY * (1.0f - (2 * fX[1] * fX[1]) - (2 * fX[3] * fX[3])) + fRefMagZ * 2 * (fX[2] * fX[3] + fX[0] * fX[1]);

	//hz = mx​ 2(qx​qz ​+ qw​qy​) + my ​2(qy​qz ​− qw​qx​) + mz​(1 − 2qx^2​ − 2qy^2​)
	fModelMag[2] = fRefMagX * 2 * (fX[1] * fX[3] + fX[0] * fX[2]) + fRefMagY * 2 * (fX[2] * fX[3] - fX[0] * fX[1]) + fRefMagZ * (1.0f - (2 * fX[1] * fX[1]) - (2 * fX[2] * fX[2]));

	//Normalizacja wektora magnetycznego i innowacja
	float fNorma = sqrtf(*(fMagne+0) * *(fMagne+0) + *(fMagne+1) * *(fMagne+1) + *(fMagne+2) * *(fMagne+2));
	for (uint8_t n=0; n<KKPOMR; n++)
	{
		fMagNormalizowany[n] = *(fMagne+n) / fNorma;
		fYm[n] = fMagNormalizowany[n] - fModelMag[n];	//Innowacja: y = z - h
	}

	//Macierz obserwacji Ha  [Pomiar x Stan]
	// Wiersz 0: pochodne hx
	fHm[0][0] =  2.0f * (fRefMagY * fX[3] - fRefMagZ * fX[2]);
	fHm[0][1] =  2.0f * (fRefMagY * fX[2] + fRefMagZ * fX[3]);
	fHm[0][2] = -4.0f *  fRefMagX * fX[2] + 2.0f * fRefMagY * fX[1] - 2.0f * fRefMagZ * fX[0];
	fHm[0][3] = -4.0f *  fRefMagX * fX[3] + 2.0f * fRefMagY * fX[0] + 2.0f * fRefMagZ * fX[1];

	// Wiersz 1: pochodne hy
	fHm[1][0] = -2.0f *  fRefMagX * fX[3] + 2.0f * fRefMagZ * fX[1];
	fHm[1][1] =  2.0f *  fRefMagX * fX[2] - 4.0f * fRefMagY * fX[1] + 2.0f * fRefMagZ * fX[0];
	fHm[1][2] =  2.0f *  fRefMagX * fX[1] + 2.0f * fRefMagZ * fX[3];
	fHm[1][3] = -2.0f *  fRefMagX * fX[0] - 4.0f * fRefMagY * fX[3] + 2.0f * fRefMagZ * fX[2];

	// Wiersz 2: pochodne hz
	fHm[2][0] =  2.0f *  fRefMagX * fX[2] - 2.0f * fRefMagY * fX[1];
	fHm[2][1] =  2.0f *  fRefMagX * fX[3] - 2.0f * fRefMagY * fX[0] - 4.0f * fRefMagZ * fX[1];
	fHm[2][2] =  2.0f *  fRefMagX * fX[0] + 2.0f * fRefMagY * fX[3] - 4.0f * fRefMagZ * fX[2];
	fHm[2][3] =  2.0f *  fRefMagX * fX[1] + 2.0f * fRefMagY * fX[2];

	//liczę współczynnik wzmocnienia Kalmana: mK,
	//najpierw transponowane H -> mTempSPc	 [Pomiar x Stan] -> [Stan x Pomiar]
	cBłąd |= arm_mat_trans_f32(&mHm, &mTempSPA);

	// P(n-1) * (H^T) -> mPHa				[Stan x Stan] * [Stan x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mP, &mTempSPA, &mPHm);

	//H * (P(n-1)*H^T) -> fTempPPcA			[Pomiar x Stan] * [Stan x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mHm, &mPHm, &mTempPPA);

	//(H*P(n-1)*H^T) + R(n) -> fTempPPcB	[Pomiar x Pomiar] + [Pomiar x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_add_f32(&mTempPPA, &mRm, &mTempPPB);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> fTempPPcA	[Pomiar x Pomiar] -> [Pomiar x Pomiar]
	cBłąd |= arm_mat_inverse_f32(&mTempPPB, &mTempPPA);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mPHm, &mTempPPA, &mKm);

	//dla odpowiednio dużych prędkosci kątowych nie licz biasu aby błędy nie przesiąkały do niego
	if (fNormaOmega > 0.1f)
	{
		for (uint8_t i = 4; i < KKSTAN; i++)
		{
			for (uint8_t j = 0; j < KKPOMR; j++)
				fKm[i][j] = 0.0f;
		}
	}

	//Estymata_x(n) = Estymata_x(n-1) + K(n) * y(n)
	//mnożenie przez K: K(n) *  y(n)		[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mKm, &mYm, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//normalizacja kwaternionu
	NormalizujWektor(fTempS1B, fTempS1A, KWATER);

	//przepisanie estymaty kwaternionu do wektora stanu
	for (uint8_t n=0; n<KWATER; n++)
		fX[n] = fTempS1A[n];

	//przepisanie estymaty błędu do wektora stanu
	for (uint8_t n=KWATER; n<(KWATER + KKPOMR); n++)
		fX[n] = fTempS1B[n];

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempSSA	 [Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mKm, &mHm, &mTempSSA);

	//odejmowanie (I - K(n) * H) -> mTempSSB
	cBłąd |= arm_mat_sub_f32(&mI, &mTempSSA, &mTempSSB);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempSSA 	[Stan x Stan] * [Stan x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSSB, &mP, &mTempSSA);

	//transpozycja: (I-K(n)*H)^T-> mTempSSC
	cBłąd |= arm_mat_trans_f32(&mTempSSB, &mTempSSC);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSC, &mTempSSB);

	//mnożenie 	K(n) * R(n)  -> mTempSPc  	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mKm, &mRm, &mTempSPA);

	//transpozycja K(n)^T -> mTempM24  		[Stan x Pomiar] -> [Pomiar x Stan]
	cBłąd |= arm_mat_trans_f32(&mKm, &mTempPSA);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T) 	 	[Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSPA, &mTempPSA, &mTempSSA);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempSSB, &mTempSSA, &mP);
	return cBłąd;
}

