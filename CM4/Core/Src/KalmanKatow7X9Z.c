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
static float32_t fX[KSTAN];					//wektor stanu: 0..3=kwaternion orientacji, 4..6= błędy prędkości kątowych
static float32_t fYa[KPOMR];				//wektor innowacji akcelerometru
static float32_t fYm[KPOMR];				//wektor innowacji magnetometru
static float32_t fF[KSTAN][KSTAN];			//macierz przejścia wektora stanu
static float32_t fP[KSTAN][KSTAN];			//macierz kowariancji predykcji
static float32_t fRa[KPOMR][KPOMR];			//macierz kowariancji pomiaru wektora przyspieszenia
static float32_t fRm[KPOMR][KPOMR];			//macierz kowariancji pomiaru wektora magnetycznego
static float32_t fG[KSTAN][KPOMR];			//macierz sterująca
static float32_t fQ[KSTAN][KSTAN];			//macierz szumu procesu
static float32_t fI[KSTAN][KSTAN];			//macierz jednostkowa
static float32_t fHa[KPOMR][KSTAN];			//macierz obserwacji przyspieszenia
static float32_t fHm[KPOMR][KSTAN];			//macierz obserwacji wektora magnetycznego
static float32_t fKa[KSTAN][KPOMR];			//macierz wzmocnień Kalmana dla IMU
static float32_t fKm[KSTAN][KPOMR];			//macierz wzmocnień Kalmana dla magnetometru

//macierze robocze do przechowywania wyników pośrednich
static float32_t fPHa[KSTAN][KPOMR];		//macierz [Stan x Pomiar] na wyniki pośrednie
static float32_t fPHm[KSTAN][KPOMR];		//macierz [Stan x Pomiar] na wyniki pośrednie
static float32_t fTempSPA[KSTAN][KPOMR];	//macierz [Stan x Pomiar] na wyniki pośrednie A
static float32_t fTempPSA[KPOMR][KSTAN];	//macierz [Pomiar x Stan] na wyniki pośrednie A
static float32_t fTempPPA[KPOMR][KPOMR];	//macierz [Pomiar x Pomiar] na wyniki pośrednie A
static float32_t fTempPPB[KPOMR][KPOMR];	//macierz [Pomiar x Pomiar] na wyniki pośrednie B
static float32_t fTempSSA[KSTAN][KSTAN];	//macierz [Stan x Stan] na wyniki pośrednie A
static float32_t fTempSSB[KSTAN][KSTAN];	//macierz [Stan x Stan] na wyniki pośrednie B
static float32_t fTempSSC[KSTAN][KSTAN];	//macierz [Stan x Stan] na wyniki pośrednie C
static float32_t fTempS1A[KSTAN];			//wektor [Stan] na wyniki pośrednie A
static float32_t fTempS1B[KSTAN];			//wektor [Stan] na wyniki pośrednie B

//zmienne z przedrostkiem m oznaczają macierze (lub wektory) w formacie biblioteki ARM DSP
static arm_matrix_instance_f32 mX   = {KSTAN, 1, fX};					//wektor stanu
static arm_matrix_instance_f32 mYa  = {KPOMR, 1, fYa};					//wektor innowacji akcelerometru
static arm_matrix_instance_f32 mYm  = {KPOMR, 1, fYm};					//wektor innowacji magnetometru
static arm_matrix_instance_f32 mF   = {KSTAN, KSTAN, &fF[0][0]};		//macierz przejścia wektora stanu
static arm_matrix_instance_f32 mP   = {KSTAN, KSTAN, &fP[0][0]};		//macierz kowariancji predykcji
static arm_matrix_instance_f32 mRa  = {KPOMR, KPOMR, &fRa[0][0]};		//macierz kowariancji pomiaru przyspieszenia
static arm_matrix_instance_f32 mRm  = {KPOMR, KPOMR, &fRm[0][0]};		//macierz kowariancji pomiaru magnetometru
static arm_matrix_instance_f32 mG   = {KSTAN, KPOMR, &fG[0][0]};		//macierz sterująca
static arm_matrix_instance_f32 mQ   = {KSTAN, KSTAN, &fQ[0][0]};		//macierz szumu procesu
static arm_matrix_instance_f32 mI   = {KSTAN, KSTAN, &fI[0][0]};		//macierz jednostkowa
static arm_matrix_instance_f32 mHa  = {KPOMR, KSTAN, &fHa[0][0]};		//macierz obserwacji wektora przyspieszenia
static arm_matrix_instance_f32 mHm  = {KPOMR, KSTAN, &fHm[0][0]};		//macierz obserwacji wektora magnetycznego
static arm_matrix_instance_f32 mKa  = {KSTAN, KPOMR, &fKa[0][0]};		//macierz wzmocnień Kalmana czujnika IMU
static arm_matrix_instance_f32 mKm  = {KSTAN, KPOMR, &fKm[0][0]};		//macierz wzmocnień Kalmana czujnika magnetometru

static arm_matrix_instance_f32 mPHa = {KSTAN, KPOMR, &fPHa[0][0]};		//macierz SxPc na iloczyn P*Ha
static arm_matrix_instance_f32 mPHm = {KSTAN, KPOMR, &fPHm[0][0]};		//macierz SxPa na iloczyn P*Hm
static arm_matrix_instance_f32 mTempSPA  = {KSTAN, KPOMR, &fTempSPA[0][0]};
static arm_matrix_instance_f32 mTempPSA  = {KPOMR, KSTAN, &fTempPSA[0][0]};
static arm_matrix_instance_f32 mTempPPA = {KPOMR, KPOMR, &fTempPPA[0][0]};		//macierz Pc x Pc na wyniki pośrednie A
static arm_matrix_instance_f32 mTempPPB = {KPOMR, KPOMR, &fTempPPB[0][0]};		//macierz Pc x Pc na wyniki pośrednie B
static arm_matrix_instance_f32 mTempSSA  = {KSTAN, KSTAN, &fTempSSA[0][0]};		//macierz S x S na wyniki pośrednie A
static arm_matrix_instance_f32 mTempSSB  = {KSTAN, KSTAN, &fTempSSB[0][0]};		//macierz S x S na wyniki pośrednie B
static arm_matrix_instance_f32 mTempSSC  = {KSTAN, KSTAN, &fTempSSC[0][0]};		//macierz S x S na wyniki pośrednie C
static arm_matrix_instance_f32 mTempS1A  = {KSTAN, 1, fTempS1A};				//macierz Sx1 na wyniki pośrednie A
static arm_matrix_instance_f32 mTempS1B  = {KSTAN, 1, fTempS1B};				//macierz Sx1 na wyniki pośrednie B

float fRefMagX;	//układ odniesienia pola magnetycznego
float fRefMagY;
float fRefMagZ;



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
	fX[0] = 1.0f;	//kwaternion w
	fX[1] = 0.0f;	//kwaternion x
	fX[2] = 0.0f;	//kwaternion y
	fX[3] = 0.0f;	//kwaternion z
	fX[4] = 0.0f;	//błąd pomiaru prędkosci kątowej P
	fX[5] = 0.0f;	//błąd pomiaru prędkosci kątowej Q
	fX[6] = 0.0f;	//błąd pomiaru prędkosci kątowej R
	arm_mat_init_f32(&mX, KSTAN, 1, fX);
	arm_mat_init_f32(&mYa, KPOMR, 1, fYa);

	memset(fF, 0, sizeof(float) * KSTAN * KSTAN);	//wypełnij zerami macierz przejścia
	arm_mat_init_f32(&mF, KSTAN, KSTAN, &fF[0][0]);

	//początkowa wariancja predykcji
	memset(fP, 0, sizeof(float) * KSTAN * KSTAN);
	fP[0][0] = 0.055;
	fP[1][1] = 1.0e-2;
	fP[2][2] = 6.0e-2;
	fP[3][3] = 1.0e-2;
	fP[4][4] = 1.0e-2;
	fP[5][5] = 1.0e-2;
	fP[6][6] = 1.0e-2;
	arm_mat_init_f32(&mP, KSTAN, KSTAN, &fP[0][0]);

	//inicjalizacja macierzy wariancji procesu
	memset(fQ, 0, sizeof(float) * KSTAN * KSTAN);	//wypełnij zerami
	arm_mat_init_f32(&mQ, KSTAN, KSTAN, &fQ[0][0]);

	//inicjalizacja macierzy wariancji pomiaru akcelerometrem
	memset(fRa, 0, sizeof(float) * KPOMR * KPOMR);
	fRa[0][0] = WARIANCJA_SZUMU_ACEL;
	fRa[1][1] = WARIANCJA_SZUMU_ACEL;
	fRa[2][2] = WARIANCJA_SZUMU_ACEL;
	arm_mat_init_f32(&mRa, KPOMR, KPOMR, &fRa[0][0]);

	memset(fRm, 0, sizeof(float) * KPOMR * KPOMR);
	fRm[0][0] = WARIANCJA_SZUMU_MAGN;
	fRm[1][1] = WARIANCJA_SZUMU_MAGN;
	fRm[2][2] = WARIANCJA_SZUMU_MAGN;
	arm_mat_init_f32(&mRm, KPOMR, KPOMR, &fRm[0][0]);

	//inicjalizacja macierzy sterowania
	memset(fG, 0, sizeof(float) * KSTAN * KPOMR);	//wypełnij zerami
	arm_mat_init_f32(&mG, KSTAN, KPOMR, &fG[0][0]);

	//inicjalziacja macierzy jednostkowej
	memset(fI, 0, sizeof(float) * KSTAN * KSTAN);	//wypełnij zerami
	for (uint8_t n=0; n<KSTAN; n++)
		fI[n][n] = 1.0f;
	arm_mat_init_f32(&mI, KSTAN, KSTAN, &fG[0][0]);

	memset(fHa, 0, sizeof(float) * KPOMR * KSTAN);
	arm_mat_init_f32(&mHa, KPOMR, KSTAN, &fHa[0][0]);	//macierz obserwacji wektora przyspieszenia

	arm_mat_init_f32(&mKa, KSTAN, KPOMR, &fKa[0][0]);	//macierz wzmocnień Kalmana

	memset(fPHa, 0, sizeof(float) * KSTAN * KPOMR);
	arm_mat_init_f32(&mPHa, KSTAN, KPOMR, &fPHa[0][0]);

	memset(fTempPPA, 0, sizeof(float) * KPOMR * KPOMR);
	arm_mat_init_f32(&mTempPPA, KPOMR, KPOMR, &fTempPPA[0][0]);

	memset(fTempPPB, 0, sizeof(float) * KPOMR * KPOMR);
	arm_mat_init_f32(&mTempPPB, KPOMR, KPOMR, &fTempPPB[0][0]);

	memset(fTempSPA, 0, sizeof(float) * KSTAN * KPOMR);
	arm_mat_init_f32(&mTempSPA, KSTAN, KPOMR, &fTempSPA[0][0]);

	memset(fTempPSA, 0, sizeof(float) * KPOMR * KSTAN);
	arm_mat_init_f32(&mTempPSA, KPOMR, KSTAN, &fTempPSA[0][0]);

	memset(fTempSSA, 0, sizeof(float) * KSTAN * KSTAN);
	arm_mat_init_f32(&mTempSSA, KSTAN, KSTAN, &fTempSSA[0][0]);

	memset(fTempSSB, 0, sizeof(float) * KSTAN * KSTAN);
	arm_mat_init_f32(&mTempSSB, KSTAN, KSTAN, &fTempSSB[0][0]);

	memset(fTempSSC, 0, sizeof(float) * KSTAN * KSTAN);
	arm_mat_init_f32(&mTempSSC, KSTAN, KSTAN, &fTempSSC[0][0]);

	memset(fTempS1A, 0, sizeof(float) * KSTAN * 1);
	arm_mat_init_f32(&mTempS1A, KSTAN, 1, &fTempS1A[0]);
	memset(fTempS1B, 0, sizeof(float) * KSTAN * 1);
	arm_mat_init_f32(&mTempS1B, KSTAN, 1, &fTempS1B[0]);

	//układ odniesienia pola magnetycznego
	fRefMagX = cosf(INKLINACJA_MAG);
	fRefMagY = 0.0f;
	fRefMagZ = sinf(INKLINACJA_MAG);

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
			fOmega[n] = dane->fZyroSur1[n];		//jeżeli błąd jest niewłaściwą liczbą  to go nie używaj aby nie eskalować błędów numerycznych
		else
			fOmega[n] = dane->fZyroSur1[n] - fX[4+n];
	}

	//sprawdź czy kwaternion jest zerem
	if (fX[0] == 0)
	{
		if ((fX[1] == 0) && (fX[2] == 0) && (fX[3] == 0))
			fX[0] = 1.0f; //napraw kwaternion
	}

	//jeżeli macierz P jest NaN to ją inicjuj
	if (isnan(fP[0][0]))
		memset(fP, 0, sizeof(float) * KSTAN * KSTAN);

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
	fTempS1A[0] = fX[0] + fDeltaCzasu * 0.5f * (-fX[1] * fOmega[0] - fX[2] * fOmega[1] - fX[3] * fOmega[2]);	//kwaternion w
	fTempS1A[1] = fX[1] + fDeltaCzasu * 0.5f * ( fX[0] * fOmega[0] + fX[2] * fOmega[2] - fX[3] * fOmega[1]);	//kwaternion x
	fTempS1A[2] = fX[2] + fDeltaCzasu * 0.5f * ( fX[0] * fOmega[1] - fX[1] * fOmega[2] + fX[3] * fOmega[0]);	//kwaternion y
	fTempS1A[3] = fX[3] + fDeltaCzasu * 0.5f * ( fX[0] * fOmega[2] + fX[1] * fOmega[1] - fX[2] * fOmega[0]);	//kwaternion z
	NormalizujWektor(fTempS1A, fTempS1B, KKWAT);

	//Przepisz wynik predykcji do zmiennych wynikowych
	for (uint8_t n=0; n<KSTAN; n++)
		dane->fKalmanKataX[n] = fX[n] = fTempS1B[n];

	//oblicz kąty orientacji z kwaternionu
	dane->stBSP.fKatIMU[0] = -atan2f(2.0f * (fX[0] * fX[1] + fX[2] * fX[3]), 1 - (2.0f * (fX[1] * fX[1] + fX[2] * fX[2])));
	dane->stBSP.fKatIMU[1] = -asinf (2.0f * (fX[0] * fX[2] - fX[1] * fX[3]));
	dane->stBSP.fKatIMU[2] = -atan2f(2.0f * (fX[0] * fX[3] + fX[1] * fX[2]), 1 - (2.0f * (fX[2] * fX[2] + fX[3] * fX[3])));

	//2 Oblicz predykcję kowariancji (niepewności) nowej wartości:  P(n+1) = F * P(n) * F^T + Q
	//Obliczenie szumu procesu Q składajacego się z szumu kwaterniony na który wpływa żyroskop i szumu błędu żyroskopów
	//Najpierw szum procesu dla żyroskopu wpływającego na kwaternion: Qzyro = Gg * Qg * Gg^T
	memset(fTempPPA, 0, sizeof(float) * KPOMR * KPOMR);	//wypełnij zerami macierz roboczą Qg
	fTempPPA[0][0] = fDeltaCzasu * WARIANCJA_SZUMU_ZYRO;
	fTempPPA[1][1] = fDeltaCzasu * WARIANCJA_SZUMU_ZYRO;
	fTempPPA[2][2] = fDeltaCzasu * WARIANCJA_SZUMU_ZYRO;

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
	fQ[4][4] = fDeltaCzasu * WARIANCJA_BLEDU_ZYRO;
	fQ[5][5] = fDeltaCzasu * WARIANCJA_BLEDU_ZYRO;
	fQ[6][6] = fDeltaCzasu * WARIANCJA_BLEDU_ZYRO;


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
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaAkcelerometremFiltraKalmanaKątów7X9Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;
	float fhAcc[KPOMR];

	//sprawdź czy kwaternion jest zerem
	if (fX[0] == 0)
	{
		if ((fX[1] == 0) && (fX[2] == 0) && (fX[3] == 0))
			fX[0] = 1.0f; //napraw kwaternion
	}

	//model pomiaru akcelerometru: Ya(n) = Za - Ha(x) = Za - R(q)^T * [0 0 g]
	fhAcc[0] = AKCEL1G * 2.0f * (fX[0] * fX[3] - fX[0] * fX[2]);
	fhAcc[1] = AKCEL1G * 2.0f * (fX[0] * fX[1] + fX[2] * fX[3]);
	fhAcc[2] = AKCEL1G * (fX[0] * fX[0] - fX[1] * fX[1] - fX[2] * fX[2] + fX[3] * fX[3]);

	//Innowacja = pomiar -
	fYa[0] = dane->fAkcel1[0] - fhAcc[0];
	fYa[1] = dane->fAkcel1[1] - fhAcc[1];
	fYa[2] = dane->fAkcel1[2] - fhAcc[2];

	//Macierz obserwacji Ha  [STAN x Pomiar]
	fHa[0][0] = -2.0f * AKCEL1G * fX[2];		// ax
	fHa[0][1] =  2.0f * AKCEL1G * fX[3];
	fHa[0][2] = -2.0f * AKCEL1G * fX[0];
	fHa[0][3] =  2.0f * AKCEL1G * fX[1];
	fHa[1][0] =  2.0f * AKCEL1G * fX[1];		// ay
	fHa[1][1] =  2.0f * AKCEL1G * fX[0];
	fHa[1][2] =  2.0f * AKCEL1G * fX[3];
	fHa[1][3] =  2.0f * AKCEL1G * fX[2];
	fHa[2][0] =  2.0f * AKCEL1G * fX[0];		// az
	fHa[2][1] = -2.0f * AKCEL1G * fX[1];
	fHa[2][2] = -2.0f * AKCEL1G * fX[2];
	fHa[2][3] =  2.0f * AKCEL1G * fX[3];

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


	//Estymata_x(n) = Estymata_x(n-1) + K(n) * y(n)
	//mnożenie przez K: K(n) *  y(n)		[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mKa, &mYa, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//normalizacja kwaternionu
	NormalizujWektor(fTempS1B, fTempS1A, KKWAT);

	//przepisanie estymaty kwaternionu do wektora stanu
	for (uint8_t n=0; n<KKWAT; n++)
		fX[n] = fTempS1A[n];

	//przepisanie estymaty błędu do wektora stanu
	for (uint8_t n=KKWAT; n<(KKWAT + KPOMR); n++)
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
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaMagnetometremFiltraKalmanaKątów7X9Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;
	float fhMag[KPOMR];

	//Normalizacja wektora magnetycznego
	float fNorma = sqrtf(dane->fMagne1[0] * dane->fMagne1[0] + dane->fMagne1[1] * dane->fMagne1[1] + dane->fMagne1[2] * dane->fMagne1[2]);
	float fMx = dane->fMagne1[0] / fNorma;
	float fMy = dane->fMagne1[1] / fNorma;
	float fMz = dane->fMagne1[2] / fNorma;

	//model magnetyczny
	//hx = mx(1 − 2qy^2​ − 2qz^2​) + my ​2(qx​qy ​+ qw​qz​) + mz​ 2(qx​qz ​− qw​qy​)
	fhMag[0] = fRefMagX * (1.0f - (2 * fX[2] * fX[2]) - (2 * fX[3] * fX[3])) + fRefMagY * 2 * (fX[1] * fX[2] + fX[0] * fX[3]) + fRefMagZ * 2 * (fX[1] * fX[3] - fX[0] * fX[2]);

	//hy ​ =mx ​2(qx​qy ​− qw​qz​) + my​(1 − 2qx^2 ​− 2qz^2​) + mz​2(qy​qz ​+ qw​qx​)
	fhMag[1] = fRefMagX * 2 * (fX[1] * fX[2] - fX[0] * fX[3]) + fRefMagY * (1.0f - (2 * fX[1] * fX[1]) - (2 * fX[3] * fX[3])) + fRefMagZ * 2 * (fX[2] * fX[3] + fX[0] * fX[1]);

	//hz = mx​ 2(qx​qz ​+ qw​qy​) + my ​2(qy​qz ​− qw​qx​) + mz​(1 − 2qx^2​ − 2qy^2​)
	fhMag[2] = fRefMagX * 2 * (fX[1] * fX[3] + fX[0] * fX[2]) + fRefMagY * 2 * (fX[2] * fX[3] - fX[0] * fX[3]) + fRefMagZ * (1.0f - (2 * fX[1] * fX[1]) - (2 * fX[2] * fX[2]));

	//Innowacja: y = z - h
	fYm[0] = dane->fMagne1[0] - fhMag[0];
	fYm[1] = dane->fMagne1[1] - fhMag[1];
	fYm[2] = dane->fMagne1[2] - fhMag[2];

	//Macierz obserwacji Ha  [Pomiar x Stan]
	fHm[0][0] = 2.0f * ( fMx * fX[0] + fMy * fX[3] - fMz * fX[2]);	// Hm = dhMag/dx
	fHm[0][1] = 2.0f * ( fMx * fX[1] + fMy * fX[2] + fMz * fX[3]);
	fHm[0][2] = 2.0f * (-fMx * fX[2] - fMy * fX[1] + fMz * fX[0]);
	fHm[0][3] = 2.0f * (-fMx * fX[3] + fMy * fX[0] + fMz * fX[1]);
	fHm[0][4] = 0.0f;
	fHm[0][5] = 0.0f;
	fHm[0][6] = 0.0f;

	fHm[1][0] = 2.0f * (-fMx * fX[3] + fMy * fX[0] + fMz * fX[1]);	// my axis
	fHm[1][1] = 2.0f * ( fMx * fX[1] + fMy * fX[2] - fMz * fX[3]);
	fHm[1][2] = 2.0f * (-fMx * fX[0] + fMy * fX[3] - fMz * fX[2]);
	fHm[1][3] = 2.0f * (-fMx * fX[2] - fMy * fX[1] + fMz * fX[0]);
	fHm[1][4] = 0.0f;
	fHm[1][5] = 0.0f;
	fHm[1][6] = 0.0f;

	fHm[2][0] = 2.0f * ( fMx * fX[2] - fMy * fX[1] + fMz * fX[0]);	// mz axis
	fHm[2][1] = 2.0f * ( fMx * fX[3] + fMy * fX[0] + fMz * fX[1]);
	fHm[2][2] = 2.0f * ( fMx * fX[0] + fMy * fX[3] - fMz * fX[2]);
	fHm[2][3] = 2.0f * ( fMx * fX[1] + fMy * fX[2] + fMz * fX[3]);
	fHm[2][4] = 0.0f;
	fHm[2][5] = 0.0f;
	fHm[2][6] = 0.0f;

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


	//Estymata_x(n) = Estymata_x(n-1) + K(n) * y(n)
	//mnożenie przez K: K(n) *  y(n)		[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mKm, &mYm, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//normalizacja kwaternionu
	NormalizujWektor(fTempS1B, fTempS1A, KKWAT);

	//przepisanie estymaty kwaternionu do wektora stanu
	for (uint8_t n=0; n<KKWAT; n++)
		fX[n] = fTempS1A[n];

	//przepisanie estymaty błędu do wektora stanu
	for (uint8_t n=KKWAT; n<(KKWAT + KPOMR); n++)
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

