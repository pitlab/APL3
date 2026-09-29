//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Liniowy Filtr Kalmana służący do kalibracji magnetometrów.
// Posiada 18-elementowy wektor stanu zawierajacy: 3 składowe wektora pola magnetycznego, 3*3 składowe biasu osi magnetometrów
// Filtr aktualizowany jest prędkościami kątowymi z żyroskopów, oraz wektoramim magnetycznym z magnetometrów
//
// (c) PitLab 2026
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <KalmanKalibracjiMagnetometrow12X9Z.h>



//zmienne z przedrostkiem f oznaczaja macierze lub wektory na liczbach float
static float32_t fX[KMSTAN];					//wektor stanu: 0..3=kwaternion orientacji, 4..6= błędy prędkości kątowych
static float32_t fF[KMSTAN][KMSTAN];			//macierz przejścia wektora stanu
static float32_t fP[KMSTAN][KMSTAN];			//macierz kowariancji predykcji
static float32_t fQ[KMSTAN][KMSTAN];			//macierz szumu procesu
static float32_t fI[KMSTAN][KMSTAN];			//macierz jednostkowa
static float32_t fS[KMAKTU][KMAKTU];			//macierz innowacji
static float32_t fHm1[KMAKTU][KMSTAN];			//macierz obserwacji wektora magnetycznego 1
static float32_t fHm2[KMAKTU][KMSTAN];			//macierz obserwacji wektora magnetycznego 2
static float32_t fHm3[KMAKTU][KMSTAN];			//macierz obserwacji wektora magnetycznego 3
static float32_t fRm1[KMAKTU][KMAKTU];			//macierz kowariancji pomiaru wektora magnetycznego
static float32_t fRm2[KMAKTU][KMAKTU];			//macierz kowariancji pomiaru wektora magnetycznego
static float32_t fRm3[KMAKTU][KMAKTU];			//macierz kowariancji pomiaru wektora magnetycznego

static float32_t fKg[KMSTAN][KMAKTU];			//macierz wzmocnień Kalmana dla żyroskopu
static float32_t fKm[KMSTAN][KMAKTU];			//macierz wzmocnień Kalmana dla magnetometru

static float32_t fTempSS[KMSTAN][KMSTAN];		//macierz robocza stan x stan

static arm_matrix_instance_f32 mX   = {KMSTAN, 1, fX};					//wektor stanu
static arm_matrix_instance_f32 mF   = {KMSTAN, KMSTAN, &fF[0][0]};		//macierz przejścia wektora stanu
static arm_matrix_instance_f32 mP   = {KMSTAN, KMSTAN, &fP[0][0]};		//macierz kowariancji predykcji
static arm_matrix_instance_f32 mQ   = {KMSTAN, KMSTAN, &fQ[0][0]};		//macierz szumu procesu
static arm_matrix_instance_f32 mI   = {KMSTAN, KMSTAN, &fI[0][0]};		//macierz jednostkowa
static arm_matrix_instance_f32 mS   = {KMAKTU, KMAKTU, &fS[0][0]};		//macierz

static arm_matrix_instance_f32 mHm1 = {KMAKTU, KMSTAN, &fHm1[0][0]};	//macierz obserwacji wektora magnetycznego 1
static arm_matrix_instance_f32 mHm2 = {KMAKTU, KMSTAN, &fHm2[0][0]};	//macierz obserwacji wektora magnetycznego 2
static arm_matrix_instance_f32 mHm3 = {KMAKTU, KMSTAN, &fHm3[0][0]};	//macierz obserwacji wektora magnetycznego 3
static arm_matrix_instance_f32 mRm1 = {KMAKTU, KMAKTU, &fRm1[0][0]};	//macierz kowariancji pomiaru magnetometru 1
static arm_matrix_instance_f32 mRm2 = {KMAKTU, KMAKTU, &fRm2[0][0]};	//macierz kowariancji pomiaru magnetometru 2
static arm_matrix_instance_f32 mRm3 = {KMAKTU, KMAKTU, &fRm3[0][0]};	//macierz kowariancji pomiaru magnetometru 3
static arm_matrix_instance_f32 mKg  = {KMSTAN, KMAKTU, &fKg[0][0]};		//macierz wzmocnień Kalmana dla żyroskopu
static arm_matrix_instance_f32 mKm  = {KMSTAN, KMAKTU, &fKm[0][0]};		//macierz wzmocnień Kalmana dla  magnetometru


static arm_matrix_instance_f32 mTempSS   = {KMSTAN, KMSTAN, &fTempSS[0][0]};		//macierz robocza stan x stan


static uint16_t sLicznikUśrednianiaŻyroskopów = 0;
static float fBiasŻyro1[3];
static float fBiasŻyro2[3];
static float fPrędkośćKątowa[3];


////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje rozszerzony filtr Kalmana dla kwaternionu kątów orientacji i błędów żyroskopów
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFiltrKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//uśrednij prędkosci kątowe żyroskopów aby wyznaczyć bieżący bias.
	//Prędkość katowa z usunisętym biasem będzie punktem odniesienia obrotu żyroskopami
	if (sLicznikUśrednianiaŻyroskopów == 0)
	{
		for (uint8_t n=0; n<3; n++)
			fBiasŻyro1[n]  = fBiasŻyro2[n] = 0.0f;
	}

	for (uint8_t n=0; n<3; n++)
	{
		fBiasŻyro1[n] += dane->fZyroKal1[n];
		fBiasŻyro2[n] += dane->fZyroKal2[n];
	}

	sLicznikUśrednianiaŻyroskopów++;
	if (sLicznikUśrednianiaŻyroskopów == LICZBA_USREDNIANIA_ZYROSKOPOW)
	{
		for (uint8_t n=0; n<3; n++)
		{
			fBiasŻyro1[n] /= LICZBA_USREDNIANIA_ZYROSKOPOW;
			fBiasŻyro2[n] /= LICZBA_USREDNIANIA_ZYROSKOPOW;
		}
		sLicznikUśrednianiaŻyroskopów = 0;	//pozwala uruchomić jeszcze raz
		dane->nZainicjowano |= INIT_KALMAN_KAL_MAGN;
	}
	else
		return cBłąd;

	for (uint8_t n=0; n<3; n++)
		fPrędkośćKątowa[n] = ((dane->fZyroKal1[n] - fBiasŻyro1[n]) + ( dane->fZyroKal2[n] - fBiasŻyro2[n])) / 2;

	//inicjuj macierze
	for (uint8_t n=0; n<KMSTAN; n++)
	{
		for (uint8_t m=0; m<KMSTAN; m++)
		{
			fF[n][m] = fI[n][m] = fQ[n][m] = 0.0f;
		}
		fX[n] = 0.0f;

		for (uint8_t m=0; m<KMAKTU; m++)
			fHm1[m][n] = fHm2[m][n] = fHm3[m][n] = 0.0f;			//macierz obserwacji wektora magnetycznego 2
	}

	//macierz kowariancji pomiaru wektora magnetycznego: zera, a w głównej przekatnej wariancja szumu
	for (uint8_t n=0; n<KMAKTU; n++)
	{
		for (uint8_t m=0; m<KMAKTU; m++)
			fRm1[n][m] = fRm2[n][m] = fRm3[n][m] = 0.0f;
	}
	for (uint8_t n=0; n<KMAKTU; n++)
	{
		fRm1[n][n] = WARIANCJA_SZUMU_MAGNETOMETRU1;
		fRm2[n][n] = WARIANCJA_SZUMU_MAGNETOMETRU2;
		fRm3[n][n] = WARIANCJA_SZUMU_MAGNETOMETRU3;
	}


	//inicjuj pierwszy pomiar i wektor stanu
	fX[0] = dane->fMagne1[0];	//składowa X wektora magnetycznego
	fX[1] = dane->fMagne1[1];	//składowa Y wektora magnetycznego
	fX[2] = dane->fMagne1[2];	//składowa Z wektora magnetycznego
	arm_mat_init_f32(&mX, KMSTAN, 1, fX);


	//macierz przejścia oblicza wartość predykcji następnego stanu
	for (uint8_t n=0; n<KMSTAN; n++)
	{
		fF[n][n] = 1.0f;	//jedynki na głównej przekątnej
		fI[n][n] = 1.0f;
		fQ[n][n] = WARIANCJA_SZUMU_PROCESU;
	}

	//macierze pomiaru mają jedynki po przekątnych w różnych miejscach wektora stanu
	for (uint8_t n=0; n<KMAKTU; n++)
	{
		fHm1[n+3][n] = 1.0f;
		fHm2[n+6][n] = 1.0f;
		fHm3[n+9][n] = 1.0f;
	}

	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja estymuje nowe wartości wektora stanu ze etapu (n) na (n+1)
// x(n+1) = F * x(n) + w. Ponieważ F = I więc: x(n+1) = x(n)
// oraz wykonuje predykcję kowariancji (niepewności) nowej wartości:
// P(n+1) = F * P(n) * F^T + Q Ponieważ F = I więc: P(n+1) = P(n) + Q
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PredykcjaFiltraKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	for (uint8_t n=0; n<10; n++)
		dane->stKalmanWys.fX[n] = fX[n];

	//2) Obliczenie niepewności nowej estymaty wektora stanu

	//dodaj macierz szumu Q procesu do P(n))
	cBłąd |= arm_mat_add_f32(&mP, &mQ, &mTempSS);

	for (uint8_t n=0; n<KMSTAN; n++)
	{
		for (uint8_t m=0; m<KMSTAN; m++)
			fP[n][m] = fTempSS[n][m];
	}

	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru magnetometrem 1
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaMag11FiltraKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane)
{

}
