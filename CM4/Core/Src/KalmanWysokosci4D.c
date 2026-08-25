//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Filtr Kalmana o 4-elementowym wektorze stanu obrabiajacy dane:
// wysokości, prędkości pionowej, przyspieszenia i biasu przyspieszenia
//
// (c) PitLab 2026
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <KalmanWysokosci4D.h>

static float32_t fX[4] = {0};		//wektor stanu: 0=wysokość, 1=prędkość, 2=kinematyczne przyspieszenie Z, 3=grawitacja + bias + dryft
static float32_t fZ[2] = {0};		//wektor pomiaru: 0=wysokość, 1=przyspieszenie
static float32_t fF[4][4];			//macierz przejścia wektora stanu
static float32_t fP[4][4];			//macierz kowariancji predykcji
static float32_t fR[2][2];			//macierz kowariancji pomiaru
static float32_t fQ[4][4];			//macierz szumu procesu
static float32_t fI[4][4];			//macierz jednostkowa
static float32_t fHh[2][4];			//macierz obserwacji przyspieszenia i wysokości, przekształca jednostkę pomiaru w jednostkę wektora stanu
static float32_t fHa[2][4];			//macierz obserwacji przyspieszenia, przekształca jednostkę pomiaru w jednostkę wektora stanu
static float32_t fK[4][2];			//macierz wzmocnień kalnama
static float32_t fPHt[4][2];		//macierz 4x2 na wyniki pośrednie
static float32_t fTempM42[4][2];	//macierz 4x2 na wyniki pośrednie
static float32_t fTempM24[2][4];	//macierz 2x4 na wyniki pośrednie
static float32_t fTempM44A[4][4];
static float32_t fTempM44B[4][4];
static float32_t fTempM44C[4][4];
static float32_t fTempM22A[2][2];
static float32_t fTempM22B[2][2];
static float32_t fTempM21A[2];
static float32_t fTempM21B[2];
static float32_t fTempM41A[4];
static float32_t fTempM41B[4];

//zmienne z przedrostkiem m oznaczają macierze (lub wektory) w formacie biblioteki ARM DSP
static arm_matrix_instance_f32 mX  = {4, 1, fX};			//wektor stanu
static arm_matrix_instance_f32 mZ  = {2, 1, fZ};			//wektor pomiaru
static arm_matrix_instance_f32 mF  = {4, 4, &fF[0][0]};		//macierz przejścia wektora stanu
static arm_matrix_instance_f32 mP  = {4, 4, &fP[0][0]};		//macierz kowariancji predykcji
static arm_matrix_instance_f32 mR  = {2, 2, &fR[0][0]};		//macierz kowariancji pomiaru
static arm_matrix_instance_f32 mQ  = {4, 4, &fQ[0][0]};		//macierz szumu procesu
static arm_matrix_instance_f32 mI  = {4, 4, &fI[0][0]};		//macierz jednostkowa
static arm_matrix_instance_f32 mHh = {2, 4, &fHh[0][0]};	//macierz obserwacji wysokości
static arm_matrix_instance_f32 mHa = {2, 4, &fHa[0][0]};	//macierz obserwacji przyspieszenia
static arm_matrix_instance_f32 mK  = {4, 2, &fK[0][0]};		//macierz wzmocnień kalmana
static arm_matrix_instance_f32 mPHt = {4, 2, &fPHt[0][0]};	//macierz 4x2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM42 = {4, 2, &fTempM42[0][0]};	//macierz 4x2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM24  = {2, 4, &fTempM24[0][0]};	//macierz 2x4 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM44A = {4, 4, &fTempM44A[0][0]};	//macierz1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM44B = {4, 4, &fTempM44B[0][0]};	//macierz2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM44C = {4, 4, &fTempM44C[0][0]};	//macierz3 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM22A = {2, 2, &fTempM22A[0][0]};	//macierz 2x2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM22B = {2, 2, &fTempM22B[0][0]};	//macierz 2x2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM21A = {2, 1, fTempM21A};	//macierz 2x1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM21B = {2, 1, fTempM21B};	//macierz 2x1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM41A = {4, 1, fTempM41A};	//macierz 4x1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM41B = {4, 1, fTempM41B};	//macierz 4x1 na wyniki pośrednie

extern float fPoczątkoweBarometryczneMSL;	//wysokość MSL wyznaczona podczas inicjalizacji
uint8_t cLicznikUśredniania = LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI;


////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje liniowy filtr Kalmana dla wysokości i jej pierwszej pochodnej - prędkości pionowej
// Filtr sterowany jest drugą pochodną wysokości - przyspieszeniem w pionie.
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFiltrKalmanaWysokości4D(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//filtr jest zainicjowany dopiero wtedy gdy trafia do niego rzeczywiste dane z czujnika o niezerowej wysokosci MSL
	if (dane->cNowyPomiar & NP_WYS1)
	{
		dane->cNowyPomiar &= ~NP_WYS1;
		fZ[0] += dane->fWysokoMSL[0];	//wysokość
		fZ[1] += dane->fAkcel1[2];		//przyspieszenie bezwzględne w osi Z
		cLicznikUśredniania--;
		if (cLicznikUśredniania == 0)
		{
			fZ[0] /= LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI;
			fZ[1] /= LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI;
			dane->nZainicjowano |= INIT_KALMAN_WYSOKOSCI;
		}
		else
			return cBłąd;
	}

	//zeruj macierze i wektory
	for (uint8_t m=0; m<4; m++)
	{
		for (uint8_t n=0; n<4; n++)
		{
			fI[m][n] = 0.0f;
			fQ[m][n] = 0.0f;
			fF[m][n] = 0.0f;
			fP[m][n] = 0.0f;
		}
		fHh[0][m] = fHa[0][m] = 0.0f;
		fHh[1][m] = fHa[1][m] = 0.0f;
	}

	for (uint8_t m=0; m<2; m++)
	{
		for (uint8_t n=0; n<2; n++)
		{
			fR[m][n] = 0.0f;
		}
		fZ[m] = 0.0f;
	}

	//inicjuj pierwszy pomiar i wektor stanu
	fX[0] = fZ[0];	//wysokość
	fX[1] = 0.001;	//prędkość pionowa
	fX[2] = 0.001;	//przyspieszenie
	fX[3] = fZ[1];	//łączne przyspieszenie w osi Z: grawitacja, kinematyka, bias
	arm_mat_init_f32(&mX, 4, 1, fX);
	arm_mat_init_f32(&mZ, 2, 1, fZ);

	//wariancja jest kwadratem standardowego odchylenia pomiaru i jest rozmieszczona w głównej przekątnej macierzy
	//pozostałe pola sa kowariancją, czyli zależnością między błędami jednego pomiaru a drugiego. Zakładam że
	//błędy pomiarów są niezależne, więc kowariancja jest ustawiona na 0. W rzeczywistosci pomiar prędkości jest liczony
	//z danych czujnika wysokości więc korelacja istnieje ale na razie nie potrafię jej obliczyć
	//fR[0][0] = 0.055;	//wariancja statycznej próbki wysokości czujnika 1 [m^2]
	fR[0][0] = 0.1;
	//fR[1][1] = 6.0e-2;	//wariancja akcelerometru 1 [m^2/s^4]
	fR[1][1] = 0.04;
	arm_mat_init_f32(&mR, 2, 2, &fR[0][0]);

	//początkowa wariancja predykcji
	fP[0][0] = 0.055;
	fP[1][1] = 1.0e-2;
	fP[2][2] = 6.0e-2;
	fP[3][3] = 1.0e-2;
	arm_mat_init_f32(&mP, 4, 4, &fP[0][0]);

	//macierz przejścia oblicza wartość predykcji następnego pomiaru
	fF[0][0] = 1.0f;				//wysokość = poprzednia wysokość
	fF[0][1] = OKRES_PETLI_GLOWNEJ;	//wysokość = prędkość * dT
	fF[0][2] = powf(OKRES_PETLI_GLOWNEJ, 2) / 2;	//wysokość = przyspieszenie * dT^2/2
	fF[1][1] = 1.0f;				//prędkość = poprzednia prędkość
	fF[1][2] = OKRES_PETLI_GLOWNEJ;	//prędkość = przyspieszenie * dT
	fF[2][2] = 1.0f;				//przyspieszenie = poprzednie przyspieszenie
	fF[3][3] = 1.0f;				//bias przyspieszenia = poprzedni bias przyspieszenia
	arm_mat_init_f32(&mF, 4, 4, &fF[0][0]);

	//inicjalizacja szumu procesu. Zakładam że szum procesu zależy od podchodnej przyspieszenia, czyli zrywu
	//Q = sigma^2 * G * G^T
	arm_mat_init_f32(&mQ, 4, 4, &fQ[0][0]);

	//inicjalizacja macierzy jednostkowej
	for (uint8_t n=0; n<4; n++)
		fI[n][n] = 1.0f;
	arm_mat_init_f32(&mI, 4, 4, &fI[0][0]);

	//inicjalizacja obu macierzy obserwacji: Hh - przyspieszenia i wysokości oraz Ha - samego przyspieszenia.
	//Oba pomiary są w tych samych jednostkach co wektor stanu
	fHh[0][0] = 1.0f;
	fHh[1][2] = 1.0f;
	fHh[1][3] = 1.0f;
	arm_mat_init_f32(&mHh, 2, 4, &fHh[0][0]);

	fHa[1][2] = 1.0f;
	fHa[1][3] = 1.0f;
	arm_mat_init_f32(&mHa, 2, 4, &fHa[0][0]);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja estymuje nowe wartości wektora stanu ze etapu (n) na (n+1)
// x(n+1) = F * x(n) + w. Nie ma G * u(n) bo w tym modelu nie ma sterowania
// oraz wwykonuje predykcję kowariancji (niepewności) nowej wartości:
// P(n+1) = F * P(n) * F^T + Q
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PredykcjaFiltraKalmanaWysokości4D(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;
	float32_t fDeltaCzasu = (float32_t)dane->ndT / 1e6;

	//aktualizuj wartości macierzy F zależne od czasu
	fF[0][1] = fF[1][2] = fDeltaCzasu;	//wysokość = prędkość * dT oraz prędkość = przyspieszenie * dT
	fF[0][2] = powf(fDeltaCzasu, 2) / 2;	//wysokość = przyspieszenie * dT^2/2

	//1) Predykcja nowej estymaty wektora stanu: x(n+1) = F * x(n)
	cBłąd |= arm_mat_mult_f32(&mF, &mX, &mX);
	dane->stBSP.fWysokoscMSL = fX[0];
	dane->stBSP.fPredkoscD 	 = fX[1];
	dane->stBSP.fWysokoscAGL = dane->stBSP.fWysokoscMSL - fPoczątkoweBarometryczneMSL;

	for (uint8_t n=0; n<4; n++)
		dane->stKalmanDebug.fX[n] = fX[n];

	//2) Obliczenie niepewności nowej estymaty wektora stanu: Temp1 = F * P(n)
	cBłąd |= arm_mat_mult_f32(&mF, &mP, &mTempM44A);

	//transpozycja macierzy F: Temp2 = F^T
	cBłąd |= arm_mat_trans_f32(&mF, &mTempM44B);

	//mnożenie Temp3 = (F * P(n)) * (F^T)
	cBłąd |= arm_mat_mult_f32(&mTempM44A, &mTempM44B, &mTempM44C);

	//obliczenie szumu procesu Q. Zakładam że szum procesu zależy od wariancji zrywu akcelerometru  [m/s^3]^2 = [m^2/s^6]
	float32_t fOkresPetli = (float32_t)dane->ndT / 1e6;	//czas od ostatniego wykonania w [sekundach]
	fQ[0][0] = powf(fOkresPetli, 6) / 36 * WARIANCJA_ZRYWU_ACEL;
	fQ[0][1] = powf(fOkresPetli, 5) / 12 * WARIANCJA_ZRYWU_ACEL;
	fQ[0][2] = powf(fOkresPetli, 4) / 6  * WARIANCJA_ZRYWU_ACEL;
	fQ[1][0] = powf(fOkresPetli, 5) / 12 * WARIANCJA_ZRYWU_ACEL;
	fQ[1][1] = powf(fOkresPetli, 4) / 4  * WARIANCJA_ZRYWU_ACEL;
	fQ[1][2] = powf(fOkresPetli, 3) / 2  * WARIANCJA_ZRYWU_ACEL;
	fQ[2][0] = powf(fOkresPetli, 4) / 6  * WARIANCJA_ZRYWU_ACEL;
	fQ[2][1] = powf(fOkresPetli, 3) / 2  * WARIANCJA_ZRYWU_ACEL;
	fQ[2][2] = powf(fOkresPetli, 2) 	 * WARIANCJA_ZRYWU_ACEL;
	fQ[0][3] = fQ[1][3] = fQ[2][3] = 0.0f;
	fQ[3][0] = fQ[3][1] = fQ[3][2] = 0.0f;
	fQ[3][3] = fOkresPetli * WARIANCJA_DRYFTU_ACEL;

	//dodaj macierz szumu Q procesu do iloczynu (F * P(n)) * (F^T) -> P
	cBłąd |= arm_mat_add_f32(&mQ, &mTempM44C, &mP);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru wysokości
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaWysokościiPrzyspieszeniaFiltraKalmanaWysokości4D(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//liczę współczynnik wzmocnienia Kalmana: mK,
	//najpierw transponowane H -> mTmpM42A	 (2x4 -> 4x2)
	cBłąd |= arm_mat_trans_f32(&mHh, &mTempM42);

	// P(n-1) * (H^T) -> mTmpM42B			(4x4 * 4x2 = 4x2)
	cBłąd |= arm_mat_mult_f32(&mP, &mTempM42, &mPHt);

	//H * (P(n-1)*H^T) -> mTempM22A			(2x4 * 4x2 = 2x2)
	cBłąd |= arm_mat_mult_f32(&mHh, &mPHt, &mTempM22A);

	//(H*P(n-1)*H^T) + R(n) -> mTempM22B	(2x2 + 2x2 = 2x2)
	cBłąd |= arm_mat_add_f32(&mTempM22A, &mR, &mTempM22B);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> mTempM22A	(2x2 -> 2x2)
	cBłąd |= arm_mat_inverse_f32(&mTempM22B, &mTempM22A);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	(4x2 * 2x2 = 4x2)
	cBłąd |= arm_mat_mult_f32(&mPHt, &mTempM22A, &mK);
	for (uint8_t n=0; n<4; n++)
		dane->stKalmanDebug.fK[n] = fK[n][0];


	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1)
	cBłąd |= arm_mat_mult_f32(&mHh, &mX, &mTempM21A);

	//pomiar
	fZ[0] = dane->fWysokoMSL[0];
	fZ[1] = dane->fAkcel1[2];	//Przyspieszenie bezwzględne osi Z

	//float32_t fInnowacjaWysokości = fZ[0] - fX[0];
	//float32_t fOdchylenieStdEstymaty = sqrtf(fP[0][0] + fR[0][0]);	//pierwiastek z sumy wariancji to odchylenie standardowe

	//sprawdzam czy wartość bezwzględna innowacji mieści sie w zakresie 3 sigma estymacji, jeżeli nie, to odrzucam taki pomiar jako niewiarygodny
	//if (fabs(fInnowacjaWysokości) < (3.0f * fOdchylenieStdEstymaty))
	{
		//innowacja: z(n) - (H*X(n-1)) ->mTempM21A		(2x1 - 2x1 = 2x1)
		cBłąd |= arm_mat_sub_f32(&mZ, &mTempM21A, &mTempM21B);

		//mnożenie przez K: K(n) * (z(n)-H*X(n-1))		(4x2 * 2x1 = 4x1)
		cBłąd |= arm_mat_mult_f32(&mK, &mTempM21B, &mTempM41A);

		//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))	(4x1 + 4x1 = 4x1)
		cBłąd |= arm_mat_add_f32(&mX, &mTempM41A, &mTempM41B);

		//przepisanie wyniku do wektora estymaty i finalnych zmiennych
		for (uint8_t n=0; n<4; n++)
			fX[n] = fTempM41B[n];
	}

	//aktualizuj zmienne wyjściowe
	dane->stBSP.fWysokoscMSL = fX[0];
	dane->stBSP.fPredkoscD 	 = fX[1];
	dane->stBSP.fWysokoscAGL = dane->stBSP.fWysokoscMSL - fPoczątkoweBarometryczneMSL;

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempM44A (4x2 * 2x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mK, &mHh, &mTempM44A);

	//odejmowanie (I - K(n) * H) -> mTempM44B
	cBłąd |= arm_mat_sub_f32(&mI, &mTempM44A, &mTempM44B);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempM44A (4x4 * 4x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mTempM44B, &mP, &mTempM44A);

	//transpozycja: (I-K(n)*H)^T-> mTempM44C
	cBłąd |= arm_mat_trans_f32(&mTempM44B, &mTempM44C);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempM44A, &mTempM44C, &mTempM44B);

	//mnożenie 	K(n) * R(n)  -> mTempM42  (4x2 * 2x2 = 4x2)
	cBłąd |= arm_mat_mult_f32(&mK, &mR, &mTempM42);

	//transpozycja K(n)^T -> mTempM24  (4x2 -> 2x4)
	cBłąd |= arm_mat_trans_f32(&mK, &mTempM24);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T)  (4x2 * 2x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mTempM42, &mTempM24, &mTempM44A);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempM44B, &mTempM44A, &mP);

	for (uint8_t n=0; n<4; n++)
		dane->stKalmanDebug.fP[n] = fP[n][n];

	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru wysokości
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaPrzyspieszeniaFiltraKalmanaWysokości4D(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//liczę współczynnik wzmocnienia Kalmana: mK,
	//najpierw transponowane H -> mTmpM42A	 (2x4 -> 4x2)
	cBłąd |= arm_mat_trans_f32(&mHa, &mTempM42);

	// P(n-1) * (H^T) -> mTmpM42B			(4x4 * 4x2 = 4x2)
	cBłąd |= arm_mat_mult_f32(&mP, &mTempM42, &mPHt);

	//H * (P(n-1)*H^T) -> mTempM22A			(2x4 * 4x2 = 2x2)
	cBłąd |= arm_mat_mult_f32(&mHh, &mPHt, &mTempM22A);

	//(H*P(n-1)*H^T) + R(n) -> mTempM22B	(2x2 + 2x2 = 2x2)
	cBłąd |= arm_mat_add_f32(&mTempM22A, &mR, &mTempM22B);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> mTempM22A	(2x2 -> 2x2)
	cBłąd |= arm_mat_inverse_f32(&mTempM22B, &mTempM22A);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	(4x2 * 2x2 = 4x2)
	cBłąd |= arm_mat_mult_f32(&mPHt, &mTempM22A, &mK);
	for (uint8_t n=0; n<4; n++)
		dane->stKalmanDebug.fK[n] = fK[n][0];

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1)
	cBłąd |= arm_mat_mult_f32(&mHa, &mX, &mTempM21A);

	//pomiar
	fZ[1] = dane->fAkcel1[2];	//Przyspieszenie bezwzględne osi Z

	float32_t fInnowacjaWysokości = fZ[0] - fX[0];
	float32_t fOdchylenieStdPomiaru = sqrtf(fP[0][0] + fR[0][0]);	//pierwiastek z sumy wariancji to odchylenie standardowe

	//sprawdzam czy wartość bezwzględna innowacji mieści sie w zakresie 3 sigma estymacji, jeżeli nie, to odrzucam taki pomiar jako niewiarygodny
	if (fabs(fInnowacjaWysokości) < (3.0f * fOdchylenieStdPomiaru))
	{
		//Uwzględnienie pomiaru: (z(n) - H * Estymata_x(n-1))
		cBłąd |= arm_mat_sub_f32(&mZ, &mTempM21A, &mTempM21B);

		//mnożenie przez K: K(n) * (z(n) - H * Estymata_x(n-1))
		cBłąd |= arm_mat_mult_f32(&mK, &mTempM21B, &mTempM41A);		//4x2 * 2x1 = 4x1

		//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
		cBłąd |= arm_mat_add_f32(&mX, &mTempM41A, &mTempM41B);

		//przepisanie wyniku do wektora estymaty i finalnych zmiennych
		fX[0] = fTempM41B[0];
		fX[1] = fTempM41B[1];
		fX[2] = fTempM41B[2];
		fX[3] = fTempM41B[3];
	}

	//aktualizuj zmienne wyjsciowe
	dane->stBSP.fWysokoscMSL = fX[0];
	dane->stBSP.fPredkoscD 	 = fX[1];
	dane->stBSP.fWysokoscAGL = dane->stBSP.fWysokoscMSL - fPoczątkoweBarometryczneMSL;

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempM44A (4x2 * 2x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mK, &mHa, &mTempM44A);

	//odejmowanie (I - K(n) * H) -> mTempM44B
	cBłąd |= arm_mat_sub_f32(&mI, &mTempM44A, &mTempM44B);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempM44A (4x4 * 4x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mTempM44B, &mP, &mTempM44A);

	//transpozycja: (I-K(n)*H)^T-> mTempM44C
	cBłąd |= arm_mat_trans_f32(&mTempM44B, &mTempM44C);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempM44A, &mTempM44C, &mTempM44B);


	//mnożenie 	K(n) * R(n)  -> mTempM42  (4x2 * 2x2 = 4x2)
	cBłąd |= arm_mat_mult_f32(&mK, &mR, &mTempM42);

	//transpozycja K(n)^T -> mTempM24  (4x2 -> 2x4)
	cBłąd |= arm_mat_trans_f32(&mK, &mTempM24);

	//mnożenie 	K(n) * R(n) * K(n)^T  (4x2 * 2x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mTempM42, &mTempM24, &mTempM44A);

	//finalne sumowanie (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempM44B, &mTempM44A, &mP);

	for (uint8_t n=0; n<4; n++)
		dane->stKalmanDebug.fP[n] = fP[n][n];
	return cBłąd;
}
