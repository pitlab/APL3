//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Filtr Kalmana obrabiajacy dane pochodzące z dwu kompletów czujników: wysokości, prędkości pionowej, i przyspieszenia
// W 5-elementowym wektorze stanu są jeszcze biasy obu przyspieszeń
// Macierz pomiaru Z ma rozmiar 6x1
// (c) PitLab 2026
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <KalmanWysokosci4X3Z.h>

#define KSTAN		5	//rozmiar wektora stanu
#define KPOMIAR		6	//rozmiar wektora pomiaru

static float32_t fX[KSTAN] = {0};			//wektor stanu: 0=wysokość, 1=prędkość, 2=kinematyczne przyspieszenie Z, 3=grawitacja + bias + dryft
static float32_t fZ[KPOMIAR] = {0};			//wektor pomiaru: 0=wysokość, 1=przyspieszenie
static float32_t fF[KPOMIAR][KSTAN];		//macierz przejścia wektora stanu
static float32_t fP[KSTAN][KSTAN];			//macierz kowariancji predykcji
static float32_t fR[KPOMIAR][KPOMIAR];		//macierz kowariancji pomiaru
static float32_t fQ[KSTAN][KSTAN];			//macierz szumu procesu
static float32_t fI[KSTAN][KSTAN];			//macierz jednostkowa
static float32_t fHh1[KPOMIAR][KSTAN];		//macierz obserwacji prędkości i wysokości z czujnika ciśnienia 1
static float32_t fHh2[KPOMIAR][KSTAN];		//macierz obserwacji prędkości i wysokości z czujnika ciśnienia 2
static float32_t fHa1[KPOMIAR][KSTAN];		//macierz obserwacji przyspieszenia z akcelerometru 1
static float32_t fHa2[KPOMIAR][KSTAN];		//macierz obserwacji przyspieszenia z akcelerometru 2
static float32_t fK[KSTAN][KPOMIAR];		//macierz wzmocnień kalnama
static float32_t fPHt[KSTAN][KPOMIAR];		//macierz na wyniki pośrednie
static float32_t fTempMSP[KSTAN][KPOMIAR];	//macierz na wyniki pośrednie
static float32_t fTempMPS[KPOMIAR][KSTAN];	//macierz na wyniki pośrednie
static float32_t fTempMSSA[KSTAN][KSTAN];
static float32_t fTempMSSB[KSTAN][KSTAN];
static float32_t fTempMSSC[KSTAN][KSTAN];
static float32_t fTempMPPA[KPOMIAR][KPOMIAR];
static float32_t fTempMPPB[KPOMIAR][KPOMIAR];
static float32_t fTempMP1A[KPOMIAR];
static float32_t fTempMP1B[KPOMIAR];
static float32_t fTempMS1A[KSTAN];			//macierz robocza [Stan, 1]
static float32_t fTempMS1B[KSTAN];			//macierz robocza [Stan, 1]

//zmienne z przedrostkiem m oznaczają macierze (lub wektory) w formacie biblioteki ARM DSP
static arm_matrix_instance_f32 mX   = {KSTAN, 1, fX};					//wektor stanu
static arm_matrix_instance_f32 mZ   = {KPOMIAR, 1, fZ};					//wektor pomiaru: wysokość, prędkość, przyspieszenie
static arm_matrix_instance_f32 mF   = {KSTAN, KSTAN, &fF[0][0]};		//macierz przejścia wektora stanu
static arm_matrix_instance_f32 mP   = {KSTAN, KSTAN, &fP[0][0]};		//macierz kowariancji predykcji
static arm_matrix_instance_f32 mR   = {KPOMIAR, KPOMIAR, &fR[0][0]};	//macierz kowariancji pomiaru
static arm_matrix_instance_f32 mQ   = {KSTAN, KSTAN, &fQ[0][0]};		//macierz szumu procesu
static arm_matrix_instance_f32 mI   = {KSTAN, KSTAN, &fI[0][0]};		//macierz jednostkowa
static arm_matrix_instance_f32 mHh1 = {KPOMIAR, KSTAN, &fHh1[0][0]};	//macierz obserwacji wysokości i prędkości czujnika 1
static arm_matrix_instance_f32 mHh2 = {KPOMIAR, KSTAN, &fHh2[0][0]};	//macierz obserwacji wysokości i prędkości czujnika 2
static arm_matrix_instance_f32 mHa1 = {KPOMIAR, KSTAN, &fHa1[0][0]};	//macierz obserwacji przyspieszenia czujnika 1
static arm_matrix_instance_f32 mHa2 = {KPOMIAR, KSTAN, &fHa2[0][0]};	//macierz obserwacji przyspieszenia czujnika 2
static arm_matrix_instance_f32 mK   = {KSTAN, KPOMIAR, &fK[0][0]};		//macierz wzmocnień kalmana
static arm_matrix_instance_f32 mPHt = {KSTAN, KPOMIAR, &fPHt[0][0]};	//macierz 4x3 na wyniki pośrednie
static arm_matrix_instance_f32 mTempMSP  = {KSTAN, KPOMIAR, &fTempMSP[0][0]};	//macierz SxP na wyniki pośrednie
static arm_matrix_instance_f32 mTempMPS  = {KPOMIAR, KSTAN, &fTempMPS[0][0]};	//macierz PxS na wyniki pośrednie
static arm_matrix_instance_f32 mTempMSSA = {KSTAN, KSTAN, &fTempMSSA[0][0]};	//macierz1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempMSSB = {KSTAN, KSTAN, &fTempMSSB[0][0]};	//macierz2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempMSSC = {KSTAN, KSTAN, &fTempMSSC[0][0]};	//macierz3 na wyniki pośrednie
static arm_matrix_instance_f32 mTempMPPA = {KPOMIAR, KPOMIAR, &fTempMPPA[0][0]};	//macierz PxP na wyniki pośrednie
static arm_matrix_instance_f32 mTempMPPB = {KPOMIAR, KPOMIAR, &fTempMPPB[0][0]};	//macierz 3x3 na wyniki pośrednie
static arm_matrix_instance_f32 mTempMP1A = {KPOMIAR, 1, fTempMP1A};		//macierz Px1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempMP1B = {KPOMIAR, 1, fTempMP1B};		//macierz Px1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempMS1A = {KSTAN, 1, fTempMS1A};		//macierz Sx1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempMS1B = {KSTAN, 1, fTempMS1B};		//macierz Sx1 na wyniki pośrednie

extern float fPoczątkoweBarometryczneMSL;	//wysokość MSL wyznaczona podczas inicjalizacji
static uint8_t cLicznikUśredniania = LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI;


////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje liniowy filtr Kalmana dla wysokości i jej pierwszej pochodnej - prędkości pionowej
// Filtr sterowany jest drugą pochodną wysokości - przyspieszeniem w pionie.
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFiltrKalmanaWysokości4X3Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//filtr jest zainicjowany dopiero wtedy gdy trafia do niego rzeczywiste dane z czujnika o niezerowej wysokosci MSL
	if (dane->cNowyPomiar & NP_WYS1)
	{
		dane->cNowyPomiar &= ~NP_WYS1;
		fZ[0] += dane->fWysokoMSL[0];	//wysokość 1
		fZ[1] += dane->fWysokoMSL[1];	//wysokość 2
		fZ[2] += dane->fWariometr[0];	//prędkość pionowa 1
		fZ[2] += dane->fWariometr[1];	//prędkość pionowa 2
		fZ[4] += dane->fAkcel1[2];		//przyspieszenie bezwzględne 1 w osi Z
		fZ[5] += dane->fAkcel2[2];		//przyspieszenie bezwzględne 2 w osi Z
		cLicznikUśredniania--;
		if (cLicznikUśredniania == 0)
		{
			for (uint8_t p=0; p<KPOMIAR; p++)
				fZ[p] /= LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI;
			dane->nZainicjowano |= INIT_KALMAN_WYSOKOSCI;
		}
		else
			return cBłąd;
	}

	//zeruj macierze i wektory
	for (uint8_t s=0; s<KSTAN; s++)
	{
		for (uint8_t n=0; n<KSTAN; n++)
		{
			fI[s][n] = 0.0f;
			fQ[s][n] = 0.0f;
			fF[s][n] = 0.0f;
			fP[s][n] = 0.0f;
		}
	}

	 i prędkości czujnika 1
	for (uint8_t p=0; p<KPOMIAR; p++)
	{
		for (uint8_t n=0; n<KPOMIAR; n++)
		{
			fR[p][n] = 0.0f;
		}

		for (uint8_t s=0; s<KSTAN; s++)
		{
			fHh1[p][s] = fHa1[p][s] = 0.0f;
			fHh2[p][s] = fHa1[p][s] = 0.0f;
		}
		fZ[p] = 0.0f;
	}

	//inicjuj pierwszy pomiar i wektor stanu
	fX[0] = (fZ[0] + fZ[1]) / 2;		//średnia wysokość
	fX[1] = (fZ[2] + fZ[3]) / 2;		//średnia prędkość pionowa
	fX[2] = 0.00001;	//przyspieszenie kinematyczne
	fX[3] = fZ[4];		//łączne przyspieszenie w osi Z: grawitacja + bias akcelerometru 1
	fX[4] = fZ[5];		//łączne przyspieszenie w osi Z: grawitacja + bias akcelerometru 2

	arm_mat_init_f32(&mX, KSTAN, 1, fX);
	arm_mat_init_f32(&mZ, KPOMIAR, 1, fZ);

	//wariancja jest kwadratem standardowego odchylenia pomiaru i jest rozmieszczona w głównej przekątnej macierzy
	//pozostałe pola sa kowariancją, czyli zależnością między błędami jednego pomiaru a drugiego. Zakładam że
	//błędy pomiarów są niezależne, więc kowariancja jest ustawiona na 0. W rzeczywistosci pomiar prędkości jest liczony
	//z danych czujnika wysokości więc korelacja istnieje ale na razie nie potrafię jej obliczyć
	fR[0][0] = 0.068;	//wariancja statycznego pomiaru wysokości czujnika 1 [m^2]
	fR[1][1] = 0.068;	//wariancja statycznego pomiaru wysokości czujnika 2 [m^2]
	fR[2][2] = 2.097;	//wariancja statycznego pomiaru prędkości wariometru 1 [m^2/s^2]
	fR[3][3] = 2.097;	//wariancja statycznego pomiaru prędkości wariometru 2 [m^2/s^2]
	fR[4][4] = 0.399;	//wariancja statycznego pomiaru przyspieszenia akcelerometru 1[m^2/s^4]
	fR[5][5] = 0.399;	//wariancja statycznego pomiaru przyspieszenia akcelerometru 2[m^2/s^4]
	arm_mat_init_f32(&mR, KPOMIAR, KPOMIAR, &fR[0][0]);

	//początkowa wariancja predykcji
	fP[0][0] = 0.055;
	fP[1][1] = 1.0e-2;
	fP[2][2] = 6.0e-2;
	fP[3][3] = 1.0e-2;
	fP[4][4] = 1.0e-2;
	arm_mat_init_f32(&mP, KSTAN, KSTAN, &fP[0][0]);

	//macierz przejścia oblicza wartość predykcji następnego pomiaru
	fF[0][0] = 1.0f;				//wysokość = poprzednia wysokość
	fF[0][1] = OKRES_PETLI_GLOWNEJ;	//wysokość = prędkość * dT
	fF[0][2] = powf(OKRES_PETLI_GLOWNEJ, 2) / 2;	//wysokość = przyspieszenie * dT^2/2
	fF[1][3] = 1.0f;				//prędkość = poprzednia prędkość
	fF[1][2] = OKRES_PETLI_GLOWNEJ;	//prędkość = przyspieszenie * dT
	fF[2][2] = 1.0f;				//przyspieszenie = poprzednie przyspieszenie
	fF[3][3] = 1.0f;				//bias przyspieszenia 1 = poprzedni bias przyspieszenia 1
	fF[4][4] = 1.0f;				//bias przyspieszenia 2 = poprzedni bias przyspieszenia 2
	arm_mat_init_f32(&mF, KSTAN, KSTAN, &fF[0][0]);

	//inicjalizacja szumu procesu. Zakładam że szum procesu zależy od podchodnej przyspieszenia, czyli zrywu
	//Q = sigma^2 * G * G^T
	arm_mat_init_f32(&mQ, KSTAN, KSTAN, &fQ[0][0]);

	//inicjalizacja macierzy jednostkowej
	for (uint8_t n=0; n<KSTAN; n++)
		fI[n][n] = 1.0f;
	arm_mat_init_f32(&mI, KSTAN, KSTAN, &fI[0][0]);

	//inicjalizacja obu macierzy obserwacji: Hh - wysokości i prędkości oraz Ha - samego przyspieszenia.
	fHh1[0][0] = 1.0f;		//wysokość obserwuje czujnik wysokości 1
	fHh1[1][2] = 1.0f;		//prędkość obserwuje wariometr 1
	arm_mat_init_f32(&mHh1, KPOMIAR, KSTAN, &fHh1[0][0]);

	fHh2[0][1] = 1.0f;		//wysokość obserwuje czujnik wysokości 2
	fHh2[1][3] = 1.0f;		//prędkość obserwuje wariometr  2
	arm_mat_init_f32(&mHh1, KPOMIAR, KSTAN, &fHh1[0][0]);

	fHa1[2][4] = 1.0f;		//przyspieszenie obserwuje oś Z akceletrometru 1
	fHa1[3][4] = 1.0f;		//bias 1 obserwuje oś Z akceletrometru 1
	arm_mat_init_f32(&mHa1, KPOMIAR, KSTAN, &fHa1[0][0]);

	fHa2[2][5] = 1.0f;		//przyspieszenie obserwuje oś Z akceletrometru 2
	fHa2[4][5] = 1.0f;		//bias 2 obserwuje oś Z akceletrometru 2
	arm_mat_init_f32(&mHa2, KPOMIAR, KSTAN, &fHa2[0][0]);
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
uint8_t PredykcjaFiltraKalmanaWysokości4X3Z(stWymianyCM4_t *dane)
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
uint8_t AktulizacjaWysokościiPrzyspieszeniaFiltraKalmanaWysokości4X3Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//liczę współczynnik wzmocnienia Kalmana: mK,
	//najpierw transponowane H -> mTmpM43	 (3x4 -> 4x3)
	cBłąd |= arm_mat_trans_f32(&mHh, &mTempM43);

	// P(n-1) * (H^T) -> mPHt				(4x4 * 4x3 = 4x3)
	cBłąd |= arm_mat_mult_f32(&mP, &mTempM43, &mPHt);

	//H * (P(n-1)*H^T) -> mTempM33A			(3x4 * 4x3 = 3x3)
	cBłąd |= arm_mat_mult_f32(&mHh, &mPHt, &mTempM33A);

	//(H*P(n-1)*H^T) + R(n) -> mTempM33B	(3x3 + 3x3 = 3x3)
	cBłąd |= arm_mat_add_f32(&mTempM33A, &mR, &mTempM33B);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> mTempM33A	(3x3 -> 3x3)
	cBłąd |= arm_mat_inverse_f32(&mTempM33B, &mTempM33A);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	(4x3 * 3x3 = 4x3)
	cBłąd |= arm_mat_mult_f32(&mPHt, &mTempM33A, &mK);
	dane->stKalmanDebug.fK[0] = fK[0][0];
	dane->stKalmanDebug.fK[1] = fK[3][0];
	dane->stKalmanDebug.fK[2] = fK[3][1];
	dane->stKalmanDebug.fK[3] = fK[3][2];

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1)
	cBłąd |= arm_mat_mult_f32(&mHh, &mX, &mTempM31A);

	//Uwzględnienie pomiaru: (z(n) - H * Estymata_x(n-1))
	fZ[0] = dane->fWysokoMSL[0];	//wysokość
	fZ[1] = dane->fWariometr[0];	//prędkość pionowa
	fZ[2] = dane->fAkcel1[2];		//Przyspieszenie bezwzględne osi Z

	//innowacja: z(n) - (H*X(n-1)) ->mTempM31B		(3x1 - 3x1 = 3x1)
	cBłąd |= arm_mat_sub_f32(&mZ, &mTempM31A, &mTempM31B);
	for (uint8_t n=0; n<3; n++)
		dane->stKalmanDebug.fP[n+1] = fTempM31B[n];	//w zmiennej fP zachowaj innowację

	//mnożenie przez K: K(n) * (z(n)-H*X(n-1))		(4x3 * 3x1 = 4x1)
	cBłąd |= arm_mat_mult_f32(&mK, &mTempM31B, &mTempM41A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))	(4x1 + 4x1 = 4x1)
	cBłąd |= arm_mat_add_f32(&mX, &mTempM41A, &mTempM41B);

	//przepisanie wyniku do wektora estymaty
	for (uint8_t n=0; n<4; n++)
		fX[n] = fTempM41B[n];

	//aktualizuj zmienne wyjściowe
	dane->stBSP.fWysokoscMSL = fX[0];
	dane->stBSP.fPredkoscD 	 = fX[1];
	dane->stBSP.fWysokoscAGL = dane->stBSP.fWysokoscMSL - fPoczątkoweBarometryczneMSL;

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempM44A (4x3 * 3x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mK, &mHh, &mTempM44A);

	//odejmowanie (I - K(n) * H) -> mTempM44B
	cBłąd |= arm_mat_sub_f32(&mI, &mTempM44A, &mTempM44B);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempM44A (4x4 * 4x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mTempM44B, &mP, &mTempM44A);

	//transpozycja: (I-K(n)*H)^T-> mTempM44C
	cBłąd |= arm_mat_trans_f32(&mTempM44B, &mTempM44C);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempM44A, &mTempM44C, &mTempM44B);

	//mnożenie 	K(n) * R(n)  -> mTempM43  (4x3 * 3x3 = 4x3)
	cBłąd |= arm_mat_mult_f32(&mK, &mR, &mTempM43);

	//transpozycja K(n)^T -> mTempM24  (4x3 -> 3x4)
	cBłąd |= arm_mat_trans_f32(&mK, &mTempM34);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T)  (4x3 * 3x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mTempM43, &mTempM34, &mTempM44A);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempM44B, &mTempM44A, &mP);

	//for (uint8_t n=0; n<4; n++)
		//dane->stKalmanDebug.fP[n] = fP[n][n];

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
uint8_t AktulizacjaPrzyspieszeniaFiltraKalmanaWysokości4X3Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//liczę współczynnik wzmocnienia Kalmana: mK,
	//najpierw transponowane H -> mTmpM42A	 (3x4 -> 4x3)
	cBłąd |= arm_mat_trans_f32(&mHa, &mTempM43);

	// P(n-1) * (H^T) -> mTmpM43B			(4x4 * 4x3 = 4x3)
	cBłąd |= arm_mat_mult_f32(&mP, &mTempM43, &mPHt);

	//H * (P(n-1)*H^T) -> mTempM33A			(3x4 * 4x3 = 3x3)
	cBłąd |= arm_mat_mult_f32(&mHh, &mPHt, &mTempM33A);

	//(H*P(n-1)*H^T) + R(n) -> mTempM22B	(3x3 + 3x3 = 3x3)
	cBłąd |= arm_mat_add_f32(&mTempM33A, &mR, &mTempM33B);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> mTempM33A	(3x3 -> 3x3)
	cBłąd |= arm_mat_inverse_f32(&mTempM33B, &mTempM33A);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	(4x3 * 3x3 = 4x3)
	cBłąd |= arm_mat_mult_f32(&mPHt, &mTempM33A, &mK);

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1)
	cBłąd |= arm_mat_mult_f32(&mHa, &mX, &mTempM31A);

	//Uwzględnienie pomiaru: (z(n) - H * Estymata_x(n-1))
	fZ[2] = dane->fAkcel1[2];	//pomiar: Przyspieszenie bezwzględne osi Z
	cBłąd |= arm_mat_sub_f32(&mZ, &mTempM31A, &mTempM31B);

	//mnożenie przez K: K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_mult_f32(&mK, &mTempM31B, &mTempM41A);		//4x3 * 3x1 = 4x1

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_add_f32(&mX, &mTempM41A, &mTempM41B);

	//przepisanie wyniku do wektora estymaty i finalnych zmiennych
	fX[0] = fTempM41B[0];
	fX[1] = fTempM41B[1];
	fX[2] = fTempM41B[2];
	fX[3] = fTempM41B[3];


	//aktualizuj zmienne wyjsciowe
	dane->stBSP.fWysokoscMSL = fX[0];
	dane->stBSP.fPredkoscD 	 = fX[1];
	dane->stBSP.fWysokoscAGL = dane->stBSP.fWysokoscMSL - fPoczątkoweBarometryczneMSL;

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempM44A (4x3 * 3x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mK, &mHa, &mTempM44A);

	//odejmowanie (I - K(n) * H) -> mTempM44B
	cBłąd |= arm_mat_sub_f32(&mI, &mTempM44A, &mTempM44B);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempM44A (4x4 * 4x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mTempM44B, &mP, &mTempM44A);

	//transpozycja: (I-K(n)*H)^T-> mTempM44C
	cBłąd |= arm_mat_trans_f32(&mTempM44B, &mTempM44C);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempM44A, &mTempM44C, &mTempM44B);


	//mnożenie 	K(n) * R(n)  -> mTempM42  (4x3 * 3x3 = 4x3)
	cBłąd |= arm_mat_mult_f32(&mK, &mR, &mTempM43);

	//transpozycja K(n)^T -> mTempM34  (4x3 -> 3x4)
	cBłąd |= arm_mat_trans_f32(&mK, &mTempM34);

	//mnożenie 	K(n) * R(n) * K(n)^T  (4x3 * 3x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mTempM43, &mTempM34, &mTempM44A);

	//finalne sumowanie (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempM44B, &mTempM44A, &mP);
	return cBłąd;
}
