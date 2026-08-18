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

static float32_t fX[4];		//wektor stanu: 0=wysokość, 1=prędkość
static float32_t fZ[2];		//wektor pomiaru: 0=wysokość, 1=prędkość
//static float32_t fY[2];		//wektor innowacji
static float32_t fW[4];		//wektor niemierzalnego szumu procesu
static float32_t fF[4][4];	//macierz przejścia wektora stanu
static float32_t fP[4][4];	//macierz kowariancji predykcji
static float32_t fR[2][2];	//macierz kowariancji pomiaru
static float32_t fQ[4][4];	//macierz szumu procesu
static float32_t fI[4][4];	//macierz jednostkowa
static float32_t fHh[2][4];	//macierz obserwacji przyspieszenia i wysokości, przekształca jednostkę pomiaru w jednostkę wektora stanu
static float32_t fHa[2][4];	//macierz obserwacji przyspieszenia, przekształca jednostkę pomiaru w jednostkę wektora stanu
static float32_t fK[4][2];	//macierz wzmocnień kalnama
static float32_t fTempW1[4], fTempW2[4];	//wektory na wyniki pośrednie
static float32_t fTempM42A[4][2];	//macierz 4x2 na wyniki pośrednie
static float32_t fTempM42B[4][2];	//macierz 4x2 na wyniki pośrednie
static float32_t fTempM24[2][4];	//macierz 2x4 na wyniki pośrednie
static float32_t fTempM1[4][4], fTempM2[4][4], fTempM3[4][4], fTempM4[4][4];	//macierze na wyniki pośrednie
static float32_t fTempM22A[2][2];
static float32_t fTempM22B[2][2];
static float32_t fTempM21A[2];
static float32_t fTempM21B[2];
static float32_t fTempM41A[4];
static float32_t fTempM41B[4];

//zmienne z przedrostkiem m oznaczają macierze (lub wektory) w formacie biblioteki ARM DSP
static arm_matrix_instance_f32 mX  = {4, 1, fX};			//wektor stanu
static arm_matrix_instance_f32 mZ  = {2, 1, fZ};			//wektor pomiaru
//static arm_matrix_instance_f32 mY  = {2, 1, fY};			//wektor innowacji
static arm_matrix_instance_f32 mW  = {4, 1, &fW[0]};		//wektor szumu procesu
static arm_matrix_instance_f32 mF  = {4, 4, &fF[0][0]};		//macierz przejścia wektora stanu
static arm_matrix_instance_f32 mP  = {4, 4, &fP[0][0]};		//macierz kowariancji predykcji
static arm_matrix_instance_f32 mR  = {2, 2, &fR[0][0]};		//macierz kowariancji pomiaru
static arm_matrix_instance_f32 mQ  = {4, 4, &fQ[0][0]};		//macierz szumu procesu
static arm_matrix_instance_f32 mI  = {4, 4, &fI[0][0]};		//macierz jednostkowa
static arm_matrix_instance_f32 mHh = {2, 4, &fHh[0][0]};	//macierz obserwacji wysokości
static arm_matrix_instance_f32 mHa = {2, 4, &fHa[0][0]};	//macierz obserwacji przyspieszenia
static arm_matrix_instance_f32 mK  = {4, 2, &fK[0][0]};		//macierz wzmocnień kalmana
static arm_matrix_instance_f32 mTempW1   = {4, 1, fTempW1};	//wektor1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempW2   = {4, 1, fTempW2};	//wektor2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM42A = {4, 2, &fTempM42A[0][0]};	//macierz 4x2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM42B = {4, 2, &fTempM42B[0][0]};	//macierz 4x2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM24  = {2, 4, &fTempM24[0][0]};	//macierz 2x4 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM1   = {4, 4, &fTempM1[0][0]};	//macierz1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM2   = {4, 4, &fTempM2[0][0]};	//macierz2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM3   = {4, 4, &fTempM3[0][0]};	//macierz3 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM4   = {4, 4, &fTempM4[0][0]};	//macierz4 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM22A = {2, 2, &fTempM22A[0][0]};	//macierz 2x2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM22B = {2, 2, &fTempM22B[0][0]};	//macierz 2x2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM21A = {2, 1, fTempM21A};	//macierz 2x1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM21B = {2, 1, fTempM21B};	//macierz 2x1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM41A = {4, 1, fTempM41A};	//macierz 4x1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM41B = {4, 1, fTempM41B};	//macierz 4x1 na wyniki pośrednie

extern float fPoczątkoweBarometryczneMSL;	//wysokość MSL wyznaczona podczas inicjalizacji



////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje liniowy filtr Kalmana dla wysokości i jej pierwszej pochodnej - prędkości pionowej
// Filtr sterowany jest drugą pochodną wysokości - przyspieszeniem w pionie.
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFiltrKalmanaWysokości4D(void)
{
	uint8_t cBłąd = BLAD_OK;

	//zeruj wszystkie macierze i wektory
	for (uint8_t m=0; m<4; m++)
	{
		for (uint8_t n=0; n<4; n++)
		{
			fI[m][n] = 0.0f;
			fQ[m][n] = 0.0f;
			fF[m][n] = 0.0f;
		}
		fZ[m] = 0.0f;
		fHh[0][m] = fHa[0][m] = 0.0f;
		fHh[1][m] = fHa[1][m] = 0.0f;
	}

	for (uint8_t m=0; m<2; m++)
	{
		for (uint8_t n=0; n<2; n++)
		{
			fR[m][n] = 0.0f;
		}
	}

	//inicjuj pierwszy pomiar i wektor stanu
	fX[0] = 0.1;	//wysokość
	fX[1] = 0.01;	//prędkość pionowa
	fX[2] = 0.001;	//przyspieszenie względne w osi Z
	fX[3] = AKCEL1G;	//bias osi Z akcelerometru
	arm_mat_init_f32(&mX, 4, 1, fX);
	
	fZ[0] = 0.1;	//wysokość
	fZ[1] = 0.001;	//przyspieszenie względne w osi Z
	arm_mat_init_f32(&mZ, 2, 1, fZ);

	//wektor szumu procesu
	fW[0] = 5e-6;		//odchylenie standardowe błędu pomiaru wysokości
	fW[1] = 1e-5;		//odchylenie standardowe błędu pomiaru prędkości pionowej
	fW[2] = 1e-8;		//odchylenie standardowe błędu pomiaru przyspieszenie wzglednego w osi Z
	fW[3] = 1e-10;		//odchylenie standardowe błędu dryftu przyspieszenia

	//wariancja jest kwadratem standardowego odchylenia pomiaru i jest rozmieszczona w głównej przekątnej macierzy
	//pozostałe pola sa kowariancją, czyli zależnością między błędami jednego pomiaru a drugiego. Zakładam że
	//błędy pomiarów są niezależne, więc kowariancja jest ustawiona na 0. W rzeczywistosci pomiar prędkości jest liczony
	//z danych czujnika wysokości więc korelacja istnieje ale na razie nie potrafię jej obliczyć
	fR[0][0] = 0.055;	//wariancja statycznej próbki wysokości czujnika 1 [m^2]
	fR[1][1] = 6.0e-2;	//wariancja akcelerometru 1 [m^2/s^6]
	arm_mat_init_f32(&mR, 2, 2, &fR[0][0]);

	//początkowa wariancja i kowariancja predykcji jest taka sama jak dla pomiaru
	for (uint8_t m=0; m<4; m++)
	{
		for (uint8_t n=0; n<4; n++)
			fP[m][n] = fR[m][n];
	}
	arm_mat_init_f32(&mP, 4, 4, &fP[0][0]);

	//macierz przejścia oblicza wartość predykcji następnego pomiaru
	fF[0][0] = 1.0f;				//wysokość = poprzednia wysokość
	fF[0][1] = OKRES_PETLI_GLOWNEJ;	//wysokość = prędkość * dT
	fF[0][2] = powf(OKRES_PETLI_GLOWNEJ, 2) / 2;	//wysokość = przyspieszenie * dT^2/2
	//fF[0][3] = 0.0f;				//wysokość nie zależy od biasu przyspieszenia

	//fF[1][0] = 0.0f;				//prędkość nie bierze się z wysokości
	fF[1][1] = 1.0f;				//prędkość = poprzednia prędkość
	fF[1][2] = OKRES_PETLI_GLOWNEJ;	//prędkość = przyspieszenie * dT
	//fF[1][3] = 0.0f;				//prędkość nie zależy od biasu przyspieszenia

	//fF[2][0] = 0.0f;				//przyspieszenie nie bierze się z wysokości
	//fF[2][1] = 0.0f;				//przyspieszenie nie bierze się z prędkości
	fF[2][2] = 1.0f;				//przyspieszenie = poprzednie przyspieszenie
	//fF[2][3] = 0.0f;				//przyspieszenie nie zależy od biasu przyspieszenia

	//fF[3][0] = 0.0f;				//bias przyspieszenia nie bierze się z wysokości
	//fF[3][1] = 0.0f;				//bias przyspieszenia nie bierze się z prędkości
	//fF[3][2] = 0.0f;				//bias przyspieszenia nie bierze się z przyspieszenia
	fF[3][3] = 1.0f;				//bias przyspieszenia = poprzedni bias przyspieszenia
	arm_mat_init_f32(&mF, 4, 4, &fF[0][0]);

	//inicjalizacja szumu procesu. Zakładam że szum procesu zależy od podchodnej przyspieszenia, czyli zrywu
	//Q = sigma^2 * G * G^T
	fQ[0][0] = powf(OKRES_PETLI_GLOWNEJ, 6) / 36;
	fQ[0][1] = powf(OKRES_PETLI_GLOWNEJ, 5) / 12;
	fQ[0][2] = powf(OKRES_PETLI_GLOWNEJ, 4) / 6;
	//fQ[0][3] = 0.0f;
	fQ[1][0] = powf(OKRES_PETLI_GLOWNEJ, 5) / 12;
	fQ[1][1] = powf(OKRES_PETLI_GLOWNEJ, 4) / 6;
	fQ[1][2] = powf(OKRES_PETLI_GLOWNEJ, 3) / 2;
	//fQ[1][3] = 0.0f;
	fQ[2][0] = powf(OKRES_PETLI_GLOWNEJ, 4) / 6;
	fQ[2][1] = powf(OKRES_PETLI_GLOWNEJ, 3) / 2;
	fQ[2][1] = powf(OKRES_PETLI_GLOWNEJ, 2);
	//fQ[2][3] = 0.0f;
	//fQ[3][0] = 0.0f;
	//fQ[3][1] = 0.0f;
	//fQ[3][2] = 0.0f;
	fQ[3][3] = 0.000001f;	//zakładam że dryft akcelerometru jest bardzo mały
	arm_mat_init_f32(&mQ, 4, 4, &fQ[0][0]);

	//inicjalizacja macierzy jednostkowej
	fI[0][0] = 1.0f;
	fI[1][1] = 1.0f;
	fI[2][2] = 1.0f;
	fI[3][3] = 1.0f;
	arm_mat_init_f32(&mI, 4, 4, &fI[0][0]);

	//inicjalizacja obu macierzy obserwacji: Hah - przyspieszenia i wysokości oraz Ha - samego przyspieszenia.
	//Oba pomiary są w tych samych jednostkach co wektor stanu, więc obie macierze Hah i Ha są macierzami jednostkowymi
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

	//1) Predykcja nowej estymaty wektora stanu: x(n+1) = F * x(n) + w
	cBłąd |= arm_mat_mult_f32(&mF, &mX, &mTempW1);
	cBłąd |= arm_mat_add_f32(&mTempW1, &mW, &mX);
	dane->stBSP.fWysokoscMSL = fX[0];
	dane->stBSP.fPredkoscD 	 = fX[1];
	dane->stBSP.fWysokoscAGL = dane->stBSP.fWysokoscMSL - fPoczątkoweBarometryczneMSL;

	for (uint8_t n=0; n<4; n++)
		dane->stKalmanDebug.fX[n] = fX[n];

	//2) Obliczenie niepewności nowej estymaty wektora stanu: Temp1 = F * P(n)
	cBłąd |= arm_mat_mult_f32(&mF, &mP, &mTempM1);

	//transpozycja macierzy F: Temp2 = F^T
	cBłąd |= arm_mat_trans_f32(&mF, &mTempM2);

	//mnożenie Temp3 = (F * P(n)) * (F^T)
	cBłąd |= arm_mat_mult_f32(&mTempM1, &mTempM2, &mTempM3);

	//obliczenie szumu procesu Q. Na podstawie przykładów zakładam że szum procesu zależy od zrywu akcelerometru
	float32_t fOkresPetli = (float32_t)dane->ndT / 1e6;	//czas od ostatniego wykonania w [sekundach]
	fQ[0][0] = powf(fOkresPetli, 6) / 36 * WARIANCJA_ZRYWU_ACEL;
	fQ[0][1] = powf(fOkresPetli, 5) / 12 * WARIANCJA_ZRYWU_ACEL;
	fQ[0][2] = powf(fOkresPetli, 4) / 6  * WARIANCJA_ZRYWU_ACEL;
	fQ[1][0] = powf(fOkresPetli, 5) / 12 * WARIANCJA_ZRYWU_ACEL;
	fQ[1][1] = powf(fOkresPetli, 4) / 6  * WARIANCJA_ZRYWU_ACEL;
	fQ[1][2] = powf(fOkresPetli, 3) / 2  * WARIANCJA_ZRYWU_ACEL;
	fQ[2][0] = powf(fOkresPetli, 4) / 6  * WARIANCJA_ZRYWU_ACEL;
	fQ[2][1] = powf(fOkresPetli, 3) / 2  * WARIANCJA_ZRYWU_ACEL;
	fQ[2][1] = powf(fOkresPetli, 2) 	 * WARIANCJA_ZRYWU_ACEL;
	fQ[0][3] = fQ[1][3] = fQ[2][3] = 0.0f;
	fQ[3][0] = fQ[3][1] = fQ[3][2] = 0.0f;
	fQ[3][3] = WARIANCJA_DRYFTU_ACEL;

	//dodaj macierz szumu Q procesu do iloczynu (F * P(n)) * (F^T) -> P
	cBłąd |= arm_mat_add_f32(&mQ, &mTempM3, &mP);
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

	//liczę współczynnik wzmocnienia Kalmana, najpierw transponowane H[2][4] -> mTmpM42A[4][2]
	cBłąd |= arm_mat_trans_f32(&mHh, &mTempM42A);

	// P(n-1) * H^T -> mTmpM42B[4][2]
	cBłąd |= arm_mat_mult_f32(&mP, &mTempM42A, &mTempM42B);

	//H[2][4] * P(n-1)[4][4] -> mTempM24[2][4]
	cBłąd |= arm_mat_mult_f32(&mHh, &mP, &mTempM24);

	//(H * P(n-1))[2][4] * (H^T)[4][2] -> mTempM22A[2][2]
	cBłąd |= arm_mat_mult_f32(&mTempM24, &mTempM42B, &mTempM22A);

	//(H * P(n-1) * H^T) + R(n) -> mTempM22B
	cBłąd |= arm_mat_add_f32(&mTempM22A, &mR, &mTempM22B);

	//licze inwersję powyższego: (H * P(n-1) * H^T + R(n))^-1 -> mTempM22A
	cBłąd |= arm_mat_inverse_f32(&mTempM22B, &mTempM22A);

	//finalne mnożenie pierwszej części: P(n-1) * H^T oraz (H * P(n-1) * H^T + R(n))^-1 -> K
	cBłąd |= arm_mat_mult_f32(&mTempM42B, &mTempM22A, &mK);
	for (uint8_t n=0; n<4; n++)
		dane->stKalmanDebug.fK[n] = fK[n][0];

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1)
	cBłąd |= arm_mat_mult_f32(&mHh, &mX, &mTempM21A);

	//Uwzględnienie pomiaru: (z(n) - H * Estymata_x(n-1))
	fZ[0] = dane->fWysokoMSL[0];
	fZ[1] = dane->fAkcel1[2] - fX[3];	//Przyspieszenie osi Z - (przyspieszenie ziemskie + dryft akcelerometru)
	cBłąd |= arm_mat_sub_f32(&mZ, &mTempM21A, &mTempM21B);

	//mnożenie przez K: K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_mult_f32(&mK, &mTempM21B, &mTempM41A);		//4x2 * 2x1 = 4x1

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_add_f32(&mX, &mTempM41A, &mTempM41B);

	//przepisanie wyniku do wektora estymaty i finalnych zmiennych
	dane->stBSP.fWysokoscMSL = fX[0] = fTempM41B[0];
	dane->stBSP.fPredkoscD 	 = fX[1] = fTempM41B[1];
	fX[2] = fTempM41B[2];
	fX[3] = fTempM41B[3];
	dane->stBSP.fWysokoscAGL = dane->stBSP.fWysokoscMSL - fPoczątkoweBarometryczneMSL;



	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempM1 (4x2 * 2x4 = 4x4)
	cBłąd |= arm_mat_mult_f32(&mK, &mHh, &mTempM1);

	//odejmowanie (I - K(n) * H) -> TempM2
	cBłąd |= arm_mat_sub_f32(&mI, &mTempM1, &mTempM2);

	//mnożenie (I - K(n) * H) * P(n-1) -> TempM1
	cBłąd |= arm_mat_mult_f32(&mI, &mTempM2, &mTempM1);

	//transpozycja (I * K(n) * H)^T
	cBłąd |= arm_mat_trans_f32(&mTempM2, &mTempM3);

	//mnożenie  (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T
	cBłąd |= arm_mat_mult_f32(&mTempM1, &mTempM3, &mTempM2);

	//mnożenie 	K(n) * R(n)    (4x2 * 2x2 = 4x2)
	cBłąd |= arm_mat_mult_f32(&mK, &mR, &mTempM42A);

	//transpozycja K(n)^T   (4x2 -> 2x4)
	cBłąd |= arm_mat_trans_f32(&mK, &mTempM24);

	//mnożenie 	K(n) * R(n) * K(n)^T
	cBłąd |= arm_mat_mult_f32(&mTempM42A, &mTempM24, &mTempM4);

	//finalne sumowanie (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempM2, &mTempM4, &mP);

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

	//liczę współczynnik wzmocnienia Kalmana, najpierw transponowane H -> TempM1
	cBłąd |= arm_mat_trans_f32(&mHa, &mTempM42A);

	// P(n-1) * H^T -> TempM2
	cBłąd |= arm_mat_mult_f32(&mP, &mTempM42A, &mTempM42B);

	//H * P(n-1) -> TempM3
	cBłąd |= arm_mat_mult_f32(&mHa, &mP, &mTempW2);

	//(H * P(n-1)) * H^T -> TempM4
	cBłąd |= arm_mat_mult_f32(&mTempW2, &mTempM1, &mTempM4);

	//(H * P(n-1) * H^T) + R(n) -> TempM1
	cBłąd |= arm_mat_add_f32(&mTempM4, &mR, &mTempM1);

	//licze inwersję powyższego: (H * P(n-1) * H^T + R(n))^-1 -> TempM3
	cBłąd |= arm_mat_inverse_f32(&mTempM1, &mTempM3);

	//finalne mnożenie pierwszej części: P(n-1) * H^T oraz (H * P(n-1) * H^T + R(n))^-1 -> K
	cBłąd |= arm_mat_mult_f32(&mTempM2, &mTempM3, &mK);

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * Estymata_x(n-1)
	cBłąd |= arm_mat_mult_f32(&mHa, &mX, &mTempW1);

	//Uwzględnienie pomiaru: (z(n) - H * Estymata_x(n-1))
	fZ[2] = dane->fAkcel1[2] - fX[3];	//Przyspieszenie osi Z - przyspieszenie ziemskie
	cBłąd |= arm_mat_sub_f32(&mZ, &mTempW1, &mTempW2);

	//mnożenie przez K: K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_mult_f32(&mK, &mTempW2, &mTempW1);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_add_f32(&mX, &mTempW1, &mTempW2);

	//przepisanie wyniku do wektora estymaty i finalnych zmiennych
	dane->stBSP.fWysokoscMSL = fX[0] = fTempW2[0];
	dane->stBSP.fPredkoscD 	 = fX[1] = fTempW2[1];
	dane->stBSP.fWysokoscAGL = dane->stBSP.fWysokoscMSL - fPoczątkoweBarometryczneMSL;

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> TempM1
	cBłąd |= arm_mat_mult_f32(&mK, &mHa, &mTempM1);

	//odejmowanie (I - K(n) * H) -> TempM2
	cBłąd |= arm_mat_sub_f32(&mI, &mTempM1, &mTempM2);

	//mnożenie (I - K(n) * H) * P(n-1) -> TempM1
	cBłąd |= arm_mat_mult_f32(&mI, &mTempM2, &mTempM1);

	//transpozycja (I * K(n) * H)^T
	cBłąd |= arm_mat_trans_f32(&mTempM2, &mTempM3);

	//mnożenie  (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T
	cBłąd |= arm_mat_mult_f32(&mTempM1, &mTempM3, &mTempM2);

	//mnożenie 	K(n) * R(n)
	cBłąd |= arm_mat_mult_f32(&mK, &mR, &mTempM1);

	//transpozycja K(n)^T
	cBłąd |= arm_mat_trans_f32(&mK, &mTempM3);

	//mnożenie 	K(n) * R(n) * K(n)^T
	cBłąd |= arm_mat_mult_f32(&mTempM1, &mTempM3, &mTempM4);

	//finalne sumowanie (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempM2, &mTempM4, &mP);
	return cBłąd;
}
