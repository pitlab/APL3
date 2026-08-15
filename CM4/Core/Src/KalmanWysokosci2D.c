//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Filtr Kalmana obrabiajacy dane wysokości i prędkości pionowej
// Przyspieszenie pionowe jest wartością sterującą
//
// (c) PitLab 2026
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <KalmanWysokosci2D.h>

static float32_t fx[2];		//wektor stanu: 0=wysokość, 1=prędkość
static float32_t fz[2];		//wektor pomiaru: 0=wysokość, 1=prędkość
static float32_t fw[2];		//wektor niemierzalnego szumu procesu
static float32_t fP[2][2];	//macierz wariancji i kowariancji procesu
static float32_t fR[2][2];	//macierz wariancji i kowariancji pomiaru
static float32_t fF[2][2];	//macierz przejścia wektora stanu
static float32_t fG[2];		//macierz przejścia sygnałów wejściowych
static float32_t fQ[2][2];	//macierz szumu procesu
static float32_t fI[2][2];	//macierz jednostkowa
static float32_t fH[2][2];	//macierz obserwacji, przekształca jednostkę pomiaru w jednostkę wektora stanu
static float32_t fK[2][2];	//macierz wzmocnień kalnama
static float32_t fEstymataX[2];	//estymata wektora stanu n+1
static float32_t fTempW1[2], fTempW2[2], fTempW3[2];	//wektory na wyniki pośrednie
static float32_t fTempM1[2][2], fTempM2[2][2], fTempM3[2][2], fTempM4[2][2];	//macierze na wyniki pośrednie

//zmienne z przedrostkiem m oznaczają macierze (lub wektory) w formacie biblioteki ARM DSP
static arm_matrix_instance_f32 mx = {2, 1, fx};			//wektor stanu
static arm_matrix_instance_f32 mz = {2, 1, fz};			//wektor pomiaru
static arm_matrix_instance_f32 mex = {2, 1, fEstymataX};	//estymata wektora stanu n+1
static arm_matrix_instance_f32 mw = {2, 1, &fw[0]};		//wektor szumu procesu
static arm_matrix_instance_f32 mP = {2, 2, &fP[0][0]};		//macierz wariancji i kowariancji predykcji
static arm_matrix_instance_f32 mR = {2, 2, &fR[0][0]};		//macierz wariancji i kowariancji pomiaru
static arm_matrix_instance_f32 mF = {2, 2, &fF[0][0]};		//macierz przejścia wektora stanu
static arm_matrix_instance_f32 mG = {2, 1, fG};			//macierz przejścia sygnałów wejściowych
static arm_matrix_instance_f32 mQ = {2, 2, &fQ[0][0]};		//macierz szumu procesu
static arm_matrix_instance_f32 mI = {2, 2, &fI[0][0]};		//macierz jednostkowa
static arm_matrix_instance_f32 mH = {2, 2, &fH[0][0]};		//macierz obserwacji
static arm_matrix_instance_f32 mK = {2, 2, &fK[0][0]};		//macierz wzmocnień kalmana
static arm_matrix_instance_f32 mTempW1 = {2, 1, fTempW1};	//wektor1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempW2 = {2, 1, fTempW2};	//wektor2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempW3 = {2, 1, fTempW3};	//wektor3 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM1 = {2, 2, &fTempM1[0][0]};	//macierz1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM2 = {2, 2, &fTempM2[0][0]};	//macierz2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM3 = {2, 2, &fTempM3[0][0]};	//macierz3 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM4 = {2, 2, &fTempM4[0][0]};	//macierz4 na wyniki pośrednie

extern float fPoczątkoweBarometryczneMSL;	//wysokość MSL wyznaczona podczas uruchomienia

////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje liniowy filtr Kalmana dla wysokości i jej pierwszej pochodnej - prędkości pionowej
// Filtr sterowany jest drugą pochodną wysokości - przyspieszeniem w pionie.
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFiltrKalmanaWysokości2D(void)
{
	uint8_t cBłąd = BLAD_OK;

	arm_mat_init_f32(&mx, 2, 1, fx);

	fz[0] = 0.0f;
	fz[1] = 0.0f;
	arm_mat_init_f32(&mz, 2, 1, fz);
	
	//wariancja jest kwadratem standardowego odchylenia pomiaru i jest rozmieszczona w głównej przekątnej macierzy
	//pozostałe pola sa kowariancją, czyli zależnością między błędami jednago pomiaru a drugiego. Zakładam że
	//błędy pomiarów są niezależne, więc kowariancja jest ustawiona na 0. W rzeczywistosci pomiar prędkości jest liczony
	//z danych czujnika wysokości więc korelacja istnieje ale na razie nie potrafię jej obliczyć
	fR[0][0] = 0.0055;	//wariancja statycznej próbki wysokości czujnika 1 [m^2]
	fR[0][1] = 0.0f;
	fR[1][0] = 6.9e-5;	//wariancja wariometru z czujnika 1 [m^2/s^2]
	fR[1][1] = 0.0f;
	arm_mat_init_f32(&mR, 2, 2, &fR[0][0]);

	//początkowa wariancja i kowariancja predykcji jest taka sama jak dla pomiaru
	fP[0][0] = fR[0][0];
	fP[0][1] = fR[0][1];
	fP[1][0] = fR[1][0];
	fP[1][1] = fR[1][1];
	arm_mat_init_f32(&mP, 2, 2, &fP[0][0]);

	//macierz przejścia oblicza wartość predykcji następnego pomiaru
	fF[0][0] = 1.0f;				//wysokość = poprzednia wysokość
	fF[0][1] = OKRES_PETLI_GLOWNEJ;	//wysokość = prędkość * dT
	fF[1][0] = 0.0f;				//prędkość nie bierze się z wysokości
	fF[1][1] = 1.0f;				//prędkość = poprzednia prędkość
	arm_mat_init_f32(&mF, 2, 2, &fF[0][0]);

	//pierwsza estymata ma wartość pomiaru
	fEstymataX[0] = fz[0];
	fEstymataX[1] = fz[1];
	arm_mat_init_f32(&mex, 2, 1, fEstymataX);

	//inicjalizacja macierzy przejscia sygnałów wejściowych - czyli zależność od przyspieszenia
	fG[0] = powf(OKRES_PETLI_GLOWNEJ, 2) / 2;	//wysokość = przyspieszenie * (dT^2) / 2
	fG[1] = OKRES_PETLI_GLOWNEJ;				//prędkość = przyspieszenie * dT
	arm_mat_init_f32(&mG, 2, 1, &fG[0]);

	//inicjalizacja szumu procesu. Na podstawie przykładów zakładam że szum procesu wygląda następująco
	//[dT^4/4, dT^3/2] * sigma^2, gdzie sigma to odchylenie standardowe przyspieszenia = 0,0025, stąd sigma^2 = 6e-6
	//[dT^3/2, dT^2  ]
	fQ[0][0] = 0.0055;	//wariancja statycznej próbki wysokości czujnika 1 [m^2]
	fQ[0][1] = 0.0f;
	fQ[1][1] = 6.9e-5;	//wariancja wariometru z czujnika 1 [m^2/s^2]
	fQ[1][1] = 0.0f;
	arm_mat_init_f32(&mQ, 2, 2, &fQ[0][0]);

	//inicjalizacja macierzy jednostkowej
	fI[0][0] = 1.0f;
	fI[0][1] = 0.0f;
	fI[1][0] = 0.0f;
	fI[1][1] = 1.0f;
	arm_mat_init_f32(&mI, 2, 2, &fI[0][0]);

	//inicjalizacja macierzy obserwacji - pomiar jest w tych samych jednostkach co wektor stanu, więc H jest macierzą jednostkową
	fH[0][0] = 1.0f;
	fH[0][1] = 0.0f;
	fH[1][0] = 0.0f;
	fH[1][1] = 1.0f;
	arm_mat_init_f32(&mH, 2, 2, &fH[0][0]);

	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja estymuje nowe wartości wektora stanu ze etapu (n) na (n+1)
// x(n+1) = F * x(n) + G * u(n) + w
// oraz wwykonuje predykcję kowariancji (niepewności) nowej wartości:
// P(n+1) = F * P(n) * F^T + Q
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PredykcjaFiltraKalmanaWysokości2D(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//1) Predykcja nowej estymaty wektora stanu
	//F * x(n) -> Temp1
	cBłąd = arm_mat_mult_f32(&mF, &mx, &mTempW1);

	//G * u(n) -> Temp2; gdzie u(n) jest zmienną sterującą, w naszym przypadku jest to różnica między przyspieszeniem w osi Z a przyspieszeniem ziemskim
	cBłąd |= arm_mat_scale_f32(&mG, (dane->fAkcel1[2] - AKCEL1G), &mTempW2);

	//dodawanie składników
	cBłąd |= arm_mat_add_f32(&mTempW1, &mTempW2, &mTempW3);
	cBłąd |= arm_mat_add_f32(&mTempW3, &mw, &mex);
	dane->stBSP.fWysokoscMSL = fEstymataX[0];
	dane->stBSP.fPredkoscD 	 = fEstymataX[1];
	dane->stBSP.fWysokoscAGL = dane->stBSP.fWysokoscMSL - fPoczątkoweBarometryczneMSL;

	//2) Obliczenie niepewności nowej estymaty wektora stanu
	//Temp1 = F * P(n)
	cBłąd |= arm_mat_mult_f32(&mF, &mP, &mTempM1);

	//transpozycja macierzy F: Temp2 = F^T
	cBłąd |= arm_mat_trans_f32(&mF, &mTempM2);

	//mnożenie Temp3 = (F * P(n)) * (F^T)
	cBłąd |= arm_mat_mult_f32(&mTempM1, &mTempM2, &mTempM3);

	//obliczenie szumu procesu Q. Na podstawie przykładów zakładam że szum procesu wygląda następująco
	//[dT^4/4, dT^3/2] * sigma^2;
	//[dT^3/2, dT^2  ];		gdzie sigma to odchylenie standardowe przyspieszenia = 0,0025[m/s^2], stąd sigma^2 = 6e-6 [m^2/s^4]
	fTempM1[0][0] = powf((float32_t)dane->ndT / 1e6, 4) / 4;
	fTempM1[0][1] = powf((float32_t)dane->ndT / 1e6, 3) / 2;
	fTempM1[1][0] = fTempM1[0][1];
	fTempM1[1][1] = powf((float32_t)dane->ndT / 1e6, 2);
	cBłąd |= arm_mat_scale_f32(&mTempM1, 6e-6, &mQ);

	//dodaj macierz szumu Q procesu do iloczynu (F * P(n)) * (F^T)
	cBłąd |= arm_mat_add_f32(&mQ, &mTempM3, &mP);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaFiltraKalmanaWysokości2D(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//liczę współczynnik wzmocnienia Kalmana, najpierw transponowane H -> TempM1
	cBłąd |= arm_mat_trans_f32(&mH, &mTempM1);

	// P(n-1) * H^T -> TempM2
	cBłąd |= arm_mat_mult_f32(&mP, &mTempM1, &mTempM2);

	//H * P(n-1) -> TempM3
	cBłąd |= arm_mat_mult_f32(&mH, &mP, &mTempM3);

	//(H * P(n-1)) * H^T -> TempM4
	cBłąd |= arm_mat_mult_f32(&mTempM3, &mTempM1, &mTempM4);

	//(H * P(n-1) * H^T) + R(n) -> TempM1
	cBłąd |= arm_mat_add_f32(&mTempM4, &mR, &mTempM1);

	//licze inwersję powyższego: (H * P(n-1) * H^T + R(n))^-1 -> TempM3
	cBłąd |= arm_mat_inverse_f32(&mTempM1, &mTempM3);

	//finalne mnożenie pierwszej części: P(n-1) * H^T oraz (H * P(n-1) * H^T + R(n))^-1 -> K
	cBłąd |= arm_mat_mult_f32(&mTempM2, &mTempM3, &mK);

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * Estymata_x(n-1)
	cBłąd |= arm_mat_mult_f32(&mH, &mex, &mTempW1);

	//Uwzględnienie pomiaru: (z(n) - H * Estymata_x(n-1))
	fz[0] = dane->fWysokoMSL[0];
	fz[1] = dane->fWariometr[0];
	cBłąd |= arm_mat_sub_f32(&mz, &mTempW1, &mTempW2);

	//mnożenie przez K: K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_mult_f32(&mK, &mTempW2, &mTempW1);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_add_f32(&mex, &mTempW1, &mTempW2);

	//przepisanie wyniku do wektora estymaty
	dane->stBSP.fWysokoscMSL = fEstymataX[0] = fTempW2[0];
	dane->stBSP.fPredkoscD 	 = fEstymataX[1] = fTempW2[1];

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> TempM1
	cBłąd |= arm_mat_mult_f32(&mK, &mH, &mTempM1);

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
