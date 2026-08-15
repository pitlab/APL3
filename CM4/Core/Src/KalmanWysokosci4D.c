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

static float32_t fx[4];		//wektor stanu: 0=wysokość, 1=prędkość
static float32_t fz[4];		//wektor pomiaru: 0=wysokość, 1=prędkość
static float32_t fw[4];		//wektor niemierzalnego szumu procesu
static float32_t fP[4][4];	//macierz wariancji i kowariancji procesu
static float32_t fR[4][4];	//macierz wariancji i kowariancji pomiaru
static float32_t fF[4][4];	//macierz przejścia wektora stanu
static float32_t fQ[4][4];	//macierz szumu procesu
static float32_t fI[4][4];	//macierz jednostkowa
static float32_t fHah[4][4];//macierz obserwacji przyspieszenia i wysokości, przekształca jednostkę pomiaru w jednostkę wektora stanu
static float32_t fHa[4][4];	//macierz obserwacji przyspieszenia, przekształca jednostkę pomiaru w jednostkę wektora stanu
static float32_t fK[4][4];	//macierz wzmocnień kalnama
static float32_t fEstymataX[4];	//estymata wektora stanu n+1
static float32_t fTempW1[4], fTempW2[4];	//wektory na wyniki pośrednie
static float32_t fTempM1[4][4], fTempM2[4][4], fTempM3[4][4], fTempM4[4][4];	//macierze na wyniki pośrednie

//zmienne z przedrostkiem m oznaczają macierze (lub wektory) w formacie biblioteki ARM DSP
static arm_matrix_instance_f32 mx  = {4, 1, fx};			//wektor stanu
static arm_matrix_instance_f32 mz  = {4, 1, fz};			//wektor pomiaru
static arm_matrix_instance_f32 mex = {4, 1, fEstymataX};	//estymata wektora stanu n+1
static arm_matrix_instance_f32 mw  = {4, 1, &fw[0]};		//wektor szumu procesu
static arm_matrix_instance_f32 mP  = {4, 4, &fP[0][0]};		//macierz wariancji i kowariancji predykcji
static arm_matrix_instance_f32 mR  = {4, 4, &fR[0][0]};		//macierz wariancji i kowariancji pomiaru
static arm_matrix_instance_f32 mF  = {4, 4, &fF[0][0]};		//macierz przejścia wektora stanu
static arm_matrix_instance_f32 mQ  = {4, 4, &fQ[0][0]};		//macierz szumu procesu
static arm_matrix_instance_f32 mI  = {4, 4, &fI[0][0]};		//macierz jednostkowa
static arm_matrix_instance_f32 mHah = {4, 4, &fHah[0][0]};		//macierz obserwacji wysokości
static arm_matrix_instance_f32 mHa = {4, 4, &fHa[0][0]};		//macierz obserwacji przyspieszenia
static arm_matrix_instance_f32 mK  = {4, 4, &fK[0][0]};		//macierz wzmocnień kalmana
static arm_matrix_instance_f32 mTempW1 = {4, 1, fTempW1};	//wektor1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempW2 = {4, 1, fTempW2};	//wektor2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM1 = {4, 4, &fTempM1[0][0]};	//macierz1 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM2 = {4, 4, &fTempM2[0][0]};	//macierz2 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM3 = {4, 4, &fTempM3[0][0]};	//macierz3 na wyniki pośrednie
static arm_matrix_instance_f32 mTempM4 = {4, 4, &fTempM4[0][0]};	//macierz4 na wyniki pośrednie

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
			fR[m][n] = 0.0f;
			fHah[m][n] = fHa[m][n] = 0.0f;
			fI[m][n] = 0.0f;
			fQ[m][n] = 0.0f;
			fF[m][n] = 0.0f;
		}
		fz[m] = 0.0f;
	}

	fx[3] = AKCEL1G;
	arm_mat_init_f32(&mx, 4, 1, fx);

	fz[3] = AKCEL1G;	//bias osi Z akcelerometru
	arm_mat_init_f32(&mz, 4, 1, fz);
	
	//wariancja jest kwadratem standardowego odchylenia pomiaru i jest rozmieszczona w głównej przekątnej macierzy
	//pozostałe pola sa kowariancją, czyli zależnością między błędami jednego pomiaru a drugiego. Zakładam że
	//błędy pomiarów są niezależne, więc kowariancja jest ustawiona na 0. W rzeczywistosci pomiar prędkości jest liczony
	//z danych czujnika wysokości więc korelacja istnieje ale na razie nie potrafię jej obliczyć
	fR[0][0] = 0.0055;	//wariancja statycznej próbki wysokości czujnika 1 [m^2]
	fR[2][2] = 6.0e-5;	//wariancja akcelerometru 1 [m^2/s^6]
	arm_mat_init_f32(&mR, 4, 4, &fR[0][0]);

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

	//pierwsza estymata ma wartość pomiaru
	fEstymataX[0] = fz[0];
	fEstymataX[1] = fz[1];
	fEstymataX[2] = fz[2];
	fEstymataX[3] = fz[3];
	arm_mat_init_f32(&mex, 4, 1, fEstymataX);

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
	fHah[1][1] = 1.0f;
	fHah[3][3] = 1.0f;
	arm_mat_init_f32(&mHah, 4, 4, &fHah[0][0]);

	fHa[3][3] = 1.0f;
	arm_mat_init_f32(&mHa, 4, 4, &fHa[0][0]);
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

	//1) Predykcja nowej estymaty wektora stanu: x(n+1) = F * x(n) + w
	cBłąd |= arm_mat_mult_f32(&mF, &mx, &mTempW1);
	cBłąd |= arm_mat_add_f32(&mTempW1, &mw, &mex);
	dane->stBSP.fWysokoscMSL = fEstymataX[0];
	dane->stBSP.fPredkoscD 	 = fEstymataX[1];
	dane->stBSP.fWysokoscAGL = dane->stBSP.fWysokoscMSL - fPoczątkoweBarometryczneMSL;

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

	//liczę współczynnik wzmocnienia Kalmana, najpierw transponowane H -> TempM1
	cBłąd |= arm_mat_trans_f32(&mHah, &mTempM1);

	// P(n-1) * H^T -> TempM2
	cBłąd |= arm_mat_mult_f32(&mP, &mTempM1, &mTempM2);

	//H * P(n-1) -> TempM3
	cBłąd |= arm_mat_mult_f32(&mHah, &mP, &mTempM3);

	//(H * P(n-1)) * H^T -> TempM4
	cBłąd |= arm_mat_mult_f32(&mTempM3, &mTempM1, &mTempM4);

	//(H * P(n-1) * H^T) + R(n) -> TempM1
	cBłąd |= arm_mat_add_f32(&mTempM4, &mR, &mTempM1);

	//licze inwersję powyższego: (H * P(n-1) * H^T + R(n))^-1 -> TempM3
	cBłąd |= arm_mat_inverse_f32(&mTempM1, &mTempM3);

	//finalne mnożenie pierwszej części: P(n-1) * H^T oraz (H * P(n-1) * H^T + R(n))^-1 -> K
	cBłąd |= arm_mat_mult_f32(&mTempM2, &mTempM3, &mK);

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * Estymata_x(n-1)
	//Ponieważ estymata przyspieszenia to pr
	cBłąd |= arm_mat_mult_f32(&mHah, &mex, &mTempW1);

	//Uwzględnienie pomiaru: (z(n) - H * Estymata_x(n-1))
	fz[0] = dane->fWysokoMSL[0];
	fz[2] = dane->fAkcel1[2] - fx[3];	//Przyspieszenie osi Z - przyspieszenie ziemskie
	cBłąd |= arm_mat_sub_f32(&mz, &mTempW1, &mTempW2);

	//mnożenie przez K: K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_mult_f32(&mK, &mTempW2, &mTempW1);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_add_f32(&mex, &mTempW1, &mTempW2);

	//przepisanie wyniku do wektora estymaty i finalnych zmiennych
	dane->stBSP.fWysokoscMSL = fEstymataX[0] = fTempW2[0];
	dane->stBSP.fPredkoscD 	 = fEstymataX[1] = fTempW2[1];
	dane->stBSP.fWysokoscAGL = dane->stBSP.fWysokoscMSL - fPoczątkoweBarometryczneMSL;

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> TempM1
	cBłąd |= arm_mat_mult_f32(&mK, &mHah, &mTempM1);

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
	cBłąd |= arm_mat_trans_f32(&mHa, &mTempM1);

	// P(n-1) * H^T -> TempM2
	cBłąd |= arm_mat_mult_f32(&mP, &mTempM1, &mTempM2);

	//H * P(n-1) -> TempM3
	cBłąd |= arm_mat_mult_f32(&mHa, &mP, &mTempM3);

	//(H * P(n-1)) * H^T -> TempM4
	cBłąd |= arm_mat_mult_f32(&mTempM3, &mTempM1, &mTempM4);

	//(H * P(n-1) * H^T) + R(n) -> TempM1
	cBłąd |= arm_mat_add_f32(&mTempM4, &mR, &mTempM1);

	//licze inwersję powyższego: (H * P(n-1) * H^T + R(n))^-1 -> TempM3
	cBłąd |= arm_mat_inverse_f32(&mTempM1, &mTempM3);

	//finalne mnożenie pierwszej części: P(n-1) * H^T oraz (H * P(n-1) * H^T + R(n))^-1 -> K
	cBłąd |= arm_mat_mult_f32(&mTempM2, &mTempM3, &mK);

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * Estymata_x(n-1)
	cBłąd |= arm_mat_mult_f32(&mHa, &mex, &mTempW1);

	//Uwzględnienie pomiaru: (z(n) - H * Estymata_x(n-1))
	fz[2] = dane->fAkcel1[2] - fx[3];	//Przyspieszenie osi Z - przyspieszenie ziemskie
	cBłąd |= arm_mat_sub_f32(&mz, &mTempW1, &mTempW2);

	//mnożenie przez K: K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_mult_f32(&mK, &mTempW2, &mTempW1);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
	cBłąd |= arm_mat_add_f32(&mex, &mTempW1, &mTempW2);

	//przepisanie wyniku do wektora estymaty i finalnych zmiennych
	dane->stBSP.fWysokoscMSL = fEstymataX[0] = fTempW2[0];
	dane->stBSP.fPredkoscD 	 = fEstymataX[1] = fTempW2[1];
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
