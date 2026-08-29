//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Filtr Kalmana o 5-elementowym wektorze stanu zawierajacym: wysokość MSL, prędkość pionową, przyspieszenie kinematyczne w osi Z i bias przyspieszenia dwóch akcelerometrów
// Filtr zasilany jest dwoma zestawami wysokości i prędkości z dwu różnych czujników ciśnienia, oraz dwoma przyspieszeniami całkowitym w osi Z z dwóch akcelerometrów
// Dane pomiarowe aktualizowane są w 4 niezależnych funkcjach:
// - AktulizacjaCzujnikiemCiśnienia1FiltraKalmanaWysokości5X6Z - wysokość i prędkość pionowa z czujnika ciśnienia 1
// - AktulizacjaCzujnikiemCiśnienia2FiltraKalmanaWysokości5X6Z - wysokość i prędkość pionowa  z czujnika ciśnienia 2
// - AktulizacjaAkcelerometrem1FiltraKalmanaWysokości5X6Z - przyspieszenia w osi Z z akcelerometru 1
// - AktulizacjaAkcelerometrem2FiltraKalmanaWysokości5X6Z - przyspieszenia w osi Z z akcelerometru 2
//
// (c) PitLab 2026
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <KalmanWysokosci4X3Z.h>

#define KSTAN		5	//rozmiar wektora stanu
//#define KPOMIAR		6	//rozmiar wektora pomiaru
#define KPCIS		2	//rozmiar wektora pomiaru ciśnienia
#define KPACC		1	//rozmiar wektora pomiaru przyspieszenia

static float32_t fX[KSTAN];					//wektor stanu: 0=wysokość, 1=prędkość, 2=przyspieszenie kinematyczne Z, 3=grawitacja + bias akcelerometru 1, 4=grawitacja + bias akcelerometru 2
static float32_t fZc[KPCIS];				//wektor pomiaru: 0=wysokość, 1=prędkość
static float32_t fZa[KPACC];				//wektor pomiaru: przyspieszenie
static float32_t fF[KSTAN][KSTAN];			//macierz przejścia wektora stanu
static float32_t fP[KSTAN][KSTAN];			//macierz kowariancji predykcji
static float32_t fRc1[KPCIS][KPCIS];		//macierz kowariancji pomiaru wysokości i prędkości z czujnika ciśnienia 1
static float32_t fRc2[KPCIS][KPCIS];		//macierz kowariancji pomiaru wysokości i prędkości z czujnika ciśnienia 2
static float32_t fRa1[KPACC][KPACC];		//macierz kowariancji pomiaru przyspieszenia 1
static float32_t fRa2[KPACC][KPACC];		//macierz kowariancji pomiaru przyspieszenia 2
static float32_t fQ[KSTAN][KSTAN];			//macierz szumu procesu
static float32_t fI[KSTAN][KSTAN];			//macierz jednostkowa
static float32_t fHc1[KPCIS][KSTAN];		//macierz obserwacji prędkości i wysokości z czujnika ciśnienia 1
static float32_t fHc2[KPCIS][KSTAN];		//macierz obserwacji prędkości i wysokości z czujnika ciśnienia 2
static float32_t fHa1[KPACC][KSTAN];		//macierz obserwacji przyspieszenia z akcelerometru 1
static float32_t fHa2[KPACC][KSTAN];		//macierz obserwacji przyspieszenia z akcelerometru 2
static float32_t fKc1[KSTAN][KPCIS];		//macierz wzmocnień Kalmana czujnika ciśnienia 1
static float32_t fKc2[KSTAN][KPCIS];		//macierz wzmocnień Kalmana czujnika ciśnienia 2
static float32_t fKa1[KSTAN][KPACC];		//macierz wzmocnień Kalmana akcelerometru 1
static float32_t fKa2[KSTAN][KPACC];		//macierz wzmocnień Kalmana akcelerometru 1
static float32_t fPHc[KSTAN][KPCIS];		//macierz [Stan x PomiarCiśn] na wyniki pośrednie
static float32_t fPHa[KSTAN][KPACC];		//macierz [Stan x PomiarAkcel] na wyniki pośrednie
static float32_t fTempSPc[KSTAN][KPCIS];	//macierz [Stan x PomiarCiśn] na wyniki pośrednie
static float32_t fTempSPa[KSTAN][KPACC];	//macierz [Stan x PomiarAkcel] na wyniki pośrednie
static float32_t fTempPPcA[KPCIS][KPCIS];	//macierz [PomiarCiśn x PomiarCiśn] na wyniki pośrednie A
static float32_t fTempPPcB[KPCIS][KPCIS];	//macierz [PomiarCiśn x PomiarCiśn] na wyniki pośrednie B
static float32_t fTempPPaA[KPACC][KPACC];	//macierz [PomiarAkcel x PomiarAkcel] na wyniki pośrednie A
static float32_t fTempPPaB[KPACC][KPACC];	//macierz [PomiarAkcel x PomiarAkcel] na wyniki pośrednie B
static float32_t fTempPc1A[KPCIS];			//wektor [PomiarCiśn] na wyniki pośrednie A
static float32_t fTempPc1B[KPCIS];			//wektor [PomiarCiśn] na wyniki pośrednie B
static float32_t fTempPa1A[KPACC];			//wektor [PomiarAkcel] na wyniki pośrednie A
static float32_t fTempPa1B[KPACC];			//wektor [PomiarAkcel] na wyniki pośrednie B
static float32_t fTempPcS[KPCIS][KSTAN];	//macierz [PomiarCiśn x Stan] na wyniki pośrednie
static float32_t fTempPaS[KPACC][KSTAN];	//macierz [PomiarAkcel x Stan] na wyniki pośrednie
static float32_t fTempSSA[KSTAN][KSTAN];	//macierz [Stan x Stan] na wyniki pośrednie A
static float32_t fTempSSB[KSTAN][KSTAN];	//macierz [Stan x Stan] na wyniki pośrednie B
static float32_t fTempSSC[KSTAN][KSTAN];	//macierz [Stan x Stan] na wyniki pośrednie C
static float32_t fTempS1A[KSTAN];			//wektor [Stan] na wyniki pośrednie A
static float32_t fTempS1B[KSTAN];			//wektor [Stan] na wyniki pośrednie B

//zmienne z przedrostkiem m oznaczają macierze (lub wektory) w formacie biblioteki ARM DSP
static arm_matrix_instance_f32 mX   = {KSTAN, 1, fX};					//wektor stanu
static arm_matrix_instance_f32 mZc  = {KPCIS, 1, fZc};					//wektor pomiaru: wysokość, prędkość
static arm_matrix_instance_f32 mZa  = {KPACC, 1, fZa};					//wektor pomiaru:  przyspieszenie
static arm_matrix_instance_f32 mF   = {KSTAN, KSTAN, &fF[0][0]};		//macierz przejścia wektora stanu
static arm_matrix_instance_f32 mP   = {KSTAN, KSTAN, &fP[0][0]};		//macierz kowariancji predykcji
static arm_matrix_instance_f32 mRc1 = {KPCIS, KPCIS, &fRc1[0][0]};		//macierz kowariancji pomiaru czujnikiem ciśnienia 1
static arm_matrix_instance_f32 mRc2 = {KPCIS, KPCIS, &fRc2[0][0]};		//macierz kowariancji pomiaru czujnikiem ciśnienia 2
static arm_matrix_instance_f32 mRa1 = {KPACC, KPACC, &fRa1[0][0]};		//macierz kowariancji pomiaru akceleromerem 1
static arm_matrix_instance_f32 mRa2 = {KPACC, KPACC, &fRa2[0][0]};		//macierz kowariancji pomiaru akceleromerem 2
static arm_matrix_instance_f32 mQ   = {KSTAN, KSTAN, &fQ[0][0]};		//macierz szumu procesu
static arm_matrix_instance_f32 mI   = {KSTAN, KSTAN, &fI[0][0]};		//macierz jednostkowa
static arm_matrix_instance_f32 mHc1 = {KPCIS, KSTAN, &fHc1[0][0]};		//macierz obserwacji wysokości i prędkości czujnika ciśnienia 1
static arm_matrix_instance_f32 mHc2 = {KPCIS, KSTAN, &fHc2[0][0]};		//macierz obserwacji wysokości i prędkości czujnika ciśnienia 2
static arm_matrix_instance_f32 mHa1 = {KPACC, KSTAN, &fHa1[0][0]};		//macierz obserwacji przyspieszenia akcelerometru  1
static arm_matrix_instance_f32 mHa2 = {KPACC, KSTAN, &fHa2[0][0]};		//macierz obserwacji przyspieszenia akcelerometru 2
static arm_matrix_instance_f32 mKc1 = {KSTAN, KPCIS, &fKc1[0][0]};		//macierz wzmocnień Kalmana czujnika ciśnienia 1
static arm_matrix_instance_f32 mKc2 = {KSTAN, KPCIS, &fKc2[0][0]};		//macierz wzmocnień Kalmana czujnika ciśnienia 2
static arm_matrix_instance_f32 mKa1 = {KSTAN, KPACC, &fKa1[0][0]};		//macierz wzmocnień Kalmana akcelerometru 1
static arm_matrix_instance_f32 mKa2 = {KSTAN, KPACC, &fKa2[0][0]};		//macierz wzmocnień Kalmana akcelerometru 2
static arm_matrix_instance_f32 mPHc = {KSTAN, KPCIS, &fPHc[0][0]};		//macierz SxPc na iloczyn P*Hc
static arm_matrix_instance_f32 mPHa = {KSTAN, KPACC, &fPHa[0][0]};		//macierz SxPa na iloczyn P*Ha
static arm_matrix_instance_f32 mTempSPc  = {KSTAN, KPCIS, &fTempSPc[0][0]};
static arm_matrix_instance_f32 mTempSPa  = {KSTAN, KPACC, &fTempSPa[0][0]};
static arm_matrix_instance_f32 mTempPPcA = {KPCIS, KPCIS, &fTempPPcA[0][0]};	//macierz Pc x Pc na wyniki pośrednie A
static arm_matrix_instance_f32 mTempPPcB = {KPCIS, KPCIS, &fTempPPcB[0][0]};	//macierz Pc x Pc na wyniki pośrednie B
static arm_matrix_instance_f32 mTempPPaA = {KPACC, KPACC, &fTempPPaA[0][0]};	//macierz Pa x Pa na wyniki pośrednie A
static arm_matrix_instance_f32 mTempPPaB = {KPACC, KPACC, &fTempPPaB[0][0]};	//macierz Pa x Pa na wyniki pośrednie B
static arm_matrix_instance_f32 mTempPc1A = {KPCIS, 1, fTempPc1A};				//macierz Pc x 1 na wyniki pośrednie A
static arm_matrix_instance_f32 mTempPc1B = {KPCIS, 1, fTempPc1B};				//macierz Pc x 1 na wyniki pośrednie B
static arm_matrix_instance_f32 mTempPa1A = {KPACC, 1, fTempPa1A};				//macierz Pa x 1 na wyniki pośrednie A
static arm_matrix_instance_f32 mTempPa1B = {KPACC, 1, fTempPa1B};				//macierz Pa x 1 na wyniki pośrednie B
static arm_matrix_instance_f32 mTempPcS  = {KPCIS, KSTAN, &fTempPcS[0][0]};		//macierz Pc x S na wyniki pośrednie
static arm_matrix_instance_f32 mTempPaS  = {KPACC, KSTAN, &fTempPaS[0][0]};		//macierz Pa x S na wyniki pośrednie
static arm_matrix_instance_f32 mTempSSA  = {KSTAN, KSTAN, &fTempSSA[0][0]};	//macierz S x S na wyniki pośrednie A
static arm_matrix_instance_f32 mTempSSB  = {KSTAN, KSTAN, &fTempSSB[0][0]};	//macierz S x S na wyniki pośrednie B
static arm_matrix_instance_f32 mTempSSC  = {KSTAN, KSTAN, &fTempSSC[0][0]};	//macierz S x S na wyniki pośrednie C



static arm_matrix_instance_f32 mTempS1A = {KSTAN, 1, fTempS1A};		//macierz Sx1 na wyniki pośrednie A
static arm_matrix_instance_f32 mTempS1B = {KSTAN, 1, fTempS1B};		//macierz Sx1 na wyniki pośrednie B

extern float fPoczątkoweBarometryczneMSL;	//wysokość MSL wyznaczona podczas inicjalizacji
static uint8_t cLicznikUśredniania = LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI;


////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje liniowy filtr Kalmana dla wysokości i jej pierwszej pochodnej - prędkości pionowej
// Filtr sterowany jest drugą pochodną wysokości - przyspieszeniem w pionie.
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFiltrKalmanaWysokości5X6Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//filtr jest zainicjowany dopiero wtedy gdy trafia do niego rzeczywiste dane z czujnika o niezerowej wysokosci MSL
	if (dane->cNowyPomiar & NP_WYS1)
	{
		//zeru wektory pomiaru w pierwszym cyklu uśredniania
		if (cLicznikUśredniania == LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI)
		{
				fZc[0] = 0.0f;
				fZc[1] = 0.0f;
				fZa[0] = 0.0f;
		}
		dane->cNowyPomiar &= ~(NP_WYS1 + NP_WYS2);
		fZc[0] += dane->fWysokoMSL[0];	//wysokość
		fZc[1] += dane->fWariometr[0];	//prędkość pionowa 1
		fZa[0] += dane->fAkcel1[2];		//przyspieszenie bezwzględne 1 w osi Z

		cLicznikUśredniania--;
		if (cLicznikUśredniania == 0)
		{
			fZc[0] /= LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI;
			fZc[1] /= LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI;
			fZa[0] /= LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI;
			dane->nZainicjowano |= INIT_KALMAN_WYSOKOSCI;
			cLicznikUśredniania = LICZBA_PROBEK_USREDNIANIA_KALMANA_WYSOKOSCI;
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

	//zeruj macierze obserwacji H i kowariancji pomiarów R
	for (uint8_t p=0; p<KPCIS; p++)
	{
		for (uint8_t n=0; n<KPCIS; n++)
		{
			fRc1[p][n] = 0.0f;
			fRc2[p][n] = 0.0f;
		}
		for (uint8_t n=0; n<KSTAN; n++)
		{
			fHc1[p][n] = 0.0f;
			fHc2[p][n] = 0.0f;
		}
	}
	fRa1[0][0] = 0.0f;
	fRa2[0][0] = 0.0f;
	for (uint8_t n=0; n<KSTAN; n++)
	{
		fHa1[0][n] = 0.0f;
		fHa2[0][n] = 0.0f;
	}

	//inicjuj pierwszy pomiar i wektor stanu
	fX[0] = fZc[0];		//średnia wysokość
	fX[1] = fZc[1];		//średnia prędkość pionowa
	fX[2] = 0.00001;		//przyspieszenie kinematyczne różne od zera
	fX[3] = dane->fAkcel1[2];		//łączne przyspieszenie w osi Z: grawitacja + bias akcelerometru 1
	fX[4] = dane->fAkcel2[2];		//łączne przyspieszenie w osi Z: grawitacja + bias akcelerometru 2

	arm_mat_init_f32(&mX, KSTAN, 1, fX);
	arm_mat_init_f32(&mZc, KPCIS, 1, fZc);
	arm_mat_init_f32(&mZa, KPACC, 1, fZa);

	//wariancja jest kwadratem standardowego odchylenia pomiaru i jest rozmieszczona w głównej przekątnej macierzy
	//pozostałe pola sa kowariancją, czyli zależnością między błędami jednego pomiaru a drugiego. Zakładam że
	//błędy pomiarów są niezależne, więc kowariancja jest ustawiona na 0. W rzeczywistosci pomiar prędkości jest liczony
	//z danych czujnika wysokości więc korelacja istnieje ale na razie nie potrafię jej obliczyć
	fRc1[0][0] = 0.068;	//wariancja statycznego pomiaru wysokości czujnika 1 [m^2]
	fRc1[1][1] = 0.068;	//wariancja statycznego pomiaru wysokości czujnika 2 [m^2]
	fRc2[0][0] = 2.097;	//wariancja statycznego pomiaru prędkości wariometru 1 [m^2/s^2]
	fRc2[1][1] = 2.097;	//wariancja statycznego pomiaru prędkości wariometru 2 [m^2/s^2]
	fRa1[0][0] = 0.399;	//wariancja statycznego pomiaru przyspieszenia akcelerometru 1 [m^2/s^4]
	fRa2[0][0] = 0.399;	//wariancja statycznego pomiaru przyspieszenia akcelerometru 2 [m^2/s^4]
	arm_mat_init_f32(&mRc1, KPCIS, KPCIS, &fRc1[0][0]);
	arm_mat_init_f32(&mRc2, KPCIS, KPCIS, &fRc2[0][0]);
	arm_mat_init_f32(&mRa1, KPACC, KPACC, &fRa1[0][0]);
	arm_mat_init_f32(&mRa2, KPACC, KPACC, &fRa2[0][0]);

	//początkowa wariancja predykcji
	fP[0][0] = 0.055;
	fP[1][1] = 1.0e-2;
	fP[2][2] = 6.0e-2;
	fP[3][3] = 1.0e-2;
	fP[4][4] = 1.0e-2;
	arm_mat_init_f32(&mP, KSTAN, KSTAN, &fP[0][0]);

	//macierz przejścia oblicza wartość predykcji następnego stanu
	fF[0][0] = 1.0f;				//wysokość = poprzednia wysokość
	fF[0][1] = OKRES_PETLI_GLOWNEJ;	//wysokość = prędkość * dT
	fF[0][2] = powf(OKRES_PETLI_GLOWNEJ, 2) / 2;	//wysokość = przyspieszenie * dT^2/2
	fF[1][1] = 1.0f;				//prędkość = poprzednia prędkość
	fF[1][2] = OKRES_PETLI_GLOWNEJ;	//prędkość = przyspieszenie * dT
	fF[2][2] = 1.0f;				//przyspieszenie = poprzednie przyspieszenie
	fF[3][3] = 1.0f;				//bias przyspieszenia 1 = poprzedni bias przyspieszenia 1
	fF[4][4] = 1.0f;				//bias przyspieszenia 2 = poprzedni bias przyspieszenia 2
	arm_mat_init_f32(&mF, KSTAN, KSTAN, &fF[0][0]);

	//inicjalizacja szumu procesu. Zakładam że szum procesu zależy od podchodnej przyspieszenia, czyli zrywu
	//Q = sigma^2 * G * G^T
	//macierz zależy do dT, więc wypełniam ją podczas predykcji
	arm_mat_init_f32(&mQ, KSTAN, KSTAN, &fQ[0][0]);

	//inicjalizacja macierzy jednostkowej
	for (uint8_t n=0; n<KSTAN; n++)
		fI[n][n] = 1.0f;
	arm_mat_init_f32(&mI, KSTAN, KSTAN, &fI[0][0]);

	//inicjalizacja obu macierzy obserwacji: Hc - dane o wysokości i prędkości z czujnika ciśnienia  oraz Ha - przyspieszenie
	fHc1[0][0] = 1.0f;		//wysokość obserwuje czujnik wysokości 1
	fHc1[1][1] = 1.0f;		//prędkość obserwuje wariometr 1
	arm_mat_init_f32(&mHc1, KPCIS, KSTAN, &fHc1[0][0]);

	fHc2[0][0] = 1.0f;		//wysokość obserwuje czujnik wysokości 2
	fHc2[1][1] = 1.0f;		//prędkość obserwuje wariometr  2
	arm_mat_init_f32(&mHc2, KPCIS, KSTAN, &fHc2[0][0]);

	fHa1[0][2] = 1.0f;		//przyspieszenie obserwuje oś Z akceletrometru 1
	fHa1[0][3] = 1.0f;		//bias 1 obserwuje oś Z akceletrometru 1
	arm_mat_init_f32(&mHa1, KPACC, KSTAN, &fHa1[0][0]);

	fHa2[0][2] = 1.0f;		//przyspieszenie obserwuje oś Z akceletrometru 2
	fHa2[0][4] = 1.0f;		//bias 2 obserwuje oś Z akceletrometru 2
	arm_mat_init_f32(&mHa2, KPACC, KSTAN, &fHa2[0][0]);
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
uint8_t PredykcjaFiltraKalmanaWysokości5X6Z(stWymianyCM4_t *dane)
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

	for (uint8_t n=0; n<5; n++)
		dane->stKalmanDebug.fX[n] = fX[n];

	//2) Obliczenie niepewności nowej estymaty wektora stanu: Temp1 = F * P(n)
	cBłąd |= arm_mat_mult_f32(&mF, &mP, &mTempSSA);

	//transpozycja macierzy F: Temp2 = F^T
	cBłąd |= arm_mat_trans_f32(&mF, &mTempSSB);

	//mnożenie Temp3 = (F * P(n)) * (F^T)
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSB, &mTempSSC);

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
	cBłąd |= arm_mat_add_f32(&mQ, &mTempSSC, &mP);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru wysokości i prędkości czujnka ciśnienia 1
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaCzujnikiemCiśnienia1FiltraKalmanaWysokości5X6Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//liczę współczynnik wzmocnienia Kalmana: mK,
	//najpierw transponowane H -> mTempSPc	 [Pomiar x Stan] -> [Stan x Pomiar]
	cBłąd |= arm_mat_trans_f32(&mHc1, &mTempSPc);

	// P(n-1) * (H^T) -> mPHc				[Stan x Stan] * [Stan x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mP, &mTempSPc, &mPHc);

	//H * (P(n-1)*H^T) -> fTempPPcA			[Pomiar x Stan] * [Stan x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mHc1, &mPHc, &mTempPPcA);

	//(H*P(n-1)*H^T) + R(n) -> fTempPPcB	[Pomiar x Pomiar] + [Pomiar x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_add_f32(&mTempPPcA, &mRc1, &mTempPPcB);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> fTempPPcA	[Pomiar x Pomiar] -> [Pomiar x Pomiar]
	cBłąd |= arm_mat_inverse_f32(&mTempPPcB, &mTempPPcA);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mPHc, &mTempPPcA, &mKc1);
	dane->stKalmanDebug.fK[0] = fKc1[0][0];	//wpływuwysokości czujnka 1 na estymowaną wysokość

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1) [Pomiar x Stan] * [Stan] = [Pomiar]
	cBłąd |= arm_mat_mult_f32(&mHc1, &mX, &mTempPc1A);

	//Uwzględnienie pomiaru: (z(n) - H * X(n-1))
	fZc[0] = dane->fWysokoMSL[0];	//wysokość
	fZc[1] = dane->fWariometr[0];	//prędkość pionowa

	//innowacja: z(n) - (H*X(n-1)) ->mTempPc1B		[Pomiar] - [Pomiar] = [Pomiar]
	cBłąd |= arm_mat_sub_f32(&mZc, &mTempPc1A, &mTempPc1B);
	dane->stKalmanDebug.fP[0] = fTempPc1B[0];	//w zmiennej fP zachowaj innowację wysokości 1

	//mnożenie przez K: K(n) * (z(n)-H*X(n-1))		[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mKc1, &mTempPc1B, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//przepisanie estymaty do wektora stanu
	for (uint8_t n=0; n<KSTAN; n++)
		fX[n] = fTempS1B[n];

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempSSA	 [Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mKc1, &mHc1, &mTempSSA);

	//odejmowanie (I - K(n) * H) -> mTempSSB
	cBłąd |= arm_mat_sub_f32(&mI, &mTempSSA, &mTempSSB);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempSSA 	[Stan x Stan] * [Stan x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSSB, &mP, &mTempSSA);

	//transpozycja: (I-K(n)*H)^T-> mTempSSC
	cBłąd |= arm_mat_trans_f32(&mTempSSB, &mTempSSC);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSC, &mTempSSB);

	//mnożenie 	K(n) * R(n)  -> mTempSPc  	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mKc1, &mRc1, &mTempSPc);

	//transpozycja K(n)^T -> mTempM24  		[Stan x Pomiar] -> [Pomiar x Stan]
	cBłąd |= arm_mat_trans_f32(&mKc1, &mTempPcS);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T) 	 	[Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSPc, &mTempPcS, &mTempSSA);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempSSB, &mTempSSA, &mP);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru wysokości i prędkości czujnka ciśnienia 1
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaCzujnikiemCiśnienia2FiltraKalmanaWysokości5X6Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//liczę współczynnik wzmocnienia Kalmana: mKc2
	//najpierw transponowane H -> mTempSPc	 [Pomiar x Stan] -> [Stan x Pomiar]
	cBłąd |= arm_mat_trans_f32(&mHc2, &mTempSPc);

	// P(n-1) * (H^T) -> mPHc				[Stan x Stan] * [Stan x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mP, &mTempSPc, &mPHc);

	//H * (P(n-1)*H^T) -> fTempPPcA			[Pomiar x Stan] * [Stan x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mHc2, &mPHc, &mTempPPcA);

	//(H*P(n-1)*H^T) + R(n) -> fTempPPcB	[Pomiar x Pomiar] + [Pomiar x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_add_f32(&mTempPPcA, &mRc2, &mTempPPcB);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> fTempPPcA	[Pomiar x Pomiar] -> [Pomiar x Pomiar]
	cBłąd |= arm_mat_inverse_f32(&mTempPPcB, &mTempPPcA);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mKc2	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mPHc, &mTempPPcA, &mKc2);
	dane->stKalmanDebug.fK[1] = fKc2[0][1];	//wpływ wysokości czujnka 2 na estymowaną wysokość

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1) [Pomiar x Stan] * [Stan] = [Pomiar]
	cBłąd |= arm_mat_mult_f32(&mHc2, &mX, &mTempPc1A);

	//Uwzględnienie pomiaru: (z(n) - H * X(n-1))
	fZc[0] = dane->fWysokoMSL[1];	//wysokość
	fZc[1] = dane->fWariometr[1];	//prędkość pionowa

	//innowacja: z(n) - (H*X(n-1)) ->mTempPc1B		[Pomiar] - [Pomiar] = [Pomiar]
	cBłąd |= arm_mat_sub_f32(&mZc, &mTempPc1A, &mTempPc1B);
	dane->stKalmanDebug.fP[1] = fTempPc1B[0];	//w zmiennej fP zachowaj innowację wysokości 1

	//mnożenie przez K: K(n) * (z(n)-H*X(n-1))		[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mKc2, &mTempPc1B, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//przepisanie estymaty do wektora stanu
	for (uint8_t n=0; n<KSTAN; n++)
		fX[n] = fTempS1B[n];

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempSSA	 [Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mKc2, &mHc2, &mTempSSA);

	//odejmowanie (I - K(n) * H) -> mTempSSB
	cBłąd |= arm_mat_sub_f32(&mI, &mTempSSA, &mTempSSB);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempSSA 	[Stan x Stan] * [Stan x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSSB, &mP, &mTempSSA);

	//transpozycja: (I-K(n)*H)^T-> mTempSSC
	cBłąd |= arm_mat_trans_f32(&mTempSSB, &mTempSSC);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSC, &mTempSSB);

	//mnożenie 	K(n) * R(n)  -> mTempSPc  	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mKc2, &mRc2, &mTempSPc);

	//transpozycja K(n)^T -> mTempM24  		[Stan x Pomiar] -> [Pomiar x Stan]
	cBłąd |= arm_mat_trans_f32(&mKc2, &mTempPcS);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T) 	 	[Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSPc, &mTempPcS, &mTempSSA);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempSSB, &mTempSSA, &mP);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru przyspieszenia akcelerometru 1
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaAkcelerometrem1FiltraKalmanaWysokości5X6Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//liczę współczynnik wzmocnienia Kalmana: mKa1,
	//najpierw transponowane H -> mTempSPa	 [Pomiar x Stan] -> [Stan x Pomiar]
	cBłąd |= arm_mat_trans_f32(&mHa1, &mTempSPa);

	// P(n-1) * (H^T) -> mPHa				[Stan x Stan] * [Stan x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mP, &mTempSPa, &mPHa);

	//H * (P(n-1)*H^T) -> fTempPPaA			[Pomiar x Stan] * [Stan x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mHa1, &mPHa, &mTempPPaA);

	//(H*P(n-1)*H^T) + R(n) -> fTempPPaB	[Pomiar x Pomiar] + [Pomiar x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_add_f32(&mTempPPaA, &mRa1, &mTempPPaB);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> fTempPPaA	[Pomiar x Pomiar] -> [Pomiar x Pomiar]
	cBłąd |= arm_mat_inverse_f32(&mTempPPaB, &mTempPPaA);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mPHa, &mTempPPaA, &mKa1);
	dane->stKalmanDebug.fK[2] = fKa1[2][4];	//wpływ akcelrometru 1 na estymowane przyspieszenie

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1) [Pomiar x Stan] * [Stan] = [Pomiar]
	cBłąd |= arm_mat_mult_f32(&mHa1, &mX, &mTempPa1A);

	//Innowacja: z(n) - (H*X(n-1)) ->mTempPa1B		[Pomiar] - [Pomiar] = [Pomiar]
	fZa[0] = dane->fAkcel1[2];		//pomiar: Przyspieszenie bezwzględne osi Z
	cBłąd |= arm_mat_sub_f32(&mZa, &mTempPa1A, &mTempPa1B);
	dane->stKalmanDebug.fP[2] = fTempPa1B[0];	//w zmiennej fP zachowaj innowację przyspieszenia 1

	//mnożenie przez K: K(n) * (z(n)-H*X(n-1))		[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mKa1, &mTempPa1B, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//przepisanie estymaty do wektora stanu
	for (uint8_t n=0; n<KSTAN; n++)
		fX[n] = fTempS1B[n];

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempSSA	 [Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mKa1, &mHa1, &mTempSSA);

	//odejmowanie (I - K(n) * H) -> mTempSSB
	cBłąd |= arm_mat_sub_f32(&mI, &mTempSSA, &mTempSSB);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempSSA 	[Stan x Stan] * [Stan x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSSB, &mP, &mTempSSA);

	//transpozycja: (I-K(n)*H)^T-> mTempSSC
	cBłąd |= arm_mat_trans_f32(&mTempSSB, &mTempSSC);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSC, &mTempSSB);

	//mnożenie 	K(n) * R(n)  -> mTempSPa  	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mKa1, &mRa1, &mTempSPa);

	//transpozycja K(n)^T -> mTempM24  		[Stan x Pomiar] -> [Pomiar x Stan]
	cBłąd |= arm_mat_trans_f32(&mKa1, &mTempPaS);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T) 	 	[Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSPa, &mTempPaS, &mTempSSA);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempSSB, &mTempSSA, &mP);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru przyspieszenia akcelerometru 2
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaAkcelerometrem2FiltraKalmanaWysokości5X6Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//liczę współczynnik wzmocnienia Kalmana: mKa2,
	//najpierw transponowane H -> mTempSPa	 [Pomiar x Stan] -> [Stan x Pomiar]
	cBłąd |= arm_mat_trans_f32(&mHa2, &mTempSPa);

	// P(n-1) * (H^T) -> mPHa				[Stan x Stan] * [Stan x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mP, &mTempSPa, &mPHa);

	//H * (P(n-1)*H^T) -> fTempPPaA			[Pomiar x Stan] * [Stan x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mHa2, &mPHa, &mTempPPaA);

	//(H*P(n-1)*H^T) + R(n) -> fTempPPaB	[Pomiar x Pomiar] + [Pomiar x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_add_f32(&mTempPPaA, &mRa2, &mTempPPaB);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> fTempPPaA	[Pomiar x Pomiar] -> [Pomiar x Pomiar]
	cBłąd |= arm_mat_inverse_f32(&mTempPPaB, &mTempPPaA);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mKa2	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mPHa, &mTempPPaA, &mKa2);
	dane->stKalmanDebug.fK[3] = fKa2[2][5];	//wpływ akcelrometru 2 na estymowane przyspieszenie

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1) [Pomiar x Stan] * [Stan] = [Pomiar]
	cBłąd |= arm_mat_mult_f32(&mHa2, &mX, &mTempPa1A);

	//Innowacja: z(n) - (H*X(n-1)) ->mTempPa1B		[Pomiar] - [Pomiar] = [Pomiar]
	fZa[0] = dane->fAkcel2[2];		//pomiar: Przyspieszenie bezwzględne osi Z
	cBłąd |= arm_mat_sub_f32(&mZa, &mTempPa1A, &mTempPa1B);
	dane->stKalmanDebug.fP[3] = fTempPa1B[0];	//w zmiennej fP zachowaj innowację przyspieszenia 2

	//mnożenie przez K: K(n) * (z(n)-H*X(n-1))		[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mKa2, &mTempPa1B, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//przepisanie estymaty do wektora stanu
	for (uint8_t n=0; n<KSTAN; n++)
		fX[n] = fTempS1B[n];

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempSSA	 [Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mKa2, &mHa2, &mTempSSA);

	//odejmowanie (I - K(n) * H) -> mTempSSB
	cBłąd |= arm_mat_sub_f32(&mI, &mTempSSA, &mTempSSB);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempSSA 	[Stan x Stan] * [Stan x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSSB, &mP, &mTempSSA);

	//transpozycja: (I-K(n)*H)^T-> mTempSSC
	cBłąd |= arm_mat_trans_f32(&mTempSSB, &mTempSSC);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSC, &mTempSSB);

	//mnożenie 	K(n) * R(n)  -> mTempSPa  	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mKa2, &mRa2, &mTempSPa);

	//transpozycja K(n)^T -> mTempM24  		[Stan x Pomiar] -> [Pomiar x Stan]
	cBłąd |= arm_mat_trans_f32(&mKa2, &mTempPaS);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T) 	 	[Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSPa, &mTempPaS, &mTempSSA);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempSSB, &mTempSSA, &mP);
	return cBłąd;
}

