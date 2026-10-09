//////////////////////////////////////////////////////////////////////////////
//
// Symulator pracy wyrzutni BSP przeznaczony do opracowania filtra Kalmana
// mierzącego prędkości wózka wyrzutni z obu rolek
//
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "Wyrzutnia.h"
#include <stdlib.h>
#include <LCD/ILI9488.h>
#include <arm_math.h>

float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fPrędkośćRolki1[LICZBA_POMIAROW];
float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fPrędkośćRolki2[LICZBA_POMIAROW];
float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fDrogaRolki1[LICZBA_POMIAROW];
float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fDrogaRolki2[LICZBA_POMIAROW];
static float32_t fX[KSTAN];					//wektor stanu: 0=droga, 1=prędkość, 2=przyspieszenie Z, 3=błąd drogi rolki 1, 4-błąd drogi rolki 2
static float32_t fZ[KPOM];					//wektor pomiaru: 0=droga, 1=prędkość
static float32_t fF[KSTAN][KSTAN];			//macierz przejścia wektora stanu
static float32_t fP[KSTAN][KSTAN];			//macierz kowariancji predykcji
static float32_t fR1[KPOM][KPOM];			//macierz kowariancji pomiaru prędkości z rolki 1
static float32_t fR2[KPOM][KPOM];			//macierz kowariancji pomiaru prędkości z rolki 2
static float32_t fQ[KSTAN][KSTAN];			//macierz szumu procesu
static float32_t fI[KSTAN][KSTAN];			//macierz jednostkowa
static float32_t fH1[KPOM][KSTAN];			//macierz obserwacji prędkości rolki 1
static float32_t fH2[KPOM][KSTAN];			//macierz obserwacji prędkości rolki 2
static float32_t fK1[KSTAN][KPOM];			//macierz wzmocnień Kalmana rolki 1
static float32_t fK2[KSTAN][KPOM];			//macierz wzmocnień Kalmana rolki 2
static float32_t fPH[KSTAN][KPOM];			//macierz [Stan x Pomiar] na wyniki pośrednie
static float32_t fTempSPc[KSTAN][KPOM];		//macierz [Stan x Pomiar] na wyniki pośrednie
static float32_t fTempPPcA[KPOM][KPOM];		//macierz [Pomiar x Pomiar] na wyniki pośrednie A
static float32_t fTempPPcB[KPOM][KPOM];		//macierz [Pomiar x Pomiar] na wyniki pośrednie B
static float32_t fTempPc1A[KPOM];			//wektor [Pomiar] na wyniki pośrednie A
static float32_t fTempPc1B[KPOM];			//wektor [Pomiar] na wyniki pośrednie B
static float32_t fTempPcS[KPOM][KSTAN];		//macierz [Pomiar x Stan] na wyniki pośrednie
static float32_t fTempSSA[KSTAN][KSTAN];	//macierz [Stan x Stan] na wyniki pośrednie A
static float32_t fTempSSB[KSTAN][KSTAN];	//macierz [Stan x Stan] na wyniki pośrednie B
static float32_t fTempSSC[KSTAN][KSTAN];	//macierz [Stan x Stan] na wyniki pośrednie C
static float32_t fTempS1A[KSTAN];			//wektor [Stan] na wyniki pośrednie A
static float32_t fTempS1B[KSTAN];			//wektor [Stan] na wyniki pośrednie B

//zmienne z przedrostkiem m oznaczają macierze (lub wektory) w formacie biblioteki ARM DSP
static arm_matrix_instance_f32 mX  = {KSTAN, 1, fX};				//wektor stanu
static arm_matrix_instance_f32 mZ  = {KPOM, 1, fZ};					//wektor pomiaru: prędkość
static arm_matrix_instance_f32 mF  = {KSTAN, KSTAN, &fF[0][0]};		//macierz przejścia wektora stanu
static arm_matrix_instance_f32 mP  = {KSTAN, KSTAN, &fP[0][0]};		//macierz kowariancji predykcji
static arm_matrix_instance_f32 mR1 = {KPOM, KPOM, &fR1[0][0]};		//macierz kowariancji pomiaru rolką 1
static arm_matrix_instance_f32 mR2 = {KPOM, KPOM, &fR2[0][0]};		//macierz kowariancji pomiaru rolka 2
static arm_matrix_instance_f32 mQ  = {KSTAN, KSTAN, &fQ[0][0]};		//macierz szumu procesu
static arm_matrix_instance_f32 mI  = {KSTAN, KSTAN, &fI[0][0]};		//macierz jednostkowa
static arm_matrix_instance_f32 mH1 = {KPOM, KSTAN, &fH1[0][0]};		//macierz obserwacji prędkości z rolki 1
static arm_matrix_instance_f32 mH2 = {KPOM, KSTAN, &fH2[0][0]};		//macierz obserwacji prędkości z rolk1 2
static arm_matrix_instance_f32 mK1 = {KSTAN, KPOM, &fK1[0][0]};		//macierz wzmocnień Kalmana czujnika ciśnienia 1
static arm_matrix_instance_f32 mK2 = {KSTAN, KPOM, &fK2[0][0]};		//macierz wzmocnień Kalmana czujnika ciśnienia 2
static arm_matrix_instance_f32 mPH = {KSTAN, KPOM, &fPH[0][0]};		//macierz Stan x Pomiar na iloczyn P*Hc
static arm_matrix_instance_f32 mTempSPc  = {KSTAN, KPOM, &fTempSPc[0][0]};
static arm_matrix_instance_f32 mTempPPcA = {KPOM, KPOM, &fTempPPcA[0][0]};	//macierz Pomiar x Pomiar na wyniki pośrednie A
static arm_matrix_instance_f32 mTempPPcB = {KPOM, KPOM, &fTempPPcB[0][0]};	//macierz Pomiar x Pomiar na wyniki pośrednie B
static arm_matrix_instance_f32 mTempPc1A = {KPOM, 1, fTempPc1A};			//macierz Pomiar x 1 na wyniki pośrednie A
static arm_matrix_instance_f32 mTempPc1B = {KPOM, 1, fTempPc1B};			//macierz Pomiar x 1 na wyniki pośrednie B
static arm_matrix_instance_f32 mTempPcS  = {KPOM, KSTAN, &fTempPcS[0][0]};	//macierz Pomiar x Stan na wyniki pośrednie
static arm_matrix_instance_f32 mTempSSA  = {KSTAN, KSTAN, &fTempSSA[0][0]};	//macierz Stan x Stan na wyniki pośrednie A
static arm_matrix_instance_f32 mTempSSB  = {KSTAN, KSTAN, &fTempSSB[0][0]};	//macierz Stan x Stan na wyniki pośrednie B
static arm_matrix_instance_f32 mTempSSC  = {KSTAN, KSTAN, &fTempSSC[0][0]};	//macierz Stan x Stan na wyniki pośrednie C
static arm_matrix_instance_f32 mTempS1A  = {KSTAN, 1, fTempS1A};			//macierz Stan x 1 na wyniki pośrednie A
static arm_matrix_instance_f32 mTempS1B  = {KSTAN, 1, fTempS1B};			//macierz Stan x 1 na wyniki pośrednie B

extern uint8_t cRysujRaz;
extern char cNapis[100];
extern RNG_HandleTypeDef hrng;



////////////////////////////////////////////////////////////////////////////////
// Zewnętzne wywołanie sumulacji i danych i filtra kalmana prędkości obu rolek
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t TestFiltraPrędkościRolek(void)
{
	uint8_t cBłąd = BLAD_OK;
	uint16_t x1, x2, y11, y12, y21, y22;
	float fSkalaY;
	float fCzas1[LICZBA_POMIAROW], fCzas2[LICZBA_POMIAROW];	//czasy pomiaru obu rolek
	uint16_t sLicznikPomiarów1, sLicznikPomiarów2;
	float fCzas;

	if (cRysujRaz)
	{
		cRysujRaz = 0;
		sprintf(cNapis, "Symulacja pr%cdko%cci w%czka", ę, ś, ó);
		BelkaTytulu(cNapis);
		setColor(SZARY60);
		//sprintf(cNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		//RysujNapis(cNapis, CENTER, 300);

	}
	setColor(BIALY);
	InicjujFiltrKalmanaPrędkościRolek5X2Z();

	SymulujPrędkościRolek(fDrogaRolki1, fDrogaRolki2, fPrędkośćRolki1, fPrędkośćRolki2, fCzas1, fCzas2);
	for (uint8_t n=0; n<14; n++)
	{
		sprintf(cNapis, "v1 = %.2f ", fPrędkośćRolki1[n]);
		RysujNapis(cNapis, 0, 30 + n * 20);
		sprintf(cNapis, "v2 = %.2f", fPrędkośćRolki2[n]);
		RysujNapis(cNapis, 120, 30 + n * 20);
	}
	for (uint8_t n=0; n<14; n++)
	{
		sprintf(cNapis, "v1 = %.2f ", fPrędkośćRolki1[n+14]);
		RysujNapis(cNapis, 240, 30 + n * 20);
		sprintf(cNapis, "v2 = %.2f", fPrędkośćRolki2[n+14]);
		RysujNapis(cNapis, 360, 30 + n * 20);
	}

	//uruchom filtr Kalmana ze stałym krokiem czasu równym 1 ms
	sLicznikPomiarów1 = sLicznikPomiarów2 = 0;
	for (uint16_t n=1; n<1300; n++)
	{
		fCzas = (float)n / 1000;	//czas w sekundach
		PredykcjaFiltraKalmanaPrędkościRolek5X2Z(fCzas);

		//aktualizacja pomiarem rolki 1
		if (fCzas >= fCzas1[sLicznikPomiarów1])
		{
			AktulizacjaRolką1FiltraKalmanaPrędkościRolek5X2Z(fDrogaRolki1[sLicznikPomiarów1], fPrędkośćRolki1[sLicznikPomiarów1]);
			sLicznikPomiarów1++;
		}

		//aktualizacja pomiarem rolki 2
		if (fCzas >= fCzas2[sLicznikPomiarów2])
		{
			AktulizacjaRolką2FiltraKalmanaPrędkościRolek5X2Z(fDrogaRolki2[sLicznikPomiarów2], fPrędkośćRolki2[sLicznikPomiarów2]);
			sLicznikPomiarów2++;
		}
	}



	//rysuj wykres
	x1 = 0;
	y11 = y21 = DISP_Y_SIZE;
	fSkalaY = DISP_Y_SIZE / fPrędkośćRolki1[LICZBA_POMIAROW - 1];	//jednostka pikseli na m/s
	for (uint8_t n=0; n<LICZBA_POMIAROW; n++)
	{
		x2 = x1 + SKOK_X_WYKRESU;
		y12 = DISP_Y_SIZE - (uint16_t)(fPrędkośćRolki1[n] * fSkalaY);
		y22 = DISP_Y_SIZE - (uint16_t)(fPrędkośćRolki2[n] * fSkalaY);
		setColor(CZERWONY);
		RysujLinie(x1, y11, x2, y12);
		setColor(ZOLTY);
		RysujLinie(x1, y21, x2, y22);
		x1 = x2;
		y11 = y12;
		y21 = y22;
	}


	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja symuluje dane pomiarowe prędkości obu rolek wyrzutni
// Zakładam siłę napędzajacą jako iloczyn masy BSP i przyspieszenia
// Parametry: *fVRolki1, *fVRolki2 - wskaźniki na symulowane prędkości obu rolek
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t SymulujPrędkościRolek(float *fDrogaRolki1, float *fDrogaRolki2, float *fVRolki1, float *fVRolki2, float *fCzas1, float *fCzas2)
{
	uint8_t cBłąd = BLAD_OK;
	float fPredkość = 0.0f;
	uint32_t nRrandom32;
	float fCzasLokalny1, fCzasLokalny2;

	fCzasLokalny1 = fCzasLokalny2 = 0;
	for (uint16_t n=0; n<LICZBA_POMIAROW; n++)
	{
		fPredkość = sqrtf(fPredkość * fPredkość + 2 * PRZYSP_STARTOWE * OBWOD_ROLKI / 1000);
		cBłąd = HAL_RNG_GenerateRandomNumber(&hrng, &nRrandom32);	//losuj liczbę
		*(fVRolki1 + n) = fPredkość + (nRrandom32 & 0xFF) / 1000.0f;
		*(fDrogaRolki1 + n) = (n + 1) * (float)OBWOD_ROLKI / 1000;
		fCzasLokalny1 += ((float)OBWOD_ROLKI / 1000) / *(fVRolki1 + n);
		*(fCzas1 + n) = fCzasLokalny1;


		cBłąd = HAL_RNG_GenerateRandomNumber(&hrng, &nRrandom32);	//losuj liczbę
		*(fVRolki2 + n) = fPredkość + (nRrandom32 & 0xFF) / 10000.0f;
		*(fDrogaRolki2 + n) = (n + 1) * (float)OBWOD_ROLKI / 1000;
		fCzasLokalny2 += ((float)OBWOD_ROLKI / 1000) / *(fVRolki2 + n);
		*(fCzas2 + n) = fCzasLokalny2;

	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje liniowy filtr Kalmana do estymacji prędkości rolek wózka wyrzutnie
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFiltrKalmanaPrędkościRolek5X2Z(void)
{
	uint8_t cBłąd = BLAD_OK;

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
	for (uint8_t p=0; p<KPOM; p++)
	{
		for (uint8_t n=0; n<KPOM; n++)
		{
			fR1[p][n] = 0.0f;
			fR2[p][n] = 0.0f;
		}
		for (uint8_t n=0; n<KSTAN; n++)
		{
			fH1[p][n] = 0.0f;
			fH2[p][n] = 0.0f;
		}
	}


	//inicjuj pierwszy pomiar i wektor stanu
	fX[0] = 0.00001f;		//droga
	fX[1] = 0.00001f;		//prędkość
	fX[2] = 0.00001f;		//przyspieszenie
	fX[3] = 0.00001f;		//błąd prędkości rolki 1
	fX[4] = 0.00001f;		//błąd prędkości rolki 2

	arm_mat_init_f32(&mX, KSTAN, 1, fX);
	arm_mat_init_f32(&mZ, KPOM, 1, fZ);


	//wariancja jest kwadratem standardowego odchylenia pomiaru i jest rozmieszczona w głównej przekątnej macierzy
	//pozostałe pola sa kowariancją, czyli zależnością między błędami jednego pomiaru a drugiego. Zakładam że
	//błędy pomiarów są niezależne, więc kowariancja jest ustawiona na 0. W rzeczywistosci pomiar prędkości jest liczony
	//z danych czujnika wysokości więc korelacja istnieje ale na razie nie potrafię jej obliczyć
	fR1[0][0] = 0.056;	//wariancja statycznego pomiaru drogi rolki 1 [m^2]
	fR1[1][1] = 0.407;	//wariancja statycznego pomiaru prędkości rolki 1 [m^2/s^2]
	fR2[0][0] = 0.0093;	//wariancja statycznego pomiaru drogi rolki 3 [m^2]
	fR2[1][1] = 0.0016;	//wariancja statycznego pomiaru prędkości rolki 2 [m^2/s^2]
	arm_mat_init_f32(&mR1, KPOM, KPOM, &fR1[0][0]);
	arm_mat_init_f32(&mR2, KPOM, KPOM, &fR2[0][0]);

	//początkowa wariancja predykcji
	fP[0][0] = 0.055;
	fP[1][1] = 1.0e-2;
	fP[2][2] = 6.0e-2;
	fP[3][3] = 1.0e-2;
	fP[4][4] = 1.0e-2;
	arm_mat_init_f32(&mP, KSTAN, KSTAN, &fP[0][0]);

	//macierz przejścia oblicza wartość predykcji następnego stanu
	fF[0][0] = 1.0f;				//droga = poprzednia droga
	fF[0][1] = OKRES_PETLI_GLOWNEJ;	//droga = prędkość * dT
	fF[0][2] = powf(OKRES_PETLI_GLOWNEJ, 2) / 2;	//droga = przyspieszenie * dT^2/2
	fF[1][1] = 1.0f;				//droga = poprzednia prędkość
	fF[1][2] = OKRES_PETLI_GLOWNEJ;	//prędkość = przyspieszenie * dT
	fF[2][2] = 1.0f;				//przyspieszenie = poprzednie przyspieszenie
	fF[3][3] = 1.0f;				//błąd prędkości rolki 1
	fF[4][4] = 1.0f;				//błąd prędkości rolki 2
	arm_mat_init_f32(&mF, KSTAN, KSTAN, &fF[0][0]);

	//inicjalizacja szumu procesu. Zakładam że szum procesu zależy od podchodnej przyspieszenia, czyli zrywu
	//Q = sigma^2 * G * G^T
	//macierz zależy do dT, więc wypełniam ją podczas predykcji
	arm_mat_init_f32(&mQ, KSTAN, KSTAN, &fQ[0][0]);

	//inicjalizacja macierzy jednostkowej
	for (uint8_t n=0; n<KSTAN; n++)
		fI[n][n] = 1.0f;
	arm_mat_init_f32(&mI, KSTAN, KSTAN, &fI[0][0]);

	//inicjalizacja obu macierzy obserwacji, takiej samej dla obu rolek
	fH1[0][0] = 1.0f;		//droga z rolki jest obserwowana przez stan drogi rolki 1
	fH1[0][3] = 1.0f;		//droga z rolki jest obserwowana przez stan błędu drogi rolki 1
	fH1[1][1] = 1.0f;		//prędkość z rolki jest obserwowana przez stan prędkości rolki 1
	arm_mat_init_f32(&mH1, KPOM, KSTAN, &fH1[0][0]);

	fH2[0][0] = 1.0f;		//droga z rolki jest obserwowana przez stan drogi rolki 2
	fH2[0][3] = 1.0f;		//droga z rolki jest obserwowana przez stan błędu drogi rolki 2
	fH2[1][1] = 1.0f;		//prędkość z rolki jest obserwowana przez stan prędkości rolki 2
	arm_mat_init_f32(&mH2, KPOM, KSTAN, &fH2[0][0]);

	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja estymuje nowe wartości wektora stanu ze etapu (n) na (n+1)
// x(n+1) = F * x(n) + w. Nie ma G * u(n) bo w tym modelu nie ma sterowania
// oraz wwykonuje predykcję kowariancji (niepewności) nowej wartości:
// P(n+1) = F * P(n) * F^T + Q
// Parametry: fDeltaCzasu - czas w sekundach od ostatniego pomiaru
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PredykcjaFiltraKalmanaPrędkościRolek5X2Z(float fDeltaCzasu)
{
	uint8_t cBłąd = BLAD_OK;

	//aktualizuj wartości macierzy F zależne od czasu
	fF[0][1] = fF[1][2] = fDeltaCzasu;	//wysokość = prędkość * dT oraz prędkość = przyspieszenie * dT
	fF[0][2] = powf(fDeltaCzasu, 2) / 2;	//wysokość = przyspieszenie * dT^2/2

	//1) Predykcja nowej estymaty wektora stanu: x(n+1) = F * x(n)
	cBłąd |= arm_mat_mult_f32(&mF, &mX, &mX);

	//2) Obliczenie niepewności nowej estymaty wektora stanu: Temp1 = F * P(n)
	cBłąd |= arm_mat_mult_f32(&mF, &mP, &mTempSSA);

	//transpozycja macierzy F: Temp2 = F^T
	cBłąd |= arm_mat_trans_f32(&mF, &mTempSSB);

	//mnożenie Temp3 = (F * P(n)) * (F^T)
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSB, &mTempSSC);

	//obliczenie szumu procesu Q. Zakładam że szum procesu zależy od wariancji zrywu akcelerometru  [m/s^3]^2 = [m^2/s^6]
	float32_t fOkresPetli = fDeltaCzasu / 1e6;	//czas od ostatniego wykonania w [sekundach]
	fQ[0][0] = powf(fOkresPetli, 6) / 36 * WARIANCJA_ZRYWUL;
	fQ[0][1] = powf(fOkresPetli, 5) / 12 * WARIANCJA_ZRYWUL;
	fQ[0][2] = powf(fOkresPetli, 4) / 6  * WARIANCJA_ZRYWUL;
	fQ[1][0] = powf(fOkresPetli, 5) / 12 * WARIANCJA_ZRYWUL;
	fQ[1][1] = powf(fOkresPetli, 4) / 4  * WARIANCJA_ZRYWUL;
	fQ[1][2] = powf(fOkresPetli, 3) / 2  * WARIANCJA_ZRYWUL;
	fQ[2][0] = powf(fOkresPetli, 4) / 6  * WARIANCJA_ZRYWUL;
	fQ[2][1] = powf(fOkresPetli, 3) / 2  * WARIANCJA_ZRYWUL;
	fQ[2][2] = powf(fOkresPetli, 2) 	 * WARIANCJA_ZRYWUL;
	fQ[3][3] = fOkresPetli * WARIANCJA_POSLIZGU;
	fQ[4][4] = fOkresPetli * WARIANCJA_POSLIZGU;

	//dodaj macierz szumu Q procesu do iloczynu (F * P(n)) * (F^T) -> P
	cBłąd |= arm_mat_add_f32(&mQ, &mTempSSC, &mP);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru prędkości rolki 1
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: fPredkość - prędkość odczytana z rolki 1
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaRolką1FiltraKalmanaPrędkościRolek5X2Z(float fDroga, float fPredkość)
{
	uint8_t cBłąd = BLAD_OK;

	fZ[0] = fPredkość;	//prędkość pionowa

	//liczę współczynnik wzmocnienia Kalmana: mK,
	//najpierw transponowane H -> mTempSPc	 [Pomiar x Stan] -> [Stan x Pomiar]
	cBłąd |= arm_mat_trans_f32(&mH1, &mTempSPc);

	// P(n-1) * (H^T) -> mPHc				[Stan x Stan] * [Stan x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mP, &mTempSPc, &mPH);

	//H * (P(n-1)*H^T) -> fTempPPcA			[Pomiar x Stan] * [Stan x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mH1, &mPH, &mTempPPcA);

	//(H*P(n-1)*H^T) + R(n) -> fTempPPcB	[Pomiar x Pomiar] + [Pomiar x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_add_f32(&mTempPPcA, &mR1, &mTempPPcB);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> fTempPPcA	[Pomiar x Pomiar] -> [Pomiar x Pomiar]
	cBłąd |= arm_mat_inverse_f32(&mTempPPcB, &mTempPPcA);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mPH, &mTempPPcA, &mK1);

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1) [Pomiar x Stan] * [Stan] = [Pomiar]
	cBłąd |= arm_mat_mult_f32(&mH1, &mX, &mTempPc1A);

	//innowacja: z(n) - (H*X(n-1)) ->mTempPc1B		[Pomiar] - [Pomiar] = [Pomiar]
	cBłąd |= arm_mat_sub_f32(&mZ, &mTempPc1A, &mTempPc1B);

	//mnożenie przez K: K(n) * (z(n)-H*X(n-1))		[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mK1, &mTempPc1B, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//przepisanie estymaty do wektora stanu
	for (uint8_t n=0; n<KSTAN; n++)
		fX[n] = fTempS1B[n];

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempSSA	 [Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mK1, &mH1, &mTempSSA);

	//odejmowanie (I - K(n) * H) -> mTempSSB
	cBłąd |= arm_mat_sub_f32(&mI, &mTempSSA, &mTempSSB);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempSSA 	[Stan x Stan] * [Stan x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSSB, &mP, &mTempSSA);

	//transpozycja: (I-K(n)*H)^T-> mTempSSC
	cBłąd |= arm_mat_trans_f32(&mTempSSB, &mTempSSC);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSC, &mTempSSB);

	//mnożenie 	K(n) * R(n)  -> mTempSPc  	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mK1, &mR1, &mTempSPc);

	//transpozycja K(n)^T -> mTempM24  		[Stan x Pomiar] -> [Pomiar x Stan]
	cBłąd |= arm_mat_trans_f32(&mK1, &mTempPcS);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T) 	 	[Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSPc, &mTempPcS, &mTempSSA);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempSSB, &mTempSSA, &mP);
	return cBłąd;
}


////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru prędkości rolki 1
// Estymata_x(n) = Estymata_x(n-1) + K(n) * (z(n) - H * Estymata_x(n-1))
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: fPredkość - prędkość odczytana z rolki 1
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaRolką2FiltraKalmanaPrędkościRolek5X2Z(float fDroga, float fPredkość)
{
	uint8_t cBłąd = BLAD_OK;

	fZ[0] = fPredkość;	//prędkość pionowa

	//liczę współczynnik wzmocnienia Kalmana: mK,
	//najpierw transponowane H -> mTempSPc	 [Pomiar x Stan] -> [Stan x Pomiar]
	cBłąd |= arm_mat_trans_f32(&mH2, &mTempSPc);

	// P(n-1) * (H^T) -> mPHc				[Stan x Stan] * [Stan x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mP, &mTempSPc, &mPH);

	//H * (P(n-1)*H^T) -> fTempPPcA			[Pomiar x Stan] * [Stan x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mH2, &mPH, &mTempPPcA);

	//(H*P(n-1)*H^T) + R(n) -> fTempPPcB	[Pomiar x Pomiar] + [Pomiar x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_add_f32(&mTempPPcA, &mR1, &mTempPPcB);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> fTempPPcA	[Pomiar x Pomiar] -> [Pomiar x Pomiar]
	cBłąd |= arm_mat_inverse_f32(&mTempPPcB, &mTempPPcA);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mPH, &mTempPPcA, &mK2);

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1) [Pomiar x Stan] * [Stan] = [Pomiar]
	cBłąd |= arm_mat_mult_f32(&mH2, &mX, &mTempPc1A);

	//innowacja: z(n) - (H*X(n-1)) ->mTempPc1B		[Pomiar] - [Pomiar] = [Pomiar]
	cBłąd |= arm_mat_sub_f32(&mZ, &mTempPc1A, &mTempPc1B);

	//mnożenie przez K: K(n) * (z(n)-H*X(n-1))		[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mK2, &mTempPc1B, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//przepisanie estymaty do wektora stanu
	for (uint8_t n=0; n<KSTAN; n++)
		fX[n] = fTempS1B[n];

	//teraz liczę macierz wariancji i kowariancji, zaczynam od  K(n) * H -> mTempSSA	 [Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mK2, &mH2, &mTempSSA);

	//odejmowanie (I - K(n) * H) -> mTempSSB
	cBłąd |= arm_mat_sub_f32(&mI, &mTempSSA, &mTempSSB);

	//mnożenie (I-K(n)*H) * P(n-1) -> mTempSSA 	[Stan x Stan] * [Stan x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSSB, &mP, &mTempSSA);

	//transpozycja: (I-K(n)*H)^T-> mTempSSC
	cBłąd |= arm_mat_trans_f32(&mTempSSB, &mTempSSC);

	//mnożenie: (I-K(n)*H)*P(n-1) * (I-K(n)*H)^T
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSC, &mTempSSB);

	//mnożenie 	K(n) * R(n)  -> mTempSPc  	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mK2, &mR1, &mTempSPc);

	//transpozycja K(n)^T -> mTempM24  		[Stan x Pomiar] -> [Pomiar x Stan]
	cBłąd |= arm_mat_trans_f32(&mK2, &mTempPcS);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T) 	 	[Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSPc, &mTempPcS, &mTempSSA);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempSSB, &mTempSSA, &mP);
	return cBłąd;
}


