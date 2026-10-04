//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Liniowy Filtr Kalmana służący do kalibracji magnetometrów.
// Posiada 12-elementowy wektor stanu zawierajacy: 3 składowe wektora pola magnetycznego oraz biasy każdej z 3 osi dla 3 magnetometrów
// Filtr aktualizowany jest wektoramim magnetycznym z magnetometrów obracanymi przez
//
// (c) PitLab 2026
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <KalmanKalibracjiMagnetometrow12X9Z.h>



//zmienne z przedrostkiem f oznaczaja macierze lub wektory na liczbach float
static float32_t fX[KMSTAN];					//wektor stanu: 0..2 wektor magnetyczny, 3..5 biasy magnetometru 1, 6..8 biasy mag 2, 9..11 biasy mag 3
static float32_t fZ[KMPMAG];					//wektor pomiaru magnetometrem
static float32_t fY[KMPMAG];					//wektor innowacji
static float32_t fF[KMSTAN][KMSTAN];			//macierz przejścia
static float32_t fP[KMSTAN][KMSTAN];			//macierz kowariancji predykcji
static float32_t fQ[KMSTAN][KMSTAN];			//macierz szumu procesu
static float32_t fI[KMSTAN][KMSTAN];			//macierz jednostkowa
static float32_t fS[KMPMAG][KMPMAG];			//macierz kowariancji innowacji: HPH^T
static float32_t fH1[KMPMAG][KMSTAN];			//macierz obserwacji wektora magnetycznego 1
static float32_t fH2[KMPMAG][KMSTAN];			//macierz obserwacji wektora magnetycznego 2
static float32_t fH3[KMPMAG][KMSTAN];			//macierz obserwacji wektora magnetycznego 3
static float32_t fR1[KMPMAG][KMPMAG];			//macierz kowariancji pomiaru wektora magnetycznego
static float32_t fR2[KMPMAG][KMPMAG];			//macierz kowariancji pomiaru wektora magnetycznego
static float32_t fR3[KMPMAG][KMPMAG];			//macierz kowariancji pomiaru wektora magnetycznego
static float32_t fK1[KMSTAN][KMPMAG];			//macierz wzmocnień Kalmana dla magnetometru 1
static float32_t fK2[KMSTAN][KMPMAG];			//macierz wzmocnień Kalmana dla magnetometru 2
//static float32_t fK3[KMSTAN][KMPMAG];			//macierz wzmocnień Kalmana dla magnetometru 3
static float32_t fTempSSA[KMSTAN][KMSTAN];		//macierz robocza A stan x stan
static float32_t fTempSSB[KMSTAN][KMSTAN];		//macierz robocza B stan x stan
static float32_t fTempSSC[KMSTAN][KMSTAN];		//macierz robocza C stan x stan
static float32_t fTempSPA[KMSTAN][KMPMAG];		//macierz robocza A stan x pomiar
static float32_t fTempSPB[KMSTAN][KMPMAG];		//macierz robocza B stan x pomiar
static float32_t fTempPS[KMPMAG][KMSTAN];		//macierz robocza C pomiar x stan
static float32_t fTempPP[KMPMAG][KMPMAG];		//macierz robocza pomiar x pomiar
static float32_t fTempP1[KMPMAG];				//macierz robocza A pomiar x 1
static float32_t fTempS1A[KMSTAN];				//macierz robocza A stan x 1
static float32_t fTempS1B[KMSTAN];				//macierz robocza B stan x 1

static arm_matrix_instance_f32 mX  = {KMSTAN, 1, fX};					//wektor stanu
static arm_matrix_instance_f32 mZ  = {KMPMAG, 1, fZ};					//wektor pomiaru magnetometrem
static arm_matrix_instance_f32 mY  = {KMPMAG, 1, fY};					//wektor innowacji
static arm_matrix_instance_f32 mF  = {KMSTAN, KMSTAN, &fF[0][0]};		//macierz przejscia
static arm_matrix_instance_f32 mP  = {KMSTAN, KMSTAN, &fP[0][0]};		//macierz kowariancji predykcji
static arm_matrix_instance_f32 mQ  = {KMSTAN, KMSTAN, &fQ[0][0]};		//macierz szumu procesu
static arm_matrix_instance_f32 mI  = {KMSTAN, KMSTAN, &fI[0][0]};		//macierz jednostkowa
static arm_matrix_instance_f32 mS  = {KMPMAG, KMPMAG, &fS[0][0]};		//macierz kowariancji innowacji
static arm_matrix_instance_f32 mH1 = {KMPMAG, KMSTAN, &fH1[0][0]};		//macierz obserwacji wektora magnetycznego 1
static arm_matrix_instance_f32 mH2 = {KMPMAG, KMSTAN, &fH2[0][0]};		//macierz obserwacji wektora magnetycznego 2
//static arm_matrix_instance_f32 mH3 = {KMPMAG, KMSTAN, &fH3[0][0]};		//macierz obserwacji wektora magnetycznego 3
static arm_matrix_instance_f32 mR1 = {KMPMAG, KMPMAG, &fR1[0][0]};		//macierz kowariancji pomiaru magnetometru 1
static arm_matrix_instance_f32 mR2 = {KMPMAG, KMPMAG, &fR2[0][0]};		//macierz kowariancji pomiaru magnetometru 2
//static arm_matrix_instance_f32 mR3 = {KMPMAG, KMPMAG, &fR3[0][0]};		//macierz kowariancji pomiaru magnetometru 3
static arm_matrix_instance_f32 mK1 = {KMSTAN, KMPMAG, &fK1[0][0]};		//macierz wzmocnień Kalmana dla  magnetometru 1
static arm_matrix_instance_f32 mK2 = {KMSTAN, KMPMAG, &fK2[0][0]};		//macierz wzmocnień Kalmana dla  magnetometru 2
//static arm_matrix_instance_f32 mK3 = {KMSTAN, KMPMAG, &fK3[0][0]};		//macierz wzmocnień Kalmana dla  magnetometru 3
static arm_matrix_instance_f32 mTempP1  = {KMPMAG, 1, &fTempP1[0]};		//macierz robocza A pomiar x 1
static arm_matrix_instance_f32 mTempS1A = {KMSTAN, 1, &fTempS1A[0]};	//macierz robocza A stan x 1
static arm_matrix_instance_f32 mTempS1B = {KMSTAN, 1, &fTempS1B[0]};	//macierz robocza B stan x 1
static arm_matrix_instance_f32 mTempSSA = {KMSTAN, KMSTAN, &fTempSSA[0][0]};	//macierz robocza A stan x stan
static arm_matrix_instance_f32 mTempSSB = {KMSTAN, KMSTAN, &fTempSSB[0][0]};	//macierz robocza B stan x stan
static arm_matrix_instance_f32 mTempSSC = {KMSTAN, KMSTAN, &fTempSSC[0][0]};	//macierz robocza C stan x stan
static arm_matrix_instance_f32 mTempSPA = {KMSTAN, KMPMAG, &fTempSPA[0][0]};	//macierz robocza A stan x pomiar
static arm_matrix_instance_f32 mTempSPB = {KMSTAN, KMPMAG, &fTempSPB[0][0]};	//macierz robocza B stan x pomiar
static arm_matrix_instance_f32 mTempPS  = {KMPMAG, KMSTAN, &fTempPS[0][0]};		//macierz robocza pomiar x stan
static arm_matrix_instance_f32 mTempPP  = {KMPMAG, KMPMAG, &fTempPP[0][0]};		//macierz robocza A pomiar x pomiar

static float fPoprzedniKat[3];	//poprzednia wartość kątów orientacji;



////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje rozszerzony filtr Kalmana dla kwaternionu kątów orientacji i błędów żyroskopów
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFiltrKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//inicjuj macierze
	for (uint8_t n=0; n<KMSTAN; n++)
	{
		for (uint8_t m=0; m<KMSTAN; m++)
		{
			fI[n][m] = fQ[n][m] = fP[n][m] = 0.0f;
		}
		fX[n] = 0.0f;

		for (uint8_t m=0; m<KMPMAG; m++)
			fH1[m][n] = fH2[m][n] = fH3[m][n] = 0.0f;	//macierz obserwacji wektora magnetycznego
	}

	//macierz kowariancji pomiaru wektora magnetycznego: zera, a w głównej przekatnej wariancja szumu
	for (uint8_t n=0; n<KMPMAG; n++)
	{
		for (uint8_t m=0; m<KMPMAG; m++)
			fR1[n][m] = fR2[n][m] = fR3[n][m] = 0.0f;
	}
	for (uint8_t n=0; n<KMPMAG; n++)
	{
		fR1[n][n] = WARIANCJA_SZUMU_MAGNETOMETRU1;
		fR2[n][n] = WARIANCJA_SZUMU_MAGNETOMETRU2;
		fR3[n][n] = WARIANCJA_SZUMU_MAGNETOMETRU3;
	}

	//inicjuj pierwszy wektor stanu	pomiarem
	if (dane->fMagne1[0] != 0.0f)
	{
		fX[0] = dane->fMagne1[0];	//składowa X wektora magnetycznego
		fX[1] = dane->fMagne1[1];	//składowa Y wektora magnetycznego
		fX[2] = dane->fMagne1[2];	//składowa Z wektora magnetycznego
	}
	else
	if (dane->fMagne2[0] != 0.0f)
	{
		fX[0] = dane->fMagne2[0];	//składowa X wektora magnetycznego
		fX[1] = dane->fMagne2[1];	//składowa Y wektora magnetycznego
		fX[2] = dane->fMagne2[2];	//składowa Z wektora magnetycznego
	}
	else
	if (dane->fMagne3[0] != 0.0f)
	{
		fX[0] = dane->fMagne3[0];	//składowa X wektora magnetycznego
		fX[1] = dane->fMagne3[1];	//składowa Y wektora magnetycznego
		fX[2] = dane->fMagne3[2];	//składowa Z wektora magnetycznego
	}
	else
		return BLAD_BRAK_DANYCH;
	arm_mat_init_f32(&mX, KMSTAN, 1, fX);

	//macierz przejścia oblicza wartość predykcji następnego stanu
	for (uint8_t n=0; n<KMSTAN; n++)
		fI[n][n] = 1.0f;

	//macierze pomiaru mają jedynki po przekątnych w różnych miejscach wektora stanu
	for (uint8_t n=0; n<KMPMAG; n++)
	{
		fH1[n][n] = 1.0f;	//pierwsza jedynka dla wektora magnetycznego
		fH2[n][n] = 1.0f;
		fH3[n][n] = 1.0f;
		fH1[n][n+3] = 1.0f;	//druga dla biasu
		fH2[n][n+6] = 1.0f;
		fH3[n][n+9] = 1.0f;
	}

	//początkowa wariancja predykcji
	for (uint8_t n=0; n<KMPMAG; n++)
	{
		fP[n][n] = WARIANCJA_SZUMU_PROCESU_OBROTU;		//główna przekątna części macierzy odpowiedzialnej za obrót wektora magnetycznego
		fQ[n][n] = WARIANCJA_SZUMU_PROCESU_OBROTU;
	}
	for (uint8_t n=KMPMAG; n<KMSTAN; n++)
	{
		fP[n][n] = WARIANCJA_SZUMU_PROCESU_BIASU;		//główna przekatna części macierzy odpowiedzialnej za estymację biasu
		fQ[n][n] = WARIANCJA_SZUMU_PROCESU_BIASU;
		fF[n][n] = 1.0f;
	}
	arm_mat_init_f32(&mP, KMSTAN, KMSTAN, &fP[0][0]);

	//wypełnij pierwszą wartość poprzedniego kąta
	for (uint8_t n=0; n<3; n++)
		fPoprzedniKat[n] = dane->fKatIMU1[n];

	dane->nZainicjowano |= INIT_KALMAN_KAL_MAGN;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Predykcja obraca wektor magnetyczny w układzie odniesienia BSP o różnicę kąta między iteracjami
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
	float fDeltaKąta[3];	//przyrost kątów orientacji w bieżącej iteracji

	for (uint8_t n=0; n<KMSTAN; n++)
		dane->stKalmanKalibrMag.fX[n] = fX[n];

	for (uint8_t n=0; n<3; n++)
	{
		fDeltaKąta[n] = dane->fKatIMU1[n] - fPoprzedniKat[n];
		fPoprzedniKat[n] = dane->fKatIMU1[n];
	}

	//obliczenia trygonometryczne różnicy kątów uzyskanych z filtra komplementarnego
	float fSinPhi = sinf(fDeltaKąta[0]);
	float fCosPhi = cosf(fDeltaKąta[0]);
	float fSinThe = sinf(fDeltaKąta[1]);
	float fCosThe = cosf(fDeltaKąta[1]);
	float fSinPsi = sinf(fDeltaKąta[2]);
	float fCosPsi = cosf(fDeltaKąta[2]);

	//Buduję macierz przejscia, która będzie obracała wektory magnetyczne. Podstawą matematyczną są 3 macierze obrotu: https://pl.wikipedia.org/wiki/Macierz_obrotu
	//		| 1	  0		  0	   |    	|  cosThe 0 sinThe |		| cosPsi -sinPsi 0 |
	// Rx = | 0	cosPhi -sinPhi |   Ry =	|    0	  1   0    |   Rz = | sinPsi  cosPsi 0 |
	//		| 0 sinPhi  cosPhi |		| -sinThe 0 cosThe |		|   0  		0	 1 |
	// mnożone w kolejności ZYX: Rz(Psi) * Ry(The) * Rx((Phi).
	fF[0][0] = fCosPsi * fCosThe;
	fF[0][1] = fCosPsi * fSinThe * fSinPhi - fSinPsi * fCosPhi;
	fF[0][2] = fCosPsi * fSinThe * fCosPhi + fSinPsi * fSinPhi;
	fF[1][0] = fSinPsi * fCosThe;
	fF[1][1] = fSinPsi * fSinThe * fSinPhi + fCosPsi * fCosPhi;
	fF[1][2] = fSinPsi * fSinThe * fCosPhi - fCosPsi * fSinPhi;
	fF[2][0] = -fSinThe;
	fF[2][1] = fCosThe * fSinPhi;
	fF[2][2] = fCosThe * fCosPhi;

	//1) Predykcja nowej estymaty wektora stanu: x(n+1) = F * x(n)
	cBłąd |= arm_mat_mult_f32(&mF, &mX, &mX);

	//2) Predykcja kowariancji czyli obliczenie niepewności nowej estymaty wektora stanu: Temp1 = F * P(n)
	cBłąd |= arm_mat_mult_f32(&mF, &mP, &mTempSSA);

	//transpozycja macierzy F: Temp2 = F^T
	cBłąd |= arm_mat_trans_f32(&mF, &mTempSSB);

	//mnożenie Temp3 = (F * P(n)) * (F^T)
	cBłąd |= arm_mat_mult_f32(&mTempSSA, &mTempSSB, &mTempSSC);

	//dodaj macierz szumu Q procesu do iloczynu (F * P(n)) * (F^T) -> P
	cBłąd |= arm_mat_add_f32(&mQ, &mTempSSC, &mP);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru magnetometrem 1
// Innowacja y(n) = z(n) - H * Estymata_x(n-1)
// Estymata_x(n) = Estymata_x(n-1) + K(n) * y(n)
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaMag1FiltraKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//Wstaw dane do wektora pomiaru
	for (uint8_t n=0; n<KMPMAG; n++)
		fZ[n] = dane->fMagne1[n];

	//Innowacja. Najpierw mnożenie: H * Estymata_x(n-1) 	[Pomiar x Stan] * [Stan] = [Pomiar]
	cBłąd |= arm_mat_mult_f32(&mH1, &mX, &mTempP1);

	//Innowacja y(n) = z(n) - (H*Estymata_x(n-1))			[Pomiar] - [Pomiar] = [Pomiar]
	cBłąd |= arm_mat_sub_f32(&mZ, &mTempP1, &mY);

	//liczę współczynnik wzmocnienia Kalmana: mK,
	//najpierw transponowane H -> mTempSPA	 [Pomiar x Stan] -> [Stan x Pomiar]
	cBłąd |= arm_mat_trans_f32(&mH1, &mTempSPA);

	// P(n-1) * (H^T) -> mTempSPB				[Stan x Stan] * [Stan x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mP, &mTempSPA, &mTempSPB);

	//H * (P(n-1)*H^T) -> fTempPPcA			[Pomiar x Stan] * [Stan x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mH1, &mTempSPB, &mTempPP);

	//(H*P(n-1)*H^T) + R(n) -> fS	[Pomiar x Pomiar] + [Pomiar x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_add_f32(&mTempPP, &mR1, &mS);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> fTempPPcA	[Pomiar x Pomiar] -> [Pomiar x Pomiar]
	cBłąd |= arm_mat_inverse_f32(&mS, &mTempPP);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mTempSPB, &mTempPP, &mK1);

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1) [Pomiar x Stan] * [Stan] = [Pomiar]
	cBłąd |= arm_mat_mult_f32(&mH1, &mX, &mTempP1);

	//mnożenie przez K: K(n) * y(n)			[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mK1, &mY, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//przepisanie estymaty do wektora stanu
	for (uint8_t n=0; n<KMSTAN; n++)
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
	cBłąd |= arm_mat_mult_f32(&mK1, &mR1, &mTempSPA);

	//transpozycja K(n)^T -> mTempM24  		[Stan x Pomiar] -> [Pomiar x Stan]
	cBłąd |= arm_mat_trans_f32(&mK1, &mTempPS);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T) 	 	[Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSPA, &mTempPS, &mTempSSA);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempSSB, &mTempSSA, &mP);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja aktualizuje stan filtra na podstawie nowego pomiaru magnetometrem 2
// Innowacja y(n) = z(n) - H * Estymata_x(n-1)
// Estymata_x(n) = Estymata_x(n-1) + K(n) * y(n)
// gdzie macierz wzmocnienia Kalmana: K(n) = P(n-1) * H^T * (H * P(n-1) * H^T + R(n))^-1
// Następnie znajduje nową macierz kowariancji P(n) = (I - K(n) * H) * P(n-1) * (I * K(n) * H)^T + K(n) * R(n) * K(n)^T
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AktulizacjaMag2FiltraKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//Wstaw dane do wektora pomiaru
	for (uint8_t n=0; n<KMPMAG; n++)
		fZ[n] = dane->fMagne2[n];

	//Innowacja. Najpierw mnożenie: H * Estymata_x(n-1) 	[Pomiar x Stan] * [Stan] = [Pomiar]
	cBłąd |= arm_mat_mult_f32(&mH1, &mX, &mTempP1);

	//Innowacja y(n) = z(n) - (H*Estymata_x(n-1))			[Pomiar] - [Pomiar] = [Pomiar]
	cBłąd |= arm_mat_sub_f32(&mZ, &mTempP1, &mY);

	//liczę współczynnik wzmocnienia Kalmana: mK,
	//najpierw transponowane H -> mTempSPA	 [Pomiar x Stan] -> [Stan x Pomiar]
	cBłąd |= arm_mat_trans_f32(&mH2, &mTempSPA);

	// P(n-1) * (H^T) -> mTempSPB				[Stan x Stan] * [Stan x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mP, &mTempSPA, &mTempSPB);

	//H * (P(n-1)*H^T) -> fTempPPcA			[Pomiar x Stan] * [Stan x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mH2, &mTempSPB, &mTempPP);

	//(H*P(n-1)*H^T) + R(n) -> fS	[Pomiar x Pomiar] + [Pomiar x Pomiar] = [Pomiar x Pomiar]
	cBłąd |= arm_mat_add_f32(&mTempPP, &mR2, &mS);

	//inwersja powyższego: (H*P(n-1)*H^T+R(n))^-1 -> fTempPPcA	[Pomiar x Pomiar] -> [Pomiar x Pomiar]
	cBłąd |= arm_mat_inverse_f32(&mS, &mTempPP);

	//finalne mnożenie: (P(n-1)*H^T) * ((H*P(n-1)*H^T+R(n))^-1) -> mK	[Stan x Pomiar] * [Pomiar x Pomiar] = [Stan x Pomiar]
	cBłąd |= arm_mat_mult_f32(&mTempSPB, &mTempPP, &mK2);

	//teraz liczę nową estymatę. Najpierw cześć w nawiasie: H * X(n-1) [Pomiar x Stan] * [Stan] = [Pomiar]
	cBłąd |= arm_mat_mult_f32(&mH2, &mX, &mTempP1);

	//mnożenie przez K: K(n) * z(y)			[Stan x Pomiar] * [Pomiar] = [Stan]
	cBłąd |= arm_mat_mult_f32(&mK2, &mY, &mTempS1A);

	//dodanie poprzedniej estymaty: Estymata_x(n-1) + K(n)*(z(n)-H*X(n-1))	[Stan] + [Stan] = [Stan]
	cBłąd |= arm_mat_add_f32(&mX, &mTempS1A, &mTempS1B);

	//przepisanie estymaty do wektora stanu
	for (uint8_t n=0; n<KMSTAN; n++)
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
	cBłąd |= arm_mat_mult_f32(&mK2, &mR2, &mTempSPA);

	//transpozycja K(n)^T -> mTempM24  		[Stan x Pomiar] -> [Pomiar x Stan]
	cBłąd |= arm_mat_trans_f32(&mK2, &mTempPS);

	//mnożenie 	(K(n)*R(n)) * (K(n)^T) 	 	[Stan x Pomiar] * [Pomiar x Stan] = [Stan x Stan]
	cBłąd |= arm_mat_mult_f32(&mTempSPA, &mTempPS, &mTempSSA);

	//finalne sumowanie ((I-K(n)*H)*P(n-1)*(I*K(n)*H)^T) + (K(n)*R(n)*K(n)^T) -> P(n)
	cBłąd |= arm_mat_add_f32(&mTempSSB, &mTempSSA, &mP);
	return cBłąd;
}
