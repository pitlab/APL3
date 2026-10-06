//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Kalibracja magnetometru polegajaca na dopasowaniu do sfery metodą najmniejszych kwadratów
// Równanie sfery: r^2 = (x - x0)^2 + (y - y0)^2 + (z - z0)^2;	gdzie x0, y0, z0 - współrzędne środka sfery, r - promień; x, y, z - współrzędne punktu na powierzchni
// Analogicznie pole magnetyczne B^2 = (mx - bx)^2 + (my - by)^2 + (mz - bz)^2; gdzie mx, my, mz - współrzędne odczytane z magnetometru, bx, by, bz - przesunięcie wzgledem środka sfery
// po rozwinieciu: B^2 = mx^2 + my*^2 + mz^2 - 2bxmx - 2bymy - 2bzmz + bx^2 + by^2 + bz^2
// po przeniesieniu stron: mx^2 + my*^2 + mz^2 = 2bxmx + 2bymy + 2bzmz + B^2 - bx^2 - by^2 - bz^2
// wyodrębniając biasy jako współczynniki: Ax = 2bx, Ay = 2by, Az = 2bz, oraz część stałą: C = B^2 - bx^2 - by^2 - bz^2
// mamy układ równań, który można rozwiazać: Axmx + Aymy + Azmz + C = mx^2 + my^2 + mz^2
// rozwiązaniem jest operacja macierzowa: Theta = (A^T * A)^-1 * A^T * Y;  gdzie Theta = [2bx, 2by, 2bz, C]; A = [N pomiarow]x[xm, my, mx, 1]; Y = [N pomiarow]x[mx^2 + my^2 + mz^2]
// Aby nie trzeba było zużywać dużej ilości pamięci na macierze A oraz y definiuję dwie zmienne akumulujące wyniki pośrednie: ATA = A * A^T, oraz ATy = A^T * Y
// (c) PitLab 2026
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <KalibracjaMagMetodaDopasowaniaDoSfery.h>


uint16_t sLicznikPomiarow;
//float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4_CM4")))	fDanePomiaroweMag[LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY][4];
//float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4_CM4")))	fSumaKwadratówMag[LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY];
//static arm_matrix_instance_f32 mA  = {LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY, 4, fDanePomiaroweMag};
//static arm_matrix_instance_f32 mY  = {LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY, 1, fSumaKwadratówMag};

stDopasowanieDoSfery_t stDopasowanieDoSferyMag1, stDopasowanieDoSferyMag2;
stWynikiDopasowania_t stWynikiDopasowaniaMag1, stWynikiDopasowaniaMag2;




////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjuje dane kalibracyjne
// Parametry: *stDopasowanieDoSfery - wskaźnik na strukturę akumulowanych danych pomiarowych
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujKalibracje(void)
{
	uint8_t cBłąd = BLAD_OK;

	//czyść akumulatory wyników cząstkowych
	for (uint8_t n=0; n<4; n++)
	{
		stDopasowanieDoSferyMag1.fATy[n] = 0.0f;
		stDopasowanieDoSferyMag2.fATy[n] = 0.0f;
		for (uint8_t m=0; m<4; m++)
		{
			stDopasowanieDoSferyMag1.fATA[n][m] = 0.0f;
			stDopasowanieDoSferyMag2.fATA[n][m] = 0.0f;
		}
	}
	stDopasowanieDoSferyMag1.sLiczbaPróbek = 0;
	stDopasowanieDoSferyMag2.sLiczbaPróbek = 0;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja zbiera dane magnetometru 1 w celu jego kalibracji metodą najmniejszyh kwadratów
// wyniki przekazuje w strukturze uRozne.f32[0..2] biasy mag1, uRozne.f32[3] natęzenie pola mag1,
// uRozne.f32[4..6] biasy mag2, uRozne.f32[7] natęzenie pola mag2
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZbierajDaneMagDoKalibracji(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	//zabezpieczenie przed przepełnieniem bufora
	if (sLicznikPomiarow >= LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY)
	{
		sLicznikPomiarow = 0;
		return BLAD_BUF_OVERRUN;
	}

	if (dane->sNowyPomiar & NP_KAL_MAG1)	//jest nowy pomiar
	{
		cBłąd = ZbierajProbki(&dane->fMagne1[0], &stDopasowanieDoSferyMag1);
		dane->uRozne.U16[POSTEP_PROCESU_U16] = stDopasowanieDoSferyMag1.sLiczbaPróbek;
		if (stDopasowanieDoSferyMag1.sLiczbaPróbek == LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY)
		{
			cBłąd = MetodaNajmniejszychKwadratow(&stDopasowanieDoSferyMag1, &stWynikiDopasowaniaMag1);
			for (uint8_t n=0; n<3; n++)
				dane->uRozne.f32[n] = stWynikiDopasowaniaMag1.fBias[n];
			dane->uRozne.f32[3] = stWynikiDopasowaniaMag1.fNatężeniePolaMag;
		}
		dane->sNowyPomiar &= ~NP_KAL_MAG1;
	}

	if (dane->sNowyPomiar & NP_KAL_MAG2)	//jest nowy pomiar
	{
		cBłąd = ZbierajProbki(&dane->fMagne2[0], &stDopasowanieDoSferyMag2);
		dane->uRozne.U16[POSTEP_PROCESU2_U16] = stDopasowanieDoSferyMag2.sLiczbaPróbek;
		if (stDopasowanieDoSferyMag2.sLiczbaPróbek == LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY)
		{
			cBłąd = MetodaNajmniejszychKwadratow(&stDopasowanieDoSferyMag2, &stWynikiDopasowaniaMag2);
			for (uint8_t n=0; n<3; n++)
				dane->uRozne.f32[4+n] = stWynikiDopasowaniaMag2.fBias[n];
			dane->uRozne.f32[7] = stWynikiDopasowaniaMag2.fNatężeniePolaMag;
		}
		dane->sNowyPomiar &= ~NP_KAL_MAG2;
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja liczy równania A^T * A i A^T * Y z danych pomiarowych magnetometru oraz akumuluje wyniki
// Parametry: *stDopasowanieDoSfery - wskaźnik na strukturę akumulowanych danych pomiarowych
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZbierajProbki(float *fMag, stDopasowanieDoSfery_t *stDopasowanieDoSfery)
{
	uint8_t cBłąd = BLAD_OK;
	float fA[4];
	float fY;

	fY = 0.0f;
	for (uint8_t n=0; n<3; n++)
	{
		fA[n] = *(fMag + n);
		fY += fA[n] * fA[n];	//akumuluj kwadraty skłądowych xyz
	}
	fA[3] = 1.0f;

	if (stDopasowanieDoSfery->sLiczbaPróbek < LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY)
		stDopasowanieDoSfery->sLiczbaPróbek++;
	else
		return BLAD_GOTOWE;

	//akumulacja A^T * A
	stDopasowanieDoSfery->fATA[0][0] += fA[0] * fA[0];
	stDopasowanieDoSfery->fATA[0][1] += fA[0] * fA[1];
	stDopasowanieDoSfery->fATA[0][2] += fA[0] * fA[2];
	stDopasowanieDoSfery->fATA[0][3] += fA[0];

	stDopasowanieDoSfery->fATA[1][0] += fA[1] * fA[0];
	stDopasowanieDoSfery->fATA[1][1] += fA[1] * fA[1];
	stDopasowanieDoSfery->fATA[1][2] += fA[1] * fA[2];
	stDopasowanieDoSfery->fATA[1][3] += fA[1];

	stDopasowanieDoSfery->fATA[2][0] += fA[2] * fA[0];
	stDopasowanieDoSfery->fATA[2][1] += fA[2] * fA[1];
	stDopasowanieDoSfery->fATA[2][2] += fA[2] * fA[2];
	stDopasowanieDoSfery->fATA[2][3] += fA[2];

	stDopasowanieDoSfery->fATA[3][0] += fA[0];
	stDopasowanieDoSfery->fATA[3][1] += fA[1];
	stDopasowanieDoSfery->fATA[3][2] += fA[2];
	stDopasowanieDoSfery->fATA[3][3] += 1.0f;

	//akumulacja A^T * y
	stDopasowanieDoSfery->fATy[0] += fA[0] * fY;
	stDopasowanieDoSfery->fATy[1] += fA[1] * fY;
	stDopasowanieDoSfery->fATy[2] += fA[2] * fY;
	stDopasowanieDoSfery->fATy[3] += fY;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja rozwiązuje układ równań wykonując operację macierzową https://home.agh.edu.pl/~mariuszp/wfiis_mmf/wyklad_mmf1_14_1112.pdf
// Theta = (A^T * A)^-1 * A^T * Y
// gdzie Theta  [2bx, 2by, 2bz, C]
// Parametry:
//	*stDopasowanieDoSfery - wskaźnik na strukturę akumulowanych danych pomiarowych
//	*stWynikiDopasowania - wskaźnik na strukturę zawierjącą wyniki dopasowania do sfery
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t MetodaNajmniejszychKwadratow(stDopasowanieDoSfery_t *stDopasowanieDoSfery, stWynikiDopasowania_t *stWynikiDopasowania)
{
	uint8_t cBłąd = BLAD_OK;
	float fATA[4][4];	//wynik operacji A * A^T
	float fATy[4];		//wynik operacji A^T * y
	float fInvATA[4][4];	//wynik inwersji (A * A^T)^1
	float fTheta[4];		//wynikowa macierz biasów
	arm_matrix_instance_f32 mATA  = {4, 4, &fATA[0][0]};
	arm_matrix_instance_f32 mATy  = {4, 1, &fATy[0]};
	arm_matrix_instance_f32 mInvATA  = {4, 4, &fInvATA[0][0]};
	arm_matrix_instance_f32 mTheta  = {4, 1, &fTheta[0]};

	arm_mat_init_f32(&mATA, 4, 4, &fATA[0][0]);
	arm_mat_init_f32(&mATy, 4, 1, &fATy[0]);
	arm_mat_init_f32(&mInvATA, 4, 4, &fInvATA[0][0]);
	arm_mat_init_f32(&mTheta, 4, 1, &fTheta[0]);

	//wstaw dane do zmiennych roboczych
	for (uint8_t n=0; n<4; n++)
	{
		fATy[n] = stDopasowanieDoSfery->fATy[n];
		for (uint8_t m=0; m<4; m++)
			fATA[n][m] = stDopasowanieDoSfery->fATA[n][m];
	}

	cBłąd = arm_mat_inverse_f32(&mATA, &mInvATA);
	if (cBłąd)
		return cBłąd;

	cBłąd = arm_mat_mult_f32(&mInvATA, &mATy, &mTheta);

	for (uint8_t n=0; n<3; n++)
		stWynikiDopasowania->fBias[n] = fTheta[n] / 2;

	//ponieważ: C = B^2 - bx^2 - by^2 - bz^2, więc: B^2 = C + bx^2 + by^2 + bz^2
	//poniewa Theta = b/2, więc: B = pierwiastek( C + theta[x]/4 + theta[y]/4 + theta[z]/4)
	stWynikiDopasowania->fNatężeniePolaMag = sqrtf(fTheta[3] + (fTheta[0] * fTheta[0] + fTheta[1] * fTheta[1] + fTheta[2] * fTheta[2]) / 4);
	return cBłąd;
}

