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
// Inny przykład:
// Ogólne równanie: ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2iz + j = 0
// Można to zapisać macierzowo: h^T * A * h + b^T * h + j = 0	gdzie A to macierz 3x3, b to wektor 3x1, j=-1
// Budujemy macierz n pomiarów o wierszu zawierajacym: D = [x^2, y^2, z^2, xy, xz, yz, x, y, z]
// Rozwiązanie metodą najmnieszych kwadratów daje wektor: p = (D^T * D)^-1 * D^T * 1
// daje to D * p = 1 gdzie p to wektor parametrów: p = [a, b, c, 2d, 2e, 2f, 2g, 2h, 2i]^T
// z wektora p budujemy macierz kwadryki (kwadratowa powierzchnia w 3D) Ae oraz wektor be
//  	|a  d  e|		|2g|
// Ae = |d  b  f|  be = |2h|
//		|e  f  c|		|2i|
// Przesunięcie środka (bias) dla Hard Iron: V = -1/2 * Ae^-1 * be
// Skalowanie i Soft Iron W^-1
// Obliczamy wartość skalującą: k = 1 + 1/4 * Ae^-1 * be
// Macierz zmienionej geometrii A = Ae/k
// Ponieważ A = (W^-1)^T * W^-1,  macierz korekcji W-^-1 wyznacza się przez deompozycję Choleskiego lub jej pierwiastek macierzowy
// Wynik jest znormalizowany
// Patrz też: https://github.com/nliaudat/magnetometer_calibration
// https://teslabs.com/articles/magnetometer-calibration/
// (c) PitLab 2026
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <KalibracjaMagMetodaDopasowaniaDoSfery.h>
#include <Fram.h>

//uint16_t sLicznikPomiarow;
//float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4_CM4")))	fDanePomiaroweMag[LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY][4];
//float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4_CM4")))	fSumaKwadratówMag[LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY];
//static arm_matrix_instance_f32 mA  = {LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY, 4, fDanePomiaroweMag};
//static arm_matrix_instance_f32 mY  = {LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY, 1, fSumaKwadratówMag};

stDopasowanieDoSfery_t stDopasowanieDoSferyMag1, stDopasowanieDoSferyMag2;
stWynikiDopasowania_t stWynikiDopasowaniaMag1, stWynikiDopasowaniaMag2;
extern float fPrzesMagn1[3], fSkaloMagn1[3];
extern float fPrzesMagn2[3], fSkaloMagn2[3];



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
// Funkcja zbiera dane magnetometrów 1 i 2 w celu ich kalibracji metodą najmniejszyh kwadratów
// wyniki przekazuje w strukturze uRozne.f32[0..2] = biasy mag1, uRozne.f32[3] = natężenie pola mag1,
// uRozne.f32[4..6] = biasy mag2, uRozne.f32[7] = natężenie pola mag2
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZbierajDaneMagDoKalibracji(stWymianyCM4_t *dane)
{
	uint8_t cBłąd = BLAD_OK;

	if ((stDopasowanieDoSferyMag1.sLiczbaPróbek < LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY) && (dane->sNowyPomiar & NP_KAL_MAG1))		//jest nowy pomiar
	{
		cBłąd = ZbierajProbki(&dane->fMagne1[0], &stDopasowanieDoSferyMag1);
		dane->uRozne.U16[POSTEP_U16_KALIBRACJI1] = stDopasowanieDoSferyMag1.sLiczbaPróbek;
		if (stDopasowanieDoSferyMag1.sLiczbaPróbek == LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY)
		{
			cBłąd = MetodaNajmniejszychKwadratow(&stDopasowanieDoSferyMag1, &stWynikiDopasowaniaMag1);
			for (uint8_t n=0; n<3; n++)
			{
				dane->uRozne.f32[n] = stWynikiDopasowaniaMag1.fPrzesunięcie[n];
				fPrzesMagn1[n] =  stWynikiDopasowaniaMag1.fPrzesunięcie[n];
				ZapiszFramFloat(FAH_MAGN1_SKLADNIK_X + 4*n, fPrzesMagn1[n]);
				//ZapiszFramFloat(FAH_MAGN1_MNOZNIK_X + 4*n, fSkaloMagn1[n]);
			}

			dane->uRozne.f32[3] = stWynikiDopasowaniaMag1.fNatężeniePolaMag;
		}
		dane->sNowyPomiar &= ~NP_KAL_MAG1;
	}

	if ((stDopasowanieDoSferyMag2.sLiczbaPróbek < LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY) && (dane->sNowyPomiar & NP_KAL_MAG2))	//jest nowy pomiar
	{
		cBłąd = ZbierajProbki(&dane->fMagne2[0], &stDopasowanieDoSferyMag2);
		dane->uRozne.U16[POSTEP_U16_KALIBRACJI2] = stDopasowanieDoSferyMag2.sLiczbaPróbek;
		if (stDopasowanieDoSferyMag2.sLiczbaPróbek == LICZBA_POMIAROW_MAG_DOPASOWANIA_DO_SFERY)
		{
			cBłąd = MetodaNajmniejszychKwadratow(&stDopasowanieDoSferyMag2, &stWynikiDopasowaniaMag2);
			for (uint8_t n=0; n<3; n++)
			{
				dane->uRozne.f32[4+n] = stWynikiDopasowaniaMag2.fPrzesunięcie[n];
				fPrzesMagn2[n] =  stWynikiDopasowaniaMag2.fPrzesunięcie[n];
				ZapiszFramFloat(FAH_MAGN2_SKLADNIK_X + 4*n, fPrzesMagn2[n]);
				//ZapiszFramFloat(FAH_MAGN2_MNOZNIK_X + 4*n, fSkaloMagn2[n]);
			}
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
		stWynikiDopasowania->fPrzesunięcie[n] = fTheta[n] / 2;

	//ponieważ: C = B^2 - bx^2 - by^2 - bz^2, więc: B^2 = C + bx^2 + by^2 + bz^2
	//poniewa Theta = b/2, więc: B = pierwiastek( C + theta[x]/4 + theta[y]/4 + theta[z]/4)
	stWynikiDopasowania->fNatężeniePolaMag = sqrtf(fTheta[3] + (fTheta[0] * fTheta[0] + fTheta[1] * fTheta[1] + fTheta[2] * fTheta[2]) / 4);
	return cBłąd;
}


//Metoda druga


////////////////////////////////////////////////////////////////////////////////
// Funkcja zmiera dane do macierzy D = [x^2, y^2, z^2, xy, xz, yz, x, y, z]
// Parametry: *stDopasowanieDoSfery - wskaźnik na strukturę akumulowanych danych pomiarowych
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZbierajProbki2(float *fMag, stDopasowanieDoSfery_t *stDopasowanieDoSfery)
{

}



////////////////////////////////////////////////////////////////////////////////
//
// Parametry:
//	*stDopasowanieDoSfery - wskaźnik na strukturę akumulowanych danych pomiarowych
//	*stWynikiDopasowania - wskaźnik na strukturę zawierjącą wyniki dopasowania do sfery
// Parametry: *dane - wskaźnik na strukturę danych autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t Metoda2NajmniejszychKwadratow(stDopasowanieDoSfery_t *stDopasowanieDoSfery, stWynikiDopasowania_t *stWynikiDopasowania)
{
}
}
