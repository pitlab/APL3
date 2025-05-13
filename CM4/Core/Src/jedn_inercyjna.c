//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obliczenia jednostki inercyjnej (IMU)

// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "jedn_inercyjna.h"
#include "wymiana.h"
#include "konfig_fram.h"
#include "fram.h"
#include "kwaterniony.h"
#include "petla_glowna.h"

// Definicje użytej konwencji
// prawoskrętny układ współrzędnych lokalnych samolotu
// oś X od środka ciężkości w stronę dziobu samolotu
// oś Y od środka ciężkości w stronę prawego skrzydła
// oś Z od środka ciężkości w stronę ziemi
//
// Układ współrzędnych globalnych związany z Ziemią: (PWD - Północ Wschód Dół, lub NED - North East Down)
// oś X skierowana na północ
// oś Y skierowana nz wschód
// oś Z skierowana w dół
//
// Prędkości kątowe (układ prawoskrętny, patrząc w punkcie 0 zgodnie ze zwrotem osi obrót jest dodatni gdy jest zgodny z kierunkiem ruchu wskazówek zegara)
// prędkość P wokół osi X dodatnia gdy prawe skrzydło się opuszcza
// prędkość Q wokół osi Y dodatnia gdy dziób się podnosi
// prędkość R wokół osi Z odatnia gdy dziób obraca sie w prawo (zgodnie z ruchem wskazówek)
//
// Kąty
// phi   = kąt przechylenia, obrót wokół osi X, index 0, dodatni gdy pochylony na prawe skrzydło, ujemny na lewe
// theta = kąt pochylenia,   obrót wokół osi Y, index 1, dodatni gdy dziób skierowany w górę, ujemny w dół
// psi   = kąt odchylenia,   obrót wokół osi Z, index 2, patrząc z ziemi na lecący od nas model: dodatni na prawo, ujemny na lewo
//
// Orientacja geograficzna
// oś X jest po szerokości geograficznej. Wschód jest dodatni, zachód ujemny
// oś Y jest po długości geograficznej. Północ jest dodatnia, południe ujemne
//
extern uint32_t ndT[4];								//czas [us] jaki upłynął od poprzeniego obiegu pętli dla 4 modułów wewnetrznych
extern volatile unia_wymianyCM4_t uDaneCM4;
extern volatile unia_wymianyCM7_t uDaneCM7;
extern float fOffsetZyro1[3], fOffsetZyro2[3];

static float fQAcc[4] = {0.0f, 0.0f, 0.0f, 1.0f};	//kwaternion znormalizowanego wektora przyspieszenia
static float fQMag[4] = {0.0f, 0.0f, 1.0f, 0.0f};	//kwaternion wektora magnetycznego
static float fQMagKompens[4];		//kwaternion magnetometru skompensowanego o pochylenie i przechylenie


////////////////////////////////////////////////////////////////////////////////
// inicjalizacja zmiennych potrzebanych do obliczeń kątów
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujJednostkeInercyjna(void)
{

	for (uint16_t n=0; n<3; n++)
	{
		uDaneCM4.dane.fKatZyro1[n] = 0;
		uDaneCM4.dane.fKatZyro2[n] = 0;
	}
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia jednostki inercyjnej metodą filtra komplementarnego łaczącego całkę z prędkosci kątowych i kąty obliczone na podstawie równań trygonometrycznych z akcelerometrów
// Parametry: chGniazdo - numer gniazda w którym jest moduł IMU
// Zwraca: kod błędu
//  - uDaneCM4.dane.fKatZyro1 - kąty uzyskane przez całkowanie żyroskopów
//  - uDaneCM4.dane.fKatAkcel1 - kąty uzyskane z trygonometrii wektora akcelerometru [0,1] i magnetometru [2]
//  - uDaneCM4.dane.fKatIMU1 - kąty uzyskane z filtracji komplementarnej obu powyższych
// Czas trwania: 24us @200Hz
////////////////////////////////////////////////////////////////////////////////
uint8_t JednostkaInercyjnaTrygonometria(uint8_t chGniazdo)
{
	//licz całkę z prędkosci kątowych żyroskopów
	for (uint16_t n=0; n<3; n++)
	{
		uDaneCM4.dane.fKatZyro1[n] +=  uDaneCM4.dane.fZyroKal1[n] * ndT[chGniazdo] / 1000000;		//[rad/s] * [us / 1000000] => [rad]
		uDaneCM4.dane.fKatZyro2[n] +=  uDaneCM4.dane.fZyroKal2[n] * ndT[chGniazdo] / 1000000;

		//w czasie kalibracji wzmocnienia nie ograniczaj przyrostu kąta
		if ((uDaneCM7.dane.chWykonajPolecenie < POL_CALKUJ_PRED_KAT)  && (uDaneCM7.dane.chWykonajPolecenie > POL_KALIBRUJ_ZYRO_WZMP))
		{
			//ogranicz przyrost kąta do +-Pi
			if (uDaneCM4.dane.fKatZyro1[n] > M_PI)
				uDaneCM4.dane.fKatZyro1[n] = -M_PI;
			if (uDaneCM4.dane.fKatZyro1[n] < -M_PI)
				uDaneCM4.dane.fKatZyro1[n] = M_PI;
			if (uDaneCM4.dane.fKatZyro2[n] > M_PI)
				uDaneCM4.dane.fKatZyro2[n] = -M_PI;
			if (uDaneCM4.dane.fKatZyro2[n] < -M_PI)
				uDaneCM4.dane.fKatZyro2[n] = M_PI;
		}
	}

	//kąt przechylenia z akcelerometru: tan(Z/-Y) = atan2(Y, Z)
	uDaneCM4.dane.fKatAkcel1[0] = atan2f(-uDaneCM4.dane.fAkcel1[1], uDaneCM4.dane.fAkcel1[2]);	//OK, dobry znak

	//kąt pochylenia z akcelerometru: tan(Z/X) = atan2(X, Z)
	uDaneCM4.dane.fKatAkcel1[1] = atan2f(uDaneCM4.dane.fAkcel1[0], uDaneCM4.dane.fAkcel1[2]);	//OK

	//oblicz kąt odchylenia w radianach z danych magnetometru: tan(Y/X) dla X=N, Y=E => atan2(X, Y)
	uDaneCM4.dane.fKatAkcel1[2] = atan2f(uDaneCM4.dane.fMagne1[1], uDaneCM4.dane.fMagne1[0]);

	//filtr komplementarny IMU
	for (uint16_t n=0; n<3; n++)
	{
		uDaneCM4.dane.fKatIMU1[n] += uDaneCM4.dane.fZyroKal1[n] * ndT[chGniazdo] / 1000000;
		uDaneCM4.dane.fKatIMU1[n] = 0.05 * uDaneCM4.dane.fKatAkcel1[n] + 0.95 * uDaneCM4.dane.fKatIMU1[n];
	}

	/*/w celu porównania metody policz katy z tych samych danych metodą kwaternionową
	float fQA[4];	//kwaternion wektora przyspieszenia
	float fQM[4];
	//float fAccNorm[3];

	WektorNaKwaternion((float*)uDaneCM4.dane.fAkcel1, fQA);
	WektorNaKwaternion((float*)uDaneCM4.dane.fMagne1, fQM);
	Normalizuj(fQA, fQA, 4);
	Normalizuj(fQM, fQM, 4);
	KatyKwaterniona(fQA, fQM, (float*)uDaneCM4.dane.fKatAkcel2); */
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja przechowuje wektory przyspieszenia i magnetyczny w kwaternionach qAcc i qMag, obraca je o kąty uzyskane z całkowania prędkosci katowej
// Parametry: chGniazdo - numer gniazda w którym jest moduł IMU
// *fZyro,  *fAkcel, *fMagn - wskaźniki na tablice 3 osi żyroskopu, akcelerometru i magnetometru użyanych do obliczeń
// Zwraca: kod błędu
// Czas trwania: 39us @200Hz
////////////////////////////////////////////////////////////////////////////////
uint8_t JednostkaInercyjnaKwaterniony(uint8_t chGniazdo, float *fZyro, float *fAkcel, float *fMagn)
{
	float fdPhi2, fdTheta2, fdPsi2;	//połowy przyrostu kąta obrotu
	float fQx[4], fQy[4], fQz[4];	//kwaterniony obortów wokół osi XYZ
	float fQzy[4], fQzyx[4];
	float fQs[4], fQ[4];
	float fWspFiltraAkc, fWspFiltraMag;
	float fAccNorm[3], fMagNorm[3];
	float fQPoch[4], fQprze[4], fQkompens[4];	//kwaterniony obrotu o pochylenie i przechylenie oraz łączny obrót kompensacji magnetometru


	//Wyznaczam kwaterniony obrotów w formie algebraicznej korzystając z danych wejściowych wprowadzonych do formy trygonometrycznej
	//fQz = cos (psi/2) + (1 / sqrt(x^2 + y^2 + z^2)*k) * sin (psi/2)
	//fdPsi2 = uDaneCM4.dane.fZyroKal1[2] * ndT[chGniazdo] / 2000000.f;		//[rad/s] * ndT [us] / 1000000 = [rad]
	fdPsi2 = *(fZyro+2) * ndT[chGniazdo] / 2000000.f;		//[rad/s] * ndT [us] / 1000000 = [rad]
	//fdPsi2 = 0.0f * DEG2RAD / 2;
	fQz[0] = cosf(fdPsi2);		//część rzeczywista kwaternionu: s0 = cos(theta) gdzie kąt oborotu to 2*theta, więc s0 = cos(kąt_obrotu/2)
	fQz[1] = 0;
	fQz[2] = 0;
	fQz[3] = sin(fdPsi2);	//z0 = vz / |v0| * sin(theta)  Oś obrotu to 1, więc moduł też będzie 1 więc można to pominąć

	//fQy = cos (theta/2) + (1 / sqrt(x^2 + y^2 + z^2)*j) * sin (theta/2)
	fdTheta2 = *(fZyro+1) * ndT[chGniazdo] / 2000000.f;		//[rad/s] * ndT [us] / 1000000 = [rad]
	//fdTheta2 = 0.2f * DEG2RAD / 2;
	fQy[0] = cosf(fdTheta2);
	fQy[1] = 0;
	fQy[2] = sinf(fdTheta2);
	fQy[3] = 0;

	//całka z predkosci kątowej P to kąt Phi a przyrost kąta w czasie ndT to dPhi
	//fdPhi = uDaneCM4.dane.fZyroKal1[0] * ndT[chGniazdo] / 1000000;
	//Ponieważ we wzorze występuje połowa kąta, więc aby nie wykonywać dzielenia wielokrotnie, od razu liczę wartość połowy kata
	//fQx = cos (phi/2) + (1 / sqrt(x^2 + y^2 + z^2)*i) * sin (phi/2)
	fdPhi2 = *(fZyro+0) * ndT[chGniazdo] / 2000000.f;		//[rad/s] * ndT [us] / 1000000 = [rad]
	//fdPhi2 = 0.0f * DEG2RAD / 2;
	fQx[0] = cosf(fdPhi2);
	fQx[1] = sinf(fdPhi2);
	fQx[2] = 0;
	fQx[3] = 0;

	//składanie 3 obrotów w jeden
	MnozenieKwaternionow(fQy, fQz, fQzy);	//obrót najpierw wokół Z * Y = ZY
	MnozenieKwaternionow(fQx, fQzy, fQzyx);	//potem obrót ZY * X = ZYX
	KwaternionSprzezony(fQzyx, fQs);		//kwaternion sprzężony z kwaternionem obrotu o wszystkie osie

	//obroty wektorów przyspieszenia i magnetycznego
	MnozenieKwaternionow(fQzyx, fQAcc, fQ);		//przyspieszenie
	MnozenieKwaternionow(fQ, fQs, fQAcc);

	MnozenieKwaternionow(fQzyx, fQMag, fQ);		//pole magnetyczne
	MnozenieKwaternionow(fQ, fQs, fQMag);

	//normalizuj wektor przyspieszenia, bo wymaga tego asinf() w funkcji liczenia kątów. Magnetometr też musi być znormalizowany bo używa atan2f(..., 1-...)
	Normalizuj(fAkcel, fAccNorm, 3);
	Normalizuj(fMagn, fMagNorm, 3);

	//synchronizuj modelowy wektor przyspieszenia  ze zmierzonym przyspieszeniem za pomocą filtra komplementarnego o wspólczynniku określonym przez filtr adaptacyjny
	fWspFiltraAkc = FiltrAdaptacyjnyAkc(fAkcel);
	fWspFiltraMag = FiltrAdaptacyjnyMag(fMagn);

	//zastosuj filtr komplementarny do korekty wektorów obracanych żyroskopami
	for (uint8_t n=0; n<3; n++)
	{
		fQAcc[n+1] = (1.0f - fWspFiltraAkc) * fQAcc[n+1] + fWspFiltraAkc * fAccNorm[n];
		fQMag[n+1] = (1.0f - fWspFiltraMag) * fQMag[n+1] + fWspFiltraMag * fMagNorm[n];
	}

	//oblicz katy tradycyjnie trygonometrią
	uDaneCM4.dane.fKatIMU2[0] = atan2f(-fQAcc[2], fQAcc[3]);	//kąt przechylenia z akcelerometru: tan(-Y/Z) = atan2(Z, Y)
	uDaneCM4.dane.fKatIMU2[1] = atan2f(fQAcc[1], fQAcc[3]);	//kąt pochylenia z akcelerometru: tan(Z/X) = atan2(X, Z)


	//Żeby policzyć kat odchylenia z wektora magnetometru najpierw skompensuj pochylenie i przechylenie obracając kopię wektora mag. o ujemne pochylenie i dodatnie przechylenie
	// zmiana znaku korekcji osi wynika prawdopodobnie ze składania obrotów
	fQprze[0] = cosf(uDaneCM4.dane.fKatIMU2[0] / 2);
	fQprze[1] = sinf(uDaneCM4.dane.fKatIMU2[0] / 2);
	fQprze[2] = 0;
	fQprze[3] = 0;

	fQPoch[0] = cosf(-uDaneCM4.dane.fKatIMU2[1] / 2);
	fQPoch[1] = 0;
	fQPoch[2] = sinf(-uDaneCM4.dane.fKatIMU2[1] / 2);
	fQPoch[3] = 0;

	MnozenieKwaternionow(fQPoch, fQprze, fQkompens);	//sumuj kwaterniony pochylenia i przechylenia w jeden kompensacyjny
	KwaternionSprzezony(fQkompens, fQs);				//kwaternion sprzężony

	MnozenieKwaternionow(fQkompens, fQMag, fQ);		//obróć kwaternion pola magnetycznego o ujemne pochylenie i dodatnie przechylenie
	MnozenieKwaternionow(fQ, fQs, fQMagKompens);

	//Oblicz katy Eulera: Phi, Theta, Psi - https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	KatyKwaterniona(fQAcc, fQMagKompens, (float*)uDaneCM4.dane.fKatKwater);	//wersja z asinf szybsza
	//KatyKwaterniona3(fQAcc, fQMagKompens, (float*)uDaneCM4.dane.fKatAkcel2);	//wersja oryginalna dzielona przez -2

	//oblicz kąt odchylenia w radianach z danych magnetometru: tan(Y/X) dla X=N, Y=E => atan2(X, Y)
	uDaneCM4.dane.fKatIMU2[2] = atan2f(fQMagKompens[2], fQMagKompens[1]);

	/*/przepisz kwaterniony do zmiennych wymiany
	for (uint8_t n=0; n<4; n++)
	{
		uDaneCM4.dane.fKwaAkc[n] = fQAcc[n];
		//uDaneCM4.dane.fKwaAkc[n] = fQMagKompens[n];
		uDaneCM4.dane.fKwaMag[n] = fQMag[n];
	}*/
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja liczy na ile pomiar akcelerometrów jest zaburzony przyspieszeniami wynikającymi z ruchu (przyspieszenia liniowe i odśrodkowe) i
// na tej podstawie wylicza wartość współczynnika dla filtra komplementarnego
// Przyjęte są 2 progi: PROG_ACC_DOBRY do którego wskazanie uznajemy za wiarygodne, PROG_ACC_ZLY - powyżej którego jest brak korekcji żyroskopów akcelerometrem
// Przyjeta jest wartość filtra dla progu PROG_ACC_DOBRY wynosząca WSP_FILTRA_ADAPT mówiaca ile ma być korekcji akcelerometru dla dobrego sygnału
// Zwraca: wartość filtra komplementarnego między 0.0 a WSP_FILTRA_ADAPT
// Czas trwania: 1,38us @200MHz
////////////////////////////////////////////////////////////////////////////////
float FiltrAdaptacyjnyAkc(float *fAkcel)
{
	float fPrzyspRuchu = 0.0f;	//przyspieszenie wynikające ze zmiany predkości
	float fWspFiltra;			//współczynnik filtra komplementarnego

	for (uint8_t n=0; n<3; n++)
		fPrzyspRuchu += *(fAkcel+n) * *(fAkcel+n);			//suma kwadratów 3 osi akcelerometru
	fPrzyspRuchu = sqrtf(fPrzyspRuchu);						//bezwzględna długość wektora przyspieszenia
	//fPrzyspRuchu = fabs(fPrzyspRuchu - AKCEL1G) / AKCEL1G;	//wartość przyspieszenia wynikającego ze zmiany prędkości [m/s^2]
	fPrzyspRuchu = fabs(fPrzyspRuchu - AKCEL1G);	//wartość przyspieszenia wynikającego ze zmiany prędkości [m/s^2]

	if (fPrzyspRuchu < PROG_ACC_DOBRY)	//poziom zakłóceń akceptowalny, można w pełni kompensować dryft kątów akcelerometrem
		fWspFiltra = WSP_FILTRA_ADAPT;
	else
	if (fPrzyspRuchu > PROG_ACC_ZLY)	//poziom zakłóceń nieakceptowalny, filtr komplementarny powinien polegać na samych żyroskopach
		fWspFiltra = 0.0f;
	else								//poziom zakłóceń pośredni, liniowo skaluj między 0 a 1
		fWspFiltra = WSP_FILTRA_ADAPT * (PROG_ACC_ZLY - fPrzyspRuchu) / (PROG_ACC_ZLY - PROG_ACC_DOBRY);
	return fWspFiltra;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja liczy na ile zaburzony jest pomiar magnetometru, analogicznie jak akcelerometr powyżej
// na tej podstawie wylicza wartość współczynnika dla filtra komplementarnego
// Zwraca: wartość filtra komplementarnego między 0.0 a WSP_FILTRA_ADAPT
// Czas trwania: 1,38us @200MHz
////////////////////////////////////////////////////////////////////////////////
float FiltrAdaptacyjnyMag(float *fMag)
{
	float fZaklocenieMag = 0.0f;	//wartość zakłóceniazmieniająca długość wektora magnetycznego
	float fWspFiltra;		//współczynnik filtra komplementarnego

	for (uint8_t n=0; n<3; n++)
		fZaklocenieMag += *(fMag+n) * *(fMag+n);	//suma kwadratów 3 osi magnetometru
	fZaklocenieMag = sqrtf(fZaklocenieMag);			//bezwzględna długość wektora magnetycznego
	//fZaklocenieMag = fabs(fZaklocenieMag - NOMINALNE_MAGN) / NOMINALNE_MAGN;				//wartość przyspieszenia wynikającego ze zmiany prędkości [m/s^2]
	fZaklocenieMag = fabs(fZaklocenieMag - NOMINALNE_MAGN);				//wartość przyspieszenia wynikającego ze zmiany prędkości [m/s^2]

	if (fZaklocenieMag < PROG_MAG_DOBRY)	//poziom zakłóceń akceptowalny, można w pełni kompensować dryft kątów akcelerometrem
		fWspFiltra = WSP_FILTRA_ADAPT;
	else
	if (fZaklocenieMag > PROG_MAG_ZLY)	//poziom zakłóceń nieakceptowalny, filtr komplementarny powinien polegać na samych żyroskopach
		fWspFiltra = 0.0f;
	else								//poziom zakłóceń pośredni, liniowo skaluj między 0 a 1
		fWspFiltra = WSP_FILTRA_ADAPT * (PROG_MAG_ZLY - fZaklocenieMag) / (PROG_MAG_ZLY - PROG_MAG_DOBRY);
	return fWspFiltra;
}


////////////////////////////////////////////////////////////////////////////////
// Funkcja do testowania i pomiaru parametrów algorytmów
// Zwraca: nic
// Czas trwania: ?
////////////////////////////////////////////////////////////////////////////////
void TestyObrotu(uint8_t chGniazdo)
{
	//float fS0, fX0, fY0, fZ0;
/*	float fdPhi2, fdTheta2, fdPsi2;	//połowy przyrostu kąta obrotu
	float fModul;	//długości osi obracajacej wektor
	float qx[4], qy[4], qz[4];	//kwaterniony obortów wokół osi XYZ
	float qv[4], vq[4];	//pośrednie etapy mnożenia
	float qp[4];	//kwaternion obracanego wektora
	float qs[4];	//kwaternion sprzężony
	float p[3], r[3], s[3], t[3];		//punkt obracany i po obrocie
	//float fOsObr[3];			//oś obrotu
	uint32_t nCzas;
	float A[4][4], B[4][4], C[4][4];	//kwaterniony w postaci macierzowej
	float fQxyz[4];	//kwaternion obrotu o wszystkie 3 osie jednoczesnie */



	//pomiary czasu
	/*nCzas = PobierzCzas();
	JednostkaInercyjna4Kwaterniony(ADR_MOD2);
	nCzas = MinalCzas(nCzas);

	nCzas = PobierzCzas();
	for (uint16_t n=0; n<100; n++)
		KatyKwaterniona(fQAcc, fQMag, (float*)uDaneCM4.dane.fKatIMU2);
	nCzas = MinalCzas(nCzas);

	nCzas = PobierzCzas();
	for (uint16_t n=0; n<100; n++)
		KatyKwaterniona2(fQAcc, fQMag, (float*)uDaneCM4.dane.fKatIMU2);
	nCzas = MinalCzas(nCzas); */
	return;
}



