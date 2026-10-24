//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obliczenia jednostki inercyjnej (IMU)

// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "JednostkaInercyjna.h"
#include "KonfigFram.h"
#include "fram.h"
#include "kwaterniony.h"
#include "PetlaGlowna.h"

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

extern volatile unia_wymianyCM4_t uDaneCM4;
extern volatile unia_wymianyCM7_t uDaneCM7;
extern float fOffsetZyro1[3], fOffsetZyro2[3];

static float fQAcc[4] = {0.0f, 0.0f, 0.0f, 1.0f};	//kwaternion znormalizowanego wektora przyspieszenia w osi Z
static float fQMag[4] = {0.0f, 1.0f, 0.0f, 0.0f};	//kwaternion wektora magnetycznego w osi X
static float fQMagKompens[4];		//kwaternion magnetometru skompensowanego o pochylenie i przechylenie




////////////////////////////////////////////////////////////////////////////////
// inicjalizacja zmiennych potrzebanych do obliczeń kątów
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujJednostkeInercyjna(void)
{
	float fKatObrotuMagn[3];

	for (uint16_t n=0; n<3; n++)
	{
		uDaneCM4.dane.fKątZyro1[n] = 0;
		uDaneCM4.dane.fKątZyro2[n] = 0;
	}

	//wykonaj wstępny obrót kwaterniona magnetycznego o kąty inklinacji i deklinacji
	fKatObrotuMagn[0] = 0.0f;
	fKatObrotuMagn[1] = -INKLINACJA_MAG;		//inklinacja magnetyczna w radianach
	fKatObrotuMagn[2] = DEKLINACJA_MAG;			//deklinacja magnetyczna w radianach

	KatyNaKwaternion(fKatObrotuMagn, fQMag);	//zwraca znormalizowany kwaternion magnetyczny
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia jednostki inercyjnej metodą filtra komplementarnego łaczącego całkę z prędkosci kątowych i kąty obliczone na podstawie równań trygonometrycznych z akcelerometrów
// Parametry: chGniazdo - numer gniazda w którym jest moduł IMU
// Zwraca: kod błędu
//  - uDaneCM4.dane.fKatZyro1 - kąty uzyskane przez całkowanie żyroskopów [rad]
//  - uDaneCM4.dane.fKątAkcel - kąty uzyskane z trygonometrii wektora akcelerometru [0,1] i magnetometru [2] [rad]
//  - uDaneCM4.dane.fKatIMU1 - kąty uzyskane z filtracji komplementarnej obu powyższych [rad]
// Czas trwania: 24us @200Hz
////////////////////////////////////////////////////////////////////////////////
uint8_t JednostkaInercyjnaTrygonometria(float *fŻyro, float *fAkcel, float *fMagn, stWymianyCM4_t *wyniki, uint32_t ndT)
{
	uint8_t cBłąd = BLAD_OK;
	float fPrzyrostKąta;

	//kąt przechylenia z akcelerometru: 90° + atan(-Z/Y) = Pi/2 + atan2(-Z, Y)
	if ((wyniki->fKątAkcel[1] > -M_PI/2) && (wyniki->fKątAkcel[1] < M_PI/2))
		wyniki->fKątAkcel[0] = atan2f(-1 * *(fAkcel + 2), *(fAkcel + 1)) + M_PI/2;	//OK
	else
		wyniki->fKątAkcel[0] = atan2f(-1 * *(fAkcel + 2), *(fAkcel + 1)) - M_PI/2;	//OK

	//kąt pochylenia z akcelerometru: atan(Z/X) = atan2(-sqrtf(y^2+Z^2), X)
	//ponieważ pierwiastek zwraca zawsze dodatnią wartość, więc kat byłby ograniczony do +- Pi/2. Aby mieć pełen kat rozbijam to na 2 warunki i w jednym
	// dodaję znak minus przed pierwiastkiem. To może nie działać dla dowolnej wartości kąta, więc jest jedynie punktem odniesienia dla prostych ruchów
	/*if (*(fAkcel + 2) > 0)
		//wyniki->fKątAkcel[1] = atan2f(*(fAkcel + 0), sqrtf(*(fAkcel + 1) * *(fAkcel + 1) + *(fAkcel + 2) * *(fAkcel + 2)));
		wyniki->fKątAkcel[1] = M_PI/2 + atan2f(-1 * sqrtf(*(fAkcel + 1) * *(fAkcel + 1) + *(fAkcel + 2) * *(fAkcel + 2)), *(fAkcel + 0));
	else
		//wyniki->fKątAkcel[1] = atan2f(*(fAkcel + 0), -1 * sqrtf(*(fAkcel + 1) * *(fAkcel + 1) + *(fAkcel + 2) * *(fAkcel + 2)));	//OK dla symulacji pochylenia, źle dla przechylenia
		wyniki->fKątAkcel[1] = M_PI/2 + atan2f(sqrtf(*(fAkcel + 1) * *(fAkcel + 1) + *(fAkcel + 2) * *(fAkcel + 2)), *(fAkcel + 0));*/

	if ((wyniki->fKątAkcel[0] > -M_PI/2) && (wyniki->fKątAkcel[0] < M_PI/2))
		wyniki->fKątAkcel[1] = atan2f(*(fAkcel + 2), *(fAkcel + 0)) - M_PI/2;	//OK
	else
		wyniki->fKątAkcel[1] = atan2f(*(fAkcel + 2), *(fAkcel + 0)) + M_PI/2;	//OK

	//oblicz kąt odchylenia z danych magnetometru: tan(Y/X) dla X=N, Y=E => atan2(X, Y)
	float fSinPhi = sinf(wyniki->fKątAkcel[0]);
	float fCosPhi = cosf(wyniki->fKątAkcel[0]);
	float fSinThe = sinf(wyniki->fKątAkcel[1]);
	float fCosThe = cosf(wyniki->fKątAkcel[1]);

	//Rzutujemy pomiar magnetometru na płaszczyzne poziomą przy użyciu kątów phi i theta
	float fXm = *(fMagn + 0) * fCosThe + *(fMagn + 1) * fSinPhi * fSinThe + *(fMagn + 2) * fCosPhi * fSinThe;
	float fYm = *(fMagn + 1) * fCosPhi - *(fMagn + 2) * fSinPhi;
	//wyniki->fKątAkcel[2] = atan2f(-fYm, fXm);
	//wyniki->fKątAkcel[2] = atan2f(*(fMagn + 1), *(fMagn + 0));	//bez kompensacji przechylenia i pochylenia
	//wyniki->fKątAkcel[2] = atan2f(*(fMagn + 1), fXm);		//kompensacja X
	//wyniki->fKątAkcel[2] = atan2f(fYm, *(fMagn + 0));	//kompensacja Y
	wyniki->fKątAkcel[2] = atan2f(fYm, fXm);

	//filtr komplementarny IMU i całka z żyroskopów
	if (ndT > MAX_DT)	//nie licz całki prędkosci kątowej z przestojów
		return BLAD_ZLE_DANE;

	for (uint16_t n=0; n<3; n++)
	{
		fPrzyrostKąta = *(fŻyro + n) * ndT / 1000000;		//[rad/s] * [us / 1000000] => [rad]
		wyniki->fKątZyro1[n] += fPrzyrostKąta;
		wyniki->fKątIMU1[n] += fPrzyrostKąta;

		//ogranicz przyrost kąta IMU1 do +-Pi
		if (wyniki->fKątIMU1[n] > M_PI)
			wyniki->fKątIMU1[n] -= 2 * M_PI;
		if (wyniki->fKątIMU1[n] < -M_PI)
			wyniki->fKątIMU1[n] += 2 * M_PI;

		if (wyniki->fKątAkcel[n] > M_PI)
			wyniki->fKątAkcel[n] -= 2 * M_PI;
		if (wyniki->fKątAkcel[n] < -M_PI)
			wyniki->fKątAkcel[n] += 2 * M_PI;

		wyniki->fKątIMU1[n] = 0.02 * wyniki->fKątAkcel[n] + 0.98 * wyniki->fKątIMU1[n];	//kasowanie dryftu dodatkiem kątów z akcelerometru i magnetometru

		//w czasie kalibracji wzmocnienia nie ograniczaj przyrostu kąta
		if ((uDaneCM7.dane.cWykonajPolecenie < POL7_CALKUJ_PRED_KAT) || (uDaneCM7.dane.cWykonajPolecenie > POL7_KALIBRUJ_ZYRO_WZMP))
		{
			//ogranicz przyrost kąta do +-Pi
			if (wyniki->fKątZyro1[n] > M_PI)
				wyniki->fKątZyro1[n] -= 2 * M_PI;
			if (wyniki->fKątZyro1[n] < -M_PI)
				wyniki->fKątZyro1[n] += 2 * M_PI;
		}
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja przechowuje wektory przyspieszenia i magnetyczny w kwaternionach qAcc i qMag, obraca je o kąty uzyskane z całkowania prędkosci katowej
// Parametry:
//  *fZyro,  *fAkcel, *fMagn - wskaźniki na tablice 3 osi żyroskopu, akcelerometru i magnetometru użyanych do obliczeń
//  ndT - czas w mikrosekundach jaki upłynął od poprzedniej iteracji
// Zwraca: kod błędu
// Czas trwania: 39us @200Hz
////////////////////////////////////////////////////////////////////////////////
uint8_t JednostkaInercyjnaKwaterniony(float *fŻyro, float *fAkcel, float *fMagn, uint32_t ndT)
{
	float fdPhi2, fdTheta2, fdPsi2;	//połowy przyrostu kąta obrotu
	float fQx[4], fQy[4], fQz[4];	//kwaterniony obortów wokół osi XYZ
	float fQzy[4], fQzyx[4];
	float fQs[4], fQ[4];
	float fWspFiltraAkc, fWspFiltraMag;
	float fAccNorm[3], fMagNorm[3];
	float fQPoch[4], fQprze[4], fQkompens[4];	//kwaterniony obrotu o pochylenie i przechylenie oraz łączny obrót kompensacji magnetometru


	//Wyznaczam kwaterniony obrotów w formie algebraicznej korzystając z danych wejściowych sprowadzonych do formy trygonometrycznej
	//Ponieważ we wzorach występuje połowa kąta, więc aby nie wykonywać dzielenia wielokrotnie, od razu liczę wartość połowy kata
	//fQz = cos (psi/2) + (1 / sqrt(x^2 + y^2 + z^2)*k) * sin (psi/2)
	fdPsi2 = *(fŻyro+2) * ndT / 2000000.f;		//[rad/s] * ndT [us] / 1000000 = [rad]
	fQz[0] = cosf(fdPsi2);		//część rzeczywista kwaternionu: s0 = cos(theta) gdzie kąt oborotu to 2*theta, więc s0 = cos(kąt_obrotu/2)
	fQz[1] = 0;
	fQz[2] = 0;
	fQz[3] = sin(fdPsi2);	//z0 = vz / |v0| * sin(theta)  Oś obrotu to 1, więc moduł też będzie 1 więc można to pominąć

	//fQy = cos (theta/2) + (1 / sqrt(x^2 + y^2 + z^2)*j) * sin (theta/2)
	fdTheta2 = *(fŻyro+1) * ndT / 2000000.f;		//[rad/s] * ndT [us] / 1000000 = [rad]
	fQy[0] = cosf(fdTheta2);
	fQy[1] = 0;
	fQy[2] = sinf(fdTheta2);
	fQy[3] = 0;

	//całka z predkosci kątowej P to kąt Phi a przyrost kąta w czasie ndT to dPhi
	//fQx = cos (phi/2) + (1 / sqrt(x^2 + y^2 + z^2)*i) * sin (phi/2)
	fdPhi2 = *(fŻyro+0) * ndT / 2000000.f;		//[rad/s] * ndT [us] / 1000000 = [rad]
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
	NormalizujWektor(fAkcel, fAccNorm, 3);
	NormalizujWektor(fMagn, fMagNorm, 3);

	//synchronizuj modelowy wektor przyspieszenia  ze zmierzonym przyspieszeniem za pomocą filtra komplementarnego o wspólczynniku określonym przez filtr adaptacyjny
	fWspFiltraAkc = FiltrAdaptacyjnyAkc(fAkcel);
	fWspFiltraMag = FiltrAdaptacyjnyMag(fMagn);

	//zastosuj filtr komplementarny do korekty wektorów obracanych żyroskopami
	for (uint8_t n=0; n<3; n++)
	{
		fQAcc[n+1] = (1.0f - fWspFiltraAkc) * fQAcc[n+1] + fWspFiltraAkc * fAccNorm[n];
		fQMag[n+1] = (1.0f - fWspFiltraMag) * fQMag[n+1] + fWspFiltraMag * fMagNorm[n];
	}

	//oblicz kąty tradycyjnie trygonometrią
	//uDaneCM4.dane.fKątIMU2[0] = atan2f(-fQAcc[2], fQAcc[3]);	//kąt przechylenia z akcelerometru: tan(-Y/Z) = atan2(Z, -Y)
	uDaneCM4.dane.fKątIMU2[0] = atan2f(-fQAcc[3], fQAcc[2]) +  M_PI/2;	//OK
	//uDaneCM4.dane.fKątIMU2[1] = atan2f(fQAcc[1], fQAcc[3]);	//kąt pochylenia z akcelerometru: tan(Z/X) = atan2(X, Z)
	uDaneCM4.dane.fKątIMU2[1] = atan2f(fQAcc[3], fQAcc[1]) - M_PI/2;	//OK

	//Żeby policzyć kat odchylenia z wektora magnetometru najpierw skompensuj pochylenie i przechylenie obracając kopię wektora mag. o ujemne pochylenie i dodatnie przechylenie
	// zmiana znaku korekcji osi wynika prawdopodobnie ze składania obrotów
	fQprze[0] = cosf(uDaneCM4.dane.fKątIMU2[0] / 2);
	fQprze[1] = sinf(uDaneCM4.dane.fKątIMU2[0] / 2);
	fQprze[2] = 0;
	fQprze[3] = 0;

	fQPoch[0] = cosf(-uDaneCM4.dane.fKątIMU2[1] / 2);
	fQPoch[1] = 0;
	fQPoch[2] = sinf(-uDaneCM4.dane.fKątIMU2[1] / 2);
	fQPoch[3] = 0;

	MnozenieKwaternionow(fQPoch, fQprze, fQkompens);	//sumuj kwaterniony pochylenia i przechylenia w jeden kompensacyjny
	KwaternionSprzezony(fQkompens, fQs);				//kwaternion sprzężony

	MnozenieKwaternionow(fQkompens, fQMag, fQ);		//obróć kwaternion pola magnetycznego o ujemne pochylenie i dodatnie przechylenie
	MnozenieKwaternionow(fQ, fQs, fQMagKompens);

	//Oblicz katy Eulera: Phi, Theta, Psi - https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	KatyKwaterniona(fQAcc, fQMagKompens, (float*)uDaneCM4.dane.fKątKwater);	//wersja z asinf szybsza
	//KatyKwaterniona3(fQAcc, fQMagKompens, (float*)uDaneCM4.dane.fKątAkcel);	//wersja oryginalna dzielona przez -2

	//oblicz kąt odchylenia w radianach z danych magnetometru: tan(Y/X) dla X=N, Y=E => atan2(X, Y)
	uDaneCM4.dane.fKątIMU2[2] = atan2f(fQMagKompens[2], fQMagKompens[1]);

	//zawiń kąty, ponieważ dodawanie i odejmowanie Pi/2 przesuwa skalę poza zakres +-Pi
	for (uint8_t n=0; n<3; n++)
	{
		if (uDaneCM4.dane.fKątIMU2[n] > M_PI)
			uDaneCM4.dane.fKątIMU2[n] -= 2 * M_PI;
		if (uDaneCM4.dane.fKątIMU2[n] < -M_PI)
			uDaneCM4.dane.fKątIMU2[n] += 2 * M_PI;
	}
	return BLAD_OK;
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





