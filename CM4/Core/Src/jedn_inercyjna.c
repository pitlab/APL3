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
//float fKatZyroskopu1[3], fKatZyroskopu2[3];		//całka z prędkosci kątowej żyroskopów
float fKatAkcel1[3], fKatAkcel2[3];					//kąty pochylenia i przechylenia policzone z akcelerometru
extern float fOffsetZyro1[3], fOffsetZyro2[3];


static float fQAcc[4] = {0.0f, 0.0f, 0.0f, 1.0f};	//kwaternion wektora przyspieszenia
static float fQMag[4] = {0.0f, 0.0f, NOMINALNE_MAGN, 0.0f};	//kwaternion wektora magnetycznego



////////////////////////////////////////////////////////////////////////////////
// inicjalizacja zmiennych potrzebanych do obliczeń kątów
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujJednostkeInercyjna(void)
{
	uint8_t chErr = ERR_OK;
	//float fVect[3] = {0.0f, 0.0f, 1.0f};

	for (uint16_t n=0; n<3; n++)
	{
		uDaneCM4.dane.fKatIMUZyro1[n] = 0;
		uDaneCM4.dane.fKatIMUZyro2[n] = 0;
	}
	//WektorNaKwaternion(fVect, fQa);
	//WektorNaKwaternion(fVect, fQm);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia jednostki inercyjnej metodą filtra komplementarnego łaczącego całkę z prędkosci kątowych i kąty obliczone na podstawie równań trygonometrycznych z akcelerometrów
// Parametry: chGniazdo - numer gniazda w którym jest moduł IMU
// Zwraca: kod błędu
// Czas trwania: 24us @200Hz
////////////////////////////////////////////////////////////////////////////////
uint8_t JednostkaInercyjna1Trygonometria(uint8_t chGniazdo)
{
	//licz całkę z prędkosci kątowych żyroskopów
	for (uint16_t n=0; n<3; n++)
	{
		if ((uDaneCM7.dane.chWykonajPolecenie >= POL_CALKUJ_PRED_KAT)  && (uDaneCM7.dane.chWykonajPolecenie <= POL_KALIBRUJ_ZYRO_WZMP))
		{
			//w czasie kalibracji wzmocnienia nie uwzgledniej wzmocnienia, jedynie offset i nie ograniczaj przyrostu kąta
			uDaneCM4.dane.fKatIMUZyro1[n] +=  (uDaneCM4.dane.fZyroSur1[n] - fOffsetZyro1[n]) * ndT[chGniazdo] / 1000000;		//[rad/s] * [us / 1000000] => [rad]
			uDaneCM4.dane.fKatIMUZyro2[n] +=  (uDaneCM4.dane.fZyroSur1[n] - fOffsetZyro2[n]) * ndT[chGniazdo] / 1000000;
		}
		else	//normalna praca
		{
			uDaneCM4.dane.fKatIMUZyro1[n] +=  uDaneCM4.dane.fZyroKal1[n] * ndT[chGniazdo] / 1000000;		//[rad/s] * [us / 1000000] => [rad]
			uDaneCM4.dane.fKatIMUZyro2[n] +=  uDaneCM4.dane.fZyroKal2[n] * ndT[chGniazdo] / 1000000;

			//ogranicz przyrost kąta do +-Pi
			if (uDaneCM4.dane.fKatIMUZyro1[n] > M_PI)
				uDaneCM4.dane.fKatIMUZyro1[n] = -M_PI;
			if (uDaneCM4.dane.fKatIMUZyro1[n] < -M_PI)
				uDaneCM4.dane.fKatIMUZyro1[n] = M_PI;
			if (uDaneCM4.dane.fKatIMUZyro2[n] > M_PI)
				uDaneCM4.dane.fKatIMUZyro2[n] = -M_PI;
			if (uDaneCM4.dane.fKatIMUZyro2[n] < -M_PI)
				uDaneCM4.dane.fKatIMUZyro2[n] = M_PI;
		}
	}

	//kąt przechylenia z akcelerometru: tan(Z/Y) = atan2(Y, Z)
	uDaneCM4.dane.fKatIMUAkcel1[0] = atan2f(uDaneCM4.dane.fAkcel1[1], uDaneCM4.dane.fAkcel1[2]);
	uDaneCM4.dane.fKatIMUAkcel2[0] = atan2f(uDaneCM4.dane.fAkcel2[1], uDaneCM4.dane.fAkcel2[2]);

	//kąt pochylenia z akcelerometru: tan(-Z/X) = atan2(X, -Z)
	uDaneCM4.dane.fKatIMUAkcel1[1] = atan2f(uDaneCM4.dane.fAkcel1[0], -uDaneCM4.dane.fAkcel1[2]);
	uDaneCM4.dane.fKatIMUAkcel2[1] = atan2f(uDaneCM4.dane.fAkcel2[0], -uDaneCM4.dane.fAkcel2[2]);

	//oblicz kąt odchylenia w radianach z danych magnetometru: tan(y/x)
	//uDaneCM4.dane.fKatIMUAkcel1[2] = atan2f((float)uDaneCM4.dane.sMagne3[1], (float)uDaneCM4.dane.sMagne3[0]);

	//kąt odchylenia z akcelerometru: tan(Y/X) = atan2(X, Y)
	uDaneCM4.dane.fKatIMUAkcel1[2] = atan2f(uDaneCM4.dane.fAkcel1[0], uDaneCM4.dane.fAkcel1[1]);
	uDaneCM4.dane.fKatIMUAkcel2[2] = atan2f(uDaneCM4.dane.fAkcel2[0], uDaneCM4.dane.fAkcel2[1]);

	//filtr komplementarny IMU
	for (uint16_t n=0; n<3; n++)
	{
		uDaneCM4.dane.fKatIMU1[n] += uDaneCM4.dane.fZyroKal1[n] * ndT[chGniazdo] / 1000000;
		uDaneCM4.dane.fKatIMU1[n] = 0.05 * uDaneCM4.dane.fKatIMUAkcel1[n] + 0.95 * uDaneCM4.dane.fKatIMU1[n];
	}
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Obliczenia IMU metodą publikowaną przez Sebastiana Madgwicka w jego pracy doktorskiej: //https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf
// Zwraca: kod błędu
// Czas trwania: ?
////////////////////////////////////////////////////////////////////////////////
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;   	//kwaterniony
float exInt = 0, eyInt = 0, ezInt = 0;				// scaled integral error
void JednostkaInercyjnaMadgwick(void)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float ax,ay,az,gx,gy,gz;

    //przypisz wartości do zmiennych roboczych
    ax = uDaneCM4.dane.fAkcel1[0];
    ay = uDaneCM4.dane.fAkcel1[1];
    az = uDaneCM4.dane.fAkcel1[2];
    gx = uDaneCM4.dane.fZyroKal1[0];
    gy = uDaneCM4.dane.fZyroKal1[1];
    gz = uDaneCM4.dane.fZyroKal1[2];

	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;

	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	// integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	//Oblicz katy Eulera: Phi, Theta, Psi
	uDaneCM4.dane.fKatIMU2[0] = atan2f(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
	uDaneCM4.dane.fKatIMU2[1] = asinf( 2 * (q0*q2 - q1*q3));
	uDaneCM4.dane.fKatIMU2[2] = atan2f(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));

	//ogranicz wartość kątów w zakresie -Pi..Pi
	for(uint8_t x=0; x<3; x++)
	{
		if (uDaneCM4.dane.fKatIMU2[x] > M_PI)
			uDaneCM4.dane.fKatIMU2[x] -= M_PI;
		else
		if (uDaneCM4.dane.fKatIMU2[x] < -M_PI)
			uDaneCM4.dane.fKatIMU2[x] += M_PI;
	}
}







////////////////////////////////////////////////////////////////////////////////
// Funkcja przechowuje wektory przyspieszenia i magnetyczny w kwaternionach qAcc i qMag, obraca je o kąty uzyskane z całkowania prędkosci katowej
// Parametry: chGniazdo - numer gniazda w którym jest moduł IMU
// Zwraca: nic
// Czas trwania: 39us @200Hz
////////////////////////////////////////////////////////////////////////////////
uint8_t JednostkaInercyjna4Kwaterniony(uint8_t chGniazdo)
{
	float fdPhi2, fdTheta2, fdPsi2;	//połowy przyrostu kąta obrotu
	float fQx[4], fQy[4], fQz[4];	//kwaterniony obortów wokół osi XYZ
	float fQzy[4], fQzyx[4];
	float fQs[4], fQ[4];
	float fWspFiltraAkc, fWspFiltraMag;
	float fAccNorm[3];

	//Wyznaczam kwaterniony obrotów w formie algebraicznej korzystając z danych wejściowych wprowadzonych do formy trygonometrycznej
	//fQz = cos (psi/2) + (1 / sqrt(x^2 + y^2 + z^2)*k) * sin (psi/2)
	fdPsi2 = uDaneCM4.dane.fZyroKal1[2] * ndT[chGniazdo] / 2000000;		//[rad/s] * ndT [us] / 1000000 = [rad]
	//fdPsi2 = 0.0f * DEG2RAD / 2;
	fQz[0] = cosf(fdPsi2);		//część rzeczywista kwaternionu: s0 = cos(theta) gdzie kąt oborotu to 2*theta, więc s0 = cos(kąt_obrotu/2)
	fQz[1] = 0;
	fQz[2] = 0;
	fQz[3] = sin(fdPsi2);	//z0 = vz / |v0| * sin(theta)  Oś obrotu to 1, więc moduł też będzie 1 więc można to pominąć

	//fQy = cos (theta/2) + (1 / sqrt(x^2 + y^2 + z^2)*j) * sin (theta/2)
	fdTheta2 = uDaneCM4.dane.fZyroKal1[1] * ndT[chGniazdo] / 2000000;		//[rad/s] * ndT [us] / 1000000 = [rad]
	//fdTheta2 = 0.2f * DEG2RAD / 2;
	fQy[0] = cosf(fdTheta2);
	fQy[1] = 0;
	fQy[2] = sinf(fdTheta2);
	fQy[3] = 0;

	//całka z predkosci kątowej P to kąt Phi a przyrost kąta w czasie ndT to dPhi
	//fdPhi = uDaneCM4.dane.fZyroKal1[0] * ndT[chGniazdo] / 1000000;
	//Ponieważ we wzorze występuje połwa kąta, więc aby nie wykonywać dzielenia wielokrotnie, od razu liczę połowę kata
	//fQx = cos (phi/2) + (1 / sqrt(x^2 + y^2 + z^2)*i) * sin (phi/2)
	fdPhi2 = uDaneCM4.dane.fZyroKal1[0] * ndT[chGniazdo] / 2000000;		//[rad/s] * ndT [us] / 1000000 = [rad]
	//fdPhi2 = 0.0f * DEG2RAD / 2;
	fQx[0] = cosf(fdPhi2);
	fQx[1] = sinf(fdPhi2);
	fQx[2] = 0;
	fQx[3] = 0;

	//składanie 3 obrotów w jeden
	MnozenieKwaternionow(fQy, fQz, fQzy);	//obrót najpierw wokół Z * Y
	MnozenieKwaternionow(fQx, fQzy, fQzyx);	//potem obrót ZY * X

	//obroty wektorów przyspieszenia i magnetycznego
	KwaternionSprzezony(fQzyx, fQs);	//kwaternion sprzężony z kwaternionem obrotu o wszystkie osie

	MnozenieKwaternionow(fQzyx, fQAcc, fQ);		//przyspieszenie
	MnozenieKwaternionow(fQ, fQs, fQAcc);

	MnozenieKwaternionow(fQzyx, fQMag, fQ);		//pole magnetyczne
	MnozenieKwaternionow(fQ, fQs, fQMag);

	//normalizuj wektor przyspieszenia, bo wymaga tego asinf() w funkcji liczenia kątów. Magnetometr może być nienormalizowany bo używa atan2f()
	Normalizuj((float*)uDaneCM4.dane.fAkcel1, fAccNorm, 3);

	//synchronizuj modelowy wektor przyspieszenia  ze zmierzonym przyspieszeniem za pomocą filtra komplementarnego o wspólczynniku określonym przez filtr adaptacyjny
	fWspFiltraAkc = FiltrAdaptacyjnyAkc((float*)uDaneCM4.dane.fAkcel1);
	fWspFiltraMag = FiltrAdaptacyjnyMag((float*)uDaneCM4.dane.fMagne3);

	//zastosuj filtr komplementarny do korekty wektorów obracanych żyroskopami
	for (uint8_t n=0; n<3; n++)
	{
		fQAcc[n+1] = (1.0f - fWspFiltraAkc) * fQAcc[n+1] + fWspFiltraAkc * fAccNorm[n];
		fQMag[n+1] = (1.0f - fWspFiltraMag) * fQMag[n+1] + fWspFiltraMag * uDaneCM4.dane.fMagne3[n];
	}

	//Normalizuj(fQAcc, fQAcc, 4);	//normalizuj kwaternion przyspieszenia po przejściu filtra komplementarnego


	//Oblicz katy Eulera: Phi, Theta, Psi - https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	KatyKwaterniona(fQAcc, fQMag, (float*)uDaneCM4.dane.fKatIMU2);
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja liczy na ile pomiar akcelerometrów jest zaburzony przyspieszeniami wynikającymi z ruchu (przyspieszenia liniowe i odśrodkowe) i
// na tej podstawie wylicza wartość współczynnika dla filtra komplementarnego
// Przyjęte są 2 progi: PROG_ACC_DOBRY do którego wskazanie uznajemy za wiarygodne, PROG_ACC_ZLY - powyżej którego jest brak korekcji żyroskopów akcelerometrem
// Przyjeta jest wartość filtra dla progu PROG_ACC_DOBRY wynosząca WSP_FILTRA_ADAPT mówiaca ile ma być korekcji akcelerometru dla dobrego sygnału
// Zwraca: wartość filtra komplementarnego między 0.0 a 1.0
// Czas trwania: 1,38us @200MHz
////////////////////////////////////////////////////////////////////////////////
float FiltrAdaptacyjnyAkc(float *fAkcel)
{
	float fPrzyspRuchu = 0.0f;	//przyspieszenie wynikające ze zmiany predkości
	float fWspFiltra;			//współczynnik filtra komplementarnego

	for (uint8_t n=0; n<3; n++)
		fPrzyspRuchu += *(fAkcel+n) * *(fAkcel+n);			//suma kwadratów 3 osi akcelerometru
	fPrzyspRuchu = sqrtf(fPrzyspRuchu);						//bezwzględna długość wektora przyspieszenia
	fPrzyspRuchu = fabs(fPrzyspRuchu - AKCEL1G) / AKCEL1G;	//wartość przyspieszenia wynikającego ze zmiany prędkości [m/s^2]

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
// Funkcja liczy na ile zaburzony jest pomiar magnetometru, alanlogicznie jak akcelerometr powyżej
// na tej podstawie wylicza wartość współczynnika dla filtra komplementarnego
// Zwraca: wartość filtra komplementarnego między 0.0 a 1.0
// Czas trwania: 1,38us @200MHz
////////////////////////////////////////////////////////////////////////////////
float FiltrAdaptacyjnyMag(float *fMag)
{
	float fZaklocenieMag = 0.0f;	//wartość zakłóceniazmieniająca długość wektora magnetycznego
	float fWspFiltra;		//współczynnik filtra komplementarnego

	for (uint8_t n=0; n<3; n++)
		fZaklocenieMag += *(fMag+n) * *(fMag+n);	//suma kwadratów 3 osi magnetometru
	fZaklocenieMag = sqrtf(fZaklocenieMag);			//bezwzględna długość wektora magnetycznego
	fZaklocenieMag = fabs(fZaklocenieMag - NOMINALNE_MAGN) / NOMINALNE_MAGN;				//wartość przyspieszenia wynikającego ze zmiany prędkości [m/s^2]

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
void ObrotWektora(uint8_t chGniazdo)
{
	//float fS0, fX0, fY0, fZ0;
	float fdPhi2, fdTheta2, fdPsi2;	//połowy przyrostu kąta obrotu
	float fModul;	//długości osi obracajacej wektor
	float qx[4], qy[4], qz[4];	//kwaterniony obortów wokół osi XYZ
	float qv[4], vq[4];	//pośrednie etapy mnożenia
	float qp[4];	//kwaternion obracanego wektora
	float qs[4];	//kwaternion sprzężony
	float p[3], r[3], s[3], t[3];		//punkt obracany i po obrocie
	//float fOsObr[3];			//oś obrotu
	uint32_t nCzas;
	float A[4][4], B[4][4], C[4][4];	//kwaterniony w postaci macierzowej
	float fQxyz[4];	//kwaternion obrotu o wszystkie 3 osie jednoczesnie

	//Moduł kwaternionu q to pierwiastek q i kwaterniony sprzężonego q*: |q| = sqrt(q * q*)
	//Po wykonaniu mmnożenia q * q* część urojona się zeruje pozostawiając część wektorową: sqrt(s^2 + x^2 + y^2 + z^2) = 1

	//postać trygonometryczna kwaternionu q to: q = |q| * (cos fi + (uxi + uyj + uzk) * sin fi)


	//utworzyć kwaternion jednostkowy reprezentujący przekształcenie obrotu o całkę z prędkosci kątowej P wokół osi X według wzoru na formę trygonometryczną kwaternionu
	//os X wokół ktorej odbywa się obrót jest zdefiniowana jako v0 = [vx=1, vy=0, vz=0], więc kwaternion obrotu będzie wygladał tak:
	//qx = cos (phi/2) + (1 / sqrt(x^2 + y^2 + z^2)*i + 0 / sqrt(x^2 + y^2 + z^2) * j + 0 / sqrt(x^2 + y^2 + z^2) * k) * sin (phi/2)	//odpadają części j i k bo w liczniki mają zero wiec zostaje część i i funkcje trygonometryczne
	//qx = cos (phi/2) + (1 / sqrt(x^2 + y^2 + z^2)*i) * sin (phi/2)

	//punkt obracany
	p[0] = 3.0f;
	p[1] = 0.0f;
	p[2] = 0.0f;



	//W pierwszej kolejnosci obracam punkt przez oś obrotu Z
	//fOsObr[0] = 0.0f;
	//fOsObr[1] = 0.0f;
	//fOsObr[2] = 1.0f;

	//analogicznie dla obrotu przez os Z [0, 0, 1] o kąt psi będący całką po ndT z prędkości R
	//qz = cos (psi/2) + (1 / sqrt(x^2 + y^2 + z^2)*k) * sin (psi/2)
//	fdPsi2 = uDaneCM4.dane.fZyroKal1[0] * ndT[chGniazdo] / 2000000;
	fdPsi2 = 90 * DEG2RAD / 2;
	nCzas = PobierzCzas();

	//Wyznaczam kwaternion obrotu w formie algebraicznej korzystając z danych wejściowych wprowadzonych do formy trygonometrycznej
	//1. wyznaczyć część rzeczywistą kwaternionu s0 = cos(theta) gdzie kąt oborotu to 2*theta, więc s0 = cos(kąt_obrotu/2)
	qz[0] = cosf(fdPsi2);

	//2. Obliczyć długość osi obrotu |v0| = sqrt(vx^2 + vy^2 + vz^2).
	//Dla osi X będzie wektor kierunkowy to [1, 0, 0], więc |v0x| = sqrt(1^2 + 0^2 + 0^2)
	//fModul_v0x = sqrtf(fOsObr[0]*fOsObr[0] + fOsObr[1]*fOsObr[1] + fOsObr[2]*fOsObr[2]);
	//fModul = sqrtf(fOsObr[2]*fOsObr[2]);

	//3. Obliczyć x0 = vx / |v0| * sin(theta)
	//qz[1] = fOsObr[0] / fModul * sin(fdPsi2);;		//dzielenie zera przez coś zawsze da zero
	qz[1] = 0;

	//4. Obliczyć y0 = vy / |v0| * sin(theta)
	//qz[2] = fOsObr[1] / fModul * sin(fdPsi2);;		//dzielenie zera przez coś zawsze da zero
	qz[2] = 0;

	//5. Obliczyć z0 = vz / |v0| * sin(theta)
	//qz[3] = fOsObr[2] / fModul * sin(fdPsi2);
	qz[3] = sin(fdPsi2);	//oś obrotu to 1, więc moduł też będzie 1 więc można to pominąć


	//teraz obrót przez oś Y
	//fOsObr[0] = 0.0f;
	//fOsObr[1] = 1.0f;
	//fOsObr[2] = 0.0f;

	//analogicznie dla obrotu przez os Y [0, 1, 0] o kąt theta będący całką po ndT z prędkości Q
	//qy = cos (theta/2) + (1 / sqrt(x^2 + y^2 + z^2)*j) * sin (theta/2)
//	fdTheta2 = uDaneCM4.dane.fZyroKal1[0] * ndT[chGniazdo] / 2000000;
	fdTheta2 = 90 * DEG2RAD / 2;
	//fModul = sqrtf(fOsObr[1]*fOsObr[1]);
	qy[0] = cosf(fdTheta2);
	//qy[1] = fOsObr[0] / fModul * sinf(fdTheta2);;
	qy[1] = 0;
	//qy[2] = fOsObr[1] / fModul * sinf(fdTheta2);;
	qy[2] = sinf(fdTheta2);
	//qy[3] = fOsObr[2] / fModul * sinf(fdTheta2);;
	qy[3] = 0;


	//Na koniec obrót przez oś X
	//fOsObr[0] = 1.0f;
	//fOsObr[1] = 0.0f;
	//fOsObr[2] = 0.0f;

	//całka z predkosci kątowej P to kąt Phi a przyrost kąta w czasie ndT to dPhi
	//fdPhi = uDaneCM4.dane.fZyroKal1[0] * ndT[chGniazdo] / 1000000;	//[rad/s] * ndT [us] / 1000000 = [rad]
	//Ponieważ we wzorze występuje połwa kąta, więc aby nie wykonywać dzielenia wielokrotnie, od razu liczę połowę kata
//	fdPhi2 = uDaneCM4.dane.fZyroKal1[0] * ndT[chGniazdo] / 2000000;
	fdPhi2 = 90 * DEG2RAD / 2;	//testowo obróć o taki kąt
	//fModul = sqrtf(fOsObr[0]*fOsObr[0]);
	qx[0] = cosf(fdPhi2);
	//qx[1] = fOsObr[0] / fModul * sinf(fdPhi2);
	qx[1] = sinf(fdPhi2);
	//qx[2] = fOsObr[1] / fModul * sinf(fdPhi2);
	qx[2] = 0;
	//qx[3] = fOsObr[2] / fModul * sinf(fdPhi2);
	qx[3] = 0;


	fModul = sqrtf(3);	//oś obrotu to [1,1,1]
	fQxyz[0] = 0;//suma 3 kątów
	fQxyz[1] = 1 / fModul * sinf(fdPhi2);
	fQxyz[2] = 1 / fModul * sinf(fdTheta2);
	fQxyz[3] = 1 / fModul * sinf(fdPsi2);



	//aby zoptymalizowac obliczenia rotacji przez 3 osie stusuję złożenie obrotów dla osi jako kwaterion w postaci macierzowej a następnie przemnażam przez siebie te macierze
	//[s1	x1	y1	z1]			[s2	x2	y2	z2]			[s3	x3	y3	z3]
	//[-x1	s1	-z1	y1]		*	[-x2 s2	-z2	y2]  *		[-x3 s3	-z3	y3]
	//[-y1	z1	s1	-x1]		[-y2 z2	s2	-x2]		[-y3 z3	s3	-x3]
	//[-z1	-y1	x1	s1]			[-z2 -y2 x2	s2]			[-z3 -y3 x3	s3]

	//Aby nie wykonywać 3 osobnych obrotów dla każdej osi, można obroty zlożyć w jeden monożąc przez siebie kwaterniony obrotu
	//Metoda 1 - działa dobrze
	nCzas = PobierzCzas();
	MnozenieKwaternionow(qy, qz, qv);	//obrót najpierw wokół Z * Y = qv
	MnozenieKwaternionow(qx, qv, vq);	//potem obrót ZY * X
	ObrotWektoraKwaternionem(p, vq, r);
	nCzas = MinalCzas(nCzas);		//6us


	//metoda 2 - niezależne obroty - działa poprawnie
	nCzas = PobierzCzas();
	ObrotWektoraKwaternionem(p, qz, r);
	ObrotWektoraKwaternionem(r, qy, s);
	ObrotWektoraKwaternionem(s, qx, t);
	nCzas = MinalCzas(nCzas);	//7us


	//metoda 3- działa poprawnie z mnożeniem 3
	nCzas = PobierzCzas();
	WektorNaKwaternion(p, qp);

	//obrót wokół Z
	MnozenieKwaternionow(qz, qp, qv);	// Wykonuję pierwsze mnożenie q * v
	KwaternionSprzezony(qz, qs);		//kwaternion sprzężony q* ma taką samą część rzeczywistą i ujemne części urojone
	MnozenieKwaternionow(qv, qs, vq);	//wykonuję drugie mnożenie (qv) * q*

	//obrót wokół Y
	MnozenieKwaternionow(qy, vq, qv);
	KwaternionSprzezony(qy, qs);
	MnozenieKwaternionow(qv, qs, vq);

	//obrót wokól X
	MnozenieKwaternionow(qx, vq, qv);
	KwaternionSprzezony(qx, qs);
	MnozenieKwaternionow(qv, qs, vq);
	nCzas = MinalCzas(nCzas);		//12us


	//metoda 4 składanie obrotów z mnożeniem macierzy - działa dobrze
	nCzas = PobierzCzas();
	KwaternionNaMacierz(qy, &A[0][0]);
	KwaternionNaMacierz(qz, &B[0][0]);
	MnozenieMacierzy4x4(&A[0][0], &B[0][0], &C[0][0]);
	//MacierzNaKwaternion(&C[0][0], vq);
	//ObrotWektoraKwaternionem(p, vq, r);

	KwaternionNaMacierz(qx, &A[0][0]);
	MnozenieMacierzy4x4(&A[0][0], &C[0][0], &B[0][0]);
	MacierzNaKwaternion(&B[0][0], vq);
	ObrotWektoraKwaternionem(p, vq, s);
	nCzas = MinalCzas(nCzas);	//18us

	//metoda 5 - zweryfikować - źle
	ObrotWektoraKwaternionem(p, fQxyz, r);


	//pomiary czasu
	nCzas = PobierzCzas();
	JednostkaInercyjna4Kwaterniony(ADR_MOD2);
	nCzas = MinalCzas(nCzas);

/*	float fAccNorm[3];

	nCzas = PobierzCzas();
	for (uint16_t n=0; n<100; n++)
		Normalizuj((float*)uDaneCM4.dane.fAkcel1, fAccNorm, 3);
	nCzas = MinalCzas(nCzas);*/
	return;
}



