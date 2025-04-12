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

// Definicje użytej konwencji
// prawoskrętny układ współrzędnych samolotu
// oś X od środka ciężkości w stronę dziobu samolotu
// oś Y od środka ciężkości w kierunku prawego skrzydła
// oś Z od środka ciężkości w kierunku ziemi
//
// Układ współrzędnych związany z Ziemią: (PWD - Północ Wschód Dół, lub NED - North East Down)
// oś X skierowana na północ
// oś Y skierowana nz wschód
// oś Z skierowana w dół
//
// Prędkości kątowe (układ prawoskrętny, patrząc w punkcie 0 zgodnie zwe zwrotem osi obrót jest dodatni gdy jest zgodny z kierunkiem ruchu wskazówek zegara)
// prędkość P wokół osi X dodatnia gdy prawe skrzydło się opuszcza
// prędkość Q wokół osi Y dodatnia gdy dziób się opuszcza
// prędkość R wokół osi Z odatnia gdy dziób obraca sie w lewo
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
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;   	//kwaterniony
float exInt = 0, eyInt = 0, ezInt = 0;				// scaled integral error
float v[4], q[4], qs[4];	//kwaterniony wektora, obrotu i sprzężony obrotu




////////////////////////////////////////////////////////////////////////////////
// inicjalizacja zmiennych potrzebanych do obliczeń kątów
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujJednostkeInercyjna(void)
{
	uint8_t chErr = ERR_OK;

	for (uint16_t n=0; n<3; n++)
	{
		uDaneCM4.dane.fKatIMUZyro1[n] = 0;
		uDaneCM4.dane.fKatIMUZyro2[n] = 0;
		v[n+1] = uDaneCM4.dane.fAkcel1[n];		//obracany wektor jest odczytem z akcelerometru

	}
	v[0] = 0;	//część rzeczywista kwaternionu wektora
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia jednostki inercyjnej metodą filtra komplementarnego łaczącego całkę z prędkosci kątowych i kąty obliczone na podstawie równań trygonometrycznych z akcelerometrów
// Parametry: chGniazdo - numer gniazda w którym jest moduł IMU
// Zwraca: kod błędu
// Czas trwania: ?
////////////////////////////////////////////////////////////////////////////////
void ObliczeniaJednostkiInercujnej(uint8_t chGniazdo)
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

	//kąt przechylenia z akcelerometru: tan(Z/Y)
	uDaneCM4.dane.fKatIMUAkcel1[0] = atan2f(uDaneCM4.dane.fAkcel1[2], uDaneCM4.dane.fAkcel1[1]) - 90 * DEG2RAD;
	uDaneCM4.dane.fKatIMUAkcel2[0] = atan2f(uDaneCM4.dane.fAkcel2[2], uDaneCM4.dane.fAkcel2[1]) - 90 * DEG2RAD;

	//kąt pochylenia z akcelerometru: tan(Z/X)
	uDaneCM4.dane.fKatIMUAkcel1[1] = atan2f(uDaneCM4.dane.fAkcel1[2], uDaneCM4.dane.fAkcel1[0]) - 90 * DEG2RAD;
	uDaneCM4.dane.fKatIMUAkcel2[1] = atan2f(uDaneCM4.dane.fAkcel2[2], uDaneCM4.dane.fAkcel2[0]) - 90 * DEG2RAD;

	//oblicz kąt odchylenia w radianach z danych magnetometru: tan(y/x)
	//uDaneCM4.dane.fKatIMUAkcel1[2] = atan2f((float)uDaneCM4.dane.sMagne3[1], (float)uDaneCM4.dane.sMagne3[0]);

	//kąt odchylenia z akcelerometru: tan(Y/X)
	uDaneCM4.dane.fKatIMUAkcel1[2] = atan2f(uDaneCM4.dane.fAkcel1[1], uDaneCM4.dane.fAkcel1[0]);// - 90 * DEG2RAD;
	uDaneCM4.dane.fKatIMUAkcel2[2] = atan2f(uDaneCM4.dane.fAkcel2[1], uDaneCM4.dane.fAkcel2[0]);// - 90 * DEG2RAD;

	//filtr komplementarny IMU
	for (uint16_t n=0; n<3; n++)
	{
		uDaneCM4.dane.fKatIMU1[n] += uDaneCM4.dane.fZyroKal1[n] * ndT[chGniazdo] / 1000000;
		uDaneCM4.dane.fKatIMU1[n] = 0.05 * uDaneCM4.dane.fKatIMUAkcel1[n] + 0.95 * uDaneCM4.dane.fKatIMU1[n];
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////
// Obliczenia metoda kwaternionow bazujace na samym IMU
// zrodlo przeksztalcenia ze strony en.wikipedia.org/wiki/Rotation_representation_(mathematics)
// Parametry: chGniazdo - numer gniazda w którym jest moduł IMU
// ax, ay, az - wartości przyspieszenia z akcelerometru
// gx, gy, gz - prędkosci kątowe z żyroskopu [rad/s]
// Zwraca: kod błędu
// Czas trwania: ?
////////////////////////////////////////////////////////////////////////////////
uint8_t JednostkaInercyjnaKwaterniony(uint8_t chGniazdo)
{
    float fdTime;
    float fNormal;
    float s0, s1, s2, s3;
    float ax,ay,az,gx,gy,gz;
    float beta = 0.1f;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    //sprawdź czas aby wyeliminować sytuacje początkowe lub długie przestoje
    if (ndT[chGniazdo] > 2*1000000/CZESTOTLIWOSC_PETLI) //czas [us] nie powinien być większy niż 2 obiegi petli
        return ERR_ZA_DLUGO;

    //czas od poprzedniego pomiaru w sekundach
    fdTime = ndT[chGniazdo] * 1.0e-6;

    //przypisz wartości do zmiennych roboczych
    ax = uDaneCM4.dane.fAkcel1[0];
    ay = uDaneCM4.dane.fAkcel1[1];
    az = uDaneCM4.dane.fAkcel1[2];
    gx = uDaneCM4.dane.fZyroKal1[0];
    gy = uDaneCM4.dane.fZyroKal1[1];
    gz = uDaneCM4.dane.fZyroKal1[2];

    //kwaternion prędkosci katowej
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    //licz tylko gdy składowe przyspieszenia ziemskiego sa niezerowe aby uniknąć dzielenia przez zero przy normalizacji
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        //normalizuj dugość wektora przyspieszenia
    	fNormal = sqrt(ax * ax + ay * ay + az * az);
        ax /= fNormal;
        ay /= fNormal;
        az /= fNormal;

        //zmienne pomocnicze aby uniknąć wielokrotnych obliczeń tego smego
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        //normalizuj odpowiedź
        fNormal = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 /= fNormal;
        s1 /= fNormal;
        s2 /= fNormal;
        s3 /= fNormal;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    //całkuj kwaternion z prędkosciami katowymi
    q0 += qDot1 * fdTime;
    q1 += qDot2 * fdTime;
    q2 += qDot3 * fdTime;
    q3 += qDot4 * fdTime;

    //normalizuj kwaternion z kątami
    fNormal = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= fNormal;
    q1 /= fNormal;
    q2 /= fNormal;
    q3 /= fNormal;

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
    return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Obliczenia IMU metodą publikowaną przez Sebastiana Madgwicka w jego pracy doktorskiej: //https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf
// Zwraca: kod błędu
// Czas trwania: ?
////////////////////////////////////////////////////////////////////////////////
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
// Obliczenia obrotu znormalizowanego wektora grawitacji o kąt z żyroskopów na podstawie instrukcji Mateusza Kowalskiego: https://youtu.be/XjFq3Slo2wo?si=5JBCD7lqXvaLayjN
// Trzeba utworzyć kwaterniony q i v gdzie v będzie zawierał współrzędne obracanego wektora a q będzie definiował obrót i jest kwaternionem jednostkowym
// Następnie trzeba wykonać mnożenie q * v * q* gdzie q* jest kwaternionem sprzężonym.
// Kwaternion sprzeżony ma części urojone ze znakiem minus. Jeżeli q = s + xi + yj + zk to q* = s - xi - yj - zk
// Mnożenie przez kwaternion sprzężony wykonuje się po to aby pozbyć się części rzeczywistej, której nie potrafimy zinterpretować gdyż nasz wektor siedzi w części urojonej.
// mnożenie przez q i następnie przez sprzezone q* spowoduje wyzerowanie części rzeczywistej. Skutkiem ubocznym jest obrót o podwójny kąt, dlatego obracamy o połowę kąta
// Wektorem obracamym jest wektor przyspieszenie ziemskiego, będący odczytem początkowym z akcelerometru. Wektor nie musi być znormalizowany więc jest to po prostu uDaneCM4.dane.fAkcel1 lub uDaneCM4.dane.fAkcel2
// Mnożenie kwaternionów wykonujemy w postaci algebraicznej a podstawianie danych w postaci trygonometrycznej, więc potrzebne jest przejscie miedzy tymi dwiema formami
// Przejscie z formy trygonometrycznej na algebraiczną:
//	q = |q| * (cos fi + (uxi + uyj + uzk) * sin fi)  ->  q = s + xi + yj + zk
//	s = |q| * cos fi
//	x = |q| * ux * sin fi
//	y = |q| * uy * sin fi
//	z = |q| * uz * sin fi
// Przejście z formy algebraicznej na trygonometryczną:
//	q = s + xi + yj + zk  ->  q = |q| * (cos fi + (uxi + uyj + uzk) * sin fi)
//	|q| = sqrt(s^2 + x^2 + y^2 + z^2)
//	ux = x / sqrt(x^2 + y^2 + z^2)		//bez s^2
//	uy = y / sqrt(x^2 + y^2 + z^2)
//	uz = z / sqrt(x^2 + y^2 + z^2)
//	cos fi = s / sqrt(s^2 + x^2 + y^2 + z^2)
//	sin fi = sqrt(x^2 + y^2 + z^2) / sqrt(s^2 + x^2 + y^2 + z^2)
//	fi = arcos(s / sqrt(s^2 + x^2 + y^2 + z^2))
//	Jeżeli przyjmiemy że moduł z q: |q| = 1 to mamy uproszczenie:
//	cos fi = s
//	sin fi = sqrt(x^2 + y^2 + z^2)
//	fi = arcos(s)
// Zwraca: nic
// Czas trwania: ?
////////////////////////////////////////////////////////////////////////////////
void ObrotWektora(uint8_t chGniazdo)
{
	//float fS0, fX0, fY0, fZ0;
	float fdPhi2, fdTheta2, fdPsi2;	//połowy przyrostu kąta obrotu
	float fModul_v0x, fModul_v0y, fModul_v0z;	//długości osi obracajacej wektor
	float qv[4], vq[4];	//pośrednie etapy mnożenia
	float qs[4];	//sprzężone q
	float p[3];		//punkt obracany


	//Moduł kwaternionu q to pierwiastek q i kwaterniony sprzężonego q*: |q| = sqrt(q * q*)
	//Po wykonaniu mmnożenia q * q* część urojona się zeruje pozostawiając część wektorową: sqrt(s^2 + x^2 + y^2 + z^2) = 1

	//postać trygonometryczna kwaternionu q to: q = |q| * (cos fi + (uxi + uyj + uzk) * sin fi)


	//utworzyć kwaternion jednostkowy reprezentujący przekształcenie obrotu o całkę z prędkosci kątowej P wokół osi X według wzoru na formę trygonometryczną kwaternionu
	//os X wokół ktorej odbywa się obrót jest zdefiniowana jako v0 = [vx=1, vy=0, vz=0], więc kwaternion obrotu będzie wygladał tak:
	//qx = cos (phi/2) + (1 / sqrt(x^2 + y^2 + z^2)*i + 0 / sqrt(x^2 + y^2 + z^2) * j + 0 / sqrt(x^2 + y^2 + z^2) * k) * sin (phi/2)	//odpadają części j i k bo w liczniki mają zero wiec zostaje część i i funkcje trygonometryczne
	//qx = cos (phi/2) + (1 / sqrt(x^2 + y^2 + z^2)*i) * sin (phi/2)

	//punkt obracany
	p[0] = 1.0f;
	p[1] = 0.0f;
	p[2] = 0.0f;

	//oś obrotu
	v[0] = 0;
	v[1] = 1.0f;
	v[2] = 1.0f;
	v[3] = 1.0f;

	//całka z predkosci kątowej P to kąt Phi a przyrost kąta w czasie ndT to dPhi
	//fdPhi = uDaneCM4.dane.fZyroKal1[0] * ndT[chGniazdo] / 1000000;	//[rad/s] * ndT [us] / 1000000 = [rad]
	//Ponieważ we wzorze występuje połwa kąta, więc aby nie wykonywać dzielenia wielokrotnie, od razu liczę połowę kata
//	fdPhi2 = uDaneCM4.dane.fZyroKal1[0] * ndT[chGniazdo] / 2000000;
	fdPhi2 = 90 * DEG2RAD / 2;	//testowo obróć o taki kąt

	//analogicznie dla obrotu przez os Y [0, 1, 0] o kąt theta będący całką po ndT z prędkości Q
	//qy = cos (theta/2) + (1 / sqrt(x^2 + y^2 + z^2)*j) * sin (theta/2)
//	fdTheta2 = uDaneCM4.dane.fZyroKal1[0] * ndT[chGniazdo] / 2000000;
	fdTheta2 = 0;

	//analogicznie dla obrotu przez os Z [0, 0, 1] o kąt psi będący całką po ndT z prędkości R
	//qz = cos (psi/2) + (1 / sqrt(x^2 + y^2 + z^2)*k) * sin (psi/2)
//	fdPsi2 = uDaneCM4.dane.fZyroKal1[0] * ndT[chGniazdo] / 2000000;
	fdPsi2 = 0;

	//procedura obrotu wektora grawitacji wokół osi X
	//Wyznaczam kwaternion obrotu w formie algebraicznej korzystając z danych wejściowych wprowadzonych do formy trygonometrycznej
	//1. wyznaczyć część rzeczywistą kwaternionu s0 = cos(theta) gdzie kąt oborotu to 2*theta, więc s0 = cos(kąt_obrotu/2)
	q[0] = cosf(fdPhi2);

	//2. Obliczyć długość osi obrotu |v0| = sqrt(vx^2 + vy^2 + vz^2).
	//Dla osi X będzie wektor kierunkowy to [1, 0, 0], więc |v0x| = sqrt(1^2 + 0^2 + 0^2)
	fModul_v0x = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);


	//3. Obliczyć x0 = vx / |v0| * sin(theta)
	q[1] = v[1] / fModul_v0x * sinf(fdPhi2);

	//4. Obliczyć y0 = vy / |v0| * sin(theta)
	q[2] = v[2] / fModul_v0x * sinf(fdPhi2);

	//5. Obliczyć z0 = vz / |v0| * sin(theta)
	q[3] = v[3] / fModul_v0x * sinf(fdPhi2);

	//aby zoptymalizowac obliczenia rotacji przez 3 osie stusuję złożenie obrotów dla osi jako kwaterion w postaci macierzowej a następnie przemnażam przez siebie te macierze
	//[s1	x1	y1	z1]			[s2	x2	y2	z2]			[s3	x3	y3	z3]
	//[-x1	s1	-z1	y1]		*	[-x2 s2	-z2	y2]  *		[-x3 s3	-z3	y3]
	//[-y1	z1	s1	-x1]		[-y2 z2	s2	-x2]		[-y3 z3	s3	-x3]
	//[-z1	-y1	x1	s1]			[-z2 -y2 x2	s2]			[-z3 -y3 x3	s3]



	//6. Macierz obrotu należy pomnożyć przez obracany punkt zapisany jako wektor [Px, Py, Pz] w kolumnie
	//https://youtu.be/ZgOmCYfw6os?si=W9FL7V7r92M1e8Zf
	//[2*(s0^2 + x0^2) - 1		2*(x0*y0 - s0*z0)		2*(s0*y0 + x0*z0)]		[Px]
	//[2*(s0*z0 + x0*y0)		2*(s0^2 + y0^2) - 1		2*(y0*z0 - s0*x0)]	* 	[Py]
	//[2*(x0*z0 - s0*y0)		2*(s0*x0 + y0*z0)		2*(s0^2 + z0^2) - 1]	[Pz]

	//mnożemie macierzy przez wektor odpowiada mnożeniu kolumn macierzy przez składowe wektora

	qv[0] = 0;
	qv[1] = p[0] * (2*(q[0]*q[0] + q[1]*q[1]) - 1) 	+ p[1] * 2*(q[1]*q[2] - q[0]*q[3]) 			+ p[2] * 2*(q[0]*q[2] + q[1]*q[3]);
	qv[2] = p[0] * 2*(q[0]*q[3] + q[1]*q[2]) 		+ p[1] * (2*(q[0]*q[0] + q[2]*q[2]) - 1) 	+ p[2] * 2*(q[2]*q[3] - q[0]*q[1]);
	qv[3] = p[0] * 2*(q[1]*q[3] - q[0]*q[2]) 		+ p[1] * 2*(q[0]*q[1] + q[2]*q[3]) 			+ p[2] * (2*(q[0]*q[0] + q[3]*q[3]) - 1);




	// Wykonuję pierwsze mnożenie q * v
	MnozenieKwaternionow(q[0], q[1], q[2], q[3], v[0], v[1], v[2], v[3], qv);
	MnozenieKwaternionow2(q, v, qv);
	//qv[0] = q[0]*v[]

	//kwaternion sprzężony q* ma taką samą część rzeczywistą i ujemne części urojone
	qs[0] = q[0];
	for (uint8_t n=1; n<4; n++)
		qs[n] = q[n] * -1.0f;

	//wykonuję drugie mnożenie (qv) * q*
	MnozenieKwaternionow(qv[0], qv[1], qv[2], qv[3], qs[0], qs[1], qs[2], qs[3], vq);
	MnozenieKwaternionow2(qv, qs, vq);
}



////////////////////////////////////////////////////////////////////////////////
// Wzór na mnożenie kwaternionów (a1 + ib1 + jc1 + kd1) * (a2 + ib2 + jc2 + kd2) =
// = (a1a2 - b1b2 - c1c2 -d1d2)
// +i(a1b2 + b1a2 + c1d2 - d1c2)
// +j(a1c2 + c1a2 + d1b2 - b1d2)
// +k(a1d2 + d1a2 + b1c2 - c1b2)
// Parametry: (abcd)1 i (abcd)2 - elementy mnożonych kwaternionó
// *wynik - wskaźnik na kwaternion będący wynikiem mnożenia
// Zwraca: nic
// Czas trwania: 1,05us
////////////////////////////////////////////////////////////////////////////////
void MnozenieKwaternionow(float a1, float b1, float c1, float d1, float a2, float b2, float c2, float d2, float *wynik)
{
	*(wynik + 0) = a1*a2 - b1*b2 - c1*c2 - d1*d2;
	*(wynik + 1) = a1*b2 + b1*a2 + c1*d2 - d1*c2;
	*(wynik + 2) = a1*c2 + c1*a2 + d1*b2 - b1*d2;
	*(wynik + 3) = a1*c2 + c1*a2 + d1*b2 - b1*d2;
}



// Dla porównania bardziej czytelny wzór, łatwiejszy do implemntacji na wskaźnikach. Mnożenie q*p =
////////////////////////////////////////////////////////////////////////////////
// Wzór na mnożenie kwaternionów (q0 + iq1 + jq2 + kq3) * (p0 + ip1 + jp2 + kp3)
// (q0p0 - q1p1 - q2p2 + q3p3)
// (q1p0 + q0p1 - q3p2 + q2p3)
// (q2p0 + q3p1 + q0p2 - q1p3)
// (q3p0 - q2p1 + q1p2 + q0p3)
// Parametry: *q i *p - wskaźniki na mnożone kwaterniony
// *wynik - wskaźnik na kwaternion będący wynikiem mnożenia
// Zwraca: nic
// Czas trwania: 1,05us
////////////////////////////////////////////////////////////////////////////////
void MnozenieKwaternionow2(float *q, float *p, float *wynik)
{
	*(wynik + 0) = *(q+0) * *(p+0) - *(q+1) * *(p+1) - *(q+2) * *(p+2) - *(q+3) * *(p+3);
	*(wynik + 1) = *(q+1) * *(p+0) + *(q+0) * *(p+1) + *(q+3) * *(p+2) - *(q+2) * *(p+3);
	*(wynik + 2) = *(q+2) * *(p+0) + *(q+3) * *(p+1) + *(q+0) * *(p+2) - *(q+1) * *(p+3);
	*(wynik + 3) = *(q+3) * *(p+0) + *(q+2) * *(p+1) + *(q+1) * *(p+2) - *(q+0) * *(p+3);
}
