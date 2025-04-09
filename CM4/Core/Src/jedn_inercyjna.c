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
// Prędkości kątowe (układ prawoskrętny)
// prędkość P wokół osi X dodatnia gdy prawe skrzydło się opuszcza
// prędkość Q wokół osi Y dodatnia gdy dziób się podnosi
// prędkość R wokół osi Z odatnia gdy dziób obraca sie w prawo
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
#define Kp 2.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f		// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.5f		// half the sample period

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

	}
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

    //czas od poprzednich pomiarów w sekundach
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

    //normalzuj kwaternion z kątami
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


uint8_t JednostkaInercyjnaMadgwick(void)
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
