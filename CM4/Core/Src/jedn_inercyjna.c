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
extern uint32_t ndT[4];						//czas jaki upłynął od poprzeniego obiegu pętli dla 4 modułów wewnetrznych
extern volatile unia_wymianyCM4_t uDaneCM4;
extern volatile unia_wymianyCM7_t uDaneCM7;
//float fKatZyroskopu1[3], fKatZyroskopu2[3];				//całka z prędkosci kątowej żyroskopów
float fKatAkcel1[3], fKatAkcel2[3];						//kąty pochylenia i przechylenia policzone z akcelerometru
float fKatMagnetometru1, fKatMagnetometru2, fKatMagnetometru3;	//kąt odchylenia z magnetometru
extern float fOffsetZyro1[3], fOffsetZyro2[3];
magn_t stMagn;
float fOffsetMagn1[3], fGainMagn1[3];
float fOffsetMagn2[3], fGainMagn2[3];
float fOffsetMagn3[3], fGainMagn3[3];

////////////////////////////////////////////////////////////////////////////////
// inicjalizacja zmiennych potrzebanych do obliczeń kątów
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjujJednostkeInercyjna(void)
{
	for (uint16_t n=0; n<3; n++)
	{
		uDaneCM4.dane.fKatIMUZyro1[n] = 0;
		uDaneCM4.dane.fKatIMUZyro2[n] = 0;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia jednostki inercyjnej
// Parametry: chGniazdo - numer gniazda w którym jest moduł IMU
// Zwraca: kod błędu
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



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia kalibracji magnetometru
// Parametry: *sMag - wskaźnik na dane z 3 osi magnetometru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KalibracjaZeraMagnetometru(int16_t *sMag)
{
	for (uint16_t n=0; n<3; n++)
	{
		if (*(sMag+n) < stMagn.sMin[n])
			stMagn.sMin[n] = *(sMag+n);

		if (*(sMag+n) > stMagn.sMax[n])
			stMagn.sMax[n] = *(sMag+n);

		uDaneCM4.dane.fRozne[2*n+0] = (float)stMagn.sMin[n];
		uDaneCM4.dane.fRozne[2*n+1] = (float)stMagn.sMax[n];
	}
}



////////////////////////////////////////////////////////////////////////////////
// Liczy i zapisuje offset zera magnetometru
// Parametry: chMagn - indeks układu magnetometru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ZapiszOffsetMagnetometru(uint8_t chMagn)
{
	for (uint16_t n=0; n<3; n++)
	{
		switch (chMagn)
		{
		case 0:
			fOffsetMagn1[n] = stMagn.sMax[n] + stMagn.sMin[n];
			FramDataWriteFloat(FAH_MAGN1_OFSTX + 4*n, fOffsetMagn1[n]);
			break;

		case 1:
			fOffsetMagn2[n] = stMagn.sMax[n] + stMagn.sMin[n];
			FramDataWriteFloat(FAH_MAGN2_OFSTX + 4*n, fOffsetMagn2[n]);
			break;

		case 2:
			fOffsetMagn3[n] = stMagn.sMax[n] + stMagn.sMin[n];
			FramDataWriteFloat(FAH_MAGN3_OFSTX + 4*n, fOffsetMagn3[n]);
			break;
		}
	}
}


