//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obliczenia jednostki inercyjnej (IMU)

// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "jedn_inercyjna.h"
#include "wymiana.h"

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
//float fKatZyroskopu1[3], fKatZyroskopu2[3];				//całka z prędkosci kątowej żyroskopów
float fKatAkcel1[3], fKatAkcel2[3];						//kąty pochylenia i przechylenia policzone z akcelerometru
float fKatMagnetometru1, fKatMagnetometru2, fKatMagnetometru3;	//kąt odchylenia z magnetometru




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
		uDaneCM4.dane.fKatIMUZyro1[n] += uDaneCM4.dane.fZyroKal1[n] * ndT[chGniazdo] / 1000000;		//[°/s] * [us / 1000000] = [°]
		uDaneCM4.dane.fKatIMUZyro2[n] += uDaneCM4.dane.fZyroKal2[n] * ndT[chGniazdo] / 1000000;

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

	//kąt przechylenia z akcelerometru: tan(Z/Y)
	uDaneCM4.dane.fKatIMUAkcel1[0] = atan2f(uDaneCM4.dane.fAkcel1[2], uDaneCM4.dane.fAkcel1[1]) - 90 * DEG2RAD;
	uDaneCM4.dane.fKatIMUAkcel2[0] = atan2f(uDaneCM4.dane.fAkcel2[2], uDaneCM4.dane.fAkcel2[1]) - 90 * DEG2RAD;

	//kąt pochylenia z akcelerometru: tan(Z/X)
	uDaneCM4.dane.fKatIMUAkcel1[1] = atan2f(uDaneCM4.dane.fAkcel1[2], uDaneCM4.dane.fAkcel1[0]) - 90 * DEG2RAD;
	uDaneCM4.dane.fKatIMUAkcel2[1] = atan2f(uDaneCM4.dane.fAkcel2[2], uDaneCM4.dane.fAkcel2[0]) - 90 * DEG2RAD;

	//oblicz kąt odchylenia w radianach z danych magnetometru: tan(y/x)
	uDaneCM4.dane.fKatIMUAkcel1[2] = atan2f((float)uDaneCM4.dane.sMagne3[1], (float)uDaneCM4.dane.sMagne3[0]);
}





