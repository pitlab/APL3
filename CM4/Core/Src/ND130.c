//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika ciśnienia różnicowego ND130 na magistrali SPI
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "ND130.h"
#include "main.h"
#include "spi.h"
#include "petla_glowna.h"
#include "wymiana_CM4.h"

uint8_t chBufND130[13];

extern SPI_HandleTypeDef hspi2;
extern volatile unia_wymianyCM4_t uDaneCM4;

// Układ ND130 pracujacy na magistrali SPI ma okres zegara 6us co odpowiada częstotliwości 166kHz

////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z PROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujND130(void)
{
	uint32_t nCzasStart;
	uint8_t chErr;

    nCzasStart = PobierzCzas();
    do
    {
    	chBufND130[0] = 0xF7;	//mode byte: 30 in H2O, BW=200Hz, Notch enabled
    	chBufND130[1] = 0x02;	//rate = 222Hz
    	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
    	chErr = HAL_SPI_TransmitReceive(&hspi2, chBufND130, chBufND130, 13, 5);
    	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
    	if (chErr)
    		return chErr;

    	//sprawdź czy układ przedstawił się jako "ND130"
       	if ((chBufND130[4] == 'N') && (chBufND130[5] == 'D') && (chBufND130[6] == '1') && (chBufND130[7] == '3') && (chBufND130[8] == '0'))
        	chErr = ERR_OK;

    	if (MinalCzas(nCzasStart) > 5000)   //czekaj maksymalnie 5000us
    		return ERR_TIMEOUT;
    }
    while (chErr);
    uDaneCM4.dane.nZainicjowano |= INIT_ND130;
    return chErr;
}


////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika do wywołania w wyższej warstwie
// Wykonuje w pętli 1 pomiar temepratury i 7 pomiarów ciśnienia
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaND130(void)
{
	uint8_t chErr;

	if ((uDaneCM4.dane.nZainicjowano & INIT_ND130) != INIT_ND130)	//jeżeli czujnik nie jest zainicjowany
	{
		chErr = InicjujND130();
	}
	else
	{
		chBufND130[0] = 0xF7;	//mode byte: 30in H2O, BW=200Hz, Notch enabled
		chBufND130[1] = 0x02;	//rate = 222Hz
		chBufND130[2] = chBufND130[3] = 0x00;
		HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
		HAL_SPI_TransmitReceive(&hspi2, chBufND130, chBufND130, 4, 5);
		HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1

		//walidacja poprawności danych
		if ((chBufND130[0] == 0) && (chBufND130[1] == 0) && (chBufND130[2] == 0) && (chBufND130[3] == 0))
			return ERR_ZLE_DANE;

		if ((chBufND130[0] == 0xFF) && (chBufND130[1] == 0xFF) && (chBufND130[2] == 0xFF) && (chBufND130[3] == 0xFF))
			return ERR_ZLE_DANE;

		uDaneCM4.dane.fCisnRozn[0] = (float)(((int16_t)chBufND130[0] <<8) + chBufND130[1]  * 30 * 249.082f) / (0.9f * 32768);		//wynik (In H2O) -> Pa
		uDaneCM4.dane.fTemper[5] = (float)chBufND130[2] + (float)chBufND130[3] / 2550;	//starszy bajt to stopnie, młodszy to ułamek będący częścią po przecinku
		//uDaneCM4.dane.fTemper[5] = PredkoscRurkiPrantla1(uDaneCM4.dane.fCisnRozn[0]);
		uDaneCM4.dane.fPredkosc[0] = PredkoscRurkiPrantla(uDaneCM4.dane.fCisnRozn[0], 101315.f);	//dla ciśnienia standardowego. Docelowo zamienić na cisnienie zmierzone
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza prędkość dla różnicy cisnienia w rurce Prandtla
// Na podstawie wzoru https://pl.wikipedia.org/wiki/Rurka_Prandtla
// Parametry:
//  fCisnRozn - ciśnienie różnicowe w Pa
//  fCisnStat = ciśnienie statyczne w Pa
// Zwraca: prędkość w m/s
////////////////////////////////////////////////////////////////////////////////
float PredkoscRurkiPrantla(float fCisnRozn, float fCisnStatP2)
{
	#define GESTOSC_POWIETRZA	1.20f		//https://pl.wikipedia.org/wiki/G%C4%99sto%C5%9B%C4%87
	#define	WYKLADNIK_ADIABATY	1.4f
	#define CZESC_PIERWSZA	((2.0f * WYKLADNIK_ADIABATY) / (WYKLADNIK_ADIABATY - 1.0f))
	#define WYKLADNIK_POTEGI ((WYKLADNIK_ADIABATY - 1.0f) / WYKLADNIK_ADIABATY)
	float fPredkosc;
	float fCisnCalkP1;
	float fZnak;

	if (fCisnRozn >= 0)
	{
		fCisnCalkP1 = fCisnRozn + fCisnStatP2;	//cisnienie całkowite
		fZnak = 1.0f;
	}
	else
	{
		fCisnCalkP1 = -1 * fCisnRozn + fCisnStatP2;	//cisnienie całkowite
		fZnak = -1.0f;
	}

	fPredkosc = sqrtf(CZESC_PIERWSZA * (fCisnCalkP1 / GESTOSC_POWIETRZA) * (1.0f - powf((fCisnStatP2 / fCisnCalkP1), WYKLADNIK_POTEGI)));

	//jednostka: kappa czyli wykładnik adiabaty jest bezwymiarowa, iloraz ciśnienia jest bezwymiarowy, wykłądnik potęgi też jest bezwymiarowy
	//pozostaje P1 / GESTOSC_POWIETRZA gdzie P1 jest w Paskalach czyli N/m^2 a gęstość jest w jednostce kg/m^3
	//jeżeli przyjmiemy że N = kg*m/s^2 to jednostka uprości się do m^2/s^2 a że jest pod pierwiastkiem to finalnie otrzymujemy [m/s]
	return fPredkosc * fZnak;
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza prędkość dla różnicy cisnienia w rurce Prandtla
// Na podstawie wzoru pierwiastek (2*deltaP / gęstość)
// Parametry:
//  fCisnRozn - ciśnienie różnicowe w Pa
// Zwraca: prędkość w m/s
////////////////////////////////////////////////////////////////////////////////
float PredkoscRurkiPrantla1(float fCisnRozn)
{
	float fPredkosc;

	if (fCisnRozn >= 0)
		fPredkosc = sqrtf(2 * fCisnRozn / GESTOSC_POWIETRZA);
	else
		fPredkosc = sqrtf(-2 * fCisnRozn / GESTOSC_POWIETRZA) * -1;
	return fPredkosc;
}
