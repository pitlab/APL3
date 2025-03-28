/*
 * wymiana.h
 *
 *  Created on: Dec 5, 2024
 *      Author: PitLab
 */

#ifndef INC_WYMIANA_H_
#define INC_WYMIANA_H_
#include "stm32h7xx_hal.h"
#include "semafory.h"


#define ROWNAJ_DO32B(adres)				((adres + 3) & 0xFFFFFFFC)

#define ROZMIAR_BUF8_WYMIANY_CM4		ROWNAJ_DO32B(sizeof(stWymianyCM4_t))
#define ROZMIAR_BUF8_WYMIANY_CM7		ROWNAJ_DO32B(sizeof(stWymianyCM7_t))
#define ROZMIAR_BUF32_WYMIANY_CM4		ROZMIAR_BUF8_WYMIANY_CM4 / 4
#define ROZMIAR_BUF32_WYMIANY_CM7		ROZMIAR_BUF8_WYMIANY_CM7 / 4
#define ROZMIAR_BUF_NAPISU_WYMIANY		32


//definicje poleceń przekazywanych z CM7 do CM4
#define POL_NIC					0	//nic nie rób
#define POL_KALIBRUJ_ZYRO_ZIM	1	//uruchom kalibrację zera żyroskopów na zimno 10°C
#define POL_KALIBRUJ_ZYRO_POK	2	//uruchom kalibrację zera żyroskopów w temperaturze pokojowej 25°C
#define POL_KALIBRUJ_ZYRO_GOR	3	//uruchom kalibrację zera żyroskopów na gorąco 40°C
#define POL_KALIBRUJ_ZYRO_WZMR	4	//uruchom kalibrację wzmocnienia żyroskopów R
#define POL_KALIBRUJ_ZYRO_WZMQ	5	//uruchom kalibrację wzmocnienia żyroskopów Q
#define POL_KALIBRUJ_ZYRO_WZMP	6	//uruchom kalibrację wzmocnienia żyroskopów P
#define POL_ZERUJ_CALKE_ZYRO	7	//zeruje całkę żyroskopów przed kalibracją

#define POL_KALIBRUJ_MAGN1		8	//uruchom kalibrację magnetometru 1
#define POL_KALIBRUJ_MAGN2		9	//uruchom kalibrację magnetometru 2
#define POL_KALIBRUJ_MAGN3		10	//uruchom kalibrację magnetometru 3
#define POL_CZYSC_BLEDY			99	//polecenie kasuje błąd zwrócony pzez poprzednie polecenie

typedef struct _GNSS
{
	double dDlugoscGeo;
	double dSzerokoscGeo;
	float fPredkoscWzglZiemi;
	float fKurs;
	float fWysokoscMSL;
	float fHdop;
	float fVdop;
	uint8_t chLiczbaSatelit;
	uint8_t chFix;
	uint8_t chGodz, chMin, chSek;
	uint8_t chDzien, chMies, chRok;
} stGnss_t;


//definicja struktury wymiany danych wychodzących z rdzenia CM4
//typedef struct _stWymianyCM4
typedef struct
{
	float fAkcel1[3];		//[m/s^2]
	float fAkcel2[3];		//[m/s^2]
	float fZyroSur1[3];		//[rad/s]
	float fZyroSur2[3];		//żyroskop surowy - potrzebne do kalibracji i wyznaczania charakterystyki dryftu
	float fZyroKal1[3];		//[rad/s]
	float fZyroKal2[3];		//żyroskop skalibrowany
	int16_t sMagne1[3];
	int16_t sMagne2[3];
	int16_t sMagne3[3];
	float fKatIMU1[3];		//[rad]
	float fKatIMU2[3];
	float fKatIMUZyro1[3];	//[rad]
	float fKatIMUZyro2[3];
	float fKatIMUAkcel1[3];	//[rad]
	float fKatIMUAkcel2[3];
	float fCisnie[2];		//[Pa]
	float fWysoko[2];		//[m]
	float fCisnRozn[2];		//0=ND130, 1=MS2545
	float fPredkosc[2];		//[m/s]
	float fTemper[7];		//0=MS5611, 1=BMP851, 2=ICM42688 [K], 3=LSM6DSV [K], 4=IIS2MDC, 5=ND130, 6=MS2545
	uint16_t sSerwa[16];
	uint8_t chErrPetliGlownej;
	uint8_t chOdpowiedzNaPolecenie;
	uint32_t nZainicjowano;
	uint16_t sPostepProcesu;	//do wizualizacji trwania postępu procesów np. kalibracji
	stGnss_t stGnss1;
	char chNapis[ROZMIAR_BUF_NAPISU_WYMIANY];
} stWymianyCM4_t;


//definicja struktury wymiany danych wychodzących z rdzenia CM7
typedef struct
{
	uint8_t chWykonajPolecenie;
} stWymianyCM7_t;

//unie do konwersji struktur na słowa 32-bitowe
typedef union
{
	stWymianyCM4_t dane;
	uint32_t nSlowa[ROZMIAR_BUF32_WYMIANY_CM4];
} unia_wymianyCM4_t;

typedef union
{
	stWymianyCM7_t dane;
	uint32_t nSlowa[ROZMIAR_BUF32_WYMIANY_CM7];
} unia_wymianyCM7_t;


#endif /* INC_WYMIANA_H_ */
