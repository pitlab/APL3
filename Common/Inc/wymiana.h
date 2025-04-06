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
#define POL_ZERUJ_CALKE_ZYRO	4	//zeruje całkę żyroskopów przed kalibracją
#define POL_CALKUJ_PRED_KAT		5	//całkuje prędkość kątową z żyroskopów
#define POL_KALIBRUJ_ZYRO_WZMR	6	//uruchom kalibrację wzmocnienia żyroskopów R
#define POL_KALIBRUJ_ZYRO_WZMQ	7	//uruchom kalibrację wzmocnienia żyroskopów Q
#define POL_KALIBRUJ_ZYRO_WZMP	8	//uruchom kalibrację wzmocnienia żyroskopów P
#define POL_CZYTAJ_WZM_ZYROP	9	//odczytaj wzmocnienia żyroskopów P
#define POL_CZYTAJ_WZM_ZYROQ	10	//odczytaj wzmocnienia żyroskopów Q
#define POL_CZYTAJ_WZM_ZYROR	11	//odczytaj wzmocnienia żyroskopów R

#define POL_KAL_ZERO_MAGN1		12	//uruchom kalibrację zera magnetometru 1
#define POL_KAL_ZERO_MAGN2		13	//uruchom kalibrację zera magnetometru 2
#define POL_KAL_ZERO_MAGN3		14	//uruchom kalibrację zera magnetometru 3
#define POL_ZAPISZ_KONF_MAGN1	15	//zapisz konfigurację magnetometru 1
#define POL_ZAPISZ_KONF_MAGN2	16	//zapisz konfigurację magnetometru 2
#define POL_ZAPISZ_KONF_MAGN3	17	//zapisz konfigurację magnetometru 3
#define POL_SPRAWDZ_MAGN1		18	//sprawdź kalibrację magnetometru 1
#define POL_SPRAWDZ_MAGN2		19	//sprawdź kalibrację magnetometru 2
#define POL_SPRAWDZ_MAGN3		20	//sprawdź kalibrację magnetometru 3
#define POL_ZERUJ_EKSTREMA		21	//zeruje wartosci zebranych ekstremów magnetometru
#define POL_INICJUJ_USREDN		22	//inicjuje licznik uśredniania
#define POL_USREDNIJ_CISN1		23	//uśredniania ciśnienia 1 czujników ciśnienia bezwzględnego
#define POL_USREDNIJ_CISN2		24	//uśredniania ciśnienia 2 czujników ciśnienia bezwzględnego


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
	float fMagne1[3];
	float fMagne2[3];
	float fMagne3[3];
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
	float fRozne[6];		//różne parametry w zależności od bieżącego kontekstu
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
