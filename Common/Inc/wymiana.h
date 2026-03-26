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
#include "pid_kanaly.h"
#include "sys_def_wspolny.h"

#define ROWNAJ_DO32B(adres)				((adres + 3) & 0xFFFFFFFC)

#define ROZMIAR_BUF8_WYMIANY_CM4		ROWNAJ_DO32B(sizeof(stWymianyCM4_t))
#define ROZMIAR_BUF8_WYMIANY_CM7		ROWNAJ_DO32B(sizeof(stWymianyCM7_t))
#define ROZMIAR_BUF32_WYMIANY_CM4		ROZMIAR_BUF8_WYMIANY_CM4 / 4
#define ROZMIAR_BUF32_WYMIANY_CM7		ROZMIAR_BUF8_WYMIANY_CM7 / 4
#define ROZMIAR_BUF_NAPISU_WYMIANY		32
//#define ROZMIAR_ROZNE					8
#define ROZMIAR_ROZNE_FLOAT				8
#define ROZMIAR_ROZNE_SHORT				16
#define ROZMIAR_ROZNE_CHAR				32

//definicje poleceń przekazywanych z CM4 do CM7
#define POL4_NIC					0	//nic nie rób
#define POL4_WLACZ_OD1				1	//włącz wyjście otwarty dren 1
#define POL4_WLACZ_OD2				2	//włącz wyjście otwarty dren 2
#define POL4_WYLACZ_OD1				3	//wyłącz wyjście otwarty dren 1
#define POL4_WYLACZ_OD2				4	//wyłącz wyjście otwarty dren 2
#define POL4_MOW_UZBROJONE			5	//rozpocznil wymowę komunikatu silniki uzbrojone
#define POL4_MOW_ROZBROJONE			6	//rozpocznil wymowę komunikatu silniki rozbrojone
#define POL4_MOW_WYSOKOSC			7
#define POL4_MOW_NAPIECIE			8
#define POL4_MOW_TEMPERAT			9
#define POL4_MOW_PREDKOSC			10
#define POL4_MOW_KIERUNEK			11



//definicje poleceń przekazywanych z CM7 do CM4
#define POL7_NIC					0	//nic nie rób
#define POL7_KALIBRUJ_ZYRO_ZIM		1	//uruchom kalibrację zera żyroskopów na zimno 10°C
#define POL7_KALIBRUJ_ZYRO_POK		2	//uruchom kalibrację zera żyroskopów w temperaturze pokojowej 25°C
#define POL7_KALIBRUJ_ZYRO_GOR		3	//uruchom kalibrację zera żyroskopów na gorąco 40°C
#define POL7_ZERUJ_CALKE_ZYRO		4	//zeruje całkę żyroskopów przed kalibracją
#define POL7_CALKUJ_PRED_KAT		5	//całkuje prędkość kątową z żyroskopów
#define POL7_KALIBRUJ_ZYRO_WZMR		6	//uruchom kalibrację wzmocnienia żyroskopów R
#define POL7_KALIBRUJ_ZYRO_WZMQ		7	//uruchom kalibrację wzmocnienia żyroskopów Q
#define POL7_KALIBRUJ_ZYRO_WZMP		8	//uruchom kalibrację wzmocnienia żyroskopów P
#define POL7_CZYTAJ_WZM_ZYROP		9	//odczytaj wzmocnienia żyroskopów P
#define POL7_CZYTAJ_WZM_ZYROQ		10	//odczytaj wzmocnienia żyroskopów Q
#define POL7_CZYTAJ_WZM_ZYROR		11	//odczytaj wzmocnienia żyroskopów R

#define POL7_KAL_ZERO_MAGN1			12	//uruchom kalibrację zera magnetometru 1
#define POL7_KAL_ZERO_MAGN2			13	//uruchom kalibrację zera magnetometru 2
#define POL7_KAL_ZERO_MAGN3			14	//uruchom kalibrację zera magnetometru 3
#define POL7_ZAPISZ_KONF_MAGN1		15	//zapisz konfigurację magnetometru 1
#define POL7_ZAPISZ_KONF_MAGN2		16	//zapisz konfigurację magnetometru 2
#define POL7_ZAPISZ_KONF_MAGN3		17	//zapisz konfigurację magnetometru 3
#define POL7_POBIERZ_KONF_MAGN1		18	//pobiera współczynniki kalibracyjne magnetometru 1
#define POL7_POBIERZ_KONF_MAGN2		19	//pobiera współczynniki kalibracyjne magnetometru 2
#define POL7_POBIERZ_KONF_MAGN3		20	//pobiera współczynniki kalibracyjne magnetometru 3
#define POL7_ZERUJ_EKSTREMA			21	//zeruje wartosci zebranych ekstremów magnetometru
#define POL7_INICJUJ_USREDN			22	//inicjuje prametry przed startem
#define POL7_ZERUJ_LICZNIK			23	//zeruje licznik uśredniana przed kolejnym cyklem
#define POL7_USREDNIJ_CISN1			24	//uśredniania ciśnienia 1 czujników ciśnienia bezwzględnego
#define POL7_USREDNIJ_CISN2			25	//uśredniania ciśnienia 2 czujników ciśnienia bezwzględnego
#define POL7_CZYTAJ_FRAM_U8			26	//odczytaj i wyślij zawartość FRAM spod podanego adresu
#define POL7_ZAPISZ_FRAM_U8			27	//zapisz przekazane dane do FRAM pod podany adres
#define POL7_CZYTAJ_FRAM_FLOAT		28	//odczytaj i wyślij zawartość FRAM spod podanego adresu
#define POL7_ZAPISZ_FRAM_FLOAT		29	//zapisz przekazane dane do FRAM pod podany adres
#define POL7_KASUJ_DRYFT_ZYRO		30	//kasuje druft katów z żyroskopu sprowadzajac je do wartości z akcelerometru
#define POL7_REKONFIG_SERWA_RC		31	//wykonuje ponowną konfigurację wejść i wyjść RC po zmianie konfiguracji we FRAM
#define POL7_ZAPISZ_KONFIG_PID		32	//zapisuje konfogurację PID do FRAM oraz do zmiennych roboczych
#define POL7_ZBIERAJ_EKSTREMA_RC	33	//rozpoczyna zbieranie ekstremalnych wartości kanałów obu odbiorników RC
#define POL7_ZAPISZ_EKSTREMA_RC		34	//kończy zbieranie ekstremalnych wartości kanałów obu odbiorników RC i zapisuje wyniki do FRAM
#define POL7_ZAPISZ_PWM_NAPEDU		35	//zapsuje nastawy wysterowania napędu dla wartości jałowej, minimalnej, zawisu i maksymalnej
#define POL7_ZAPISZ_TRYB_REG		36	//zapisuje tryb pracy 4 podstawowych regulatorów
#define POL7_RESETUJ_CM4			37	//resetuj rdzeń CM4, zwykle po zmianie konfiguracji
#define POL7_NORMALIZUJ_WEJ_RC		38	//normalizuj wskazane wejście RC
#define POL7_PRZELADUJ_WSKAZN_LED	39	//odczytaj konfigurację wskaźników LED z pamieci FRAM i załaduj do zmiennych aby wprowadzona zmiana stała się widoczna

#define POL7_CZYSC_BLEDY			99	//polecenie kasuje błąd zwrócony przez poprzednie polecenie

#define ROZMIAR_BUFORA_IMU	8		//rozmiar bufora kołowego przechowującego ostarnie dane z szybkiego IMU
#define MASKA_BUFORA_IMU	0x03	//maska do zawijania bufora kołowego

//definicje pól zmiennej chNowyPomiar
#define NP_MAG1		0x01
#define NP_MAG2		0x02
#define NP_MAG3		0x04
#define NP_EXT_IAS	0x08


//definicje kątow
#define PRZE	0
#define POCH	1
#define ODCH	2

typedef struct _GNSS
{
	double dDlugoscGeo;
	double dSzerokoscGeo;
	float fPredkoscWzglZiemi;
	float fKurs;
	float fPredkoscN;
	float fPredkoscE;
	float fWysokoscMSL;
	float fHdop;
	float fVdop;
	uint8_t chLiczbaSatelit;
	uint8_t chFix;
	uint8_t chGodz, chMin, chSek;
	uint8_t chDzien, chMies, chRok;
} stGnss_t;

//definicje struktury zawierajacej syntetyczne dane bezzałogowca (niezależne od konkretnych czujników)
typedef struct _BSP
{
	float fAkcel[3];		//[m/s^2]
	float fZyro[3];			//[rad/s]
	float fMagne[3];		//[uT]
	float fKatIMU[3];
	float fIAS;
	double dDlugoscGeo;
	double dSzerokoscGeo;
	float fWysokoscMSL;
	float fWysokoscAGL;
	float fPredkoscN;
	float fPredkoscE;
	float fPredkoscD;
	float fKursGeo;
} stBSP_t;

//definicje struktury danych IMU potrzebnych do szybkich obliczeń typu filtry czy FFT
typedef struct
{
	float fAkcel[ROZMIAR_BUFORA_IMU][3];		//[m/s^2]
	float fZyro[ROZMIAR_BUFORA_IMU][3];			//[rad/s]
	uint8_t chIndeksProbki;
} stSzybkieIMU;

typedef union
{
	float f32[ROZMIAR_ROZNE_FLOAT];
	uint32_t U32[ROZMIAR_ROZNE_FLOAT];
	uint16_t U16[ROZMIAR_ROZNE_SHORT];
	uint8_t U8[ROZMIAR_ROZNE_CHAR];
}  uRozne_t;

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
	float fKatZyro1[3];		//[rad]
	float fKatZyro2[3];
	float fKatAkcel1[3];	//[rad]
	float fKatAkcel2[3];
	float fKatKwater[3];
	float fCisnieBzw[2];	//Ciśnienie bezwzgleędne [Pa]
	float fWysokoMSL[2];	//Wysokość nad poziomem morza [m]
	float fWariometr[2];	//prędkość pionowa [m/s]
	float fCisnRozn[2];		//0=ND130, 1=MS2545
	float fPredkosc[2];		//[m/s]
	float fTemper[6];		//0=MS5611, 1=BMP851, 2=ICM42688 [K], 3=LSM6DSV [K], 4=ND130, 5=MS2545
	float fKwaAkc[4];		//kwaternion wektora przyspieszenia
	float fKwaMag[4];		//kwaternion wektora magnetycznego
	float fNapiecieBat[2];	//napięcie baterii podłączonych do obu wejść zasilania
	float fPradBat[2];		//prąd pobierany z obu baterii
	float fEnergiaPobr[2];	//energia pobrana z obu baterii
	uRozne_t uRozne;		//unia różnych typów danych ogólnego zastosowania
	uint8_t chRozmiar;		//rozmiar danych przekazywanych w polu fRozne
	uint16_t sAdres;		//adres danych przekazywanych w polu fRozne
	stGnss_t stGnss1;		//struktura danych GNSS1
	stWyjPID_t stWyjPID[LICZBA_PID];	//tablica struktur danych wyjściowych regulatorów PID
	int16_t sSilnik[KANALY_MIKSERA];	//wartości wysterowania silników wychodzące z miksera
	int16_t sWyjscieRC[KANALY_WYJSC_RC];	//finalne wartość wysterowania serw lub silników po uwzględnieniu konfiguracji wyjść
	int16_t sKanalRC[KANALY_ODB_RC];
	uint8_t chTrybLotu;		//tryb lotu jako zestaw bitów określających funkcjonalności realizowane w danym czasie
	uint8_t chNowyPomiar;	//zestaw flag informujacychpo pojawieniu się nowego pomiaru z wolno aktualizowanych czujników po I2C
	uint8_t chErrPetliGlownej;
	uint8_t chOdpowiedzNaPolecenie;	//potwierdzenie wykonania polecenia
	uint32_t nZainicjowano;		//zestaw flag inicjalizacji sprzętu
	uint32_t nBrakCzujnika;		//zestaw flag obecnosci czujników
	uint16_t sPostepProcesu;	//do wizualizacji trwania postępu procesów np. kalibracji
	uint8_t chWykonajPolecenie;	//numer polecenia do wykonania przez CM7
	uint8_t chPotwierdzenieWykonania;	//potwierdza wykonanie polecenia przysłanego przez CM7
	uint32_t ndT;
	stBSP_t stBSP;				//struktura zawierajaca syntetyczne dane bezzałogowca (niezależne od konkretnych czujników)
	stSzybkieIMU stSzybkieIMU;	//struktura zawierajaca bufor kołowy szybkich  i indeks danych z IMU aby na styku procesorów nie dochodziło do gubienia i powtarzania danych
} stWymianyCM4_t;


//definicja struktury wymiany danych wychodzących z rdzenia CM7
typedef struct
{
	uint8_t chOdbiornikRC;			//wybór źródła danych odbiornika RC dla stWymianyCM4.sKanalRC[]: RC1, RC2, oba zdywersyfikowane
	uint8_t chWykonajPolecenie;		//numer polecenia do wykonania przez CM4
	uint8_t chPotwierdzenieWykonania;	//potwierdza wykonanie polecenia przysłanego przez CM4
	uRozne_t uRozne;				//unia różnych typów danych ogólnego zastosowania
	uint8_t chRozmiar;				//rozmiar danych przekazywanych w polu fRozne
	uint16_t sAdres;				//adres danych przekazywanych w polu fRozne

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
