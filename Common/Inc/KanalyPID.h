#ifndef SRC_PID_KANALY_H_
#define SRC_PID_KANALY_H_

#include "stm32h7xx_hal.h"

//#define NUM_AXIS  6 //liczba regulowanych osi: pochylenie, przechylenie, odchylenie, wysokość, prędkość + rezerwa
//#define FRAM_FLOAT_SIZE     4   //rozmiar liczby float


//Rodzaj regulatora
#define REG_KAT     1   //regulator kątowy
#define REG_LIN     0   //regulator liniowy


//definicje nazw regulatorów
#define PID_KĄTA_PRZE 	0   //regulator sterowania przechyleniem (lotkami w samolocie)
#define PID_PRED_PRZE 	1   //regulator sterowania prędkością kątową przechylenia (żyroskop P)
#define PID_KĄTA_POCH 	2   //regulator sterowania pochyleniem (sterem wysokości)
#define PID_PRED_POCH 	3   //regulator sterowania prędkością kątową pochylenia (żyroskop Q)
#define PID_KĄTA_ODCH 	4  	//regulator sterowania odchyleniem (sterem kierunku)
#define PID_PRED_ODCH	5   //regulator sterowania prędkością kątową odchylenia (żyroskop R)
#define PID_WYSOKOSCI 	6   //regulator sterowania wysokością
#define PID_PRED_ZWYS	7   //regulator sterowani prędkością zmiany wysokości (wario)
#define PID_NAWIG_PÓŁN 	8   //regulator sterowania nawigacją w kierunku północnym
#define PID_PRED_PÓŁN	9  	//regulator sterowania prędkością w kierunku północnym
#define PID_NAWIG_WSCH 	10  //regulator sterowania nawigacją w kierunku wschodnim
#define PID_PRED_WSCH	11 	//regulator sterowania prędkością w kierunku wschodnim

#define LICZBA_PID  12 //liczba regulatorów
#define LICZBA_KAN_RC_DO_STROJENIA_PID	2	//tyle kanałów RC jest używanych do strojenia wybranych parametrów PID

//definicje trybów pracy regulatora
#define REG_OFF           0   //wyłączony regulator
#define REG_MAN           1   //regulacja ręczna prosto z drążka
#define REG_ACRO          2   //regulacja pierwszej pochodnej parametru (prędkość katowa, prędkość wznoszenia)
#define REG_STAB          3   //regulacja parametru właściwego (kąt , wysokość)
#define REG_GPS_SPEED     4   //regulacja prędkości liniowej w osiach XYZ
#define REG_GPS_POS       5   //regulacja prędkości liniowej w osiach XYZ ze stabilizacją położenia

#define NUM_REG_MOD       6   //liczba trybów regulatora


//definicje bitów konfiguracji PID
#define PID_KATOWY				0x01



typedef struct	//struktura konfiguracji regulatora PID
{
	float fWzmP;   				//wzmocnienie członu P
	float fWzmI;   				//wzmocnienie członu I
	float fWzmD;   				//wzmocnienie członu D
	float fWzmWyprz;			//wzmocnienia wyprzedzenia
	float fOgrCalki; 			//ogranicznik wartości całki członu I
	float fMaxWyj;				//maksymalna wartość wyjściowa regulatora
	float fMinWyj;				//minimalna wartość wyjściowa regulatora
	float fPrzesunWartZadanej;	//wartość dodawana do wartości zadanej (umożliwia lot pod niezerowym kątem)
	float fSkalaWartZadanej;	//skalowanie wartości zadanej
	uint8_t cPodstFiltraWej;	//podstawa filtra IIR wartości wejściowej
	uint8_t cPodstFiltraD; 		//podstawa filtra IIR błędu
	uint8_t cPodstFiltraWZad; 	//podstawa filtra IIR wartości zadanej
	uint8_t cFlagi;				//0x80 - regulator kątowy, 0x40 - regulator wyłączony
} stKonfPID_t;

typedef struct	//struktura danych roboczych regulatora PID
{
	//wartości wejsciowe
	float fWejscie;  		//wartość mierzona regulowanego parametru
	float fZadana;  		//wartość zadana
	float fWyprzedzenie;	//wartość wyprzedzajacą (feedforward)

	//zmienne robocze członów dynamicznych
	float fCalka;  			//zmianna przechowująca całkę z błędu
	float fFiltrWeD;	 	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
	float fFiltrWartZad;	//przefiltrowana wartość zadana do liczenia wartosci wyprzedzającej

	//zmienne wyjściowe
	float fWyjsciePID; 		//wartość wyjściowa z całego regulatora
	float fWyjscieP;  		//wartość wyjściowa z członu P
	float fWyjscieI;  		//wartość wyjściowa z członu I
	float fWyjscieD;  		//wartość wyjściowa z członu D
	float fWyjscieWyprz;	//wartość wyprzedzająca
} stPID_t;


typedef struct	//struktura przechowujaca konfigurację strojenia parametru PID kanałem RC
{
	uint8_t cNrKanałuRC;	//numer kanału RC używany do strojenia parametru
	uint8_t cNrParametru;	//numer strojonego parametru
	float fWartośćMin;	//minimalna wartość parametru dla minimalnej wartości kanału
	float fWartośćMax;	//maksymalna wartość parametru dla maksymalnej wartości kanału
} stStrojPID_t;


#endif
