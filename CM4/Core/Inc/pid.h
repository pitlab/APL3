#include "pid_kanaly.h"
#include "sys_def_CM4.h"

//#define MAX_INT   (float)3.0   //maksymalna wartość całki członu I
//#define MAX_TIME  1   //podstawa czasu różniczkowania = MAX_TIME* 10ms
#define MAX_PID   (float)100.0  	//maksymalna wartość wyjściowa regulatora PID = 100%
#define MIN_WZM_CALK	0.001f		//minimalna wartość wzmocnienia członu całkowania
#define MIN_WZM_ROZN	0.0001f		//minimalna wartość wzmocnienia członu różniczkowania

typedef struct
{
	//wartości wejsciowe
	float fWejscie;  		//wartość wejściowa
	float fZadana;  		//wartość zadana

	//nastawy regulatorów
	float fWzmP;   			//wzmocnienie członu P
	float fWzmI;   			//wzmocnienie członu I
	float fWzmD;   			//wzmocnienie członu D
	float fOgrCalki; 		//ogranicznik wartości całki członu I
	uint8_t chPodstFiltraD; //podstawa różniczkującego filtra błędu o nieskończonej odpowiedzi fimpulsowej IIR
	float fMaxWyj;			//maksymalna wartość wyjściowa regulatora
	float fMinWyj;			//minimalna wartość wyjściowa regulatora

	//zmienne robocze członów dynamicznych
	float fCalka;  			//zmianna przechowująca całkę z błędu
	float fPoprzBlad; 		//poprzednia, przefiltrowana wartość błędu do liczenia akcji różniczkującej

	//zmienne wyjściowe
	float fWyjsciePID; 		//wartość wyjściowa z całego regulatora
	float fWyjscieP;  		//wartość wyjściowa z członu P
	float fWyjscieI;  		//wartość wyjściowa z członu I
	float fWyjscieD;  		//wartość wyjściowa z członu D
} stPID_t;


//definicje funkcji
uint8_t InicjujPID(void);
float RegulatorPID(uint32_t ndT, uint8_t chKanal, uint8_t chKatowy);
void ResetujCalkePID(void);
