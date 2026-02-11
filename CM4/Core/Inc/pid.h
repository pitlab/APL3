#include "pid_kanaly.h"
#include "sys_def_CM4.h"
#include "wymiana.h"
#include "konfig_fram.h"

//#define MAX_INT   (float)3.0   //maksymalna wartość całki członu I
//#define MAX_TIME  1   //podstawa czasu różniczkowania = MAX_TIME* 10ms
#define MAX_PID   		NORMA_SYGNALU  	//maksymalna wartość wyjściowa regulatora PID = odpowiada znormalizowanemu sygnałowi
#define MIN_WZM_CALK	0.001f		//minimalna wartość wzmocnienia członu całkowania
#define MIN_WZM_ROZN	0.0001f		//minimalna wartość wzmocnienia członu różniczkowania


//definicje funkcji
uint8_t InicjujPID(void);
float RegulatorPID(uint32_t ndT, uint8_t chKanal, stWymianyCM4_t *dane, stKonfPID_t *konfig);
void ResetujCalkePID(void);
uint8_t StabilizacjaPID(uint32_t ndT, stWymianyCM4_t *dane, stKonfPID_t *konfig);
void TestPID(void);
