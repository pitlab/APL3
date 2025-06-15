#include "pid_kanaly.h"
#include "sys_def_CM4.h"
#include "wymiana.h"
#include "konfig_fram.h"

//#define MAX_INT   (float)3.0   //maksymalna wartość całki członu I
//#define MAX_TIME  1   //podstawa czasu różniczkowania = MAX_TIME* 10ms
#define MAX_PID   (float)100.0  	//maksymalna wartość wyjściowa regulatora PID = 100%
#define MIN_WZM_CALK	0.001f		//minimalna wartość wzmocnienia członu całkowania
#define MIN_WZM_ROZN	0.0001f		//minimalna wartość wzmocnienia członu różniczkowania



//#define MASKA_FILTRA_D		0x3F
//#define MASKA_WYLACZONY		0x40
//#define MASKA_KATOWY		0x80


//definicje funkcji
uint8_t InicjujPID(stWymianyCM4_t *dane);
float RegulatorPID(uint32_t ndT, uint8_t chKanal, stWymianyCM4_t *dane);
void ResetujCalkePID(void);
uint8_t StabilizacjaPID(uint32_t ndT, stWymianyCM4_t *dane);
