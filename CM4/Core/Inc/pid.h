#include "pid_kanaly.h"
#include "sys_def_CM4.h"

//#define MAX_INT   (float)3.0   //maksymalna wartość całki członu I
//#define MAX_TIME  1   //podstawa czasu różniczkowania = MAX_TIME* 10ms
#define MAX_PID   (float)100.0   //maksymalna wartość wyjściowa regulatora PID = 100%




//definicje funkcji
float CalcPID(unsigned int ndT, unsigned char chN, unsigned char chKatowy);
unsigned char InitPID(void);
void ResetPIDintegrator(void);

//deklaracje funkcji zewnętrznych
extern void FramDataWriteFloat(unsigned short sAddress, float fData);
extern unsigned char FramDataReadFloatValid(unsigned short sAddress, float *fValue, float fValMin, float fValMax, float fValDef, unsigned char chErrCode);
extern unsigned char FramDataRead(unsigned short sAddress);
