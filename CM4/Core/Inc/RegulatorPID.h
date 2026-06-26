#include <KanalyPID.h>
#include "SysDefCM4.h"
#include "Wymiana.h"
#include "KonfigFram.h"

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
float StrojeniePID_KanałemRC(stStrojPID_t *stStrój, uint8_t chNrKan, stKonfPID_t *Konf, stWymianyCM4_t *WymianaCM4);
float ObliczWartośćParametruStrojenia(uint16_t sWartośćKanałuRC, stStrojPID_t *stStrój);
uint8_t ZapiszWartośćStrojeniaPID_KanałemRC(stStrojPID_t *stStrój, stKonfPID_t *Konf, stWymianyCM4_t *WymianaCM4);
void TestPID(void);
