//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v2.0
// Moduł ubsługi regulatorów PID
//
// (c) Pit Lab 
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "pid.h"
#include "konfig_fram.h"




//definicje zmiennych
float fPgain[NUM_PIDS];   //wzmocnienie członu P
float fIgain[NUM_PIDS];   //wzmocnienie członu I
float fDgain[NUM_PIDS];   //wzmocnienie członu D
float fILimit[NUM_PIDS];    //ogranicznik wartości całki członu I
float fPidInVal[NUM_PIDS];  //wartość wejściowa
float fSetPoint[NUM_PIDS];  //wartość zadana
float fDevFilter[NUM_PIDS]; //filtr uśredniający z wartość do różniczkowania
float fDeFilTime[NUM_PIDS]; //sta�a czasowa filtra
unsigned char chDeFilTime[NUM_PIDS]; //sta�a czasowa filtra

//zmienne debugujące
float fPidOut[NUM_PIDS];  //wartość wyjściowa z całego regulatora
float fPidPOut[NUM_PIDS];  //wartość wyjściowa z członu P
float fPidIOut[NUM_PIDS];  //wartość wyjściowa z członu I
float fPidDOut[NUM_PIDS];  //wartość wyjściowa z członu D



//zmienne wewnętrzne
static float fIntegrator[NUM_PIDS];  //zmianna przechowująca całka z błędu
//static float fLastOutVal[NUM_PIDS];  //ostatnia wartość wyjściowa służy do okre�lenia pr�dko�ci narastania sygna�u
static float fLastErrVal[NUM_PIDS];  //ostatnia wartość wyjściowa służy do różniczkowania

//deklaracje zmiennych zewnętrznych
extern signed short sServoOutVal[];  //zmienna zawiera pozycj� serw wyj�ciowych +-150% z rozdzieczo�ci� 0,25%
extern signed short sServoInVal[];


////////////////////////////////////////////////////////////////////////////////
// Funkcja liczy odpowiedź regulatora PID
// Parametry: 
// [i] ndT - czas od ostatniego cyklu [us]
// [i] chN - indeks regulatora i zmiennych
// [i] chKatowy - określa sposób licznia błędu: liniowy czy katowy (+-180�)
// Zwraca: znormalizowaną odpowiedź regulatora +-100%
// Czas wykonania: 29us, sporadycznie 40us
////////////////////////////////////////////////////////////////////////////////
float CalcPID(unsigned int ndT, unsigned char chN, unsigned char chKatowy)
{
    float fOut, fErr;   //wartość wyjściowa i błąd sterowania (odchyłka)
    float fTemp, fdT;

    fdT = (float)ndT/1000000;    //czas obiegu petli w sekundach (optymalizacja kilkukrotnie wykorzystywanej zmiennej)

    //człon proporocjonalny
    fErr = fSetPoint[chN] - fPidInVal[chN];
    if (chKatowy)  //czy regulator pracuje na wartościach kątowych?
    {
        if (fErr > M_PI)
            fErr -= 2*M_PI;
        if (fErr < -M_PI)
            fErr += 2*M_PI;
    }
    fOut = fErr * fPgain[chN];

    fPidPOut[chN] = fOut;  //debugowanie: wartość wyjściowa z członu P

    //człon całkujący - liczy sumę błędu od początku do teraz
    if (fIgain[chN] > 0.001f)    //sprawdź warunek !=0 ze wzglądu na dzielenie przez fIgain[] oraz ogranicz zbyt szybkie całkowanie nastawione pomyłkowo jako 0 a będące bardzo małą liczbą
    {
        fIntegrator[chN] += fErr * fdT;   //całkowanie odchy�ki
        fTemp = fIntegrator[chN] / fIgain[chN];

        //ogranicznik wartości całki
        if (fTemp > fILimit[chN])
        {
            fIntegrator[chN] = fILimit[chN]*fIgain[chN];
            fTemp = fILimit[chN];
        }
        else
        if (fTemp < -fILimit[chN])
        {
            fIntegrator[chN] = -fILimit[chN]*fIgain[chN];
            fTemp = -fILimit[chN];
        }
    
        
        fPidIOut[chN] = fTemp;  //debugowanie: wartość wyjściowa z członu I
        fOut += fTemp;
    }
    else
        fPidIOut[chN] = 0.0f;

    //człon różniczkujący
    if ((chDeFilTime[chN] > 0) && (fDgain[chN] > 0.0001f))
    {
        //najpierw filtruj wartość błędu aby uzyskać gładką akcję różniczkującą
        fDevFilter[chN] = ((chDeFilTime[chN] - 1) * fDevFilter[chN] + fLastErrVal[chN]) / chDeFilTime[chN];

        //zamiast liczyć różnicę od ostatniej wartości, licz od wartosci przefiltrowanej
        //fTemp = (fErr - fLastErrVal[chN]) * fDgain[chN] / fdT;
        fTemp = (fErr - fDevFilter[chN]) * fDgain[chN] / fdT;
    }
    else
        fTemp = 0;

    fPidDOut[chN] = fTemp;  //wartość wyjściowa z członu D
    fOut += fTemp;
    fLastErrVal[chN] = fErr;
  
    //ograniczenie wartości wyjściowej
  /*  if (fOut > MAX_PID)
        fOut = MAX_PID;
    else
    if (fOut < -MAX_PID)
        fOut = -MAX_PID;*/

    fPidOut[chN] = fOut;    //wartość wyjściowa z całego regulatora
    return fOut;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja przeładowuje konfigurację regulatora PID
// Parametry: brak
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
unsigned char InitPID(void)
{
    unsigned char x, chAdrOffset, chErr = ERR_OK;

    for (x=0; x<NUM_PIDS; x++)
    {
        chAdrOffset = (x * PID_FRAM_CH_SIZE * FRAM_FLOAT_SIZE);	//offset danych kolejnego kana�u regulatora
        //odczytaj wartość wzmocnienienia członu P regulatora 0
        chErr |= FramDataReadFloatValid(FAU_PID_P0 + chAdrOffset, &fPgain[x], VMIN_PID_WZMP, VMAX_PID_WZMP, VDEF_PID_WZMP, ERR_NASTAWA_PID);

        //odczytaj wartość wzmocnienienia członu I regulatora 0
        chErr |= FramDataReadFloatValid(FAU_PID_I0 + chAdrOffset, &fIgain[x], VMIN_PID_WZMI, VMAX_PID_WZMI, VDEF_PID_WZMI, ERR_NASTAWA_PID);
    
        //odczytaj wartość wzmocnienienia członu D regulatora 0
        chErr |= FramDataReadFloatValid(FAU_PID_D0 + chAdrOffset, &fDgain[x], VMIN_PID_WZMD, VMAX_PID_WZMD, VDEF_PID_WZMD, ERR_NASTAWA_PID);
    
        //odczytaj granic� nasycenia członu całkującego
        chErr |= FramDataReadFloatValid(FAU_PID_ILIM0 + chAdrOffset, &fILimit[x], VMIN_PID_ILIM, VMAX_PID_ILIM, VDEF_PID_ILIM, ERR_NASTAWA_PID);
    
        //odczytaj stałą czasow� filtru członu różniczkowania
        chDeFilTime[x] = FramDataRead(FAU_D_FILTER + x);
      
        //zeruj integrator
        fIntegrator[x] = 0.0;   //zmianna przechowująca całkę z błędu
        fLastErrVal[x] = 0.0;   //ostatnia wartość wyjściowa służy do różniczkowania
    }

    return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja zeruje wartości całek członu całkującego
// Parametry: brak
// Zwraca: nic
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
void ResetPIDintegrator(void)
{
    unsigned char x;

    for (x=0; x<NUM_PIDS; x++)
        fIntegrator[x] = 0.0;   //zmianna przechowująca całka z błędu
}
