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
#include "fram.h"


//definicje zmiennych
stPID_t stPID[NUM_PIDS];


//deklaracje zmiennych zewnętrznych
extern signed short sServoOutVal[];  //zmienna zawiera pozycj� serw wyj�ciowych +-150% z rozdzieczo�ci� 0,25%
extern signed short sServoInVal[];



////////////////////////////////////////////////////////////////////////////////
// Funkcja przeładowuje konfigurację regulatora PID
// Parametry: brak
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujPID(void)
{
	uint8_t chAdrOffset, chErr = ERR_OK;

    for (uint8_t n=0; n<NUM_PIDS; n++)
    {
        chAdrOffset = (n * PID_FRAM_CH_SIZE * FRAM_FLOAT_SIZE);	//offset danych kolejnego kanału regulatora
        //odczytaj wartość wzmocnienienia członu P regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_P0 + chAdrOffset, &stPID[n].fWzmP, VMIN_PID_WZMP, VMAX_PID_WZMP, VDEF_PID_WZMP, ERR_NASTAWA_PID);

        //odczytaj wartość wzmocnienienia członu I regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_I0 + chAdrOffset, &stPID[n].fWzmI, VMIN_PID_WZMI, VMAX_PID_WZMI, VDEF_PID_WZMI, ERR_NASTAWA_PID);

        //odczytaj wartość wzmocnienienia członu D regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_D0 + chAdrOffset, &stPID[n].fWzmD, VMIN_PID_WZMD, VMAX_PID_WZMD, VDEF_PID_WZMD, ERR_NASTAWA_PID);

        //odczytaj granicę nasycenia członu całkującego
        chErr |= CzytajFramZWalidacja(FAU_PID_ILIM0 + chAdrOffset, &stPID[n].fOgrCalki, VMIN_PID_ILIM, VMAX_PID_ILIM, VDEF_PID_ILIM, ERR_NASTAWA_PID);

        //odczytaj stałą czasową filtru członu różniczkowania
        stPID[n].chPodstFiltraD = CzytajFRAM(FAU_D_FILTER + n);

        //zeruj integrator
        stPID[n].fCalka = 0.0f;   	//zmianna przechowująca całkę z błędu
        stPID[n].fPoprzBlad = 0.0f;	//poprzednia wartość błędu
    }

    return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja liczy odpowiedź regulatora PID
// Parametry: 
// [i] ndT - czas od ostatniego cyklu [us]
// [i] chKanal - indeks regulatora i zmiennych
// [i] chKatowy - określa sposób licznia błędu: liniowy czy katowy (+-180�)
// Zwraca: znormalizowaną odpowiedź regulatora +-100%
// Czas wykonania: ?
////////////////////////////////////////////////////////////////////////////////
float RegulatorPID(uint32_t ndT, uint8_t chKanal, uint8_t chKatowy)
{
    float fWyjscieReg, fBladReg;   //wartość wyjściowa i błąd sterowania (odchyłka)
    float fTemp, fdT;

    fdT = (float)ndT/1000000;    //czas obiegu petli w sekundach (optymalizacja kilkukrotnie wykorzystywanej zmiennej)

    //człon proporocjonalny
    fBladReg = stPID[chKanal].fZadana - stPID[chKanal].fWejscie;
    if (chKatowy)  //czy regulator pracuje na wartościach kątowych?
    {
        if (fBladReg > M_PI)
        	fBladReg -= 2*M_PI;
        if (fBladReg < -M_PI)
        	fBladReg += 2*M_PI;
    }
    fWyjscieReg = fBladReg * stPID[chKanal].fWzmP;
    stPID[chKanal].fWyjscieP = fWyjscieReg;  //debugowanie: wartość wyjściowa z członu P

    //człon całkujący - liczy sumę błędu od początku do teraz
    if (stPID[chKanal].fWzmI > MIN_WZM_CALK)    //sprawdź warunek !=0 ze wzglądu na dzielenie przez fWzmI[] oraz ogranicz zbyt szybkie całkowanie nastawione pomyłkowo jako 0 a będące bardzo małą liczbą
    {
    	stPID[chKanal].fCalka += fBladReg * fdT;   //całkowanie odchyłki
        fTemp = stPID[chKanal].fCalka / stPID[chKanal].fWzmI;

        //ogranicznik wartości całki
        if (fTemp > stPID[chKanal].fOgrCalki)
        {
        	stPID[chKanal].fCalka = stPID[chKanal].fOgrCalki * stPID[chKanal].fWzmI;
            fTemp = stPID[chKanal].fOgrCalki;
        }
        else
        if (fTemp < -stPID[chKanal].fOgrCalki)
        {
        	stPID[chKanal].fCalka = -stPID[chKanal].fOgrCalki * stPID[chKanal].fWzmI;
            fTemp = -stPID[chKanal].fOgrCalki;
        }
        fWyjscieReg += fTemp;
    }
    else
    	 fTemp = 0.0f;
    stPID[chKanal].fWyjscieI = fTemp;  //debugowanie: wartość wyjściowa z członu I


    //człon różniczkujący
    if (stPID[chKanal].fWzmD > MIN_WZM_ROZN)
    {
        fTemp = (fBladReg - stPID[chKanal].fPoprzBlad) * stPID[chKanal].fWzmD / fdT;
        fWyjscieReg += fTemp;

        //filtruj IIR wartość błędu aby uzyskać gładką akcję różniczkującą
    	if (stPID[chKanal].chPodstFiltraD >= 2)
    		stPID[chKanal].fPoprzBlad = ((stPID[chKanal].chPodstFiltraD - 1) * stPID[chKanal].fPoprzBlad + fBladReg) / stPID[chKanal].chPodstFiltraD;
    	else
    		stPID[chKanal].fPoprzBlad = fBladReg;
    }
    else
        fTemp = 0.0f;
    stPID[chKanal].fWyjscieD = fTemp;  //wartość wyjściowa z członu D
  
    //ograniczenie wartości wyjściowej
    if (fWyjscieReg > stPID[chKanal].fMaxWyj)
    	fWyjscieReg = stPID[chKanal].fMaxWyj;
    else
    if (fWyjscieReg < stPID[chKanal].fMinWyj)
    	fWyjscieReg = stPID[chKanal].fMinWyj;

    stPID[chKanal].fWyjsciePID = fWyjscieReg;    //wartość wyjściowa z całego regulatora
    return fWyjscieReg;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja zeruje wartości całek członu całkującego
// Parametry: brak
// Zwraca: nic
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
void ResetujCalkePID(void)
{
    for (uint8_t n=0; n<NUM_PIDS; n++)
    	stPID[n].fCalka = 0.0f;   //zmianna przechowująca całka z błędu
}
