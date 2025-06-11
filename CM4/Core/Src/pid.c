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

//Przyjmuję że regulator będzie mógł być szeregowy lub równoległy z preferencja szeregowego
//Człon różniczkujący jest zasilany ujemnym sygnałem wejsciowym a nie błędem aby uniknąć zmiany odpowiedzi od zmiany wartosci zadanej.
// w klasycznym członie: błąd = wartość zadana - wejscie
// w układzie zmodyfikowanym u usuwamy wartość zadaną, wiec: błąd = - wejscie
//Człon różniczkujący bazuje na różnicy między wartością bieżącą i poprzednią sygnału wejsciowego. Aby zmniejszyć szum regulacji, wartość poprzednia
// jest przefiltrowaną wartością n poprzednich pomiarów gdzie n jest regulowaną nastawą filra D

//definicje zmiennych
//stPID_t stPID[LICZBA_PID];	//zmienna przechowująca dane dotyczące regulatora PID

//deklaracje zmiennych zewnętrznych
extern signed short sServoOutVal[];  //zmienna zawiera pozycję serw wyjściowych +-150% z rozdzieczością 0,25%
extern signed short sServoInVal[];
extern unia_wymianyCM4_t uDaneCM4;


////////////////////////////////////////////////////////////////////////////////
// Funkcja przeładowuje konfigurację regulatora PID
// Parametry: brak
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujPID(void)
{
	uint8_t chAdrOffset, chErr = ERR_OK;
	uint8_t chTemp;

    for (uint8_t n=0; n<LICZBA_PID; n++)
    {
        chAdrOffset = (n * PID_FRAM_CH_SIZE * FRAM_FLOAT_SIZE);	//offset danych kolejnego kanału regulatora
        //odczytaj wartość wzmocnienienia członu P regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_P0 + chAdrOffset, &stPID[n].fWzmP, VMIN_PID_WZMP, VMAX_PID_WZMP, VDEF_PID_WZMP, ERR_NASTAWA_PID);

        //odczytaj wartość wzmocnienienia członu I regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_I0 + chAdrOffset, &stPID[n].fWzmI, VMIN_PID_WZMI, VMAX_PID_WZMI, VDEF_PID_WZMI, ERR_NASTAWA_PID);

        //odczytaj wartość wzmocnienienia członu D regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_D0 + chAdrOffset, &stPID[n].fWzmD, VMIN_PID_WZMD, VMAX_PID_WZMD, VDEF_PID_WZMD, ERR_NASTAWA_PID);

        //odczytaj granicę nasycenia członu całkującego
        chErr |= CzytajFramZWalidacja(FAU_PID_OGR_I0 + chAdrOffset, &stPID[n].fOgrCalki, VMIN_PID_ILIM, VMAX_PID_ILIM, VDEF_PID_ILIM, ERR_NASTAWA_PID);

        //odczytaj minimalną wartość wyjścia
        chErr |= CzytajFramZWalidacja(FAU_PID_MIN_WY0 + chAdrOffset, &stPID[n].fOgrCalki, VMIN_PID_MINWY, VMAX_PID_MINWY, VDEF_PID_MINWY, ERR_NASTAWA_PID);

        //odczytaj maksymalną wartość wyjścia
        chErr |= CzytajFramZWalidacja(FAU_PID_MAX_WY0 + chAdrOffset, &stPID[n].fOgrCalki, VMIN_PID_MAXWY, VMAX_PID_MAXWY, VDEF_PID_MAXWY, ERR_NASTAWA_PID);

        //odczytaj stałą czasową filtru członu różniczkowania (bity 0..5), właczony (bit 6) i to czy regulator jest kątowy (bit 7)
        chTemp = CzytajFRAM(FAU_FILTRD_TYP + n);
        stPID[n].chPodstFiltraD = chTemp & MASKA_FILTRA_D;
        stPID[n].chFlagi = chTemp & (MASKA_WYLACZONY | MASKA_KATOWY);

        //zeruj integrator
        stPID[n].fCalka = 0.0f;   	//zmianna przechowująca całkę z błędu
        stPID[n].fFiltrWePoprz = 0.0f;	//poprzednia wartość błędu
    }

    return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja liczy odpowiedź regulatora PID
// Parametry: 
// [i] ndT - czas od ostatniego cyklu [us]
// [i] chKanal - indeks regulatora i zmiennych
// Zwraca: znormalizowaną odpowiedź regulatora +-100%
// Czas wykonania: ?
////////////////////////////////////////////////////////////////////////////////
float RegulatorPID(uint32_t ndT, uint8_t chKanal)
{
    float fWyjscieReg, fOdchylka;   //wartość wyjściowa i błąd sterowania (odchyłka)
    float fTemp, fdT;

    fdT = (float)ndT/1000000;    //czas obiegu petli w sekundach (optymalizacja kilkukrotnie wykorzystywanej zmiennej)

    //człon proporocjonalny
    fOdchylka = *stPID[chKanal].fZadana - *stPID[chKanal].fWejscie;
    if (stPID[chKanal].chFlagi & MASKA_KATOWY)  //czy regulator pracuje na wartościach kątowych?
    {
        if (fOdchylka > M_PI)
        	fOdchylka -= 2*M_PI;
        if (fOdchylka < -M_PI)
        	fOdchylka += 2*M_PI;
    }
    fWyjscieReg = fOdchylka * stPID[chKanal].fWzmP;
    stPID[chKanal].fWyjscieP = fWyjscieReg;  //debugowanie: wartość wyjściowa z członu P

    //człon całkujący - liczy sumę błędu od początku do teraz
    if (stPID[chKanal].fWzmI > MIN_WZM_CALK)    //sprawdź warunek !=0 ze wzglądu na dzielenie przez fWzmI[] oraz ogranicz zbyt szybkie całkowanie nastawione pomyłkowo jako 0 a będące bardzo małą liczbą
    {
    	//stPID[chKanal].fCalka += fBladReg * fdT;   //całkowanie odchyłki - regulator równoległy
    	//stPID[chKanal].fCalka += fWyjscieReg * fdT;   //całkowanie wzmocnionego wejścia - regulator szeregowy
        //fTemp = stPID[chKanal].fCalka / stPID[chKanal].fWzmI;

    	stPID[chKanal].fCalka += fWyjscieReg * fdT / stPID[chKanal].fWzmI;   //całkowanie wzmocnionego wejścia - regulator szeregowy

        //ogranicznik wartości całki
        if (stPID[chKanal].fCalka > stPID[chKanal].fOgrCalki)
        {
        	stPID[chKanal].fCalka = stPID[chKanal].fOgrCalki * stPID[chKanal].fWzmI;
            //fTemp = stPID[chKanal].fOgrCalki;
        }
        else
        if (stPID[chKanal].fCalka < -stPID[chKanal].fOgrCalki)
        {
        	stPID[chKanal].fCalka = -stPID[chKanal].fOgrCalki * stPID[chKanal].fWzmI;
            //fTemp = -stPID[chKanal].fOgrCalki;
        }
        fWyjscieReg += stPID[chKanal].fCalka;
        stPID[chKanal].fWyjscieI = stPID[chKanal].fCalka;  //debugowanie: wartość wyjściowa z członu I
    }
    else
    	 stPID[chKanal].fWyjscieI = 0.0f;  //debugowanie: wartość wyjściowa z członu I


    //człon różniczkujący
    if (stPID[chKanal].fWzmD > MIN_WZM_ROZN)
    {
        fTemp = (*stPID[chKanal].fWejscie - stPID[chKanal].fFiltrWePoprz) * stPID[chKanal].fWzmD / fdT;
        fWyjscieReg += fTemp;

        //filtruj wartość wejścia aby uzyskać gładką akcję różniczkującą
        stPID[chKanal].fFiltrWePoprz = (stPID[chKanal].chPodstFiltraD * stPID[chKanal].fFiltrWePoprz + *stPID[chKanal].fWejscie) / (stPID[chKanal].chPodstFiltraD + 1);
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
    for (uint8_t n=0; n<LICZBA_PID; n++)
    	stPID[n].fCalka = 0.0f;   //zmianna przechowująca całka z błędu
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja łączy regualtory PID kątów i wysokości z regulatorami prędkości katowych i wario realizując sterowanie stabilnością
// Parametry: brak
// [i] ndT - czas od ostatniego cyklu [us]
// [i] *pid - wskaźnik na strukturę danych regulatorów PID
// [i] *wron - wskaźnik na strukturę danych parametrów wrona
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t StabilizacjaPID(uint32_t ndT, stWymianyCM4_t *dane)
{
	//regulacja przechylenia
	dane->pid[PID_PRZE].fWejscie = dane->fKatIMU1[PRZE];
	RegulatorPID(ndT, PID_PRZE);

	dane->pid[PID_PK_PRZE].fWejscie = dane->fZyroKal1[PRZE];
	dane->pid[PID_PK_PRZE].fZadana = dane->pid[PID_PRZE].fWyjsciePID;
	RegulatorPID(ndT, PID_PK_PRZE);

	//regulacja pochylenia
	dane->pid[PID_POCH].fWejscie = dane->fKatIMU1[POCH];
	RegulatorPID(ndT, PID_POCH);

	dane->pid[PID_PK_POCH].fWejscie = dane->fZyroKal1[POCH];
	dane->pid[PID_PK_POCH].fZadana = dane->pid[PID_POCH].fWyjsciePID;
	RegulatorPID(ndT, PID_PK_POCH);

	//regulacja odchylenia
	dane->pid[PID_ODCH].fWejscie = dane->fKatIMU1[ODCH];
	RegulatorPID(ndT, PID_ODCH);

	dane->pid[PID_PK_ODCH].fWejscie = dane->fZyroKal1[ODCH];
	dane->pid[PID_PK_ODCH].fZadana = dane->pid[PID_ODCH].fWyjsciePID;
	RegulatorPID(ndT, PID_PK_ODCH);

	//regulacja wysokości
	dane->pid[PID_WYSO].fWejscie = dane->fWysoko[0];
	RegulatorPID(ndT, PID_WYSO);

	dane->pid[PID_WARIO].fWejscie = dane->fWariometr[0];
	dane->pid[PID_WARIO].fZadana = dane->pid[PID_WYSO].fWyjsciePID;
	RegulatorPID(ndT, PID_WARIO);
	return ERR_OK;
}
