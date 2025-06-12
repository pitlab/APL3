//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v2.0
// Moduł ubsługi regulatorów PID
//
// (c) Pit Lab 
// https://www.pitlab.pl
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
uint8_t InicjujPID(stWymianyCM4_t *dane)
{
	uint8_t chAdrOffset, chErr = ERR_OK;
	uint8_t chTemp;

    for (uint8_t n=0; n<LICZBA_PID; n++)
    {
        chAdrOffset = (n * PID_FRAM_CH_SIZE * FRAM_FLOAT_SIZE);	//offset danych kolejnego kanału regulatora
        //odczytaj wartość wzmocnienienia członu P regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_P0 + chAdrOffset, &dane->pid[n].fWzmP, VMIN_PID_WZMP, VMAX_PID_WZMP, VDEF_PID_WZMP, ERR_NASTAWA_FRAM);

        //odczytaj wartość wzmocnienienia członu I regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_I0 + chAdrOffset, &dane->pid[n].fWzmI, VMIN_PID_WZMI, VMAX_PID_WZMI, VDEF_PID_WZMI, ERR_NASTAWA_FRAM);

        //odczytaj wartość wzmocnienienia członu D regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_D0 + chAdrOffset, &dane->pid[n].fWzmD, VMIN_PID_WZMD, VMAX_PID_WZMD, VDEF_PID_WZMD, ERR_NASTAWA_FRAM);

        //odczytaj granicę nasycenia członu całkującego
        chErr |= CzytajFramZWalidacja(FAU_PID_OGR_I0 + chAdrOffset, &dane->pid[n].fOgrCalki, VMIN_PID_ILIM, VMAX_PID_ILIM, VDEF_PID_ILIM, ERR_NASTAWA_FRAM);

        //odczytaj minimalną wartość wyjścia
        chErr |= CzytajFramZWalidacja(FAU_PID_MIN_WY0 + chAdrOffset, &dane->pid[n].fOgrCalki, VMIN_PID_MINWY, VMAX_PID_MINWY, VDEF_PID_MINWY, ERR_NASTAWA_FRAM);

        //odczytaj maksymalną wartość wyjścia
        chErr |= CzytajFramZWalidacja(FAU_PID_MAX_WY0 + chAdrOffset, &dane->pid[n].fOgrCalki, VMIN_PID_MAXWY, VMAX_PID_MAXWY, VDEF_PID_MAXWY, ERR_NASTAWA_FRAM);

        //odczytaj stałą czasową filtru członu różniczkowania (bity 0..5), właczony (bit 6) i to czy regulator jest kątowy (bit 7)
        chTemp = CzytajFRAM(FAU_FILTRD_TYP + n);
        dane->pid[n].chPodstFiltraD = chTemp & MASKA_FILTRA_D;
        dane->pid[n].chFlagi = chTemp & (MASKA_WYLACZONY | MASKA_KATOWY);

        //zeruj integrator
        dane->pid[n].fCalka = 0.0f;   	//zmianna przechowująca całkę z błędu
        dane->pid[n].fFiltrWePoprz = 0.0f;	//poprzednia wartość błędu
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
float RegulatorPID(uint32_t ndT, uint8_t chKanal, stWymianyCM4_t *dane)
{
    float fWyjscieReg, fOdchylka;   //wartość wyjściowa i błąd sterowania (odchyłka)
    float fTemp, fdT;

    fdT = (float)ndT/1000000;    //czas obiegu petli w sekundach (optymalizacja kilkukrotnie wykorzystywanej zmiennej)

    //człon proporocjonalny
    fOdchylka = dane->pid[chKanal].fZadana - dane->pid[chKanal].fWejscie;
    if (dane->pid[chKanal].chFlagi & MASKA_KATOWY)  //czy regulator pracuje na wartościach kątowych?
    {
        if (fOdchylka > M_PI)
        	fOdchylka -= 2*M_PI;
        if (fOdchylka < -M_PI)
        	fOdchylka += 2*M_PI;
    }
    fWyjscieReg = fOdchylka * dane->pid[chKanal].fWzmP;
    dane->pid[chKanal].fWyjscieP = fWyjscieReg;  //debugowanie: wartość wyjściowa z członu P

    //człon całkujący - liczy sumę błędu od początku do teraz
    if (dane->pid[chKanal].fWzmI > MIN_WZM_CALK)    //sprawdź warunek !=0 ze wzglądu na dzielenie przez fWzmI[] oraz ogranicz zbyt szybkie całkowanie nastawione pomyłkowo jako 0 a będące bardzo małą liczbą
    {
    	//stPID[chKanal].fCalka += fBladReg * fdT;   //całkowanie odchyłki - regulator równoległy
    	//stPID[chKanal].fCalka += fWyjscieReg * fdT;   //całkowanie wzmocnionego wejścia - regulator szeregowy
        //fTemp = stPID[chKanal].fCalka / stPID[chKanal].fWzmI;

    	dane->pid[chKanal].fCalka += fWyjscieReg * fdT / dane->pid[chKanal].fWzmI;   //całkowanie wzmocnionego wejścia - regulator szeregowy

        //ogranicznik wartości całki
        if (dane->pid[chKanal].fCalka > dane->pid[chKanal].fOgrCalki)
        {
        	dane->pid[chKanal].fCalka = dane->pid[chKanal].fOgrCalki * dane->pid[chKanal].fWzmI;
            //fTemp = stPID[chKanal].fOgrCalki;
        }
        else
        if (dane->pid[chKanal].fCalka < -dane->pid[chKanal].fOgrCalki)
        {
        	dane->pid[chKanal].fCalka = -dane->pid[chKanal].fOgrCalki * dane->pid[chKanal].fWzmI;
            //fTemp = -stPID[chKanal].fOgrCalki;
        }
        fWyjscieReg += dane->pid[chKanal].fCalka;
        dane->pid[chKanal].fWyjscieI = dane->pid[chKanal].fCalka;  //debugowanie: wartość wyjściowa z członu I
    }
    else
    	dane->pid[chKanal].fWyjscieI = 0.0f;  //debugowanie: wartość wyjściowa z członu I


    //człon różniczkujący
    if (dane->pid[chKanal].fWzmD > MIN_WZM_ROZN)
    {
        fTemp = (dane->pid[chKanal].fWejscie - dane->pid[chKanal].fFiltrWePoprz) * dane->pid[chKanal].fWzmD / fdT;
        fWyjscieReg += fTemp;

        //filtruj wartość wejścia aby uzyskać gładką akcję różniczkującą
        dane->pid[chKanal].fFiltrWePoprz = (dane->pid[chKanal].chPodstFiltraD * dane->pid[chKanal].fFiltrWePoprz + dane->pid[chKanal].fWejscie) / (dane->pid[chKanal].chPodstFiltraD + 1);
    }
    else
        fTemp = 0.0f;
    dane->pid[chKanal].fWyjscieD = fTemp;  //wartość wyjściowa z członu D
  
    //ograniczenie wartości wyjściowej
    if (fWyjscieReg > dane->pid[chKanal].fMaxWyj)
    	fWyjscieReg = dane->pid[chKanal].fMaxWyj;
    else
    if (fWyjscieReg < dane->pid[chKanal].fMinWyj)
    	fWyjscieReg = dane->pid[chKanal].fMinWyj;

    dane->pid[chKanal].fWyjsciePID = fWyjscieReg;    //wartość wyjściowa z całego regulatora
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
    	uDaneCM4.dane.pid[n].fCalka = 0.0f;   //zmianna przechowująca całka z błędu
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
	RegulatorPID(ndT, PID_PRZE, dane);

	dane->pid[PID_PK_PRZE].fWejscie = dane->fZyroKal1[PRZE];
	dane->pid[PID_PK_PRZE].fZadana = dane->pid[PID_PRZE].fWyjsciePID;
	RegulatorPID(ndT, PID_PK_PRZE, dane);

	//regulacja pochylenia
	dane->pid[PID_POCH].fWejscie = dane->fKatIMU1[POCH];
	RegulatorPID(ndT, PID_POCH, dane);

	dane->pid[PID_PK_POCH].fWejscie = dane->fZyroKal1[POCH];
	dane->pid[PID_PK_POCH].fZadana = dane->pid[PID_POCH].fWyjsciePID;
	RegulatorPID(ndT, PID_PK_POCH, dane);

	//regulacja odchylenia
	dane->pid[PID_ODCH].fWejscie = dane->fKatIMU1[ODCH];
	RegulatorPID(ndT, PID_ODCH, dane);

	dane->pid[PID_PK_ODCH].fWejscie = dane->fZyroKal1[ODCH];
	dane->pid[PID_PK_ODCH].fZadana = dane->pid[PID_ODCH].fWyjsciePID;
	RegulatorPID(ndT, PID_PK_ODCH, dane);

	//regulacja wysokości
	dane->pid[PID_WYSO].fWejscie = dane->fWysokoMSL[0];
	RegulatorPID(ndT, PID_WYSO, dane);

	dane->pid[PID_WARIO].fWejscie = dane->fWariometr[0];
	dane->pid[PID_WARIO].fZadana = dane->pid[PID_WYSO].fWyjsciePID;
	RegulatorPID(ndT, PID_WARIO, dane);
	return ERR_OK;
}
