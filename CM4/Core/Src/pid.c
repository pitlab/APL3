//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł ubsługi regulatorów PID
//
// (c) Pit Lab 2025
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "pid.h"
#include "fram.h"

//Przyjmuję że regulator będzie mógł być szeregowy lub równoległy z preferencja szeregowego
//Człon różniczkujący jest zasilany ujemnym sygnałem wejsciowym a nie błędem aby uniknąć zmiany odpowiedzi od zmiany wartosci zadanej.
// w klasycznym członie: błąd = wartość zadana - wejscie
// w układzie zmodyfikowanym u usuwamy wartość zadaną, wiec: błąd = - wejscie
//Człon różniczkujący bazuje na różnicy między wartością bieżącą i poprzednią sygnału wejsciowego. Aby zmniejszyć szum regulacji, wartość poprzednia
// jest przefiltrowaną wartością n poprzednich pomiarów gdzie n jest regulowaną nastawą filtra D

//definicje zmiennych
//stPID_t stPID[LICZBA_PID];	//zmienna przechowująca dane dotyczące regulatora PID

//deklaracje zmiennych zewnętrznych
extern unia_wymianyCM4_t uDaneCM4;
stKonfPID_t stKonfigPID[LICZBA_PID];

////////////////////////////////////////////////////////////////////////////////
// Funkcja przeładowuje konfigurację regulatora PID
// Parametry: brak
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujPID(void)
{
	uint8_t chAdrOffset, chErr = BLAD_OK;
	uint8_t chTemp;

    for (uint8_t n=0; n<LICZBA_PID; n++)
    {
        chAdrOffset = (n * ROZMIAR_REG_PID);	//offset danych kolejnego kanału regulatora
        //odczytaj wartość wzmocnienienia członu P regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_P0 + chAdrOffset, &stKonfigPID[n].fWzmP, VMIN_PID_WZMP, VMAX_PID_WZMP, VDEF_PID_WZMP, ERR_NASTAWA_FRAM);
        assert(stKonfigPID[n].fWzmP > 0.0);

        //odczytaj wartość wzmocnienienia członu I regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_I0 + chAdrOffset, &stKonfigPID[n].fWzmI, VMIN_PID_WZMI, VMAX_PID_WZMI, VDEF_PID_WZMI, ERR_NASTAWA_FRAM);
        assert(stKonfigPID[n].fWzmI >= 0.0);

        //odczytaj wartość wzmocnienienia członu D regulatora
        chErr |= CzytajFramZWalidacja(FAU_PID_D0 + chAdrOffset, &stKonfigPID[n].fWzmD, VMIN_PID_WZMD, VMAX_PID_WZMD, VDEF_PID_WZMD, ERR_NASTAWA_FRAM);
        assert(stKonfigPID[n].fWzmD >= 0.0);

        //odczytaj granicę nasycenia członu całkującego
        chErr |= CzytajFramZWalidacja(FAU_PID_OGR_I0 + chAdrOffset, &stKonfigPID[n].fOgrCalki, VMIN_PID_ILIM, VMAX_PID_ILIM, VDEF_PID_ILIM, ERR_NASTAWA_FRAM);
        assert(stKonfigPID[n].fOgrCalki > 0.0);

        //odczytaj minimalną wartość wyjścia
        chErr |= CzytajFramZWalidacja(FAU_PID_MIN_WY0 + chAdrOffset, &stKonfigPID[n].fMinWyj, VMIN_PID_MINWY, VMAX_PID_MINWY, VDEF_PID_MINWY, ERR_NASTAWA_FRAM);
        //assert(stKonfigPID[n].fMinWyj >-1000.0);

        //odczytaj maksymalną wartość wyjścia
        chErr |= CzytajFramZWalidacja(FAU_PID_MAX_WY0 + chAdrOffset, &stKonfigPID[n].fMaxWyj, VMIN_PID_MAXWY, VMAX_PID_MAXWY, VDEF_PID_MAXWY, ERR_NASTAWA_FRAM);
        assert(stKonfigPID[n].fMaxWyj < 1000.0);

        //odczytaj stałą czasową filtru członu różniczkowania (bity 0..5), właczony (bit 6) i to czy regulator jest kątowy (bit 7)
        chTemp = CzytajFRAM(FAU_FILTRD_TYP + n);
        stKonfigPID[n].chPodstFiltraD = chTemp & PID_MASKA_FILTRA_D;
        stKonfigPID[n].chFlagi = chTemp & (PID_WLACZONY | PID_KATOWY);

        //zeruj integrator
        uDaneCM4.dane.stWyjPID[n].fCalka = 0.0f;   	//zmianna przechowująca całkę z błędu
        uDaneCM4.dane.stWyjPID[n].fFiltrWePoprz = 0.0f;	//poprzednia wartość błędu
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
float RegulatorPID(uint32_t ndT, uint8_t chKanal, stWymianyCM4_t *dane, stKonfPID_t *konfig)
{
    float fWyjscieReg, fOdchylka;   //wartość wyjściowa i błąd sterowania (odchyłka)
    float fTemp, fdT;

    fdT = (float)ndT/1000000;    //czas obiegu petli w sekundach (optymalizacja kilkukrotnie wykorzystywanej zmiennej)

    //człon proporcjonalny
    fOdchylka = konfig[chKanal].fZadana - konfig[chKanal].fWejscie;
    if (konfig[chKanal].chFlagi & PID_KATOWY)  //czy regulator pracuje na wartościach kątowych?
    {
        if (fOdchylka > M_PI)
        	fOdchylka -= 2*M_PI;
        if (fOdchylka < -M_PI)
        	fOdchylka += 2*M_PI;
    }
    fWyjscieReg = fOdchylka * konfig[chKanal].fWzmP;
    dane->stWyjPID[chKanal].fWyjscieP = fWyjscieReg;  //debugowanie: wartość wyjściowa z członu P

    //człon całkujący - liczy sumę błędu od początku do teraz
    if (konfig[chKanal].fWzmI > MIN_WZM_CALK)    //sprawdź warunek !=0 ze wzglądu na dzielenie przez fWzmI[] oraz ogranicz zbyt szybkie całkowanie nastawione pomyłkowo jako 0 a będące bardzo małą liczbą
    {
    	//stPID[chKanal].fCalka += fBladReg * fdT;   //całkowanie odchyłki - regulator równoległy
    	//stPID[chKanal].fCalka += fWyjscieReg * fdT;   //całkowanie wzmocnionego wejścia - regulator szeregowy
        //fTemp = stPID[chKanal].fCalka / stPID[chKanal].fWzmI;

    	dane->stWyjPID[chKanal].fCalka += fWyjscieReg * fdT / konfig[chKanal].fWzmI;   //całkowanie wzmocnionego wejścia - regulator szeregowy

        //ogranicznik wartości całki
        if (dane->stWyjPID[chKanal].fCalka > konfig[chKanal].fOgrCalki)
        {
        	dane->stWyjPID[chKanal].fCalka = konfig[chKanal].fOgrCalki * konfig[chKanal].fWzmI;
            //fTemp = stPID[chKanal].fOgrCalki;
        }
        else
        if (dane->stWyjPID[chKanal].fCalka < -konfig[chKanal].fOgrCalki)
        {
        	dane->stWyjPID[chKanal].fCalka = -konfig[chKanal].fOgrCalki * konfig[chKanal].fWzmI;
            //fTemp = -stPID[chKanal].fOgrCalki;
        }
        fWyjscieReg += dane->stWyjPID[chKanal].fCalka;
        dane->stWyjPID[chKanal].fWyjscieI = dane->stWyjPID[chKanal].fCalka;  //debugowanie: wartość wyjściowa z członu I
    }
    else
    	dane->stWyjPID[chKanal].fWyjscieI = 0.0f;  //debugowanie: wartość wyjściowa z członu I


    //człon różniczkujący
    if (konfig[chKanal].fWzmD > MIN_WZM_ROZN)
    {
        fTemp = (konfig[chKanal].fWejscie - dane->stWyjPID[chKanal].fFiltrWePoprz) * konfig[chKanal].fWzmD / fdT;
        fWyjscieReg += fTemp;

        //filtruj wartość wejścia aby uzyskać gładką akcję różniczkującą
        dane->stWyjPID[chKanal].fFiltrWePoprz = (konfig[chKanal].chPodstFiltraD * dane->stWyjPID[chKanal].fFiltrWePoprz + konfig[chKanal].fWejscie) / (konfig[chKanal].chPodstFiltraD + 1);
    }
    else
        fTemp = 0.0f;
    dane->stWyjPID[chKanal].fWyjscieD = fTemp;  //wartość wyjściowa z członu D
  
    //ograniczenie wartości wyjściowej
    if (fWyjscieReg > konfig[chKanal].fMaxWyj)
    	fWyjscieReg = konfig[chKanal].fMaxWyj;
    else
    if (fWyjscieReg < konfig[chKanal].fMinWyj)
    	fWyjscieReg = konfig[chKanal].fMinWyj;

    dane->stWyjPID[chKanal].fWyjsciePID = fWyjscieReg;    //wartość wyjściowa z całego regulatora
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
    	uDaneCM4.dane.stWyjPID[n].fCalka = 0.0f;   //zmianna przechowująca całka z błędu
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja łączy regualtory PID kątów i wysokości z regulatorami prędkości katowych i wario realizując sterowanie stabilnością
// Parametry: brak
// [i] ndT - czas od ostatniego cyklu [us]
// [i] *konfig - wskaźnik na strukturę danych regulatorów PID
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t StabilizacjaPID(uint32_t ndT, stWymianyCM4_t *dane, stKonfPID_t *konfig)
{
	//regulacja przechylenia
	konfig[PID_PRZE].fWejscie = dane->fKatIMU1[PRZE];
	RegulatorPID(ndT, PID_PRZE, dane, konfig);

	konfig[PID_PK_PRZE].fWejscie = dane->fZyroKal1[PRZE];
	konfig[PID_PK_PRZE].fZadana = dane->stWyjPID[PID_PRZE].fWyjsciePID;
	RegulatorPID(ndT, PID_PK_PRZE, dane, konfig);

	//regulacja pochylenia
	konfig[PID_POCH].fWejscie = dane->fKatIMU1[POCH];
	RegulatorPID(ndT, PID_POCH, dane, konfig);

	konfig[PID_PK_POCH].fWejscie = dane->fZyroKal1[POCH];
	konfig[PID_PK_POCH].fZadana = dane->stWyjPID[PID_POCH].fWyjsciePID;
	RegulatorPID(ndT, PID_PK_POCH, dane, konfig);

	//regulacja odchylenia
	konfig[PID_ODCH].fWejscie = dane->fKatIMU1[ODCH];
	RegulatorPID(ndT, PID_ODCH, dane, konfig);

	konfig[PID_PK_ODCH].fWejscie = dane->fZyroKal1[ODCH];
	konfig[PID_PK_ODCH].fZadana = dane->stWyjPID[PID_ODCH].fWyjsciePID;
	RegulatorPID(ndT, PID_PK_ODCH, dane, konfig);

	//regulacja wysokości
	konfig[PID_WYSO].fWejscie = dane->fWysokoMSL[0];
	RegulatorPID(ndT, PID_WYSO, dane, konfig);

	konfig[PID_WARIO].fWejscie = dane->fWariometr[0];
	konfig[PID_WARIO].fZadana = dane->stWyjPID[PID_WYSO].fWyjsciePID;
	RegulatorPID(ndT, PID_WARIO, dane, konfig);
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja testująca regulatory. Sprawdza czy dla wybranych warunków i nastaw uzyskuje się właściwą odpowiedź .
// Test bazuje na rzeczywistych nastawach i wymyślonych danych wejściowych
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void TestPID(void)
{
	uint32_t ndT = 5000;	//czas obiegu pętli 200Hz [us]

	//sprawdź odpowiedź członu proporcjonalnego regulatora kątowego na zawijanie kątów wokół 2Pi
	stKonfigPID[PID_POCH].fZadana = 370 * DEG2RAD;	//odpowiada +10°
	stKonfigPID[PID_POCH].fWejscie = -5 * DEG2RAD;
	RegulatorPID(ndT, PID_POCH, &uDaneCM4.dane, stKonfigPID);
	assert(uDaneCM4.dane.stWyjPID[PID_POCH].fWyjscieP < 15.001 * DEG2RAD * stKonfigPID[PID_POCH].fWzmP);
	assert(uDaneCM4.dane.stWyjPID[PID_POCH].fWyjscieP > 14.999 * DEG2RAD * stKonfigPID[PID_POCH].fWzmP);

	//sprawdź działanie członu całkującego regulatora wysokości. Całka to czas zdwojenia, więc przy wzmocnieniu Kp=1 błąd podwaja się po sekundzie.
	//czas trwania testu=10*5ms, błąd=10m, więc przyrost powinien wynosić 50/1000 * (20 * Kp) * Ti
	stKonfigPID[PID_WYSO].fZadana = 60;
	stKonfigPID[PID_WYSO].fWejscie = 40;
	stKonfigPID[PID_WYSO].fWzmI = 1;			//Testowo - usunąć po ustawieniu wartosci
	uDaneCM4.dane.stWyjPID[PID_WYSO].fCalka = 0;
	for (uint8_t n=0; n<10; n++)
		RegulatorPID(ndT, PID_WYSO, &uDaneCM4.dane, stKonfigPID);
	assert(uDaneCM4.dane.stWyjPID[PID_WYSO].fWyjscieI == 0.1 * stKonfigPID[PID_WYSO].fWzmP * stKonfigPID[PID_WYSO].fWzmI);
	uDaneCM4.dane.stWyjPID[PID_WYSO].fCalka = 0;	//usuń całkę po teście
	assert(uDaneCM4.dane.stWyjPID[PID_WYSO].fWyjscieI == 5);

}
