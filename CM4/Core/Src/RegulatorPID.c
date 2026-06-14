//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł ubsługi regulatorów PID
//
// (c) Pit Lab 2025
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "RegulatorPID.h"
#include "FRAM.h"

//Przyjmuję że regulator będzie mógł być szeregowy lub równoległy z preferencja szeregowego
//Człon różniczkujący jest zasilany ujemnym sygnałem wejściowym a nie błędem, aby uniknąć zmiany odpowiedzi od zmiany wartości zadanej.
// w klasycznym członie: błąd = wartość zadana - wejście
// w układzie zmodyfikowanym usuwamy wartość zadaną, wiec: błąd = - wejscie
//Człon różniczkujący bazuje na różnicy między wartością bieżącą i poprzednią sygnału wejsciowego. Aby zmniejszyć szum regulacji, wartość poprzednia
// jest przefiltrowaną wartością n poprzednich pomiarów gdzie n jest regulowaną nastawą filtra D

//definicje zmiennych
stKonfPID_t stKonfigPID[LICZBA_PID];
stStrojPID_t stStrojPID[LICZBA_KAN_RC_DO_STROJENIA_PID];

//deklaracje zmiennych zewnętrznych
extern unia_wymianyCM4_t uDaneCM4;




////////////////////////////////////////////////////////////////////////////////
// Funkcja przeładowuje konfigurację regulatora PID
// Parametry: brak
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujPID(void)
{
	uint8_t cBłąd = BLAD_OK;
	uint8_t chTemp;
	uint16_t sAdrOffset;

    for (uint16_t n=0; n<LICZBA_PID; n++)
    {
    	sAdrOffset = (n * ROZMIAR_REG_PID);	//offset danych kolejnego kanału regulatora
        //odczytaj wartość wzmocnienienia członu P regulatora
    	cBłąd |= CzytajFramFloatZWalidacja(FAU_PID_KP + sAdrOffset, &stKonfigPID[n].fWzmP, VMIN_PID_WZMP, VMAX_PID_WZMP, VDOM_PID_WZMP);

        //odczytaj wartość wzmocnienienia członu I regulatora
        cBłąd |= CzytajFramFloatZWalidacja(FAU_PID_TI + sAdrOffset, &stKonfigPID[n].fWzmI, VMIN_PID_WZMI, VMAX_PID_WZMI, VDOM_PID_WZMI);

        //odczytaj wartość wzmocnienienia członu D regulatora
        cBłąd |= CzytajFramFloatZWalidacja(FAU_PID_TD + sAdrOffset, &stKonfigPID[n].fWzmD, VMIN_PID_WZMD, VMAX_PID_WZMD, VDOM_PID_WZMD);

        //odczytaj granicę nasycenia członu całkującego
        cBłąd |= CzytajFramFloatZWalidacja(FAU_PID_OGR_CALK + sAdrOffset, &stKonfigPID[n].fOgrCalki, VMIN_PID_ILIM, VMAX_PID_ILIM, VDOM_PID_ILIM);

        //odczytaj minimalną wartość wyjścia
        cBłąd |= CzytajFramFloatZWalidacja(FAU_PID_MIN_WY + sAdrOffset, &stKonfigPID[n].fMinWyj, VMIN_PID_MINWY, VMAX_PID_MINWY, VDOM_PID_MINWY);

        //odczytaj maksymalną wartość wyjścia
        cBłąd |= CzytajFramFloatZWalidacja(FAU_PID_MAX_WY + sAdrOffset, &stKonfigPID[n].fMaxWyj, VMIN_PID_MAXWY, VMAX_PID_MAXWY, VDOM_PID_MAXWY);

        //odczytaj mnożnik wartości zadanej
        cBłąd |= CzytajFramFloatZWalidacja(FAU_PID_MNOZN_WZAD + sAdrOffset, &stKonfigPID[n].fSkalaWartZadanej, VMIN_PID_MNOZWZ, VMAX_PID_MNOZWZ, VDOM_PID_MNOZWZ);

        //odczytaj stałą wartość dodawaną do wyjścia regulatora
        cBłąd |= CzytajFramFloatZWalidacja(FAU_PID_STAŁE_PRZES + sAdrOffset, &stKonfigPID[n].fPrzesunWyjscie, VMIN_PID_STWYPRZ, VMAX_PID_STWYPRZ, VDOM_PID_STWYPRZ);

        //odczytaj stałą czasową filtru członu różniczkowania (bity 0..5), właczony (bit 6) i to czy regulator jest kątowy (bit 7)
        chTemp = CzytajFRAM(FAU_FILTRD_TYP + sAdrOffset);
        stKonfigPID[n].chPodstFiltraD = chTemp & PID_MASKA_FILTRA_D;
        stKonfigPID[n].chFlagi = chTemp & PID_KATOWY;

        //odczytaj jaki procent pochodnej wartości zadanej ma wchodzić na wejście wyprzedzające
        stKonfigPID[n].chProcWartZadWyprz = CzytajFRAM(FAU_PID_PROC_WYPRZ + sAdrOffset);

        //zeruj integrator
        uDaneCM4.dane.stPID[n].fCalka = 0.0f;   	//zmienna przechowująca całkę z błędu
        uDaneCM4.dane.stPID[n].fFiltrWeD = 0.0f;	//poprzednia wartość błędu
    }

    for (uint16_t n=0; n<LICZBA_KAN_RC_DO_STROJENIA_PID; n++)
    {
    	cBłąd |= CzytajFramU8zWalidacja(FAU_STROJ1_PARAMETR + n,  &stStrojPID[n].chNrParametru,  VMIN_STRPID_PAR,  VMAX_STRPID_PAR,  VDOM_STRPID_PAR);	//numer strojonego parametru
    	cBłąd |= CzytajFramFloatZWalidacja(FAU_STROJ1_WART_MIN + n*sizeof(float), &stStrojPID[n].fWartośćMin, VMIN_STRPID_MIN, VMAX_STRPID_MIN, VDOM_STRPID_MIN);	//minimalna wartość parametru dla minimalnej wartości kanału
    	cBłąd |= CzytajFramFloatZWalidacja(FAU_STROJ1_WART_MAX + n*sizeof(float), &stStrojPID[n].fWartośćMax, VMIN_STRPID_MAX, VMAX_STRPID_MAX, VDOM_STRPID_MAX);	//maksymalna wartość parametru dla maksymalnej wartości kanału
    }
    return cBłąd;
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

    //Człon proporcjonalny.
    fOdchylka = dane->stPID[chKanal].fZadana - dane->stPID[chKanal].fWejscie;
    if (konfig[chKanal].chFlagi & PID_KATOWY)  //czy regulator pracuje na wartościach kątowych?
    {
        if (fOdchylka > M_PI)
        	fOdchylka -= 2*M_PI;
        if (fOdchylka < -M_PI)
        	fOdchylka += 2*M_PI;
    }
    fWyjscieReg = fOdchylka * konfig[chKanal].fWzmP;
    if (fWyjscieReg > MAX_PID)
    	fWyjscieReg = MAX_PID;
    else
    if (fWyjscieReg < -MAX_PID)
    	fWyjscieReg = -MAX_PID;
    dane->stPID[chKanal].fWyjscieP = fWyjscieReg;  //debugowanie: wartość wyjściowa z członu P

    //człon całkujący - liczy sumę błędu od początku do teraz
    if (konfig[chKanal].fWzmI > MIN_WZM_CALK)    //sprawdź warunek !=0 ze wzglądu na dzielenie przez fWzmI[] oraz ogranicz zbyt szybkie całkowanie nastawione pomyłkowo jako 0 a będące bardzo małą liczbą
    {
    	dane->stPID[chKanal].fCalka += fWyjscieReg * fdT / konfig[chKanal].fWzmI;   //całkowanie odchyłki po wzmocnieniu - regulator szeregowy

        //ogranicznik wartości całki
        if (dane->stPID[chKanal].fCalka > konfig[chKanal].fOgrCalki)
        	dane->stPID[chKanal].fCalka = konfig[chKanal].fOgrCalki;
        if (dane->stPID[chKanal].fCalka < -konfig[chKanal].fOgrCalki)
        	dane->stPID[chKanal].fCalka = -konfig[chKanal].fOgrCalki;

        fWyjscieReg += dane->stPID[chKanal].fCalka;
        dane->stPID[chKanal].fWyjscieI = dane->stPID[chKanal].fCalka;  //debugowanie: wartość wyjściowa z członu I
    }
    else
    	dane->stPID[chKanal].fWyjscieI = 0.0f;  //debugowanie: wartość wyjściowa z członu I


    //człon różniczkujący
    if (konfig[chKanal].fWzmD > MIN_WZM_ROZN)
    {
        fTemp = (dane->stPID[chKanal].fWejscie - dane->stPID[chKanal].fFiltrWeD) * konfig[chKanal].fWzmD / fdT;
        if (fTemp > MAX_PID)
        	fTemp = MAX_PID;
	   else
	   if (fTemp < -MAX_PID)
		   fTemp = -MAX_PID;
        fWyjscieReg += fTemp;

        //filtruj wartość wejścia aby uzyskać gładką akcję różniczkującą
        dane->stPID[chKanal].fFiltrWeD = (konfig[chKanal].chPodstFiltraD * dane->stPID[chKanal].fFiltrWeD + dane->stPID[chKanal].fWejscie) / (konfig[chKanal].chPodstFiltraD + 1);
    }
    else
        fTemp = 0.0f;
    dane->stPID[chKanal].fWyjscieD = fTemp;  //wartość wyjściowa z członu D
  
    //sprawdź czy kanał dotyczy regulatora wartości podstawowej (kąta, wysokości, pozycji) czyli wartosci parzystej czy też jego pochodnej będąca liczbą nieparzystą
    if (chKanal & 0x01)
    {
    	//liczba nieparzysta, czyli regulator pochodnej. Oblicz pochodną wartości zadanej
        float fPochodnaWartZadanej = (dane->stPID[chKanal].fZadana - dane->stPID[chKanal].fFiltrWartZad)  / fdT;
        dane->stPID[chKanal].fFiltrWartZad = (konfig[chKanal].chPodstFiltraD * dane->stPID[chKanal].fFiltrWartZad + dane->stPID[chKanal].fZadana) / (konfig[chKanal].chPodstFiltraD + 1);

        //dodanie pierwszej pochodnej wartości zadanej do wejścia wyprzedzającego
    	fTemp = fPochodnaWartZadanej * konfig[chKanal].chProcWartZadWyprz / 100.0f;
    	dane->stPID[chKanal].fWyjscieWyprz  = fTemp;
        fWyjscieReg += fTemp;
    }
    else
    {
    	//Dla regulatorów wartosci podstawowych jest możliwe stałe przesunięcie odpowiedzi regulatora
	    fWyjscieReg += konfig[chKanal].fPrzesunWyjscie;
    }

    //ograniczenie wartości wyjściowej
    if (fWyjscieReg > konfig[chKanal].fMaxWyj)
    	fWyjscieReg = konfig[chKanal].fMaxWyj;
    else
    if (fWyjscieReg < konfig[chKanal].fMinWyj)
    	fWyjscieReg = konfig[chKanal].fMinWyj;

    dane->stPID[chKanal].fWyjsciePID = fWyjscieReg;    //wartość wyjściowa z całego regulatora
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
    	uDaneCM4.dane.stPID[n].fCalka = 0.0f;   //zmianna przechowująca całka z błędu
}



////////////////////////////////////////////////////////////////////////////////
// Zmiana wartości kanału RC nadpisuje wartości nastawy wybrnego parametru regulatora PID
// Parametry:
//   *Stroj - wskaźnik na strukture strojenia PID kanałem RC
//	 chNrKan - numer kanału RC zdefiniowanego do strojenia bieżącego parametru
//   *Konf - wskaźnik na strukturę konfiguracji PID
//   *WymianaCM4 - wskaźnik na strukturę wymiany danych rdzenia CM4 zawierajacą dane autopilota
// Zwraca: wartość parametru strojącego
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
float StrojeniePID_KanałemRC(stStrojPID_t *Stroj, uint8_t chNrKan, stKonfPID_t *Konf, stWymianyCM4_t *WymianaCM4)
{
	float fKanalNorm, fParametr;

	//policz wartość parametru
	fKanalNorm = (float)WymianaCM4->sKanalRC[chNrKan] / (WE_RC_P100 - WE_RC_M100);	//wartość kanału RC znormalizowana do zakresu 0..1
	fParametr = Stroj->fWartośćMin + fKanalNorm * (Stroj->fWartośćMax - Stroj->fWartośćMin);

	switch(Stroj->chNrParametru)
	{
	case STRP_NIC:			break;		//strojenie wyłączone
	case STRP_KATA_PRZE_KP:	Konf[PID_KĄTA_PRZE].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze przechylenia
	case STRP_KATA_PRZE_TI:	Konf[PID_KĄTA_PRZE].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze przechylenia
	case STRP_KATA_PRZE_TD:	Konf[PID_KĄTA_PRZE].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze przechylenia
	case STRP_PRED_PRZE_KP:	Konf[PID_PRED_PRZE].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze prędkości kątowej przechylenia
	case STRP_PRED_PRZE_TI:	Konf[PID_PRED_PRZE].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze prędkości kątowej przechylenia
	case STRP_PRED_PRZE_TD:	Konf[PID_PRED_PRZE].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze prędkości kątowej przechylenia

	case STRP_KATA_POCH_KP:	Konf[PID_KĄTA_POCH].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze pochylenia
	case STRP_KATA_POCH_TI:	Konf[PID_KĄTA_POCH].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze pochylenia
	case STRP_KATA_POCH_TD:	Konf[PID_KĄTA_POCH].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze pochylenia
	case STRP_PRED_POCH_KP:	Konf[PID_PRED_POCH].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze prędkości kątowej pochylenia
	case STRP_PRED_POCH_TI:	Konf[PID_PRED_POCH].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze prędkości kątowej pochylenia
	case STRP_PRED_POCH_TD:	Konf[PID_PRED_POCH].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze prędkości kątowej pochylenia

	case STRP_KATA_ODCH_KP:	Konf[PID_KĄTA_ODCH].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze odchylenia
	case STRP_KATA_ODCH_TI:	Konf[PID_KĄTA_ODCH].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze odchylenia
	case STRP_KATA_ODCH_TD:	Konf[PID_KĄTA_ODCH].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze odchylenia
	case STRP_PRED_ODCH_KP:	Konf[PID_PRED_ODCH].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze prędkości kątowej odchylenia
	case STRP_PRED_ODCH_TI:	Konf[PID_PRED_ODCH].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze prędkości kątowej odchylenia
	case STRP_PRED_ODCH_TD:	Konf[PID_PRED_ODCH].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze prędkości kątowej odchylenia

	case STRP_WYSOKOSCI_KP:	Konf[PID_WYSOKOSCI].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze wysokości
	case STRP_WYSOKOSCI_TI:	Konf[PID_WYSOKOSCI].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze wysokości
	case STRP_WYSOKOSCI_TD:	Konf[PID_WYSOKOSCI].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze wysokości
	case STRP_PRED_ZWYS_KP:	Konf[PID_PRED_ZWYS].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze prędkości zmiany wysokości
	case STRP_PRED_ZWYS_TI:	Konf[PID_PRED_ZWYS].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze prędkości zmiany wysokości
	case STRP_PRED_ZWYS_TD:	Konf[PID_PRED_ZWYS].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze prędkości zmiany wysokości

	case STRP_NAWI_PÓŁN_KP:	Konf[PID_NAWIG_PÓŁN].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze nawigacji w kierunku północnym
	case STRP_NAWI_PÓŁN_TI:	Konf[PID_NAWIG_PÓŁN].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze nawigacji w kierunku północnym
	case STRP_NAWI_PÓŁN_TD:	Konf[PID_NAWIG_PÓŁN].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze nawigacji w kierunku północnym
	case STRP_PRED_PÓŁN_KP:	Konf[PID_PRED_PÓŁN].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze prędkości w kierunku północnym
	case STRP_PRED_PÓŁN_TI:	Konf[PID_PRED_PÓŁN].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze prędkości w kierunku północnym
	case STRP_PRED_PÓŁN_TD:	Konf[PID_PRED_PÓŁN].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze prędkości w kierunku północnym

	case STRP_NAWI_WSCH_KP:	Konf[PID_NAWIG_WSCH].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze nawigacji w kierunku wschodnim
	case STRP_NAWI_WSCH_TI:	Konf[PID_NAWIG_WSCH].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze nawigacji w kierunku wschodnim
	case STRP_NAWI_WSCH_TD:	Konf[PID_NAWIG_WSCH].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze nawigacji w kierunku wschodnim
	case STRP_PRED_WSCH_KP:	Konf[PID_PRED_WSCH].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze prędkości w kierunku wschodnim
	case STRP_PRED_WSCH_TI:	Konf[PID_PRED_WSCH].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze prędkości w kierunku wschodnim
	case STRP_PRED_WSCH_TD:	Konf[PID_PRED_WSCH].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze prędkości w kierunku wschodnim
	}
	return fParametr;
}



#ifdef TESTY		//testy algorytmów
////////////////////////////////////////////////////////////////////////////////
// Funkcja testująca regulatory. Sprawdza czy dla wybranych warunków i nastaw uzyskuje się właściwą odpowiedź .
// Test bazuje na rzeczywistych nastawach i wymyślonych danych wejściowych
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void TestPID(void)
{
	uint32_t ndT = 5000;	//czas obiegu pętli 200Hz [us]
	float fOdpowiedzNominalna;	//taka powinna być nominalna odpowiedź regulatora
	float fProgGor, fProgDol;	//progi testowania wartosci górny i dolny

	//sprawdź odpowiedź członu proporcjonalnego regulatora kątowego na zawijanie kątów wokół 2Pi
	uDaneCM4.dane.stPID[PID_KĄTA_POCH].fZadana = 370 * DEG2RAD;	//odpowiada +10°
	uDaneCM4.dane.stPID[PID_KĄTA_POCH].fWejscie = -5 * DEG2RAD;
	RegulatorPID(ndT, PID_KĄTA_POCH, &uDaneCM4.dane, stKonfigPID);

	assert(uDaneCM4.dane.stPID[PID_KĄTA_POCH].fWyjscieP < 15.001 * DEG2RAD * stKonfigPID[PID_KĄTA_POCH].fWzmP);
	assert(uDaneCM4.dane.stPID[PID_KĄTA_POCH].fWyjscieP > 14.999 * DEG2RAD * stKonfigPID[PID_KĄTA_POCH].fWzmP);

	//sprawdź działanie członu całkującego regulatora wysokości. Całka to czas zdwojenia, więc przy wzmocnieniu Kp=1 i Ti=1 całka po sekundzie osiaga dwukrotność uchybu.
	//czas trwania testu=10*5ms, błąd=20m, więc przyrost powinien wynosić 50/1000 * (20 * Kp) / Ti
	if (stKonfigPID[PID_WYSOKOSCI].fWzmI)	//zapobiegnie dzieleniu przez zero
	{
		uDaneCM4.dane.stPID[PID_WYSOKOSCI].fZadana = 60;
		uDaneCM4.dane.stPID[PID_WYSOKOSCI].fWejscie = 40;
		uDaneCM4.dane.stPID[PID_WYSOKOSCI].fCalka = 0;
		for (uint8_t n=0; n<10; n++)
			RegulatorPID(ndT, PID_WYSOKOSCI, &uDaneCM4.dane, stKonfigPID);

		fOdpowiedzNominalna = 0.005 * 10 * 20 * stKonfigPID[PID_WYSOKOSCI].fWzmP / stKonfigPID[PID_WYSOKOSCI].fWzmI;
		fProgGor = fOdpowiedzNominalna + 0.001;
		fProgDol = fOdpowiedzNominalna - 0.001;
		assert(uDaneCM4.dane.stPID[PID_WYSOKOSCI].fWyjscieI < fProgGor);
		assert(uDaneCM4.dane.stPID[PID_WYSOKOSCI].fWyjscieI > fProgDol);
		uDaneCM4.dane.stPID[PID_WYSOKOSCI].fCalka = 0;	//wyczyść całkę po teście
	}
}
#endif
