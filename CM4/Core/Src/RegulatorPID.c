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

        //odczytaj stałą wartość dodawaną do wartości zadanej regulatora
        cBłąd |= CzytajFramFloatZWalidacja(FAU_PID_PRZES + sAdrOffset, &stKonfigPID[n].fPrzesunWartZadanej, VMIN_PID_STWYPRZ, VMAX_PID_STWYPRZ, VDOM_PID_STWYPRZ);

        //odczytaj flagi regulatora: regulator wyłączony (bit 6), Regulator kątowy (bit 7)
        stKonfigPID[n].chFlagi = CzytajFRAM(FAU_PID_FLAGI + sAdrOffset);

        //odczytaj podstawę filtra członu różniczkowania (bity 0..5), właczony (bit 6) i to czy regulator jest kątowy (bit 7)
        stKonfigPID[n].chPodstFiltraD = CzytajFRAM(FAU_PID_FD + sAdrOffset);

        // Podstawa filtra IIR wartości zadanej do liczenia członu wyprzedzajacego
        stKonfigPID[n].chPodstFiltraWZad = CzytajFRAM(FAU_PID_F_WZ + sAdrOffset);

        //odczytaj jaki procent pochodnej wartości zadanej ma wchodzić na wejście wyprzedzające
        stKonfigPID[n].chProcWartZadWyprz = CzytajFRAM(FAU_PID_WYPRZ + sAdrOffset);

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
   	fOdchylka = dane->stPID[chKanal].fZadana - dane->stPID[chKanal].fWejscie + konfig[chKanal].fPrzesunWartZadanej;
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
        if (fTemp > MAX_PID)
        	fTemp = MAX_PID;
	   else
	   if (fTemp < -MAX_PID)
		   fTemp = -MAX_PID;
    	dane->stPID[chKanal].fWyjscieWyprz  = (3 * dane->stPID[chKanal].fWyjscieWyprz + fTemp) / 4;	//lekko odfiltrowane wyjście wyprzedzające
        fWyjscieReg += fTemp;
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
float StrojeniePID_KanałemRC(stStrojPID_t *stStrój, uint8_t chNrKan, stKonfPID_t *Konf, stWymianyCM4_t *WymianaCM4)
{
	float fParametr;
	uint16_t sParam;

	fParametr = ObliczWartośćParametruStrojenia(WymianaCM4->sKanalRC[chNrKan], stStrój);		//oblicz wartość parametru

	switch(stStrój->chNrParametru)
	{
	case STRP_NIC:			break;		//strojenie wyłączone
	case STRP_KATA_PRZE_KP:		Konf[PID_KĄTA_PRZE].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze przechylenia
	case STRP_KATA_PRZE_TI:		Konf[PID_KĄTA_PRZE].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze przechylenia
	case STRP_KATA_PRZE_TD:		Konf[PID_KĄTA_PRZE].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze przechylenia
	case STRP_KATA_PRZE_FD:		Konf[PID_KĄTA_PRZE].chPodstFiltraD = (uint8_t)fParametr;	break;//Strojenie filtra sygnału różniczkowanego
	case STRP_KATA_PRZE_FWZ:	Konf[PID_KĄTA_PRZE].chPodstFiltraWZad = (uint8_t)fParametr;	break;	//Strojenie filtra wartości zadanej
	case STRP_KATA_PRZE_WYPRZ:	Konf[PID_KĄTA_PRZE].chProcWartZadWyprz = (uint8_t)fParametr;	break;	//strojenie wielkości akcji wyprzedzającej
	case STRP_PRED_PRZE_KP:		Konf[PID_PRED_PRZE].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze prędkości kątowej przechylenia
	case STRP_PRED_PRZE_TI:		Konf[PID_PRED_PRZE].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze prędkości kątowej przechylenia
	case STRP_PRED_PRZE_TD:		Konf[PID_PRED_PRZE].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze prędkości kątowej przechylenia
	case STRP_PRED_PRZE_FD:		sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_CZLONU_D)
			sParam = MAX_FILTR_CZLONU_D;
		Konf[PID_PRED_PRZE].chPodstFiltraD = (uint8_t)sParam;		break;//Strojenie filtra sygnału różniczkowanego

	case STRP_PRED_PRZE_FWZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_WART_ZAD)
			sParam = MAX_FILTR_WART_ZAD;
		Konf[PID_PRED_PRZE].chPodstFiltraWZad = sParam;		break;	//Strojenie filtra wartości zadanej

	case STRP_PRED_PRZE_WYPRZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_PROC_WYPRZEDZENIA)
			sParam = MAX_PROC_WYPRZEDZENIA;
		Konf[PID_PRED_PRZE].chProcWartZadWyprz = (uint8_t)sParam;	break;//strojenie wielkości akcji wyprzedzającej

	case STRP_KATA_POCH_KP:		Konf[PID_KĄTA_POCH].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze pochylenia
	case STRP_KATA_POCH_TI:		Konf[PID_KĄTA_POCH].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze pochylenia
	case STRP_KATA_POCH_TD:		Konf[PID_KĄTA_POCH].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze pochylenia
	case STRP_KATA_POCH_FD:		sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_CZLONU_D)
			sParam = MAX_FILTR_CZLONU_D;
		Konf[PID_KĄTA_POCH].chPodstFiltraD =(uint8_t) sParam;	break;	//Strojenie filtra sygnału różniczkowanego

	case STRP_KATA_POCH_FWZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_WART_ZAD)
			sParam = MAX_FILTR_WART_ZAD;
		Konf[PID_KĄTA_POCH].chPodstFiltraWZad = (uint8_t)sParam;	break;	//Strojenie filtra wartości zadanej

	case STRP_KATA_POCH_WYPRZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_PROC_WYPRZEDZENIA)
			sParam = MAX_PROC_WYPRZEDZENIA;
		Konf[PID_KĄTA_POCH].chProcWartZadWyprz = (uint8_t)sParam;	break;		//strojenie wielkości akcji wyprzedzającej

	case STRP_PRED_POCH_KP:		Konf[PID_PRED_POCH].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze prędkości kątowej pochylenia
	case STRP_PRED_POCH_TI:		Konf[PID_PRED_POCH].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze prędkości kątowej pochylenia
	case STRP_PRED_POCH_TD:		Konf[PID_PRED_POCH].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze prędkości kątowej pochylenia
	case STRP_PRED_POCH_FD:		sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_CZLONU_D)
			sParam = MAX_FILTR_CZLONU_D;
		Konf[PID_PRED_POCH].chPodstFiltraD = (uint8_t)sParam;	break;	//Strojenie filtra sygnału różniczkowanego

	case STRP_PRED_POCH_FWZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_WART_ZAD)
			sParam = MAX_FILTR_WART_ZAD;
		Konf[PID_PRED_POCH].chPodstFiltraWZad = (uint8_t)sParam;	break;		//Strojenie filtra wartości zadanej

	case STRP_PRED_POCH_WYPRZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_PROC_WYPRZEDZENIA)
			sParam = MAX_PROC_WYPRZEDZENIA;
		Konf[PID_PRED_POCH].chProcWartZadWyprz = (uint8_t)sParam;	break;	//strojenie wielkości akcji wyprzedzającej

	case STRP_KATA_ODCH_KP:		Konf[PID_KĄTA_ODCH].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze odchylenia
	case STRP_KATA_ODCH_TI:		Konf[PID_KĄTA_ODCH].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze odchylenia
	case STRP_KATA_ODCH_TD:		Konf[PID_KĄTA_ODCH].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze odchylenia
	case STRP_KATA_ODCH_FD:		sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_CZLONU_D)
			sParam = MAX_FILTR_CZLONU_D;
		Konf[PID_KĄTA_ODCH].chPodstFiltraD = (uint8_t)sParam;	break;	//Strojenie filtra sygnału różniczkowanego

	case STRP_KATA_ODCH_FWZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_WART_ZAD)
			sParam = MAX_FILTR_WART_ZAD;
		Konf[PID_KĄTA_ODCH].chPodstFiltraWZad = (uint8_t)sParam;	break;		//Strojenie filtra wartości zadanej

	case STRP_KATA_ODCH_WYPRZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_PROC_WYPRZEDZENIA)
			sParam = MAX_PROC_WYPRZEDZENIA;
		Konf[PID_KĄTA_ODCH].chProcWartZadWyprz = (uint8_t)sParam;	break;		//strojenie wielkości akcji wyprzedzającej

	case STRP_PRED_ODCH_KP:		Konf[PID_PRED_ODCH].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze prędkości kątowej odchylenia
	case STRP_PRED_ODCH_TI:		Konf[PID_PRED_ODCH].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze prędkości kątowej odchylenia
	case STRP_PRED_ODCH_TD:		Konf[PID_PRED_ODCH].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze prędkości kątowej odchylenia
	case STRP_PRED_ODCH_FD:		sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_CZLONU_D)
			sParam = MAX_FILTR_CZLONU_D;
		Konf[PID_PRED_ODCH].chPodstFiltraD = (uint8_t)sParam;	break;	//Strojenie filtra sygnału różniczkowanego

	case STRP_PRED_ODCH_FWZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_WART_ZAD)
			sParam = MAX_FILTR_WART_ZAD;
		Konf[PID_PRED_ODCH].chPodstFiltraWZad = (uint8_t)sParam;	break;		//Strojenie filtra wartości zadanej

	case STRP_PRED_ODCH_WYPRZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_PROC_WYPRZEDZENIA)
			sParam = MAX_PROC_WYPRZEDZENIA;
		Konf[PID_PRED_ODCH].chProcWartZadWyprz = (uint8_t)sParam;	break;	//strojenie wielkości akcji wyprzedzającej

	case STRP_WYSOKOSCI_KP:		Konf[PID_WYSOKOSCI].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze wysokości
	case STRP_WYSOKOSCI_TI:		Konf[PID_WYSOKOSCI].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze wysokości
	case STRP_WYSOKOSCI_TD:		Konf[PID_WYSOKOSCI].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze wysokości
	case STRP_WYSOKOSCI_FD:		sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_CZLONU_D)
			sParam = MAX_FILTR_CZLONU_D;
		Konf[PID_WYSOKOSCI].chPodstFiltraD = (uint8_t)sParam;	break;		//Strojenie filtra sygnału różniczkowanego

	case STRP_WYSOKOSCI_FWZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_WART_ZAD)
			sParam = MAX_FILTR_WART_ZAD;
		Konf[PID_WYSOKOSCI].chPodstFiltraWZad = (uint8_t)sParam;	break;		//Strojenie filtra wartości zadanej

	case STRP_WYSOKOSCI_WYPRZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_PROC_WYPRZEDZENIA)
			sParam = MAX_PROC_WYPRZEDZENIA;
		Konf[PID_WYSOKOSCI].chProcWartZadWyprz = (uint8_t)sParam;	break;	//strojenie wielkości akcji wyprzedzającej

	case STRP_PRED_ZWYS_KP:		Konf[PID_PRED_ZWYS].fWzmP = fParametr;	break;	//strojenie wzmocnienia w regulatorze prędkości zmiany wysokości
	case STRP_PRED_ZWYS_TI:		Konf[PID_PRED_ZWYS].fWzmI = fParametr;	break;	//strojenie członu całkujacego w regulatorze prędkości zmiany wysokości
	case STRP_PRED_ZWYS_TD:		Konf[PID_PRED_ZWYS].fWzmD = fParametr;	break;	//strojenie członu różniczkującego w regulatorze prędkości zmiany wysokości
	case STRP_PRED_ZWYS_FD:		sParam = (uint16_t)fParametr;
		if (sParam)
			sParam = MAX_FILTR_CZLONU_D;
		Konf[PID_PRED_ZWYS].chPodstFiltraD = (uint8_t)sParam;	break;	//Strojenie filtra sygnału różniczkowanego

	case STRP_PRED_ZWYS_FWZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_FILTR_WART_ZAD)
			sParam = MAX_FILTR_WART_ZAD;
		Konf[PID_PRED_ZWYS].chPodstFiltraWZad = (uint8_t)sParam;	break;	//Strojenie filtra wartości zadanej

	case STRP_PRED_ZWYS_WYPRZ:	sParam = (uint16_t)fParametr;
		if (sParam > MAX_PROC_WYPRZEDZENIA)
			sParam = MAX_PROC_WYPRZEDZENIA;
		Konf[PID_PRED_ZWYS].chProcWartZadWyprz = (uint8_t)sParam;	break;	//strojenie wielkości akcji wyprzedzającej

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



////////////////////////////////////////////////////////////////////////////////
// Funkcja liczy wartość parametru strojenia dla bieżącej wartosci kanału RC
// Parametry:
//   sWartośćKanałuRC - wartość wysterowania kanału regulującego strojenie
//   *Strój2 - wskaźnik na strukture strojenia PID
// Zwraca: wartość strojenia
////////////////////////////////////////////////////////////////////////////////
float ObliczWartośćParametruStrojenia(uint16_t sWartośćKanałuRC, stStrojPID_t *stStrój)
{
	float fKanalNorm = (float)sWartośćKanałuRC / (WE_RC_P100 - WE_RC_M100);	//wartość kanału RC znormalizowana do zakresu 0..1
	float fParametr = stStrój->fWartośćMin + fKanalNorm * (stStrój->fWartośćMax - stStrój->fWartośćMin);
	return fParametr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja uruchamiana po zakończeniu strojenia zapisuje nastawy do FRAM konfiguracji
// Parametry:
//   *Strój - wskaźnik na strukture strojenia PID kanałem RC
//   *Konf - wskaźnik na strukturę konfiguracji PID
//   *WymianaCM4 - wskaźnik na strukturę wymiany danych rdzenia CM4 zawierajacą dane autopilota
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszWartośćStrojeniaPID_KanałemRC(stStrojPID_t *stStrój, stKonfPID_t *Konf, stWymianyCM4_t *WymianaCM4)
{
	uint8_t cBłąd = BLAD_OK;
	float fParametr;
	uint16_t sAdres;

	fParametr = ObliczWartośćParametruStrojenia(WymianaCM4->sKanalRC[stStrój->chNrKanałuRC], stStrój);		//oblicz wartość parametru

	//oblicz adres parametru
	switch (stStrój->chNrParametru)
	{
	case STRP_NIC:				cBłąd = BLAD_NIC_DO_ROBOTY;	break;	//strojenie wyłączone
	case STRP_KATA_PRZE_KP:		sAdres = FAU_PID_KP + PID_KĄTA_PRZE * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze przechylenia
	case STRP_KATA_PRZE_TI:		sAdres = FAU_PID_TI + PID_KĄTA_PRZE * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze przechylenia
	case STRP_KATA_PRZE_TD:		sAdres = FAU_PID_TD + PID_KĄTA_PRZE * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze przechylenia
	case STRP_KATA_PRZE_FD:		sAdres = FAU_PID_FD + PID_KĄTA_PRZE * ROZMIAR_REG_PID;		break;	//Strojenie filtra sygnału różniczkowanego
	case STRP_KATA_PRZE_FWZ:	sAdres = FAU_PID_F_WZ + PID_KĄTA_PRZE * ROZMIAR_REG_PID;	break;	//Strojenie filtra wartości zadanej
	case STRP_KATA_PRZE_WYPRZ:	sAdres = FAU_PID_WYPRZ + PID_KĄTA_PRZE * ROZMIAR_REG_PID;	break;	//strojenie wielkości akcji wyprzedzającej
	case STRP_PRED_PRZE_KP:		sAdres = FAU_PID_KP + PID_PRED_PRZE * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze prędkości kątowej przechylenia
	case STRP_PRED_PRZE_TI:		sAdres = FAU_PID_TI + PID_PRED_PRZE * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze prędkości kątowej przechylenia
	case STRP_PRED_PRZE_TD:		sAdres = FAU_PID_TD + PID_PRED_PRZE * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze prędkości kątowej przechylenia
	case STRP_PRED_PRZE_FD:		sAdres = FAU_PID_FD + PID_PRED_PRZE * ROZMIAR_REG_PID;		break;	//Strojenie filtra sygnału różniczkowanego
	case STRP_PRED_PRZE_FWZ:	sAdres = FAU_PID_F_WZ + PID_PRED_PRZE * ROZMIAR_REG_PID;	break;	//Strojenie filtra wartości zadanej
	case STRP_PRED_PRZE_WYPRZ:	sAdres = FAU_PID_WYPRZ + PID_PRED_PRZE * ROZMIAR_REG_PID;	break;	//strojenie wielkości akcji wyprzedzającej

	case STRP_KATA_POCH_KP:		sAdres = FAU_PID_KP + PID_KĄTA_POCH * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze pochylenia
	case STRP_KATA_POCH_TI:		sAdres = FAU_PID_TI + PID_KĄTA_POCH * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze pochylenia
	case STRP_KATA_POCH_TD:		sAdres = FAU_PID_KP + PID_KĄTA_POCH * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze pochylenia
	case STRP_KATA_POCH_FD:		sAdres = FAU_PID_FD + PID_KĄTA_POCH * ROZMIAR_REG_PID;		break;	//Strojenie filtra sygnału różniczkowanego
	case STRP_KATA_POCH_FWZ:	sAdres = FAU_PID_F_WZ + PID_KĄTA_POCH * ROZMIAR_REG_PID;	break;	//Strojenie filtra wartości zadanej
	case STRP_KATA_POCH_WYPRZ:	sAdres = FAU_PID_WYPRZ + PID_KĄTA_POCH * ROZMIAR_REG_PID;	break;	//strojenie wielkości akcji wyprzedzającej
	case STRP_PRED_POCH_KP:		sAdres = FAU_PID_KP + PID_PRED_POCH * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze prędkości kątowej pochylenia
	case STRP_PRED_POCH_TI:		sAdres = FAU_PID_TI + PID_PRED_POCH * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze prędkości kątowej pochylenia
	case STRP_PRED_POCH_TD:		sAdres = FAU_PID_TD + PID_PRED_POCH * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze prędkości kątowej pochylenia
	case STRP_PRED_POCH_FD:		sAdres = FAU_PID_FD + PID_PRED_POCH * ROZMIAR_REG_PID;		break;	//Strojenie filtra sygnału różniczkowanego
	case STRP_PRED_POCH_FWZ:	sAdres = FAU_PID_F_WZ + PID_PRED_POCH * ROZMIAR_REG_PID;	break;	//Strojenie filtra wartości zadanej
	case STRP_PRED_POCH_WYPRZ:	sAdres = FAU_PID_WYPRZ + PID_PRED_POCH * ROZMIAR_REG_PID;	break;	//strojenie wielkości akcji wyprzedzającej

	case STRP_KATA_ODCH_KP:		sAdres = FAU_PID_KP + PID_KĄTA_ODCH * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze odchylenia
	case STRP_KATA_ODCH_TI:		sAdres = FAU_PID_TI + PID_KĄTA_ODCH * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze odchylenia
	case STRP_KATA_ODCH_TD:		sAdres = FAU_PID_TD + PID_KĄTA_ODCH * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze odchylenia
	case STRP_KATA_ODCH_FD:		sAdres = FAU_PID_FD + PID_KĄTA_ODCH * ROZMIAR_REG_PID;		break;	//Strojenie filtra sygnału różniczkowanego
	case STRP_KATA_ODCH_FWZ:	sAdres = FAU_PID_F_WZ + PID_KĄTA_ODCH * ROZMIAR_REG_PID;	break;	//Strojenie filtra wartości zadanej
	case STRP_KATA_ODCH_WYPRZ:	sAdres = FAU_PID_WYPRZ + PID_KĄTA_ODCH * ROZMIAR_REG_PID;	break;	//strojenie wielkości akcji wyprzedzającej
	case STRP_PRED_ODCH_KP:		sAdres = FAU_PID_KP + PID_PRED_ODCH * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze prędkości kątowej odchylenia
	case STRP_PRED_ODCH_TI:		sAdres = FAU_PID_TI + PID_PRED_ODCH * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze prędkości kątowej odchylenia
	case STRP_PRED_ODCH_TD:		sAdres = FAU_PID_TD + PID_PRED_ODCH * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze prędkości kątowej odchylenia
	case STRP_PRED_ODCH_FD:		sAdres = FAU_PID_FD + PID_PRED_ODCH * ROZMIAR_REG_PID;		break;	//Strojenie filtra sygnału różniczkowanego
	case STRP_PRED_ODCH_FWZ:	sAdres = FAU_PID_F_WZ + PID_PRED_ODCH * ROZMIAR_REG_PID;	break;	//Strojenie filtra wartości zadanej
	case STRP_PRED_ODCH_WYPRZ:	sAdres = FAU_PID_WYPRZ + PID_PRED_ODCH * ROZMIAR_REG_PID;	break;	//strojenie wielkości akcji wyprzedzającej

	case STRP_WYSOKOSCI_KP:		sAdres = FAU_PID_KP + PID_WYSOKOSCI * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze wysokości
	case STRP_WYSOKOSCI_TI:		sAdres = FAU_PID_TI + PID_WYSOKOSCI * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze wysokości
	case STRP_WYSOKOSCI_TD:		sAdres = FAU_PID_TD + PID_WYSOKOSCI * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze wysokości
	case STRP_WYSOKOSCI_FD:		sAdres = FAU_PID_FD + PID_WYSOKOSCI * ROZMIAR_REG_PID;		break;	//Strojenie filtra sygnału różniczkowanego
	case STRP_WYSOKOSCI_FWZ:	sAdres = FAU_PID_F_WZ + PID_WYSOKOSCI * ROZMIAR_REG_PID;	break;	//Strojenie filtra wartości zadanej
	case STRP_WYSOKOSCI_WYPRZ:	sAdres = FAU_PID_WYPRZ + PID_WYSOKOSCI * ROZMIAR_REG_PID;	break;	//strojenie wielkości akcji wyprzedzającej
	case STRP_PRED_ZWYS_KP:		sAdres = FAU_PID_KP + PID_PRED_ZWYS * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze prędkości zmiany wysokości
	case STRP_PRED_ZWYS_TI:		sAdres = FAU_PID_TI + PID_PRED_ZWYS * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze prędkości zmiany wysokości
	case STRP_PRED_ZWYS_TD:		sAdres = FAU_PID_TD + PID_PRED_ZWYS * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze prędkości zmiany wysokości
	case STRP_PRED_ZWYS_FD:		sAdres = FAU_PID_FD + PID_PRED_ZWYS * ROZMIAR_REG_PID;		break;	//Strojenie filtra sygnału różniczkowanego
	case STRP_PRED_ZWYS_FWZ:	sAdres = FAU_PID_F_WZ + PID_PRED_ZWYS * ROZMIAR_REG_PID;	break;	//Strojenie filtra wartości zadanej
	case STRP_PRED_ZWYS_WYPRZ:	sAdres = FAU_PID_WYPRZ + PID_PRED_ZWYS * ROZMIAR_REG_PID;	break;	//strojenie wielkości akcji wyprzedzającej

	case STRP_NAWI_PÓŁN_KP:		sAdres = FAU_PID_KP + PID_NAWIG_PÓŁN * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze nawigacji w kierunku północnym
	case STRP_NAWI_PÓŁN_TI:		sAdres = FAU_PID_TI + PID_NAWIG_PÓŁN * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze nawigacji w kierunku północnym
	case STRP_NAWI_PÓŁN_TD:		sAdres = FAU_PID_TD + PID_NAWIG_PÓŁN * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze nawigacji w kierunku północnym
	case STRP_PRED_PÓŁN_KP:		sAdres = FAU_PID_KP + PID_PRED_PÓŁN * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze prędkości w kierunku północnym
	case STRP_PRED_PÓŁN_TI:		sAdres = FAU_PID_KP + PID_PRED_PÓŁN * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze prędkości w kierunku północnym
	case STRP_PRED_PÓŁN_TD:		sAdres = FAU_PID_TD + PID_PRED_PÓŁN * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze prędkości w kierunku północnym
	case STRP_NAWI_WSCH_KP:		sAdres = FAU_PID_KP + PID_NAWIG_WSCH * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze nawigacji w kierunku wschodnim
	case STRP_NAWI_WSCH_TI:		sAdres = FAU_PID_KP + PID_NAWIG_WSCH * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze nawigacji w kierunku wschodnim
	case STRP_NAWI_WSCH_TD:		sAdres = FAU_PID_TD + PID_NAWIG_WSCH * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze nawigacji w kierunku wschodnim
	case STRP_PRED_WSCH_KP:		sAdres = FAU_PID_KP + PID_PRED_WSCH * ROZMIAR_REG_PID;		break;	//strojenie wzmocnienia w regulatorze prędkości w kierunku wschodnim
	case STRP_PRED_WSCH_TI:		sAdres = FAU_PID_TI + PID_PRED_WSCH * ROZMIAR_REG_PID;		break;	//strojenie członu całkujacego w regulatorze prędkości w kierunku wschodnim
	case STRP_PRED_WSCH_TD:		sAdres = FAU_PID_TD + PID_PRED_WSCH * ROZMIAR_REG_PID;		break;	//strojenie członu różniczkującego w regulatorze prędkości w kierunku wschodnim
	default:					cBłąd = BLAD_ZLE_DANE;	break;
	}

	ZapiszFramFloat(sAdres, fParametr);
	return cBłąd;
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
