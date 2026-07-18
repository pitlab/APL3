//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł kontroli lotu wielowirnikowca
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "KontrolerLotu.h"
#include "FRAM.h"
#include "RegulatorPID.h"


uint8_t cTrybRegulacji[LICZBA_REG_PARAM];	//rodzaj regulacji dla podstawowych parametrów
extern uint8_t cKanalDrazkaRC[LICZBA_DRAZKOW];	//przypisanie kanałów odbiornika RC do funkcji drążków aparatury
extern stStrojPID_t stStrojPID[LICZBA_KAN_RC_DO_STROJENIA_PID];
extern stKonfPID_t stKonfigPID[LICZBA_PID];
uint8_t cNumerSesji;

////////////////////////////////////////////////////////////////////////////////
// Wczytuje konfigurację kontrolera lotu
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujKontrolerLotu(void)
{
	uint8_t cBłąd = BLAD_OK;

	//odczytaj i aktualizuj numer sesji
	cNumerSesji = CzytajFRAM(FAS_NUMER_SESJI);
	cNumerSesji++;
	ZapiszFRAM(FAS_NUMER_SESJI, cNumerSesji);

	CzytajBuforFRAM(FAU_TRYB_REG, cTrybRegulacji, LICZBA_REG_PARAM);	//6*1U Tryb pracy regulatorów 4 podstawowych wartości przypisanych do drążków i 2 regulatorów pozycji N i E
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia stabilizacji lotu w różnych trybach pracy
// Parametry:
// [i] *cTrybRegulacji - wskaźnik na stopień automatyzacji procesu: REG_RECZNA, REG_AKRO, REG_STAB, REG_AUTO
// [i] ndT - czas od ostatniego cyklu [us]
// [i] *dane - wskaźnik na dane CM4 zawierajace m.inn. stan regulatorów PID
// [i] *konfig - wskaźnik na strukturę danych regulatorów PID
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KontrolerLotu(uint8_t *cTryb, uint32_t ndT, stWymianyCM4_t *dane, stKonfPID_t *konfig)
{
	float fZnormalizowaneRC, fPodstawa;
	uint8_t cIndeksPID_Kata, cIndeksPID_Predk;
	uint8_t cBłąd = BLAD_OK;

	//sprowadź wartość kanałów wejsciowych z drążków aparatury RC do znormalizowanej wartości symetrycznej wzgledem zera: +-NORMA_SYGNALU
	for (uint16_t n=0; n<LICZBA_DRAZKOW; n++)
	{
		fZnormalizowaneRC = (float)(dane->sKanalRC[cKanalDrazkaRC[n]] - WE_RC_NEUTR) / (WE_RC_MAX - WE_RC_NEUTR) * NORMA_SYGNALU;
		//filtruj wejście aby uzyskać płynność zmian sygnału RC między kolejnymi ramkami (50 Hz dla Sbus, 150 Hz dla Crossfire). Istotne dla akcji wyprzedzającej
		if (cTryb[n] >= REG_STAB)
			fPodstawa = (float)konfig[2*n+0].cPodstFiltraWZad;	//podstawa filtra dla regulatora podstawowego
		else
			fPodstawa = (float)konfig[2*n+1].cPodstFiltraWZad;	//podstawa filtra dla regulatora pochodnej

		dane->fFiltrowaneKanalyRC[n] = ((fPodstawa * dane->fFiltrowaneKanalyRC[n]) + fZnormalizowaneRC) / (fPodstawa + 1.0);
	}

	for (uint16_t n=0; n<LICZBA_REG_PARAM; n++)
	{
		cIndeksPID_Kata = 2*n + 0;	//indeks regulatora parametru głównego: kątów i wysokości
		cIndeksPID_Predk =  2*n + 1;	//indeks regulatora pochodnej: prędkosci kątowych i prędkosci zmiany wysokości

		if (cTryb[n] == REG_RECZNA)
		{
			//regulatory nie działają, wstaw dane z drążka
			switch(n)
			{
			case PRZE:
			case POCH:
			case ODCH:
			case WYSO:  dane->stPID[cIndeksPID_Predk].fWyjsciePID = dane->fFiltrowaneKanalyRC[n];	break;	//regulator sterowania prędkością kątową lub zmiany wysokości
			case POZN:	break;	//regulatory pozycji nie są ustawiane z drążka
			case POZE:	break;
			}
		}
		else	//regulatory działają
		{
			if (cTryb[n] == REG_WYLACZ)
				break;

			if (cTryb[n] == REG_AUTO)
			{
				//tutaj będzie obsługa regulatora nawigacyjnego produkującego wartości zadane dla regulatorów kątów oraz dla trybów automatycznego startu i lądowania generujacych wartosci zadane dla regulatora wysokości
			}

			if (cTryb[n] >= REG_STAB)		//czy ma pracować regulator parametru głównego
			{
				//ustaw wartości zadane dla regualtorów parametru głównego
				if (cTryb[n] == REG_STAB)
					//dane->stPID[cIndeksPID_Kata].fZadana = dane->fFiltrowaneKanalyRC[n] * konfig[cIndeksPID_Kata].fSkalaWartZadanej / NORMA_SYGNALU;	//wartością zadaną jest drążek aparatury
					dane->stPID[cIndeksPID_Kata].fZadana = (dane->fFiltrowaneKanalyRC[n] * konfig[cIndeksPID_Kata].fSkalaWartZadanej / NORMA_SYGNALU) + konfig[cIndeksPID_Kata].fPrzesunWartZadanej;	//wartością zadaną jest drążek aparatury
				else
				{
					if (n < POZN)	//tylko na przechylenia, pochylenia, odchylenia i wysokości
					{
						//wykonaj przeliczenie prędkości zmiany położenia na wartość zadaną
						dane->stPID[cIndeksPID_Kata].fZadana = 0;		//wartoscią zadaną jest wyjście PID nadrzędnego
					}
				}
				//ustaw parametr wejściowy
				switch(n)
				{
				case PRZE:
				case POCH:
				case ODCH:  dane->stPID[cIndeksPID_Kata].fWejscie = dane->stBSP.fKatIMU[n];		break;	//regulator sterowania kątami
				case WYSO:  dane->stPID[cIndeksPID_Kata].fWejscie = dane->stBSP.fWysokoscMSL;	break;	//regulator sterowania wysokością
				case POZN:	dane->stPID[cIndeksPID_Kata].fWejscie = (float)dane->stBSP.dSzerokoscGeo;	break;	//regulator sterowania zmianą położenia północnego
				case POZE:	dane->stPID[cIndeksPID_Kata].fWejscie = (float)dane->stBSP.dDlugoscGeo;	break;	//regulator sterowania zmianą położenia wschodniego
				}
				RegulatorPID(ndT, cIndeksPID_Kata, dane, konfig);	//licz regulator parametru głównego
			}

			if (cTryb[n] >= REG_AKRO)		//czy ma pracować regulator pochodnej parametru głównego
			{
				//ustaw wartości zadane dla regualtorów pochodnej
				if (cTryb[n] == REG_AKRO)
					dane->stPID[cIndeksPID_Predk].fZadana = dane->fFiltrowaneKanalyRC[n] * konfig[cIndeksPID_Predk].fSkalaWartZadanej / NORMA_SYGNALU;	//wartością zadaną jest drążek aparatury
				else
					dane->stPID[cIndeksPID_Predk].fZadana = dane->stPID[cIndeksPID_Kata].fWyjsciePID * konfig[cIndeksPID_Predk].fSkalaWartZadanej / NORMA_SYGNALU;	//wartoscią zadaną jest wyjście PID nadrzędnego

				//ustaw parametr wejściowy
				switch(n)
				{
				case PRZE:
				case POCH:
				case ODCH:  dane->stPID[cIndeksPID_Predk].fWejscie = dane->stBSP.fZyro[n];		break;	//regulator sterowania prędkościami kątowymi
				case WYSO:  dane->stPID[cIndeksPID_Predk].fWejscie = dane->stBSP.fPredkoscD;	break;	//regulator sterowania prędkością zmiany wysokości
				case POZN:	dane->stPID[cIndeksPID_Predk].fWejscie = dane->stBSP.fPredkoscN; 	break;	//regulator sterowania prędkością zmiany położenia północnego
				case POZE:	dane->stPID[cIndeksPID_Predk].fWejscie = dane->stBSP.fPredkoscE; 	break;	//regulator sterowania prędkością zmiany położenia wschodniego
				}
				RegulatorPID(ndT, cIndeksPID_Predk, dane, konfig); 	//licz regulator pochodnej
			}
		}
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Uzbraja silniki powodujac że zaczynają pracować na obrotach jałowych. Przed uzbrojeniem sprawdza poprawność pracy kluczowych czujników
// Parametry:
// [i] *dane - wskaźnik na dane CM4
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UzbrojSilniki(stWymianyCM4_t *daneCM4, stWymianyCM7_t *daneCM7)
{
	uint8_t cBłąd = BLAD_OK;
	daneCM4->cTrybLotu |= BTR_UZBROJONY;
	if (daneCM7->cPotwierdzenieWykonania != POL4_MOW_UZBROJONE)
		daneCM4->cWykonajPolecenie = POL4_MOW_UZBROJONE;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Rozbraja silniki
// [i] *dane - wskaźnik na dane CM4
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t RozbrojSilniki(stWymianyCM4_t *daneCM4, stWymianyCM7_t *daneCM7)
{
	uint8_t cBłąd = BLAD_OK;

	daneCM4->cTrybLotu &= ~BTR_UZBROJONY;
	if (daneCM7->cPotwierdzenieWykonania != POL4_MOW_ROZBROJONE)
		daneCM4->cWykonajPolecenie = POL4_MOW_ROZBROJONE;

	//Zapisz wartość strojenia kanałem RC jeżeli ta funkcja jest włączona
	cBłąd |= ZapiszWartośćStrojeniaPID_KanałemRC(&stStrojPID[0], stKonfigPID, daneCM4);
	cBłąd |= ZapiszWartośćStrojeniaPID_KanałemRC(&stStrojPID[1], stKonfigPID, daneCM4);
	return cBłąd;
}

