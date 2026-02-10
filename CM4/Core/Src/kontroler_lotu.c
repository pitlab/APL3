//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł kontroli lotu wielowirnikowca
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "kontroler_lotu.h"
#include "konfig_fram.h"
#include "fram.h"
#include "pid.h"


uint8_t chTrybRegulacji[LICZBA_REG_PARAM];	//rodzaj regulacji dla podstawowych parametrów
extern uint8_t chKanalDrazkaRC[LICZBA_DRAZKOW];	//przypisanie kanałów odbiornika RC do funkcji drążków aparatury

////////////////////////////////////////////////////////////////////////////////
// Wczytuje konfigurację kontrolera lotu
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujKontrolerLotu(void)
{
	uint8_t chErr = BLAD_OK;

	CzytajBuforFRAM(FAU_TRYB_REG, chTrybRegulacji, LICZBA_REG_PARAM);	//6*1U Tryb pracy regulatorów 4 podstawowych wartości przypisanych do drążków i 2 regulatorów pozycji N i E
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia stabilizacji lotu w różnych trybach pracy
// Parametry:
// [i] *chTrybRegulacji - wskaźnik na stopień automatyzacji procesu: REG_RECZNA, REG_AKRO, REG_STAB, REG_AUTO
// [i] ndT - czas od ostatniego cyklu [us]
// [i] *dane - wskaźnik na dane CM4 zawierajace m.inn. stan regulatorów PID
// [i] *konfig - wskaźnik na strukturę danych regulatorów PID
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KontrolerLotu(uint8_t *chTrybRegulacji, uint32_t ndT, stWymianyCM4_t *dane, stKonfPID_t *konfig)
{
	float fWeRc[LICZBA_DRAZKOW];
	uint8_t chIndeksPID_Kata, chIndeksPID_Predk;
	uint8_t chErr = BLAD_OK;

	//sprowadź wartość kanałów wejsciowych z drążków aparatury RC do znormalizowanej wartości symetrycznej wzgledem zera: +-1.0
	for (uint16_t n=0; n<LICZBA_DRAZKOW; n++)
		fWeRc[n] = (float)(dane->sKanalRC[chKanalDrazkaRC[n]] - PPM_NEUTR) / (PPM_MAX - PPM_NEUTR);

	for (uint16_t n=0; n<LICZBA_REG_PARAM; n++)
	{
		chIndeksPID_Kata = 2*n + 0;	//indeks regulatora parametru głównego: kątów i wysokości
		chIndeksPID_Predk =  2*n + 1;	//indeks regulatora pochodnej: prędkosci kątowych i prędkosci zmiany wysokości

		if (chTrybRegulacji[n] == REG_RECZNA)
		{
			//regulatory nie działają, wstaw dane z drążka
			switch(n)
			{
			case PRZE:
			case POCH:
			case ODCH:
			case WYSO:  dane->stWyjPID[chIndeksPID_Predk].fWyjsciePID = fWeRc[n];	break;	//regulator sterowania prędkością zmiany wysokości
			case POZN:	break;	//regulatory pozycji nie są ustawiane z drążka
			case POZE:	break;
			}
		}
		else	//regulatory działają
		{
			if (chTrybRegulacji[n] == REG_WYLACZ)
				break;

			if (chTrybRegulacji[n] == REG_AUTO)
			{
				//tutaj będzie obsługa regulatora nawigacyjnego produkującego wartości zadane dla regulatorów kątów oraz dla trybów automatycznego startu i lądowania generujacych wartosci zadane dla regulatora wysokości

			}

			if (chTrybRegulacji[n] >= REG_STAB)		//czy ma pracować regulator parametru głównego
			{
				//ustaw wartości zadane dla regualtorów parametru głównego
				if (chTrybRegulacji[n] == REG_STAB)
					dane->stWyjPID[chIndeksPID_Kata].fZadana = fWeRc[n] * konfig[chIndeksPID_Kata].fSkalaWZadanej;	//wartością zadaną jest drążek aparatury
				else
				{
					if (n < POZN)	//tylko na przechylenia, pochylenia, odchylenia i wysokości
					{
						//wykonaj przeliczenie prędkości zmiany położenia na wartość zadaną
						dane->stWyjPID[chIndeksPID_Kata].fZadana = 0;		//wartoscią zadaną jest wyjście PID nadrzędnego
					}
				}
				//ustaw parametr wejściowy
				switch(n)
				{
				case PRZE:
				case POCH:
				case ODCH:  dane->stWyjPID[chIndeksPID_Kata].fWejscie = dane->fKatIMU1[n];		break;	//regulator sterowania kątami
				case WYSO:  dane->stWyjPID[chIndeksPID_Kata].fWejscie = dane->fWysokoMSL[0];	break;	//regulator sterowania wysokością
				case POZN:	dane->stWyjPID[chIndeksPID_Kata].fWejscie = (float)dane->stGnss1.dSzerokoscGeo;	break;	//regulator sterowania zmian położenia północnego
				case POZE:	dane->stWyjPID[chIndeksPID_Kata].fWejscie = (float)dane->stGnss1.dDlugoscGeo;	break;	//regulator sterowania zmianą położenia wschodniego
				}
				RegulatorPID(ndT, chIndeksPID_Kata, dane, konfig);	//licz regulator parametru głównego
			}

			if (chTrybRegulacji[n] >= REG_AKRO)		//czy ma pracować regulator pochodnej parametru głównego
			{
				//ustaw wartości zadane dla regualtorów pochodnej
				if (chTrybRegulacji[n] == REG_AKRO)
					dane->stWyjPID[chIndeksPID_Predk].fZadana = fWeRc[n] * konfig[chIndeksPID_Predk].fSkalaWZadanej;	//wartością zadaną jest drążek aparatury
				else
					dane->stWyjPID[chIndeksPID_Predk].fZadana = dane->stWyjPID[chIndeksPID_Kata].fWyjsciePID * konfig[chIndeksPID_Predk].fSkalaWZadanej;	//wartoscią zadaną jest wyjście PID nadrzędnego

				//ustaw parametr wejściowy
				switch(n)
				{
				case PRZE:
				case POCH:
				case ODCH:  dane->stWyjPID[chIndeksPID_Predk].fWejscie = dane->fKatZyro1[n];	break;	//regulator sterowania prędkościami kątowymi
				case WYSO:  dane->stWyjPID[chIndeksPID_Predk].fWejscie = dane->fWariometr[0];	break;	//regulator sterowania prędkością zmiany wysokości
				case POZN:	dane->stWyjPID[chIndeksPID_Predk].fWejscie = dane->stGnss1.fPredkoscN; break;	//regulator sterowania prędkością zmiany położenia północnego
				case POZE:	dane->stWyjPID[chIndeksPID_Predk].fWejscie = dane->stGnss1.fPredkoscE; break;	//regulator sterowania prędkością zmiany położenia wschodniego
				}
				RegulatorPID(ndT, chIndeksPID_Predk, dane, konfig); 	//licz regulator pochodnej parametru głównego
			}
		}
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Uzbraja silniki powodujac że zaczynają pracować na obrotach jałowych. Przed uzbrojeniem sprawdza poprawność pracy kluczowych czujników
// Parametry:
// [i] *dane - wskaźnik na dane CM4
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UzbrojSilniki(stWymianyCM4_t *dane)
{
	uint8_t chBlad = BLAD_OK;
	dane->chTrybLotu |= BTR_UZBROJONY;
	return chBlad;
}



////////////////////////////////////////////////////////////////////////////////
// Rozbraja silniki
// [i] *dane - wskaźnik na dane CM4
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RozbrojSilniki(stWymianyCM4_t *dane)
{
	dane->chTrybLotu &= ~BTR_UZBROJONY;
}

