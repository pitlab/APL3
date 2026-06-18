//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł ubsługi miksera silników napędowych i obsługi serw
//
// (c) PitLab 2025
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <Mikser.h>
#include <Fram.h>
#include "RegulatorPID.h"
#include <DShot.h>

stMikser_t stMikser;	//struktura zmiennych miksera
uint16_t sWysterowanieMin;		//wartość wysterowania regulatorów dla uzyskania obrotów minimalnych w trakcie lotu i na ziemi po uzbrojeniu
uint16_t sWysterowanieMax;		//wartość wysterowania regulatorów dla uzyskania obrotów maksymalnych. Dalsze zwiększanie wysterowania nic nie daje, więc w ten sposób wykluczamy go z zakresu regulacji
uint16_t sWysterowanieZawisu;	//wartość wysterowania regulatorów dla uzyskania obrotów pozwalajacych na zawis
extern unia_wymianyCM4_t uDaneCM4;
extern volatile uint8_t chNumerKanSerw;
extern uint8_t chTrybRegulacji[LICZBA_DRAZKOW];	//rodzaj regulacji dla 4 podstawowych parametrów sterowanych z aparatury
extern uint8_t chKanalDrazkaRC[LICZBA_DRAZKOW];	//przypisanie kanałów odbiornika RC do funkcji drążków aparatury



////////////////////////////////////////////////////////////////////////////////
// Funkcja wczytuje z FRAM konfigurację miksera
// Parametry: *mikser - wskaźnik na lokalną strukturę danych miksera
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujMikser(void)
{
	uint8_t cBłąd = BLAD_OK;
	float fNormPrze, fNormPoch;

	for (uint8_t n=0; n<KANALY_WYJSC_RC; n++)
		uDaneCM4.dane.sSilnik[n] = 10 * n;	//sterowane kanałów serw
	chNumerKanSerw = 8;		//wskaźnik kanału obsługuje dekoder serw, wiec inicjuj go wartoscią pierwszego kanału
	stMikser.chLiczbaSilnikow = 0;
	fNormPrze = fNormPoch = 0.0f;

	for (uint8_t n=0; n<KANALY_MIKSERA; n++)
	{
		cBłąd |= CzytajFramFloatZWalidacja(FAU_MIX_PRZECH + 4*n, &stMikser.fPrze[n], VMIN_MIX_PRZEPOCH, VMAX_MIX_PRZEPOCH, VDOM_MIX_PRZEPOCH);	//8*4F składowa przechylenia długości ramienia koptera w mikserze [mm], max 1m
		cBłąd |= CzytajFramFloatZWalidacja(FAU_MIX_POCHYL + 4*n, &stMikser.fPoch[n], VMIN_MIX_PRZEPOCH, VMAX_MIX_PRZEPOCH, VDOM_MIX_PRZEPOCH);	//8*4F składowa pochylenia długości ramienia koptera w mikserze [mm], max 1m
		cBłąd |= CzytajFramFloatZWalidacja(FAU_MIX_ODCHYL + 4*n, &stMikser.fOdch[n], VMIN_MIX_ODCH, VMAX_MIX_ODCH, VDOM_MIX_ODCH);	//8*4F współczynnik wpływu kierunku obrotów silnika na odchylenie

		//suma kwadratów
		fNormPrze += stMikser.fPrze[n] * stMikser.fPrze[n];
		fNormPoch += stMikser.fPoch[n] * stMikser.fPoch[n];

		if ((stMikser.fPrze[n] > 1.0f) || (stMikser.fPrze[n] < -1.0f) || (stMikser.fPoch[n] > 1.0f) || (stMikser.fPoch[n] < -1.0f))
			stMikser.chLiczbaSilnikow++;
	}

	//normalizuj składowe pochylenia i przechylenia miksera aby pozbyć się długosci ramienia
	fNormPrze = sqrtf(fNormPrze / (stMikser.chLiczbaSilnikow / 2));
	fNormPoch = sqrtf(fNormPoch / (stMikser.chLiczbaSilnikow / 2));

	for (uint8_t n=0; n<KANALY_MIKSERA; n++)
	{
		if (fNormPrze)
			stMikser.fPrze[n] /= fNormPrze;
		else
			stMikser.fPrze[n] = 0;

		if (fNormPoch)
			stMikser.fPoch[n] /= fNormPoch;
		else
			stMikser.fPoch[n] = 0;
	}

	sWysterowanieMin 	= CzytajFramU16(FAU_RC_WY_MIN);      	//minimalne wysterowanie regulatorów w trakcie lotu w jednostkach standardowych 0-2000
	sWysterowanieMax  	= CzytajFramU16(FAU_RC_WY_MAX);     	//maksymalne wysterowanie silników w trakcie lotu w jednostkach standardowych 0-2000
	sWysterowanieZawisu = CzytajFramU16(FAU_RC_WY_ZAWISU);  	//wysterowanie silników dla zawisu
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia na sygnałach wychodzących z PID pochodnej. Formuje sygnałów dla silników
// Parametry:
//	*mikser - wskaźnik na strukturę konfiguracyjną miksera
//	*dane - wskaźnik na strukturę danych autopilota
//	*konfig - wskaźnik na strukturę konfiguracji regulatorów PID
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t LiczMikser(stMikser_t *mikser, stWymianyCM4_t *dane, stKonfPID_t *konfig)
{
	int16_t sTmp1[KANALY_MIKSERA], sTmp2[KANALY_MIKSERA];	//sumy cząstkowe sygnału wysterowania silnika
	int16_t sTmpSilnik[KANALY_MIKSERA];
	int16_t sMaxTmp1, sMaxTmp2;	//maksymalne wartości sygnału wysterowania
	int16_t sGaz;
	float fCosPrze, fCosPoch;
	uint8_t cBłąd = BLAD_OK;

	if(dane->chTrybLotu & BTR_UZBROJONY)
	{
		fCosPrze = cosf(dane->stBSP.fKatIMU[PRZE]);
		fCosPoch = cosf(dane->stBSP.fKatIMU[POCH]);

		//jeżeli drążek gazu podniesie się powyżej -90% to zaczynamy lot dodając wysterowanie dla zawisu
		if (dane->sKanalRC[chKanalDrazkaRC[WYSO]] < WE_RC_M90)
		{
			dane->chTrybLotu &= ~BTR_TRWA_LOT;
			sGaz = sWysterowanieMin;
		}
		else
		{
			dane->chTrybLotu |= BTR_TRWA_LOT;
			//sGaz = sWysterowanieZawisu;
			sGaz = dane->sKanalRC[chKanalDrazkaRC[WYSO]];

			//pionowa składowa ciągu statycznego potrzebnego do zawisu ma być niezależna od pochylenia i przechylenia
			if (fCosPrze != 0.0f)	//niezerowy kosinus kąta
				sGaz /= fCosPrze;

			if (fCosPoch != 0.0f)
				sGaz /= fCosPoch;
		}

		sMaxTmp1 = sMaxTmp2 = -100 * PPM1PROC_BIP; //inicjuj maksima wartością minimalną aby wyłapać wartości ponizej zera
		for (uint8_t n=0; n<stMikser.chLiczbaSilnikow; n++)
		{
			//w pierwszej kolejności sumuj pochylenie i przechylenie
			sTmp1[n] = (mikser->fPoch[n] * dane->stPID[PID_PRED_POCH].fWyjsciePID - mikser->fPrze[n] * dane->stPID[PID_PRED_PRZE].fWyjsciePID) * PPM1PROC_BIP;
			if (sTmp1[n] > sMaxTmp1)
				sMaxTmp1 = sTmp1[n];

			//osobno sumuj mniej ważne regulatory ciągu i odchylenia
			sTmp2[n] = (dane->stPID[PID_PRED_ZWYS].fWyjsciePID + mikser->fOdch[n] * dane->stPID[PID_PRED_ODCH].fWyjsciePID ) * PPM1PROC_BIP;
			if (sTmp2[n] > sMaxTmp2)
				sMaxTmp2 = sTmp2[n];
		}

		 //jeżeli suma kanałów jest większa niż 100% to najpierw skaluj ważniejsze regulatory odpowiadajace za stabilizację a jeżeli jeszcze jest miejsce do potem mniej ważne
		for (uint8_t n=0; n<stMikser.chLiczbaSilnikow; n++)
		{
			if ((sMaxTmp1 + sMaxTmp2 + sGaz) < sWysterowanieMax)     //jeżeli nie przekraczamy zakresu, to sumuj wszystkie regulatory
				sTmpSilnik[n] = sTmp1[n] + sTmp2[n] + sGaz;
			else
			{
				if ((sMaxTmp1 + sGaz) < sWysterowanieMax)
					sTmpSilnik[n] = sTmp1[n] + sGaz + (sTmp2[n] * (sWysterowanieMax - sMaxTmp1 - sGaz) / sMaxTmp2);   //weź Tmp1 + gaz i doskaluj tyle Tmp2 ile jest miejsca
				else
				{
					if (sMaxTmp1 < sWysterowanieMax)
						sTmpSilnik[n] = sTmp1[n] + (sWysterowanieMax - sMaxTmp1);  //weź Tmp1 i doskaluj tyle gazu ile jest miejsca
					else
						sTmpSilnik[n] = sTmp1[n] * sWysterowanieMax / sMaxTmp1; //jeżeli nie mieści sie w zakresie sterowania to przeskaluj
				}
			}

			//nie schodź poniżej obrotów jałowych
			if (sTmpSilnik[n] < sWysterowanieMin)
				sTmpSilnik[n] = sWysterowanieMin;

			if (sTmpSilnik[n] > sWysterowanieMax)	//tymczasowa pułapka
				sTmpSilnik[n] = sWysterowanieMax;
		}
	}
	else	//silniki nie są uzbrojone
	{
		for (uint8_t n=0; n<stMikser.chLiczbaSilnikow; n++)
			sTmpSilnik[n] = WE_RC_MIN;
		dane->cPolecenieDShot = DSHOT_CMD_MOTOR_STOP;
	}

	//przepisz roboczą zmienną do zmiennej stanu silników
	for (uint8_t n=0; n<stMikser.chLiczbaSilnikow; n++)
		dane->sSilnik[n] = sTmpSilnik[n];

	return cBłąd;
}


