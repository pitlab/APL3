//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł ubsługi miksera silników napędowych i obsługi serw
//
// (c) PitLab 2025
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "mikser.h"
#include "konfig_fram.h"
#include "fram.h"
#include "pid.h"

stMikser_t stMikser[KANALY_MIKSERA];	//struktura zmiennych miksera
uint16_t sWysterowanieJalowe;	//wartość wysterowania regulatorów dla uzyskania obrotów jałowych
uint16_t sWysterowanieMin;		//wartość wysterowania regulatorów dla uzyskania obrotów minimalnych w trakcie lotu
uint16_t sWysterowanieZawisu;	//wartość wysterowania regulatorów dla uzyskania obrotów pozwalajacych na zawis
uint16_t sWysterowanieMax;		//wartość wysterowania regulatorów dla uzyskania obrotów maksymalnych. Dalsze zwiększanie wysterowania nic nie daje, więc w ten sposób wykluczamy go z zakresu regulacji


extern unia_wymianyCM4_t uDaneCM4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
//extern TIM_HandleTypeDef htim8;
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
//uint8_t InicjujMikser(stMikser_t* mikser)
{
	uint8_t chErr = BLAD_OK;
	uint8_t chLiczbaSilnikow;
	float fNormPrze, fNormPoch;

	/*/włącz odbługę timerów do generowania sygnałów serw i dekodowania PPM z odbiorników RC
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);		//generowanie impulsów dla serw [8..15] 50Hz
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);		//generowanie impulsów dla serwo[2]
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);		//odczyt wejscia RC2
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3);		//generowanie impulsów dla serwo[3]
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_4);		//generowanie impulsów dla serwo[4]
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_4);		//generowanie impulsów dla serwo[0]
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);		//odczyt wejscia RC1
	//HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);		//generowanie impulsów dla serwo[5]
	//HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_3);		//generowanie impulsów dla serwo[7]

	//kanał 7 korzysta z wyjscia 3NE. Nie potrafię ustawić go z poziomu HAL-a, więc niech będzie z poziomu CMSIS
	//htim8.Instance->CCER |= TIM_CCER_CC3NE; */

	//przeładuj 32-bitowy rejestr compare wartością startową, bo pełen obrót timera to 2^32us = 1,2 godziny. Jeżeli nie będzie zainicjowany, to generowanie impulsów ruszy dopiero po tym czasie
	//htim2.Instance->CCR1 += 1500;

	for (uint8_t n=0; n<KANALY_SERW; n++)
		uDaneCM4.dane.sSerwo[n] = 1000 + 50*n;	//sterowane kanałów serw
	chNumerKanSerw = 8;		//wskaźnik kanału obsługuje dekoder serw, wiec inicjuj go wartoscią pierwszego kanału
	chLiczbaSilnikow = 0;
	fNormPrze = fNormPoch = 0.0f;

	for (uint8_t n=0; n<KANALY_MIKSERA; n++)
	{
		chErr |= CzytajFramZWalidacja(FAU_MIX_PRZECH + 4*n, &stMikser[n].fPrze, VMIN_MIX_PRZEPOCH, VMAX_MIX_PRZEPOCH, VDOM_MIX_PRZEPOCH, ERR_NASTAWA_FRAM);	//8*4F składowa przechylenia długości ramienia koptera w mikserze [mm], max 1m
		chErr |= CzytajFramZWalidacja(FAU_MIX_POCHYL + 4*n, &stMikser[n].fPoch, VMIN_MIX_PRZEPOCH, VMAX_MIX_PRZEPOCH, VDOM_MIX_PRZEPOCH, ERR_NASTAWA_FRAM);	//8*4F składowa pochylenia długości ramienia koptera w mikserze [mm], max 1m
		chErr |= CzytajFramZWalidacja(FAU_MIX_ODCHYL + 4*n, &stMikser[n].fOdch, VMIN_MIX_ODCH, VMAX_MIX_ODCH, VDOM_MIX_ODCH, ERR_NASTAWA_FRAM);	//8*4F współczynnik wpływu kierunku obrotów silnika na odchylenie

		//suma kwadratów
		fNormPrze += stMikser[n].fPrze * stMikser[n].fPrze;
		fNormPoch += stMikser[n].fPoch * stMikser[n].fPoch;

		if ((stMikser[n].fPrze > 1.0f) || (stMikser[n].fPrze < -1.0f) || (stMikser[n].fPoch > 1.0f) || (stMikser[n].fPoch < -1.0f))
			chLiczbaSilnikow++;
	}

	//normalizuj składowe pochylenia i przechylenia miksera aby pozbyć się długosci ramienia
	fNormPrze = sqrtf(fNormPrze / (chLiczbaSilnikow / 2));
	fNormPoch = sqrtf(fNormPoch / (chLiczbaSilnikow / 2));

	for (uint8_t n=0; n<KANALY_MIKSERA; n++)
	{
		if (fNormPrze)
			stMikser[n].fPrze /= fNormPrze;
		else
			stMikser[n].fPrze = 0;

		if (fNormPoch)
			stMikser[n].fPoch /= fNormPoch;
		else
			stMikser[n].fPoch = 0;
	}

	sWysterowanieMin 	= CzytajFramU16(FAU_PWM_MIN);      	//minimalne wysterowanie silników [us]
	sWysterowanieJalowe = CzytajFramU16(FAU_PWM_JALOWY);    //wysterowanie silników dla biegu jałowego [us]
	sWysterowanieZawisu = CzytajFramU16(FAU_WPM_ZAWISU);  	//wysterowanie silników dla zawisu [us]
	sWysterowanieMax  	= CzytajFramU16(FAU_PWM_MAX);     	//maksymalne wysterowanie silników [us]
	return chErr;
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
	int16_t sTmpSerwo[KANALY_MIKSERA];
	int16_t sMaxTmp1, sMaxTmp2;	//maksymalne wartości sygnału wysterowania
	int16_t sPWMGazu;
	float fCosPrze, fCosPoch;
	uint8_t chErr = BLAD_OK;

	if(dane->chTrybLotu & BTR_UZBROJONY)
	{
		//czy poziom gazu sugeruje że Wron jest w locie
		if ((dane->sKanalRC[chKanalDrazkaRC[WYSO]] > PPM_JALOWY) || (dane->chTrybLotu == TL_LAD_AUTO) || (dane->chTrybLotu == TL_LAD_STAB))
		{
			sPWMGazu = sWysterowanieZawisu - sWysterowanieJalowe;
			fCosPrze = cosf(dane->fKatIMU1[PRZE]);
			fCosPoch = cosf(dane->fKatIMU1[POCH]);
			//pionowa składowa ciągu statycznego ma być niezależna od pochylenia i przechylenia
			if (fCosPrze &&	fCosPoch)	//kosinusy kątów są niezerowe
				sPWMGazu /= (fCosPrze * fCosPoch); //skaluj tylko zakres roboczy
			sPWMGazu += sWysterowanieJalowe;   //resztę "nieregulowalnego" gazu dodaj bez korekcji
		}
		else	//gaz zdjęty i nie jest to lądowanie
		{
				sPWMGazu = sWysterowanieJalowe;
		}


		sMaxTmp1 = sMaxTmp2 = -200 * PPM1PROC_BIP; //inicjuj maksima wartością minimalną aby wyłapać wartości ponizej zera
		for (uint8_t n=0; n<KANALY_MIKSERA; n++)
		{
			//w pierwszej kolejności sumuj pochylenie i przechylenie
			sTmp1[n] = (mikser + n)->fPoch * dane->stWyjPID[PID_PK_POCH].fWyjsciePID - (mikser + n)->fPrze * dane->stWyjPID[PID_PK_PRZE].fWyjsciePID * PPM1PROC_BIP;
			if (sTmp1[n] > sMaxTmp1)
				sMaxTmp1 = sTmp1[n];

			//osobno sumuj mniej ważne regulatory ciągu i odchylenia
			sTmp2[n] = (dane->stWyjPID[PID_WARIO].fWyjsciePID + (mikser + n)->fOdch * dane->stWyjPID[PID_PK_ODCH].fWyjsciePID ) * PPM1PROC_BIP;
			if (sTmp2[n] > sMaxTmp2)
				sMaxTmp2 = sTmp2[n];
		}

		 //jeżeli suma kanałów jest większa niż 100% to najpierw skaluj ważniejsze regulatory odpowiadajace za stabilizację a jeżeli jeszcze jest miejsce do potem mniej ważne
		for (uint8_t n=0; n<KANALY_MIKSERA; n++)
		{
			if ((sMaxTmp1 + sMaxTmp2 + sPWMGazu) < sWysterowanieMax)     //jeżeli nie przekraczamy zakresu to sumuj wszystkie regulatory
				sTmpSerwo[n] = sTmp1[n] + sTmp2[n] + sPWMGazu;
			else
			{
				if ((sMaxTmp1 + sPWMGazu) < sWysterowanieMax)
					sTmpSerwo[n] = sTmp1[n] + sPWMGazu + (sTmp2[n] * (sWysterowanieMax - sMaxTmp1 - sPWMGazu) / sMaxTmp2);   //weź Tmp1 + gaz i doskaluj tyle Tmp2 ile jest miejsca
				else
				{
					if (sMaxTmp1 < sWysterowanieMax)
						sTmpSerwo[n] = sTmp1[n] + (sWysterowanieMax - sMaxTmp1);  //weź Tmp1 i doskaluj tyle gazu ile jest miejsca
					else
						sTmpSerwo[n] = sTmp1[n] * sWysterowanieMax / sMaxTmp1; //jeżeli nie mieści sie w zakresie sterowania to przeskaluj
				}
			}

			//nie schodź poniżej obrotów jałowych
			if (sTmpSerwo[n] < sWysterowanieJalowe)
				sTmpSerwo[n] = sWysterowanieJalowe;
		}
	}
	else	//silniki nie są uzbrojone
	{
		for (uint8_t n=0; n<KANALY_MIKSERA; n++)
			sTmpSerwo[n] = PPM_MIN;
	}

	//przepisz roboczą zmienną do zmiennej stanu serw
	for (uint8_t n=0; n<KANALY_MIKSERA; n++)
		dane->sSerwo[n] = sTmpSerwo[n];

	return chErr;
}


