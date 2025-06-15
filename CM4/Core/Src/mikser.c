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

stMikser_t stMikser;	//struktura zmiennych miksera
uint16_t sPWMMin;		//wysterowanie regulatorów pozwalajace na kręcenie silnikiem z minimalna prędkością
uint16_t sPWMJalowy;	//wysterowanie regulatorów pozwalajace na kręcenie silnikiem z prędkością jałową, odrobinę większą niż minimalna
uint16_t sPWMZawisu;	//wysterowanie regulatorów pozwalajace na zawis Wrona
uint16_t sPWMMax;		//wysterowanie regulatorów pozwalajace na kręcenie silnikiem z maksymalną prędkością. Dalsze zwiększanie wysterowania nic nie daje, więc w ten sposób wykluczamy go z zakresu regulacji

extern unia_wymianyCM4_t uDaneCM4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern volatile uint8_t chNumerKanSerw;




////////////////////////////////////////////////////////////////////////////////
// Funkcja wczytuje z FRAM konfigurację miksera
// Parametry: *mikser - wskaźnik na lokalną strukturę danych miksera
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujMikser(void)
//uint8_t InicjujMikser(stMikser_t* mikser)
{
	uint8_t chErr = ERR_OK;
	//włącz odbługę timerów do generowania sygnałów serw i dekodowania PPM z odbiorników RC
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);		//generowanie impulsów dla serw [8..15] 50Hz
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);		//generowanie impulsów dla serwo[2]
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);		//odczyt wejscia RC2
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3);		//generowanie impulsów dla serwo[3]
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_4);		//generowanie impulsów dla serwo[4]
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_4);		//generowanie impulsów dla serwo[0]
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);		//odczyt wejscia RC1
	HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);		//generowanie impulsów dla serwo[5]
	HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_3);		//generowanie impulsów dla serwo[7]

	//kanał 7 korzysta z wyjscia 3NE. Nie potrafię ustawić go z poziomu HAL-a, więc niech będzie z poziomu CMSIS
	htim8.Instance->CCER |= TIM_CCER_CC3NE;

	//przeładuj 32-bitowy rejestr compare wartością startową, bo pełen obrót timera to 2^32us = 1,2 godziny. Jeżeli nie będzie zainicjowany, to generowanie impulsów ruszy dopiero po tym czasie
	htim2.Instance->CCR1 += 1500;

	for (uint8_t n=0; n<KANALY_SERW; n++)
		uDaneCM4.dane.sSerwo[n] = 1000 + 50*n;	//sterowane kanałów serw
	chNumerKanSerw = 8;		//wskaźnik kanału obsługuje dekoder serw, wiec inicjuj go wartoscią pierwszego kanału

	for (uint8_t n=0; n<KANALY_MIKSERA; n++)
	{
		chErr |= CzytajFramZWalidacja(FAU_MIX_PRZECH + 2*n, stMikser.fPrze, VMIN_MIX_PRZE, VMAX_MIX_PRZE, VDEF_MIX_PRZE, ERR_NASTAWA_FRAM);	//8*4F współczynnik wpływu przechylenia na dany silnik
		chErr |= CzytajFramZWalidacja(FAU_MIX_POCHYL + 2*n, stMikser.fPoch, VMIN_MIX_PRZE, VMAX_MIX_PRZE, VDEF_MIX_PRZE, ERR_NASTAWA_FRAM);	//8*4F współczynnik wpływu pochylenia na dany silnik
		chErr |= CzytajFramZWalidacja(FAU_MIX_ODCHYL + 2*n, stMikser.fOdch, VMIN_MIX_PRZE, VMAX_MIX_PRZE, VDEF_MIX_PRZE, ERR_NASTAWA_FRAM);	//8*4F współczynnik wpływu odchylenia na dany silnik
	}

	sPWMMin 	= CzytajFramU16(FAU_PWM_MIN);      	//minimalne wysterowanie silników [us]
	sPWMJalowy 	= CzytajFramU16(FAU_PWM_JALOWY);    //wysterowanie silników dla biegu jałowego [us]
	sPWMZawisu 	= CzytajFramU16(FAU_WPM_ZAWISU);  	//wysterowanie silników dla zawisu [us]
	sPWMMax  	= CzytajFramU16(FAU_PWM_MAX);     	//maksymalne wysterowanie silników [us]
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia na sygnałach wychodzących z PID pochodnej. Formuje sygnałów dla silników
// Parametry: brak
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t LiczMikser(stMikser_t* mikser, stWymianyCM4_t *dane)
{
	int16_t sTmp1[KANALY_MIKSERA], sTmp2[KANALY_MIKSERA];	//sumy cząstkowe sygnału wysterowania silnika
	int16_t sTmpSerwo[KANALY_MIKSERA];
	int16_t sMaxTmp1, sMaxTmp2;	//maksymalne wartości sygnału wysterowania
	int16_t sPWMGazu;
	float fCosPrze, fCosPoch;
	uint8_t chErr = ERR_OK;


	//czy poziom gazu sugeruje że Wron jest w locie
	if ((dane->sKanalRC[KANRC_GAZ] > PPM_JALOWY) || (dane->chTrybLotu == TRLOT_LADOWANIE))
	{
		sPWMGazu = sPWMZawisu - sPWMJalowy;
		fCosPrze = cosf(dane->fKatIMU1[PRZE]);
		fCosPoch = cosf(dane->fKatIMU1[POCH]);
		//pionowa składowa ciągu statycznego ma być niezależna od pochylenia i przechylenia
		if ((dane->pid[PID_WYSO].chFlagi & PID_WLACZONY) && fCosPrze &&	fCosPoch)	//działa regulator wysokości i kosinusy kątów są niezerowe
			sPWMGazu /= (fCosPrze * fCosPoch); //skaluj tylko zakres roboczy
		sPWMGazu += sPWMJalowy;   //resztę "nieregulowalnego" gazu dodaj bez korekcji
	}
	else	//gaz zdjety
	{
		//jeżeli jesteśmy w trybie ręcznym zdjęcie gazu oznacza ląduj automatycznie
		if (dane->chTrybLotu == TRLOT_LOT_RECZNY)
		{
			dane->chTrybLotu = TRLOT_LADOWANIE;
		}
		else
			sPWMGazu = sPWMJalowy;
	}

	sMaxTmp1 = sMaxTmp2 = -200 * PPM1PROC_BIP; //inicjuj maksima wartością minimalną aby wyłapać wartości ponizej zera
	for (uint8_t n=0; n<mikser->chSilnikow; n++)
	{
		//w pierwszej kolejności sumuj pochylenie i przechylenie
		sTmp1[n] = (mikser->fPoch[n] * dane->pid[PID_PK_POCH].fWyjsciePID - mikser->fPrze[n] * dane->pid[PID_PK_PRZE].fWyjsciePID) * PPM1PROC_BIP;
		if (sTmp1[n] > sMaxTmp1)
			sMaxTmp1 = sTmp1[n];

		//osobno sumuj mniej ważne regulatory ciągu i odchylenia
		sTmp2[n] = (dane->pid[PID_WARIO].fWyjsciePID + mikser->fOdch[n] * dane->pid[PID_PK_ODCH].fWyjsciePID ) * PPM1PROC_BIP;
		if (sTmp2[n] > sMaxTmp2)
			sMaxTmp2 = sTmp2[n];
	}

	 //jeżeli suma kanałów jest większa niż 100% to najpierw skaluj ważniejsze regulatory odpowiadajace za stabilizację a jeżeli jeszcze jest miejsce do potem mniej ważne
	for (uint8_t n=0; n<mikser->chSilnikow; n++)
	{
		if ((sMaxTmp1 + sMaxTmp2 + sPWMGazu) < sPWMMax)     //jeżeli nie przekraczamy zakresu to sumuj wszystkie regulatory
			sTmpSerwo[n] = sTmp1[n] + sTmp2[n] + sPWMGazu;
		else
		{
			if ((sMaxTmp1 + sPWMGazu) < sPWMMax)
				sTmpSerwo[n] = sTmp1[n] + sPWMGazu + (sTmp2[n] * (sPWMMax - sMaxTmp1 - sPWMGazu) / sMaxTmp2);   //weź Tmp1 + gaz i doskaluj tyle Tmp2 ile jest miejsca
			else
			{
				if (sMaxTmp1 < sPWMMax)
					sTmpSerwo[n] = sTmp1[n] + (sPWMMax - sMaxTmp1);  //weź Tmp1 i doskaluj tyle gazu ile jest miejsca
				else
					sTmpSerwo[n] = sTmp1[n] * sPWMMax / sMaxTmp1; //jeżeli nie mieści sie w zakresie sterowania to przeskaluj
			}
		}

		//w locie nie schodź poniżej obrotów minimalnych
		if (dane->sKanalRC[KANRC_GAZ] > sPWMJalowy)
		{
			//dolny limit wysterowania w czasie lotu
			if (sTmpSerwo[n] < sPWMMin)
				sTmpSerwo[n] = sPWMMin;
		}
		else
		{
			if(dane->chTrybLotu & WL_TRWA_LOT)
			{
				//dolny limit wysterowania podczas lądowania
				if (sTmpSerwo[n] < sPWMMin)
					sTmpSerwo[n] = sPWMMin;
			}
			else
			{
				//dolny limit wysterowania przed lotem
				if (sTmpSerwo[n] < sPWMJalowy)
					sTmpSerwo[n] = sPWMJalowy;
			}
		}

		//jeżeli jesteśmy w jednym z trybów lotnych
		if (dane->chTrybLotu > TRLOT_UZBROJONY)
			dane->sSerwo[n] = sTmpSerwo[n];  //przepisz roboczą zmienną do zmiennej stanu serw
		else
			dane->sSerwo[n] = sPWMMin;   //wartość wyłączajaca silniki
	}
	return chErr;
}


