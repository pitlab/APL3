//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v2.0
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
uint8_t InicjujMikser(stMikser_t* mikser)
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
		chErr |= CzytajFramZWalidacja(FAU_MIX_PRZECH + 2*n, mikser->fPrze, VMIN_MIX_PRZE, VMAX_MIX_PRZE, VDEF_MIX_PRZE, ERR_NASTAWA_FRAM);	//8*4F współczynnik wpływu przechylenia na dany silnik
		chErr |= CzytajFramZWalidacja(FAU_MIX_POCHYL + 2*n, mikser->fPoch, VMIN_MIX_PRZE, VMAX_MIX_PRZE, VDEF_MIX_PRZE, ERR_NASTAWA_FRAM);	//8*4F współczynnik wpływu pochylenia na dany silnik
		chErr |= CzytajFramZWalidacja(FAU_MIX_ODCHYL + 2*n, mikser->fOdch, VMIN_MIX_PRZE, VMAX_MIX_PRZE, VDEF_MIX_PRZE, ERR_NASTAWA_FRAM);	//8*4F współczynnik wpływu odchylenia na dany silnik
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



/*///////////////////////////////////////////////////////////////////////////////
// mikser łączący wyjścia regulatorów w układ sterowania. obsługuje MIXER_CHAN kanałów
// Parametry: brak
// Zwraca: kod błędu
// Czas wykonania: 380us
////////////////////////////////////////////////////////////////////////////////
void Mixer(void)
{
    unsigned short sGas;    //wartość gazu do uzyskania zerowej pływalności
    unsigned char x;
    signed short sTmpServo[MIXER_CHAN];
    signed short sTmp1[MIXER_CHAN], sTmp2[MIXER_CHAN];
    signed short sMaxTmp1, sMaxTmp2;
    extern float fVectAngle[3];      //kąty finalnego wektora inercji: phi, theta, psi w [rad]
    extern float fUserSensVolt[];    //napięcie z czujnika prądu i napiecia

    //ogranicz wartości PID-ów wchodzących do miksera
    for (x=0; x<4; x++)
    {
        if (fPidOut[x] > 100.0)
            fPidOut[x] = 100.0;
        else
        if (fPidOut[x] < -100.0)
            fPidOut[x] = -100.0;
    }

    //raz obliczone kosinusy katów będą używane kilkukrotnie
    fCosPhi = cosf(fVectAngle[0]);
    fCosTheta = cosf(fVectAngle[1]);


    //wartość gazu
    if ((sLastServoInVal[2] > PPM_IDLE) || (chLandingMode == LANDING_MODE_ON))    //1200us, 3 ząbki poniżej minimum na aparaturze z 33 ząbkami na gazie
    {
        if ((sLastServoInVal[2] > PPM_IDLE) && (chEngineArmed))
        {
            chWasInFlight = 1;  //poziom gazu uruchomił regulatory - można lądować!
            chLandingMode = LANDING_MODE_OFF; // Jeśli lądowałeś to przerwij to i radośnie leć dalej!
        }

        //Uzależnij wartość gazu od spadku napięcia pakietu
        //Dla hexakoptera zasilanego 4x LiFe opracowałem równanie współczynnika [us/V]: y = 5,8 * dUwe + 28
        //Część stałą można dodać do ciagu statycznego, ograniczamy się do samego współczynika 5,8 [us/V]
        //sGas_dUcorr = 5.8 * (13.2 - fUmin); //fVoltDropCompensation

        //pionowa składowa ciągu statycznego ma być niezależna od pochylenia i przechylenia
        sGas = (sHoverPWM-sIdlePWM+sGas_dUcorr);
        if(chRegMode[ALTI] >= REG_STAB)
            sGas /= (fCosPhi * fCosTheta); //skaluj tylko zakres roboczy
        sGas += sIdlePWM;   //resztę "nieregulowalnego" gazu dodaj bez korekcji
    }
    else
    {
        if(chWasInFlight)
        {
            chLandingMode = LANDING_MODE_ON; // Ląduj - lądowanie rozbroi silniki
            StartLanding(); //inicjuj procedurę ladowania
        }
        else
        {
            sGas = sIdlePWM;
            ResetPIDintegrator();   //zerowanie całkowania w czasie gdy nie leci
            ResetGPSSetPoint();        //zerowanie wartości zadanej regulatorów pozycji
            fSetPoint[PID_PSI] = fVectAngle[2] * RAD2DEG; //zerowanie całkowania wartości zadanej w regulatorze odchylenia.
            fUmin = fUzas;          //inicjuj detektor minimalnego napięcia bez obciążenia silnikami
        }
    }

    sMaxTmp1 = sMaxTmp2 = -200*PPM1P_BIP; //inicjuj maksima wartością minimalną aby wyłapać wartości ponizej zera
    fMixerSum = 0.0;
    for (x=0; x<chSilnikow; x++)
    {
        //w pierwszej kolejności sumuj pochylenie i przechylenie
        sTmp1[x] = (fMixerPith[x]*fPidOut[PID_GYQ] - fMixerRoll[x]*fPidOut[PID_GYP] )*PPM1P_BIP;
        if (sTmp1[x] > sMaxTmp1)
            sMaxTmp1 = sTmp1[x];

        //osobno sumuj mniej ważne regulatory ciągu i odchylenia
        sTmp2[x] = (fPidOut[PID_VAR] + fMixerYaw[x]*fPidOut[PID_GYR] )*PPM1P_BIP;
        if (sTmp2[x] > sMaxTmp2)
            sMaxTmp2 = sTmp2[x];

        //sterowanie odchyleniem za: http://diydrones.com/profiles/blogs/stability-patch-for-motor-max-and-min-and-yaw-curve
        /*So what we want to do is increase the RPM of half the motors less than we decrease the RPM of the other half and keep it the same as when all motors are at the same RPM.
        Ok, some maths.
        x = RPM of all motors at stationary hover.
        y = RPM reduction of half the motors during Yaw.
        z = RPM increase of half the motors during Yaw.
        Total lifting force at hover for a quad is:
        Total Force = 4*x^2
        When inducing a Yaw it is:
        Total Force = 2*(x - y)^2 + 2*(x + z)^2
        We want these two equations to be equal if we don’t want the quad to start accelerating vertically therefore:
        4*x^2 = 2*(x - y)^2 + 2*(x + z)^2
        2*x^2 = (x - y)^2 + (x + z)^2
        2*x^2 = x^2 + y^2 -2*x*y + x^2 + z^2 + 2*x*z
        2*x^2 = 2*x^2 + y^2 -2*x*y + z^2 + 2*x*z
        0 = y^2 -2*x*y + z^2 + 2*x*z
        0 = z^2 + 2*x*z + y^2 -2*x*y
        if we solve this for z we get:
        z = -x + sqrt(x^2 – y^2 + 2*x*y)

        The current ratio line is the ratio we are currently considering using, ie 0.7 up and 1.42 down. As you can see, the ratio starts off 1:1 then reduces to sqrt(2)-1 or 0.414. We could get pretty close to the ideal line using a straight line approximation:
        z = 1 - (2-SQRT(2)) * x/y
        */
        /*fMixerSum += sTmp1[x] + sTmp2[x];
    }
    fMixerSum /= MIXER_CHAN;    //policz średnią wartość wysterowania silników [us]
    fMixerSum += sGas;

    //jeżeli suma kanałów jest większa niż 100% to najpierw skaluj ważniejsze regulatory odpowiadajace za stabilizację a jeżeli jeszcze jest miejsce do potem mniej ważne
    for (x=0; x<chSilnikow; x++)
    {
        if ((sMaxTmp1 + sMaxTmp2 + sGas) < sMaxPWM)     //jeżeli nie przekraczamy zakresu to sumuj wszystkie regulatory
            sTmpServo[x] = sTmp1[x] + sTmp2[x] + sGas;
        else
        {
            if ((sMaxTmp1 + sGas) < sMaxPWM)
                sTmpServo[x] = sTmp1[x] + sGas + (sTmp2[x] * (sMaxPWM - sMaxTmp1 - sGas) / sMaxTmp2);   //weź Tmp1 + gaz i doskaluj tyle Tmp2 ile jest miejsca
            else
            {
                if (sMaxTmp1 < sMaxPWM)
                    sTmpServo[x] = sTmp1[x] + (sMaxPWM - sMaxTmp1);  //weź Tmp1 i doskaluj tyle gazu ile jest miejsca
                else
                    sTmpServo[x] = sTmp1[x] * sMaxPWM / sMaxTmp1; //jeżeli nie mieści sie w zakresie sterowania to przeskaluj
            }
        }

        //w locie nie schodź poniżej obrotów minimalnych
        if (sLastServoInVal[2] > PPM_IDLE)
        {
            //dolny limit wysterowania w czasie lotu
            if (sTmpServo[x] < sMinPWM)
                sTmpServo[x] = sMinPWM;
        }
        else
        {
            if(chWasInFlight)
            {
                //dolny limit wysterowania podczas lądowania
                if (sTmpServo[x] < sMinPWM)
                    sTmpServo[x] = sMinPWM;
            }
            else
            {
                //dolny limit wysterowania przed lotem
                if (sTmpServo[x] < sIdlePWM)
                    sTmpServo[x] = sIdlePWM;
            }
        }

        //jeżeli silniki uzbrojone to włącz je
        if (chEngineArmed)
            sServoOutVal[x] = sTmpServo[x];  //przepisz roboczą zmienną do zmiennej stanu serw
        else
            sServoOutVal[x] = PPM_MINVAL;   //wartość wyłączajaca silniki
    }
}
*/
