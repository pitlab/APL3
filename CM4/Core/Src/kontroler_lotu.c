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

float fSkalaWartosciZadanejAkro[ROZMIAR_DRAZKOW];	//wartość zadana dla pełnego wychylenia drążka aparatury w trybie AKRO
float fSkalaWartosciZadanejStab[ROZMIAR_DRAZKOW];	//wartość zadana dla pełnego wychylenia drążka aparatury w trybie STAB
uint16_t sWysterowanieJalowe;	//wartość wysterowania regulatorów dla uzyskania obrotów jałowych
uint16_t sWysterowanieMin;		//wartość wysterowania regulatorów dla uzyskania obrotów minimalnych w trakcie lotu
uint16_t sWysterowanieZawisu;	//wartość wysterowania regulatorów dla uzyskania obrotów pozwalajacych na zawis
uint16_t sWysterowanieMax;		//wartość wysterowania regulatorów dla uzyskania obrotów maksymalnych
uint8_t chTrybRegulatora[ROZMIAR_DRAZKOW];	//rodzaj regulacji dla 4 podstawowych parametrów sterowanych z aparatury

////////////////////////////////////////////////////////////////////////////////
// Wczytuje konfigurację kontrolera lotu
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujKontrolerLotu(void)
{
	uint8_t chErr = BLAD_OK;

	CzytajBuforFRAM(FA_TRYB_REG, chTrybRegulatora, ROZMIAR_DRAZKOW);	//4*1U Tryb pracy regulatorów 4 podstawowych wartości przypisanych do drążków

    for (uint16_t n=0; n<ROZMIAR_DRAZKOW; n++)
    {
        //odczytaj wartość skalowania wartości zadanej dla trybów STAB i AKRO
        chErr |= CzytajFramZWalidacja(FAU_ZADANA_AKRO + n*4, &fSkalaWartosciZadanejAkro[n], VMIN_PID_WZMP, VMAX_PID_WZMP, VDEF_PID_WZMP, ERR_NASTAWA_FRAM);	//4x4F wartość zadana z drążków aparatury dla regulatora Akro
        chErr |= CzytajFramZWalidacja(FAU_ZADANA_STAB + n*4, &fSkalaWartosciZadanejStab[n], VMIN_PID_WZMP, VMAX_PID_WZMP, VDEF_PID_WZMP, ERR_NASTAWA_FRAM);	//4x4F wartość zadana z drążków aparatury dla regulatora Stab
        chTrybRegulatora[n] = REG_STAB;	//na razie przypisz na sztywno
    }


    sWysterowanieJalowe = CzytajFramU16(FAU_PWM_JALOWY);	//2U wysterowanie regulatorów na biegu jałowym [us]
    sWysterowanieMin =  CzytajFramU16(FAU_PWM_MIN);			//2U minimalne wysterowanie regulatorów w trakcie lotu [us]
    sWysterowanieZawisu = CzytajFramU16(FAU_WPM_ZAWISU);	//2U wysterowanie regulatorów w zawisie [us]
    sWysterowanieMax = CzytajFramU16(FAU_PWM_MAX);			//2U maksymalne wysterowanie silników w trakcie lotu [us]

return chErr;
}





////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia stabilizacji lotu w różnych trybach pracy
// Parametry:
// [i] *chTrybRegulatora - wskaźnik na stopień automatyzacji procesu: REG_RECZNA, REG_AKRO, REG_STAB, REG_AUTO
// [i] ndT - czas od ostatniego cyklu [us]
// [i] *konfig - wskaźnik na strukturę danych regulatorów PID
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KontrolerLotu(uint8_t *chTrybRegulatora, uint32_t ndT, stWymianyCM4_t *dane, stKonfPID_t *konfig)
{
	float fWeRc[ROZMIAR_DRAZKOW];
	uint8_t chIndeksPID_Kata, chIndeksPID_Predk;
	uint8_t chErr = BLAD_OK;

	for (uint16_t n=0; n<ROZMIAR_DRAZKOW; n++)
	{
		//sprowadź wartość kanałów wejsciowych z drążków aparatury RC do znormalizowanej wartości symetrycznej wzgledem zera: +-100
		fWeRc[n] = (float)(dane->sKanalRC[n] - PPM_NEUTR) / (PPM_MAX - PPM_NEUTR) * 100.0f;

		chIndeksPID_Kata = 2*n + 0;	//indeks regulatora parametru głównego: kątów i wysokości
		chIndeksPID_Predk =  2*n + 1;	//indeks regulatora pochodnej: prędkosci kątowych i prędkosci zmiany wysokości

		if (chTrybRegulatora[n] > REG_WYLACZ)
		{
			if (chTrybRegulatora[n] == REG_RECZNA)
				dane->stWyjPID[chIndeksPID_Predk].fWyjsciePID = fWeRc[n];	//regulatory nie działają, wstaw dane z drążka
			else	//regulatory działają
			{
				//określ czy regulator kata przechylenia ma pracować
				if (chTrybRegulatora[n] > REG_AKRO)
				{

					//określ co ma być wartością zadaną dla regulatora kąta: regulatory nawigacji czy drążek RC
					if (chTrybRegulatora[n] == REG_AUTO)
					{
						//tutaj będzie obsługa regulatora nawigacyjnego
						//dane->stWyjPID[chIndeksPID_Predk].fZadana = składowa przechylenia z regulatora prędkosci zmiany pozycji geograficznej
					}
					else
					{
						dane->stWyjPID[chIndeksPID_Kata].fZadana = fSkalaWartosciZadanejStab[n] * fWeRc[n];
					}

					//regulator kąta
					dane->stWyjPID[chIndeksPID_Kata].fWejscie = dane->fKatIMU1[n];
					RegulatorPID(ndT, chIndeksPID_Kata, dane, konfig);
					dane->stWyjPID[chIndeksPID_Predk].fZadana = dane->stWyjPID[chIndeksPID_Kata].fWyjsciePID;
				}
				else
				{
					//regulatory nawigacyjne i stabilizacyjny nie działają, jesteśmy w trybie akrobacyjnym, wartość zadana prędkości kątowej pochodzi z drążka
					dane->stWyjPID[chIndeksPID_Predk].fZadana = fSkalaWartosciZadanejAkro[n] * fWeRc[n];
				}

				//regulator prędkosci kątowej
				dane->stWyjPID[chIndeksPID_Predk].fWejscie = dane->fZyroKal1[n];
				RegulatorPID(ndT, chIndeksPID_Predk, dane, konfig);
			}
		}

	}

	/*/regulacja przechylenia
	if (chTrybRegulatora[PRZE] > REG_WYLACZ)
	{
		if (chTrybRegulatora[PRZE] == REG_RECZNY)
			dane->stWyjPID[PID_PK_PRZE].fWyjsciePID = fWeRc[PRZE];	//regulatory nie działają, wstaw dane z drążka
		else	//regulatory działają
		{
			//określ czy regulator kata przechylenia ma pracować
			if (chTrybRegulatora[PRZE] > REG_AKRO)
			{

				//określ co ma być wartością zadaną dla regulatora kąta: regulatory nawigacji czy drążek RC
				if (chTrybRegulatora[PRZE] == REG_AUTO)
				{
					//tutaj będzie obsługa regulatora nawigacyjnego
					//dane->stWyjPID[PID_PRZE].fZadana = składowa przechylenia z regulatora prędkosci zmiany pozycji geograficznej
				}
				else
				{
					dane->stWyjPID[PID_PRZE].fZadana = fWeRc[PRZE];
				}

				//regulator kąta
				dane->stWyjPID[PID_PRZE].fWejscie = dane->fKatIMU1[PRZE];
				RegulatorPID(ndT, PID_PRZE, dane, konfig);
				dane->stWyjPID[PID_PK_PRZE].fZadana = dane->stWyjPID[PID_PRZE].fWyjsciePID;
			}
			else
			{
				//regulatory nawigacyjne i stabilizacyjny nie działają, jesteśmy w trybie akrobacyjnym, wartość zadana prędkości kątowej pochodzi z drążka
				dane->stWyjPID[PID_PK_PRZE].fZadana = fWeRc[PRZE];
			}

			//regulator prędkosci kątowej
			dane->stWyjPID[PID_PK_PRZE].fWejscie = dane->fZyroKal1[PRZE];
			RegulatorPID(ndT, PID_PK_PRZE, dane, konfig);
		}
	}

	//regulacja pochylenia
	if (chTrybRegulatora[POCH] > REG_WYLACZ)
	{
		if (chTrybRegulatora[POCH] == REG_RECZNY)
			dane->stWyjPID[PID_PK_POCH].fWyjsciePID = fWeRc[POCH];	//regulatory nie działają, wstaw dane z drążka
		else	//regulatory działają
		{
			//określ czy regulator kata pochylenia ma pracować
			if (chTrybRegulatora[POCH] > REG_AKRO)
			{
				//określ co ma być wartością zadaną dla regulatora kąta: regulatory nawigacji czy drążek RC
				if (chTrybRegulatora[POCH] == REG_AUTO)
				{
					//tutaj będzie obsługa regulatora nawigacyjnego
					//dane->stWyjPID[PID_PK_POCH].fZadana = składowa pochylenia z regulatora prędkosci zmiany pozycji geograficznej
				}
				else
				{
					dane->stWyjPID[PID_POCH].fZadana = fWeRc[POCH];
				}

				//regulator kąta
				dane->stWyjPID[PID_POCH].fWejscie = dane->fKatIMU1[POCH];
				RegulatorPID(ndT, PID_POCH, dane, konfig);
				dane->stWyjPID[PID_PK_POCH].fZadana = dane->stWyjPID[PID_POCH].fWyjsciePID;
			}
			else
			{
				//regulatory nawigacyjne i stabilizacyjny nie działają, jesteśmy w trybie akrobacyjnym, wartość zadana prędkości kątowej pochodzi z drążka
				dane->stWyjPID[PID_PK_POCH].fZadana = fWeRc[PID_PRZE];
			}

			//regulator prędkosci kątowej
			dane->stWyjPID[PID_PK_POCH].fWejscie = dane->fZyroKal1[POCH];
			RegulatorPID(ndT, PID_PK_POCH, dane, konfig);
		}
	}

	//regulacja odchylenia
	if (chTrybRegulatora[ODCH] > REG_WYLACZ)
	{
		if (chTrybRegulatora[ODCH] == REG_RECZNY)
			dane->stWyjPID[PID_PK_ODCH].fWyjsciePID = fWeRc[ODCH];	//regulatory nie działają, wstaw dane z drążka
		else	//regulatory działają
		{
			//określ czy regulator kata odchylenia ma pracować
			if (chTrybRegulatora[ODCH] > REG_AKRO)
			{
				//określ co ma być wartością zadaną dla regulatora kąta: regulatory nawigacji czy drążek RC
				if (chTrybRegulatora[ODCH] == REG_AUTO)
				{
					//tutaj będzie obsługa regulatora nawigacyjnego
					//dane->stWyjPID[PID_PK_ODCH].fZadana = składowa odchylenia z regulatora prędkosci zmiany pozycji geograficznej
				}
				else
				{
					dane->stWyjPID[PID_ODCH].fZadana = fWeRc[ODCH];
				}

				//regulator kąta
				dane->stWyjPID[PID_ODCH].fWejscie = dane->fKatIMU1[ODCH];
				RegulatorPID(ndT, PID_ODCH, dane, konfig);
				dane->stWyjPID[PID_PK_ODCH].fZadana = dane->stWyjPID[PID_ODCH].fWyjsciePID;
			}
			else
			{
				//regulatory nawigacyjne i stabilizacyjny nie działają, jesteśmy w trybie akrobacyjnym, wartość zadana prędkości kątowej pochodzi z drążka
				dane->stWyjPID[PID_PK_ODCH].fZadana = fWeRc[PID_PRZE];
			}

			//regulator prędkosci kątowej
			dane->stWyjPID[PID_PK_ODCH].fWejscie = dane->fZyroKal1[ODCH];
			RegulatorPID(ndT, PID_PK_ODCH, dane, konfig);
		}
	}

	//regulacja wysokości
	if (chTrybRegulatora[WYSO] > REG_WYLACZ)
	{
		if (chTrybRegulatora[WYSO] == REG_RECZNY)
			dane->stWyjPID[PID_WARIO].fWyjsciePID = fWeRc[WYSO];	//regulatory nie działają, wstaw dane z drążka
		else	//regulatory działają
		{
			//określ czy regulator wysokości ma pracować
			if (chTrybRegulatora[WYSO] > REG_AKRO)
			{
				//określ co ma być wartością zadaną dla regulatora kąta: regulatory nawigacji czy drążek RC
				if (chTrybRegulatora[WYSO] == REG_AUTO)
				{
					//tutaj będzie obsługa regulatora nawigacyjnego
					//dane->stWyjPID[PID_WARIO].fZadana = składowa wysokości z regulatora prędkosci zmiany pozycji geograficznej
				}
				else
				{
					dane->stWyjPID[PID_WYSO].fZadana = fWeRc[WYSO];
				}

				//regulator kąta
				dane->stWyjPID[PID_WYSO].fWejscie = dane->fKatIMU1[WYSO];
				RegulatorPID(ndT, PID_WYSO, dane, konfig);
				dane->stWyjPID[PID_WARIO].fZadana = dane->stWyjPID[PID_WYSO].fWyjsciePID;
			}
			else
			{
				//regulatory nawigacyjne i stabilizacyjny nie działają, jesteśmy w trybie akrobacyjnym, wartość zadana prędkości kątowej pochodzi z drążka
				dane->stWyjPID[PID_WARIO].fZadana = fWeRc[PID_PRZE];
			}

			//regulator prędkosci kątowej
			dane->stWyjPID[PID_WARIO].fWejscie = dane->fZyroKal1[WYSO];
			RegulatorPID(ndT, PID_WARIO, dane, konfig);
		}
	}*/
	return chErr;
}
