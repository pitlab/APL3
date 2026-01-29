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

uint16_t sWysterowanieJalowe;	//wartość wysterowania regulatorów dla uzyskania obrotów jałowych
uint16_t sWysterowanieMin;		//wartość wysterowania regulatorów dla uzyskania obrotów minimalnych w trakcie lotu
uint16_t sWysterowanieZawisu;	//wartość wysterowania regulatorów dla uzyskania obrotów pozwalajacych na zawis
uint16_t sWysterowanieMax;		//wartość wysterowania regulatorów dla uzyskania obrotów maksymalnych
uint8_t chTrybRegulacji[LICZBA_REG_PARAM];	//rodzaj regulacji dla podstawowych parametrów

////////////////////////////////////////////////////////////////////////////////
// Wczytuje konfigurację kontrolera lotu
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujKontrolerLotu(void)
{
	uint8_t chErr = BLAD_OK;

	CzytajBuforFRAM(FA_TRYB_REG, chTrybRegulacji, LICZBA_REG_PARAM);	//6*1U Tryb pracy regulatorów 4 podstawowych wartości przypisanych do drążków i 2 regulatorów pozycji N i E

    sWysterowanieJalowe = CzytajFramU16(FAU_PWM_JALOWY);	//2U wysterowanie regulatorów na biegu jałowym [us]
    sWysterowanieMin =  CzytajFramU16(FAU_PWM_MIN);			//2U minimalne wysterowanie regulatorów w trakcie lotu [us]
    sWysterowanieZawisu = CzytajFramU16(FAU_WPM_ZAWISU);	//2U wysterowanie regulatorów w zawisie [us]
    sWysterowanieMax = CzytajFramU16(FAU_PWM_MAX);			//2U maksymalne wysterowanie silników w trakcie lotu [us]

return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia stabilizacji lotu w różnych trybach pracy
// Parametry:
// [i] *chTrybRegulacji - wskaźnik na stopień automatyzacji procesu: REG_RECZNA, REG_AKRO, REG_STAB, REG_AUTO
// [i] ndT - czas od ostatniego cyklu [us]
// [i] *konfig - wskaźnik na strukturę danych regulatorów PID
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KontrolerLotu(uint8_t *chTrybRegulacji, uint32_t ndT, stWymianyCM4_t *dane, stKonfPID_t *konfig)
{
	float fWeRc[LICZBA_DRAZKOW];
	uint8_t chIndeksPID_Kata, chIndeksPID_Predk;
	uint8_t chErr = BLAD_OK;

	//sprowadź wartość kanałów wejsciowych z drążków aparatury RC do znormalizowanej wartości symetrycznej wzgledem zera: +-100
	for (uint16_t n=0; n<LICZBA_DRAZKOW; n++)
		fWeRc[n] = (float)(dane->sKanalRC[n] - PPM_NEUTR) / (PPM_MAX - PPM_NEUTR);

	for (uint16_t n=0; n<LICZBA_REG_PARAM; n++)
	{
		chIndeksPID_Kata = 2*n + 0;	//indeks regulatora parametru głównego: kątów i wysokości
		chIndeksPID_Predk =  2*n + 1;	//indeks regulatora pochodnej: prędkosci kątowych i prędkosci zmiany wysokości

		if (chTrybRegulacji[n] == REG_RECZNA)
			dane->stWyjPID[chIndeksPID_Predk].fWyjsciePID = fWeRc[n];	//regulatory nie działają, wstaw dane z drążka
		else	//regulatory działają
		{
			//określ czy regulator kata przechylenia ma pracować
			if (chTrybRegulacji[n] > REG_AKRO)
			{

				//określ co ma być wartością zadaną dla regulatora kąta: regulatory nawigacji czy drążek RC
				if (chTrybRegulacji[n] == REG_AUTO)
				{
					//tutaj będzie obsługa regulatora nawigacyjnego
					//dane->stWyjPID[chIndeksPID_Predk].fZadana = składowa przechylenia z regulatora prędkosci zmiany pozycji geograficznej
				}
				else
				{
					dane->stWyjPID[chIndeksPID_Kata].fZadana = fWeRc[n] * konfig[chIndeksPID_Kata].fSkalaWZadanej;
				}

				//regulator kąta
				dane->stWyjPID[chIndeksPID_Kata].fWejscie = dane->fKatIMU1[n];
				RegulatorPID(ndT, chIndeksPID_Kata, dane, konfig);
				dane->stWyjPID[chIndeksPID_Predk].fZadana = dane->stWyjPID[chIndeksPID_Kata].fWyjsciePID;
			}
			else
			{
				//regulatory nawigacyjne i stabilizacyjny nie działają, jesteśmy w trybie akrobacyjnym, wartość zadana prędkości kątowej pochodzi z drążka
				dane->stWyjPID[chIndeksPID_Predk].fZadana = fWeRc[n] * konfig[chIndeksPID_Predk].fSkalaWZadanej;
			}

			//regulator prędkosci kątowej
			dane->stWyjPID[chIndeksPID_Predk].fWejscie = dane->fZyroKal1[n];
			RegulatorPID(ndT, chIndeksPID_Predk, dane, konfig);
		}
	}

	return chErr;
}
