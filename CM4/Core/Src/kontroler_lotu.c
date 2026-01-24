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

float fSkalaWartosciZadanejAkro[ROZMIAR_DRAZKOW];	//wartość zadana dla pełnego wychylenia drążka aparatury w trybie AKRO
float fSkalaWartosciZadanejStab[ROZMIAR_DRAZKOW];	//wartość zadana dla pełnego wychylenia drążka aparatury w trybie STAB
uint16_t sWysterowanieJalowe;	//wartość wysterowania regulatorów dla uzyskania obrotów jałowych
uint16_t sWysterowanieMin;		//wartość wysterowania regulatorów dla uzyskania obrotów minimalnych w trakcie lotu
uint16_t sWysterowanieZawisu;	//wartość wysterowania regulatorów dla uzyskania obrotów pozwalajacych na zawis
uint16_t sWysterowanieMax;		//wartość wysterowania regulatorów dla uzyskania obrotów maksymalnych


////////////////////////////////////////////////////////////////////////////////
// Wczytuje konfigurację kontrolera lotu
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujKontrolerLotu(void)
{
	uint8_t chErr = BLAD_OK;

    for (uint16_t n=0; n<ROZMIAR_DRAZKOW; n++)
    {
        //odczytaj wartość wzmocnienienia członu P regulatora
        chErr |= CzytajFramZWalidacja(FAU_ZADANA_AKRO + n*4, &fSkalaWartosciZadanejAkro[n], VMIN_PID_WZMP, VMAX_PID_WZMP, VDEF_PID_WZMP, ERR_NASTAWA_FRAM);	//4x4F wartość zadana z drążków aparatury dla regulatora Akro
        chErr |= CzytajFramZWalidacja(FAU_ZADANA_STAB + n*4, &fSkalaWartosciZadanejStab[n], VMIN_PID_WZMP, VMAX_PID_WZMP, VDEF_PID_WZMP, ERR_NASTAWA_FRAM);	//4x4F wartość zadana z drążków aparatury dla regulatora Stab
    }

    sWysterowanieJalowe = CzytajFramU16(FAU_PWM_JALOWY);	//2U wysterowanie regulatorów na biegu jałowym [us]
    sWysterowanieMin =  CzytajFramU16(FAU_PWM_MIN);			//2U minimalne wysterowanie regulatorów w trakcie lotu [us]
    sWysterowanieZawisu = CzytajFramU16(FAU_WPM_ZAWISU);	//2U wysterowanie regulatorów w zawisie [us]
    sWysterowanieMax = CzytajFramU16(FAU_PWM_MAX);			//2U maksymalne wysterowanie silników w trakcie lotu [us]
    return chErr;
}
