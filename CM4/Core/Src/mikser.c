//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v2.0
// Moduł ubsługi miksera silników napędowych
//
// (c) PitLab 2025
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "mikser.h"
#include "konfig_fram.h"
#include "fram.h"


stMikser_t stMikser[KANALY_MIKSERA];





////////////////////////////////////////////////////////////////////////////////
// Funkcja wczytuje z FRAM konfigurację miksera
// Parametry: *mikser - wskaźnik na lokalną strukturę danych miksera
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujMikser(stMikser_t* mikser)
{
	uint8_t chErr = ERR_OK;

	for (uint8_t n=0; n<KANALY_MIKSERA; n++)
	{
		chErr |= CzytajFramZWalidacja(FAU_MIX_PRZECH + 2*n, &mikser->fPrze, VMIN_MIX_PRZE, VMAX_MIX_PRZE, VDEF_MIX_PRZE, ERR_NASTAWA_FRAM);	//8*4F współczynnik wpływu przechylenia na dany silnik
		chErr |= CzytajFramZWalidacja(FAU_MIX_POCHYL + 2*n, &mikser->fPoch, VMIN_MIX_PRZE, VMAX_MIX_PRZE, VDEF_MIX_PRZE, ERR_NASTAWA_FRAM);	//8*4F współczynnik wpływu pochylenia na dany silnik
		chErr |= CzytajFramZWalidacja(FAU_MIX_ODCHYL + 2*n, &mikser->fOdch, VMIN_MIX_PRZE, VMAX_MIX_PRZE, VDEF_MIX_PRZE, ERR_NASTAWA_FRAM);	//8*4F współczynnik wpływu odchylenia na dany silnik
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje obliczenia na sygnałach wychodzących z PID pochodnej i formowanie sygnałów dla silników
// Parametry: brak
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t LiczMikser(stMikser_t* mikser, stWymianyCM4_t *dane)
{

}

