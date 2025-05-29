//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł osługi pamięci FRAM w CM7
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "fram.h"



////////////////////////////////////////////////////////////////////////////////
// Wysyła dane typu float do zapisu we FRAM w rdzeniu CM4 o rozmiarze ROZMIAR_ROZNE
// Parametry:
// chIn - odbierany bajt
// chInterfejs - identyfikator interfejsu odbierająceg znak
//Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszFramFloat()
{
	uint16_t sAdres = uDaneCM7.dane.sAdres;
	uDaneCM7.dane.chWykonajPolecenie = POL_ZAPISZ_FRAM;


	case :			//odczytaj i wyślij zawartość FRAM spod podanego adresu
		for (uint16t_n=0; n<ROZMIAR_ROZNE; n++)
			uDaneCM4.dane.fRozne[n] = CzytajFramFloat(uDaneCM4.dane.sAdres + n*4);
		break;

	case :			//zapisz przekazane dane do FRAM pod podany adres
		for (uint16t_n=0; n<ROZMIAR_ROZNE; n++)
			ZapiszFramFloat(uDaneCM7.dane.sAdres + n*4, uDaneCM7.dane.fRozne[n]);
		break;
}


////////////////////////////////////////////////////////////////////////////////
// Wysyła dane do zapisu we FRAM w rdzeniu CM4
// Parametry:
// chIn - odbierany bajt
// chInterfejs - identyfikator interfejsu odbierająceg znak
//Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajFramFloat()
{
	uint16_t sAdres = uDaneCM4.dane.sAdres;
	uDaneCM4.dane.chWykonajPolecenie = POL_ODCZYTAJ_FRAM;
}
