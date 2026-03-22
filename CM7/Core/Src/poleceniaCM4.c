//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi poleceń rdzenia CM4
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <poleceniaCM4.h>
#include "audio.h"
#include <stdio.h>
#include "moduly_SPI.h"
#include "sample_audio.h"
#include "wymiana_CM7.h"



extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
extern uint8_t chPort_exp_wysylany[LICZBA_EXP_SPI_ZEWN];

////////////////////////////////////////////////////////////////////////////////
// Funkcja sprawdza czy rdzeń CM4 życzy sobie wykonać jakieś polecenie z zakresu funkconalności rdzenia CM7
// np. sterowanie wyjsciami OD lub wymową komunikatów głosowych
// Parametry: brak - ponieważ jest wykonywany w pętli głównej
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaPolecenCM4(void)
{
	uint8_t chBłąd = BLAD_OK;

	//Jeżeli CM4 ma coś do powiedzenia wtedy wstawia to do uDaneCM4.dane.chWymowSampla. Aby nie wymawiać w kółko, wymowa jest potwierdzana w uDaneCM7.dane.uRozne.U8[31]
	if (uDaneCM4.dane.chWykonajPolecenie != uDaneCM7.dane.chPotwierdzenieWykonania)
	{
		switch (uDaneCM4.dane.chWykonajPolecenie)
		{
		case POL4_NIC:			break;	//nic nie rób
		case POL4_WLACZ_OD1:	chPort_exp_wysylany[0] |= EXP06_MOD_OD1;	break;	//włącz wyjście otwarty dren 1
		case POL4_WLACZ_OD2:	chPort_exp_wysylany[0] |= EXP07_MOD_OD2;	break;	//włącz wyjście otwarty dren 2
		case POL4_WYLACZ_OD1:	chPort_exp_wysylany[0] &= ~EXP06_MOD_OD1;	break;	//wyłącz wyjście otwarty dren 1
		case POL4_WYLACZ_OD2:	chPort_exp_wysylany[0] &= ~EXP07_MOD_OD2;	break;	//wyłącz wyjście otwarty dren 2
		case POL4_MOW_UZBROJONE:	chBłąd = PrzygotujKomunikat(KOMG_UZBROJONE, 0.0f);	break;	//rozpocznij wymowę komunikatu silniki uzbrojone
		case POL4_MOW_ROZBROJONE:	chBłąd = PrzygotujKomunikat(KOMG_ROZBROJONE, 0.0f);break;			//rozpocznij wymowę komunikatu silniki rozbrojone
		case POL4_MOW_WYSOKOSC:	chBłąd = PrzygotujKomunikat(KOMG_WYSOKOSC, uDaneCM4.dane.stBSP.fWysokoscMSL);	break;
		case POL4_MOW_NAPIECIE:	chBłąd = PrzygotujKomunikat(KOMG_NAPIECIE, uDaneCM4.dane.fNapiecieBat[0]);	break;
		case POL4_MOW_TEMPERAT:	chBłąd = PrzygotujKomunikat(KOMG_TEMPERATURA, uDaneCM4.dane.fTemper[0]);	break;
		case POL4_MOW_PREDKOSC:	chBłąd = PrzygotujKomunikat(KOMG_PREDKOSC, uDaneCM4.dane.stBSP.fIAS);		break;
		case POL4_MOW_KIERUNEK:	chBłąd = PrzygotujKomunikat(KOMG_KIERUNEK, uDaneCM4.dane.stBSP.fKursGeo);	break;
		}
		uDaneCM7.dane.chPotwierdzenieWykonania = uDaneCM4.dane.chWykonajPolecenie;	//potwierdź wykonanie polecenia
	}
	return chBłąd;
}
