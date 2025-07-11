//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł obsługi komunikacji ze światem przy pomocy protokołu komunikacyjnego
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "komunikacja.h"
#include "flash_konfig.h"
#include "flash_nor.h"
#include "telemetria.h"
#include "protokol_kom.h"
#include "polecenia_komunikacyjne.h"
#include "wymiana_CM7.h"
#include "kamera.h"

uint32_t nOffsetDanych;
int16_t sSzerZdjecia, sWysZdjecia;
uint16_t sAdres;
uint8_t chStatusZdjecia;		//status gotowości wykonania zdjęcia


static un8_32_t un8_32;
static un8_16_t un8_16;


extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
extern struct st_KonfKam strKonfKam;
extern uint16_t sOkresTelemetrii[MAX_INDEKSOW_TELEMETR_W_RAMCE];	//zmienna definiujaca okres wysyłania telemetrii dla wszystkich zmiennych
extern  uint8_t chPolecenie;
extern  uint8_t chRozmDanych;
extern  uint8_t chDane[ROZM_DANYCH_UART];
extern uint16_t sBuforSektoraFlash[ROZMIAR16_BUF_SEKT];	//Bufor sektora Flash NOR umieszczony w AXI-SRAM
extern uint8_t chTrybPracy;
extern uint16_t sBuforLCD[];
extern uint16_t sWskBufSektora;	//wskazuje na poziom zapełnienia bufora
extern stBSP_t stBSP;	//struktura zawierajaca adres i nazwę BSP

uint8_t UruchomPolecenie(uint8_t chPolecenie, uint8_t* chDane, uint8_t chRozmDanych, uint8_t chInterfejs, uint8_t chAdresZdalny)
{
	uint8_t n, chErr;
	uint8_t chRozmiar;

	switch (chPolecenie)
	{
	case PK_OK:	//odeslij polecenie OK
		chErr = Wyslij_OK(PK_OK, 0, chInterfejs);
		break;

	case PK_ZROB_ZDJECIE:		//polecenie wykonania zdjęcia. We: [0..1] - sSzerokosc zdjecia, [2..3] - wysokość zdjecia
		sSzerZdjecia = (uint16_t)chDane[1] * 0x100 + chDane[0];
		sWysZdjecia  = (uint16_t)chDane[3] * 0x100 + chDane[2];
		chTrybPracy = TP_ZDJECIE;
		//chStatusZdjecia = SGZ_CZEKA;	//oczekiwania na wykonanie zdjęcia
		//chStatusZdjecia = SGZ_BLAD;		//dopóki nie ma kamery niech zgłasza bład
		chStatusZdjecia = SGZ_GOTOWE;
		//generuj testową strukturę obrazu
		/*if (chStatusZdjecia == SGZ_GOTOWE)
		{
			for (int x=0; x<ROZM_BUF32_KAM; x++)
			{
				sPix = (x*2) & 0xFFFF;
				nBuforKamery[x] = (sPix+1)*0x10000 + sPix;
			}
		}*/

		//CzytajPamiecObrazu(0, 0, 200, 320, (uint8_t*)sBuforLCD);	//odczytaj pamięć obrazu do bufora LCD
		chErr = Wyslij_OK(PK_ZROB_ZDJECIE, 0, chInterfejs);
		break;

	case PK_POB_STAT_ZDJECIA:	//pobierz status gotowości zdjęcia
		chDane[0] = chStatusZdjecia;
		chErr = WyslijRamke(chAdresZdalny, PK_POB_STAT_ZDJECIA, 1, chDane, chInterfejs);
		break;

	case PK_POBIERZ_ZDJECIE:		//polecenie przesłania fragmentu zdjecia. We: [0..3] - wskaźnik na pozycje bufora, [4] - rozmiar danych do przesłania
		for (n=0; n<4; n++)
			un8_32.dane8[n] = chDane[n];
		nOffsetDanych = un8_32.dane32;
		//WyslijRamke(chAdresZdalny[chInterfejs], PK_POBIERZ_ZDJECIE, chDane[4], (uint8_t*)(nBuforKamery + nOffsetDanych),  chInterfejs);
		WyslijRamke(chAdresZdalny, PK_POBIERZ_ZDJECIE, chDane[4], (uint8_t*)(sBuforLCD + nOffsetDanych),  chInterfejs);
		break;

	case PK_USTAW_BSP:		//ustawia identyfikator/adres urządzenia
		stBSP.chAdres = chDane[0];
		for (n=0; n<DLUGOSC_NAZWY; n++)
			stBSP.chNazwa[n] = chDane[n+1];
		for (n=0; n<4; n++)
			stBSP.chAdrIP[n] = chDane[n+DLUGOSC_NAZWY+1];
		ZapiszPaczkeKonfigu(FKON_NAZWA_ID_BSP, (uint8_t*)&stBSP);
		chErr = Wyslij_OK(PK_USTAW_BSP, 0, chInterfejs);
		break;

	case PK_POBIERZ_BSP:		//pobiera identyfikator/adres urządzenia
		chDane[0] = stBSP.chAdres;
		for (n=0; n<DLUGOSC_NAZWY; n++)
			chDane[n+1] = stBSP.chNazwa[n];
		for (n=0; n<4; n++)
			chDane[n+DLUGOSC_NAZWY+1] = stBSP.chAdrIP[n];
		chErr = WyslijRamke(chAdresZdalny, PK_POBIERZ_BSP, DLUGOSC_NAZWY+5, chDane, chInterfejs);
		break;

	case PK_UST_TR_PRACY:	//ustaw tryb pracy
		chTrybPracy = chDane[0];
		chErr = Wyslij_OK(PK_UST_TR_PRACY, 0, chInterfejs);
		break;

	case PK_POB_PAR_KAMERY:	//pobierz parametry pracy kamery
		chDane[0] = (uint8_t)(strKonfKam.sSzerWy / SKALA_ROZDZ_KAM);
		chDane[1] = (uint8_t)(strKonfKam.sWysWy / SKALA_ROZDZ_KAM);
		chDane[2] = (uint8_t)(strKonfKam.sSzerWe / SKALA_ROZDZ_KAM);
		chDane[3] = (uint8_t)(strKonfKam.sWysWe / SKALA_ROZDZ_KAM);
		chDane[4] = strKonfKam.chTrybDiagn;
		chDane[5] = strKonfKam.chFlagi;
		chErr = WyslijRamke(chAdresZdalny, PK_POB_PAR_KAMERY, 6, chDane, chInterfejs);
		break;

	case PK_UST_PAR_KAMERY:	//ustaw parametry pracy kamery
		strKonfKam.sSzerWy = chDane[0] * SKALA_ROZDZ_KAM;
		strKonfKam.sWysWy = chDane[1] * SKALA_ROZDZ_KAM;
		strKonfKam.sSzerWe = chDane[2] * SKALA_ROZDZ_KAM;
		strKonfKam.sWysWe = chDane[3] * SKALA_ROZDZ_KAM;
		strKonfKam.chTrybDiagn = chDane[4];
		strKonfKam.chFlagi = chDane[5];
		chErr = Wyslij_OK(chPolecenie, 0, chInterfejs);
		break;

	case PK_ZAPISZ_BUFOR:
		un8_16.dane8[0] = chDane[0];
		un8_16.dane8[1] = chDane[1];
		sWskBufSektora = un8_16.dane16;	//adres bezwzględny bufora
		//przepisz dane 8-bitowe  i zapisz po konwersji w buforze 16-bitowym
		for (uint8_t n=0; n<chDane[2]; n++)	//chDane[2] - rozmiar wyrażony w słowach
		{
			un8_16.dane8[0] = chDane[2*n+3];
			un8_16.dane8[1] = chDane[2*n+4];
			sBuforSektoraFlash[sWskBufSektora + n] = un8_16.dane16;
		}
		chErr = Wyslij_OK(chPolecenie, 0, chInterfejs);
		break;

	case PK_ZAPISZ_FLASH: 	//zapisz bufor 256 słów do sektora flash NOR o przekazanym adresie
		for (n=0; n<4; n++)
			un8_32.dane8[n] = chDane[n];	//adres sektora
		chErr = ZapiszDaneFlashNOR(un8_32.dane32, sBuforSektoraFlash, ROZMIAR16_BUF_SEKT);
		if (chErr == ERR_OK)
			chErr = Wyslij_OK(chPolecenie, 0, chInterfejs);
		else
			chErr = Wyslij_ERR(chErr, 0, chInterfejs);
		sWskBufSektora = 0;
		break;

	case PK_KASUJ_FLASH:	//kasuj sektor 128kB flash
		for (n=0; n<4; n++)
			un8_32.dane8[n] = chDane[n];
		chErr = KasujSektorFlashNOR(un8_32.dane32);

		//na potwierdzenie wyślij ramkę OK lub ramkę z kodem błędu
		if (chErr == ERR_OK)
			chErr = Wyslij_OK(chPolecenie, 0, chInterfejs);
		else
			chErr = Wyslij_ERR(chErr, 0, chInterfejs);
		break;

	case PK_CZYTAJ_FLASH:	//odczytaj zawartość Flash
		for (uint8_t n=0; n<4; n++)
			un8_32.dane8[n] = chDane[n];	//adres sektora

		if (chDane[4] > ROZMIAR16_BUF_SEKT)	//jeżeli zażądano odczytu więcej niż pomieści bufor sektora to zwróc błąd
			chErr = Wyslij_ERR(ERR_ZLA_ILOSC_DANYCH, 0, chInterfejs);
		if (2* chDane[4] > ROZM_DANYCH_UART)	//jeżeli zażądano odczytu więcej niż pomieści ramka komunikacyjna to zwróc błąd
			chErr = Wyslij_ERR(ERR_ZLA_ILOSC_DANYCH, 0, chInterfejs);

		CzytajDaneFlashNOR(un8_32.dane32, sBuforSektoraFlash, chDane[4]);
		chErr = WyslijRamke(chAdresZdalny, PK_CZYTAJ_FLASH, 2*chDane[4], (uint8_t*)sBuforSektoraFlash, chInterfejs);
		break;

	case PK_CZYTAJ_OKRES_TELE:	//odczytaj a APL3 okresy telemetrii: chDane[0] == liczba pozycji okresu telemetrii  do odczytania, chDane[1] == numer zmiennej względen początku
		chRozmiar = chDane[0];
		for (uint8_t n=chDane[1]; n<chRozmiar; n++)
		{
			chDane[2*n+0] = (uint8_t)(sOkresTelemetrii[n] & 0x00FF);	//młodszy przodem
			chDane[2*n+1] = (uint8_t)(sOkresTelemetrii[n] >> 8);
		}
		chErr = WyslijRamke(chAdresZdalny, PK_CZYTAJ_OKRES_TELE, 2*chRozmiar, chDane, chInterfejs);
		break;

	case PK_ZAPISZ_OKRES_TELE:	//zapisz okresy telemetrii
		uint16_t sPrzesuniecie = chDane[0] + chDane[1] * 0x100;	//indeks pierwszej zmiennej, młodszy przodem

		for (n=0; n<chRozmDanych/2; n++)
			sOkresTelemetrii[n] = chDane[2*n+2] + chDane[2*n+3] * 0x100;	//kolejnne okresy telemetrii, młodszy przodem
		chErr = ZapiszKonfiguracjeTelemetrii(sPrzesuniecie);
		if (chErr)
			chErr = Wyslij_ERR(chErr, 0, chInterfejs);		//zwróć kod błedu zapisu konfiguracji telemetrii
		else
		{
			InicjalizacjaTelemetrii();
			chErr = Wyslij_OK(chDane[1] - chDane[0], 0, chInterfejs);	//odeslij rozmiar zapisanych danych
		}
		break;

	case PK_ZAPISZ_FRAM_U8:	//zapisuje bajty do FRAM
		if (chDane[0] > 4*ROZMIAR_ROZNE)	//liczba danych uint8_t
		{
			Wyslij_ERR(ERR_ZLA_ILOSC_DANYCH, 0, chInterfejs);
			break;
		}
		un8_16.dane8[0] = chDane[1];	//adres zapisu
		un8_16.dane8[1] = chDane[2];
		sAdres = un8_16.dane16;						//zapamietaj adres do sprawdzenia czy już się zapisało
		uDaneCM7.dane.sAdres = un8_16.dane16;		//adres zapisu bloku liczb
		uDaneCM7.dane.chWykonajPolecenie = POL_ZAPISZ_FRAM_U8;
		uDaneCM7.dane.chRozmiar = chDane[0];		//ilość liczb uint8_t
		for (n=0; n<chDane[0]; n++)
			uDaneCM7.dane.uRozne.U8[n] = chDane[3+n];
		chErr = Wyslij_OK(0, 0, chInterfejs);
		break;

	case PK_ZAPISZ_FRAM_FLOAT:				//Wysyła dane typu float do zapisu we FRAM w rdzeniu CM4 o rozmiarze ROZMIAR_ROZNE
		if (chDane[0] > ROZMIAR_ROZNE)	//liczba danych float (nie uint8_t)
		{
			Wyslij_ERR(ERR_ZLA_ILOSC_DANYCH, 0, chInterfejs);
			break;
		}
		un8_16.dane8[0] = chDane[1];	//adres zapisu
		un8_16.dane8[1] = chDane[2];
		sAdres = un8_16.dane16;						//zapamietaj adres do sprawdzenia czy już się zapisało
		uDaneCM7.dane.sAdres = un8_16.dane16;		//adres zapisu bloku liczb
		uDaneCM7.dane.chWykonajPolecenie = POL_ZAPISZ_FRAM_FLOAT;
		uDaneCM7.dane.chRozmiar = chDane[0];		//ilość liczb float
		for (n=0; n<chDane[0]; n++)
		{
			for (uint8_t i=0; i<4; i++)
				un8_32.dane8[i] = chDane[3+n*4+i];
			uDaneCM7.dane.uRozne.f32[n] = un8_32.daneFloat;
		}
		chErr = Wyslij_OK(0, 0, chInterfejs);
		break;

	case PK_WYSLIJ_POTW_ZAPISU:	//jeżeli dane się zapisały to odeslij ERR_OK. jeeli jeszcze nie to ERR_PROCES_TRWA
		if ((uDaneCM4.dane.chOdpowiedzNaPolecenie != POL_ZAPISZ_FRAM_FLOAT) || (uDaneCM4.dane.sAdres != sAdres))
		{
			Wyslij_ERR(ERR_PROCES_TRWA, 0, chInterfejs);	//dane jeszcze nie przyszły
			break;
		}
		chErr = Wyslij_OK(0, 0, chInterfejs);
		uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//wyłącz wykonywanie polecenia POL_ZAPISZ_FRAM_FLOAT
		break;

	case PK_CZYTAJ_FRAM_U8:
		if (chDane[0] > 4*ROZMIAR_ROZNE)
		{
			Wyslij_ERR(ERR_ZLA_ILOSC_DANYCH, 0, chInterfejs);
			break;
		}
		un8_16.dane8[0] = chDane[1];	//adres do odczytu
		un8_16.dane8[1] = chDane[2];
		sAdres = un8_16.dane16;			//zapamiętaj adres
		uDaneCM7.dane.sAdres = un8_16.dane16;
		uDaneCM7.dane.chRozmiar = chDane[0];
		uDaneCM7.dane.chWykonajPolecenie = POL_CZYTAJ_FRAM_U8;
		chErr = Wyslij_OK(0, 0, chInterfejs);
		break;

	case PK_CZYTAJ_FRAM_FLOAT:			//odczytaj i wyślij do bufora fRozne[] zawartość FRAM spod podanego adresu w chDane[1..2] o rozmiarze podanym w chDane[0]
		if (chDane[0] > ROZMIAR_ROZNE)
		{
			Wyslij_ERR(ERR_ZLA_ILOSC_DANYCH, 0, chInterfejs);
			break;
		}
		un8_16.dane8[0] = chDane[1];	//adres do odczytu
		un8_16.dane8[1] = chDane[2];
		sAdres = un8_16.dane16;			//zapamiętaj adres
		uDaneCM7.dane.sAdres = un8_16.dane16;
		uDaneCM7.dane.chRozmiar = chDane[0];
		uDaneCM7.dane.chWykonajPolecenie = POL_CZYTAJ_FRAM_FLOAT;
		chErr = Wyslij_OK(0, 0, chInterfejs);
		break;

	case PK_WYSLIJ_ODCZYT_FRAM:	//wysyła odczytane wcześniej dane o rozmiarze podanym w chDane[0]
		if (uDaneCM4.dane.sAdres != sAdres)
		{
			Wyslij_ERR(ERR_PROCES_TRWA, 0, chInterfejs);	//dane jeszcze nie przyszły
			break;
		}
		chRozmiar = chDane[0];	//zapamiętaj w zmiennej, bo dane będą nadpisane;
		for (n=0; n<chRozmiar; n++)
			chDane[n] = uDaneCM4.dane.uRozne.U8[n];

		chErr = WyslijRamke(chAdresZdalny, PK_WYSLIJ_ODCZYT_FRAM, chRozmiar, chDane, chInterfejs);
		uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//wyłącz wykonywanie polecenia PK_WYSLIJ_ODCZYT_FRAM
		break;

	case PK_REKONFIG_SERWA_RC:	//wykonuje ponowną konfigurację wejść i wyjść RC po zmianie zawartosci FRAM
		uDaneCM7.dane.chWykonajPolecenie = POL_REKONFIG_SERWA_RC;
		chErr = Wyslij_OK(0, 0, chInterfejs);
		break;

	}
    return chErr;
}
