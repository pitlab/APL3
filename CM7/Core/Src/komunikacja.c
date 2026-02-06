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
#include "display.h"
#include "konfig_fram.h"


uint32_t nOffsetDanych;
int16_t sSzerZdjecia, sWysZdjecia;
uint16_t sAdres;
uint8_t chStatusZdjecia;		//status gotowości wykonania zdjęcia


static un8_32_t un8_32;
static un8_16_t un8_16;


extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
extern stKonfKam_t stKonfKam;
extern uint16_t sOkresTelemetrii[LICZBA_RAMEK_TELEMETR * OKRESOW_TELEMETRII_W_RAMCE];	//zmienna definiujaca okres wysyłania telemetrii dla wszystkich zmiennych
extern  uint8_t chPolecenie;
extern  uint8_t chRozmDanych;
extern  uint8_t chDane[ROZMIAR_DANYCH_KOMUNIKACJI];
extern uint16_t sBuforSektoraFlash[ROZMIAR16_BUF_SEKT];	//Bufor sektora Flash NOR umieszczony w AXI-SRAM
extern uint8_t chTrybPracy;
extern uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforKamerySRAM[];	//bufor na klatki filmu
extern uint16_t sWskBufSektora;	//wskazuje na poziom zapełnienia bufora
extern stBSP_t stBSP;	//struktura zawierajaca adres i nazwę BSP
extern uint8_t chStatusPolaczenia;
extern uint8_t chWstrzymajTelemetrie;	//wartość niezerowa tymczasowo wstrzymuje działanie telemetrii

uint8_t UruchomPolecenie(uint8_t chPolecenie, uint8_t* chDane, uint8_t chRozmDanych, uint8_t chInterfejs, uint8_t chAdresZdalny)
{
	uint8_t n, chErr = BLAD_OK;
	uint8_t chRozmiar;
	uint16_t sPrzesuniecie;

	switch (chPolecenie)
	{
	case PK_OK:	//odeslij polecenie OK
		chErr = Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
		break;

	case PK_ZROB_ZDJECIE:		//polecenie wykonania zdjęcia.
		//chTrybPracy = TP_ZDJECIE;
		//chStatusZdjecia = SGZ_CZEKA;	//oczekiwania na wykonanie zdjęcia
		//chStatusZdjecia = SGZ_BLAD;		//dopóki nie ma kamery niech zgłasza bład
		chStatusZdjecia = SGZ_GOTOWE;
		chErr = ZrobZdjecie(sBuforKamerySRAM, DISP_X_SIZE * DISP_Y_SIZE / 2);
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		break;

	case PK_POB_STAT_ZDJECIA:	//pobierz status gotowości zdjęcia - trzeba dopracować metodę ustawiania stautus po wykonaniu zdjecia w jakims callbacku
		chDane[0] = chStatusZdjecia;
		chErr = WyslijRamke(chAdresZdalny, PK_POB_STAT_ZDJECIA, 1, chDane, chInterfejs);
		for (uint16_t y=0; y<320; y++)
		{
			for (uint16_t x=0; x<480; x++)
			{
				if (x == 5)
					sBuforKamerySRAM[y*480 + x] = ZOLTY;
				if (y == 5)
					sBuforKamerySRAM[y*480 + x] = CZERWONY;
			}
		}
		break;

	case PK_POBIERZ_ZDJECIE:		//polecenie przesłania fragmentu zdjecia. We: [0..3] - wskaźnik na pozycje bufora, [4] - rozmiar danych do przesłania
		for (n=0; n<4; n++)
			un8_32.dane8[n] = chDane[n];
		nOffsetDanych = un8_32.dane32;
		chErr = WyslijRamke(chAdresZdalny, PK_POBIERZ_ZDJECIE, chDane[4], (uint8_t*)(sBuforKamerySRAM + nOffsetDanych),  chInterfejs);
		break;

	case PK_USTAW_BSP:		//ustawia identyfikator/adres urządzenia
		stBSP.chAdres = chDane[0];
		for (n=0; n<DLUGOSC_NAZWY; n++)
			stBSP.chNazwa[n] = chDane[n+1];
		for (n=0; n<4; n++)
			stBSP.chAdrIP[n] = chDane[n+DLUGOSC_NAZWY+1];
		chErr = ZapiszPaczkeKonfigu(FKON_NAZWA_ID_BSP, (uint8_t*)&stBSP);
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		break;

	case PK_POBIERZ_BSP:		//pobiera identyfikator/adres urządzenia
		chDane[0] = stBSP.chAdres;
		for (n=0; n<DLUGOSC_NAZWY; n++)
			chDane[n+1] = stBSP.chNazwa[n];
		for (n=0; n<4; n++)
			chDane[n+DLUGOSC_NAZWY+1] = stBSP.chAdrIP[n];
		chErr = WyslijRamke(chAdresZdalny, PK_POBIERZ_BSP, DLUGOSC_NAZWY+5, chDane, chInterfejs);
		//polecenie PK_POBIERZ_BSP otwiera połączenie UART, więc zmień stan na otwarty
		if (chInterfejs == INTERF_UART)
		{
			chStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_UART);
			chStatusPolaczenia |= (STAT_POL_OTWARTY << STAT_POL_UART);
		}
		break;

	case PK_UST_TR_PRACY:	//ustaw tryb pracy
		chTrybPracy = chDane[0];
		chErr = Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
		break;

	case PK_POB_PAR_KAMERY:	//pobierz parametry pracy kamery
		chDane[0] = stKonfKam.chSzerWy;
		chDane[1] = stKonfKam.chWysWy;
		chDane[2] = stKonfKam.chSzerWe;
		chDane[3] = stKonfKam.chWysWe;
		chDane[4] = stKonfKam.chPrzesWyPoz;
		chDane[5] = stKonfKam.chPrzesWyPio;
		chDane[6] = stKonfKam.chObracanieObrazu;
		chDane[7] = stKonfKam.chFormatObrazu;
		chDane[8] = (uint8_t)(stKonfKam.sWzmocnienieR >> 8);
		chDane[9] = (uint8_t)(stKonfKam.sWzmocnienieR & 0xFF);
		chDane[10] = (uint8_t)(stKonfKam.sWzmocnienieG >> 8);
		chDane[11] = (uint8_t)(stKonfKam.sWzmocnienieG & 0xFF);
		chDane[12] = (uint8_t)(stKonfKam.sWzmocnienieB >> 8);
		chDane[13] = (uint8_t)(stKonfKam.sWzmocnienieB & 0xFF);
		chDane[14] = stKonfKam.chKontrBalBieli;
		chDane[15] = stKonfKam.chNasycenie;
		chDane[16] = (uint8_t)(stKonfKam.sCzasEkspozycji >> 8);		//AEC Long Channel Exposure [19:0]: 0x3500..02
		chDane[17] = (uint8_t)(stKonfKam.sCzasEkspozycji & 0xFF);
		chDane[18] = stKonfKam.chKontrolaExpo;
		chDane[19] = stKonfKam.chTrybyEkspozycji;
		chDane[20] = (uint8_t)(stKonfKam.sAGCLong >> 8);
		chDane[21] = (uint8_t)(stKonfKam.sAGCLong & 0xFF);
		chDane[22] = (uint8_t)(stKonfKam.sAGCVTS >> 8);
		chDane[23] = (uint8_t)(stKonfKam.sAGCVTS & 0xFF);
		chDane[24] = stKonfKam.chKontrolaISP0;		//0x5000
		chDane[25] = stKonfKam.chKontrolaISP1;		//0x50001
		chDane[26] = stKonfKam.chProgUsuwania;		//0x5080 Even CTRL 00 Treshold for even odd  cancelling
		chDane[27] = (uint8_t)(stKonfKam.sAGCAdjust >> 8);
		chDane[28] = (uint8_t)(stKonfKam.sAGCAdjust & 0xFF);
		chDane[29] = stKonfKam.chPoziomEkspozycji;
		chErr = WyslijRamke(chAdresZdalny, PK_POB_PAR_KAMERY, 30, chDane, chInterfejs);
		break;

	case PK_UST_PAR_KAMERY:	//ustaw parametry pracy kamery
		stKonfKam.chSzerWy = chDane[0];
		stKonfKam.chWysWy = chDane[1];
		stKonfKam.chSzerWe = chDane[2];
		stKonfKam.chWysWe = chDane[3];
		stKonfKam.chPrzesWyPoz = chDane[4];
		stKonfKam.chPrzesWyPio = chDane[5];
		stKonfKam.chObracanieObrazu = chDane[6];
		stKonfKam.chFormatObrazu = chDane[7];
		stKonfKam.sWzmocnienieR = ((uint16_t)chDane[8] << 8) + chDane[9];
		stKonfKam.sWzmocnienieG = ((uint16_t)chDane[10] << 8) + chDane[11];
		stKonfKam.sWzmocnienieB = ((uint16_t)chDane[12] << 8) + chDane[13];
		stKonfKam.chKontrBalBieli = chDane[14];
		stKonfKam.chNasycenie = chDane[15];
		stKonfKam.sCzasEkspozycji = ((uint16_t)chDane[16] << 8) + chDane[17];
		stKonfKam.chKontrolaExpo = chDane[18];
		stKonfKam.chTrybyEkspozycji = chDane[19];
		stKonfKam.sAGCLong = ((uint16_t)chDane[20] << 8) + chDane[21];
		stKonfKam.sAGCVTS = ((uint16_t)chDane[22] << 8) + chDane[23];
		stKonfKam.chKontrolaISP0 = chDane[24];		//0x5000
		stKonfKam.chKontrolaISP1 = chDane[25];		//0x50001
		stKonfKam.chProgUsuwania = chDane[26];		//0x5080 Even CTRL 00 Treshold for even odd  cancelling
		stKonfKam.sAGCAdjust = ((uint16_t)chDane[27] << 8) + chDane[28];
		stKonfKam.chPoziomEkspozycji = chDane[29];
		chErr = UstawKamere(&stKonfKam);	//wersja z rejestrami osobno
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		break;

	case PK_UST_PAR_KAMERY_GRUP:	//ustaw parametry pracy kamery grupowo
		stKonfKam.chSzerWy = chDane[0];
		stKonfKam.chWysWy = chDane[1];
		stKonfKam.chSzerWe = chDane[2];
		stKonfKam.chWysWe = chDane[3];
		stKonfKam.chPrzesWyPoz = chDane[4];
		stKonfKam.chPrzesWyPio = chDane[5];
		stKonfKam.chObracanieObrazu = chDane[6];
		stKonfKam.chFormatObrazu = chDane[7];
		stKonfKam.sWzmocnienieR = ((uint16_t)chDane[8] << 8) + chDane[9];
		stKonfKam.sWzmocnienieG = ((uint16_t)chDane[10] << 8) + chDane[11];
		stKonfKam.sWzmocnienieB = ((uint16_t)chDane[12] << 8) + chDane[13];
		stKonfKam.chKontrBalBieli = chDane[14];
		stKonfKam.chNasycenie = chDane[15];
		stKonfKam.sCzasEkspozycji = ((uint16_t)chDane[16] << 8) + chDane[17];
		stKonfKam.chKontrolaExpo = chDane[18];
		stKonfKam.chTrybyEkspozycji = chDane[19];
		stKonfKam.sAGCLong = ((uint16_t)chDane[20] << 8) + chDane[21];
		stKonfKam.sAGCVTS = ((uint16_t)chDane[22] << 8) + chDane[23];
		stKonfKam.chKontrolaISP0 = chDane[24];		//0x5000
		stKonfKam.chKontrolaISP1 = chDane[25];		//0x50001
		stKonfKam.chProgUsuwania = chDane[26];		//0x5080 Even CTRL 00 Treshold for even odd  cancelling
		stKonfKam.sAGCAdjust = ((uint16_t)chDane[27] << 8) + chDane[28];
		stKonfKam.chPoziomEkspozycji = chDane[29];
		chErr = UstawKamere2(&stKonfKam);	//wersja z grupami rejestrów
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		break;

	case PK_RESETUJ_KAMERE:
		chErr = InicjalizujKamere();
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
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
		chErr = Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
		break;

	case PK_ZAPISZ_FLASH: 	//zapisz bufor 256 słów do sektora flash NOR o przekazanym adresie
		for (n=0; n<4; n++)
			un8_32.dane8[n] = chDane[n];	//adres sektora
		chErr = ZapiszDaneFlashNOR(un8_32.dane32, sBuforSektoraFlash, ROZMIAR16_BUF_SEKT);
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		sWskBufSektora = 0;
		break;

	case PK_KASUJ_FLASH:	//kasuj sektor 128kB flash
		for (n=0; n<4; n++)
			un8_32.dane8[n] = chDane[n];
		chErr = KasujSektorFlashNOR(un8_32.dane32);
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		break;

	case PK_CZYTAJ_FLASH:	//odczytaj zawartość Flash
		for (uint8_t n=0; n<4; n++)
			un8_32.dane8[n] = chDane[n];	//adres sektora

		if (chDane[4] > ROZMIAR16_BUF_SEKT)	//jeżeli zażądano odczytu więcej niż pomieści bufor sektora to zwróc błąd
		{
			chErr = ERR_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
			break;
		}
		if (2* chDane[4] > ROZMIAR_DANYCH_KOMUNIKACJI)	//jeżeli zażądano odczytu więcej niż pomieści ramka komunikacyjna to zwróc błąd
		{
			chErr = ERR_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
			break;
		}

		CzytajDaneFlashNOR(un8_32.dane32, sBuforSektoraFlash, chDane[4]);
		chErr = WyslijRamke(chAdresZdalny, PK_CZYTAJ_FLASH, 2*chDane[4], (uint8_t*)sBuforSektoraFlash, chInterfejs);
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		break;

	case PK_CZYTAJ_OKRES_TELE:	//odczytaj a APL3 okresy telemetrii: chDane[0] == liczba pozycji okresu telemetrii  do odczytania, chDane[1] == numer zmiennej względen początku
		chRozmiar = chDane[0];
		sPrzesuniecie = chDane[1] + chDane[2] * 0x100;	//indeks pierwszej zmiennej, młodszy przodem
		for (uint8_t n=0; n<chRozmiar; n++)
		{
			chDane[2*n+0] = (uint8_t)(sOkresTelemetrii[sPrzesuniecie+n] & 0x00FF);	//młodszy przodem
			chDane[2*n+1] = (uint8_t)(sOkresTelemetrii[sPrzesuniecie+n] >> 8);
		}
		chErr = WyslijRamke(chAdresZdalny, PK_CZYTAJ_OKRES_TELE, 2*chRozmiar, chDane, chInterfejs);
		break;

	case PK_ZAPISZ_OKRES_TELE:	//zapisz okresy telemetrii
		sPrzesuniecie = chDane[0] + chDane[1] * 0x100;	//indeks pierwszej zmiennej, młodszy przodem
		chRozmiar = (chRozmDanych - 2) / 2;
		for (uint8_t n=0; n<chRozmiar; n++)
			sOkresTelemetrii[n + sPrzesuniecie] = chDane[2*n+2] + chDane[2*n+3] * 0x100;	//kolejnne okresy telemetrii, młodszy przodem
		chErr = ZapiszKonfiguracjeTelemetrii(sPrzesuniecie);
		if (chErr == BLAD_OK)
			InicjalizacjaTelemetrii();
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		break;

	case PK_ZAPISZ_FRAM_U8:	//zapisuje bajty do FRAM
		if (chDane[0] > ROZMIAR_ROZNE_CHAR)	//liczba danych uint8_t
		{
			chErr = ERR_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
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
		Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
		break;

	case PK_ZAPISZ_FRAM_FLOAT:				//Wysyła dane typu float do zapisu we FRAM w rdzeniu CM4 o rozmiarze ROZMIAR_ROZNE_FLOAT
		if (chDane[0] > ROZMIAR_ROZNE_FLOAT)	//liczba danych float (nie uint8_t)
		{
			chErr = ERR_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
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
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		break;

	case PK_WYSLIJ_POTW_ZAPISU:	//jeżeli dane się zapisały to odeslij BLAD_OK. jeeli jeszcze nie to ERR_PROCES_TRWA
		if ((uDaneCM4.dane.chOdpowiedzNaPolecenie != POL_ZAPISZ_FRAM_FLOAT) || (uDaneCM4.dane.sAdres != sAdres))
		{
			chErr = ERR_PROCES_TRWA;		//dane jeszcze nie przyszły
			Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
			break;
		}
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//wyłącz wykonywanie polecenia POL_ZAPISZ_FRAM_FLOAT
		break;

	case PK_CZYTAJ_FRAM_U8:
		if (chDane[0] > ROZMIAR_ROZNE_CHAR)
		{
			chErr = ERR_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
			break;
		}
		un8_16.dane8[0] = chDane[1];	//adres do odczytu
		un8_16.dane8[1] = chDane[2];
		sAdres = un8_16.dane16;			//zapamiętaj adres
		uDaneCM7.dane.sAdres = un8_16.dane16;
		uDaneCM7.dane.chRozmiar = chDane[0];
		uDaneCM7.dane.chWykonajPolecenie = POL_CZYTAJ_FRAM_U8;
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		break;

	case PK_CZYTAJ_FRAM_FLOAT:			//odczytaj i wyślij do bufora fRozne[] zawartość FRAM spod podanego adresu w chDane[1..2] o rozmiarze podanym w chDane[0]
		if (chDane[0] > ROZMIAR_ROZNE_FLOAT)
		{
			chErr = ERR_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
			break;
		}
		un8_16.dane8[0] = chDane[1];	//adres do odczytu
		un8_16.dane8[1] = chDane[2];
		sAdres = un8_16.dane16;			//zapamiętaj adres
		uDaneCM7.dane.sAdres = un8_16.dane16;
		uDaneCM7.dane.chRozmiar = chDane[0];
		uDaneCM7.dane.chWykonajPolecenie = POL_CZYTAJ_FRAM_FLOAT;
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		break;

	case PK_WYSLIJ_ODCZYT_FRAM:	//wysyła odczytane wcześniej dane o rozmiarze podanym w chDane[0]
		if (uDaneCM4.dane.sAdres != sAdres)
		{
			chErr = ERR_PROCES_TRWA;		//dane jeszcze nie przyszły
			Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
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
		Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
		break;

	case PK_ZAMKNIJ_POLACZENIE:	// zamyka połączenie UART, więc zmień stan na STAT_POL_GOTOWY
		if (chInterfejs == INTERF_UART)
		{
			chStatusPolaczenia &= ~(STAT_POL_MASKA_GOT << STAT_POL_UART);
		}
		Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
		break;

	case PK_ZAPISZ_KONFIG_PID:			//Wysyła dane do zapisu we FRAM w rdzeniu CM4 oraz zapisania do zmienych regulatora
		if ((chDane[0] >= LICZBA_PID) || (chRozmDanych > ROZMIAR_ROZNE_CHAR))	//indeks kanału regulatora nie powinien przekraczać liczby regulatorów i nie powinna zostać przepełniona struktura danych przekazywanych
		{
			chErr = ERR_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(chErr, chPolecenie, chInterfejs);
			break;
		}
#ifdef TESTY
		assert(chRozmDanych == ROZMIAR_REG_PID);
#endif
		uDaneCM7.dane.chWykonajPolecenie = POL_ZAPISZ_KONFIG_PID;
		uDaneCM7.dane.chRozmiar = (chRozmDanych / sizeof(float)) - 1;		//ilość liczb float zawierających konfigurację regulatora, ostatnia liczba zawiera flagi
		uDaneCM7.dane.sAdres = chDane[0];	//indeks regulatora
		//chDane[2] i chDane[3] są wolne do wykorzystania
		for (n=0; n<uDaneCM7.dane.chRozmiar; n++)
		{
			for (uint8_t i=0; i<sizeof(float); i++)
				un8_32.dane8[i] = chDane[4 + n*sizeof(float) + i];
			uDaneCM7.dane.uRozne.f32[n] = un8_32.daneFloat;
		}
		//w ostatnich 4 bajtach zamiast float przeslij konfigurację zawartą w bajtach
		uDaneCM7.dane.uRozne.U8[sizeof(float) * uDaneCM7.dane.chRozmiar] = chDane[1];	//stała czasowa filtra D
		uDaneCM7.dane.chRozmiar++;
		Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
		break;

	case PK_ZBIERAJ_EKSTREMA_RC:	//rozpoczyna zbieranie ekstremalnych wartości kanałów obu odbiorników RC
		uDaneCM7.dane.chWykonajPolecenie = POL_ZBIERAJ_EKSTREMA_RC;
		Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
		break;

	case PK_ZAPISZ_EKSTREMA_RC:		//kończy zbieranie ekstremalnych wartości kanałów obu odbiorników RC i zapisuje wyniki do FRAM
		uDaneCM7.dane.chWykonajPolecenie = POL_ZAPISZ_EKSTREMA_RC;
		Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
		break;

	case PK_ZAPISZ_WYSTER_NAPEDU:		//zapisuje nastawy wysterowania napędu dla wartości jałowej, minimalnej, zawisu i maksymalnej
#ifdef TESTY
		assert(chRozmDanych == 2*LICZBA_DRAZKOW);
#endif
		for (n=0; n<chRozmDanych; n++)
		{
			for (uint8_t i=0; i<2; i++)
				un8_32.dane8[i] = chDane[n*2+i];
			uDaneCM7.dane.uRozne.U16[n] = un8_32.dane16[0];
		}
		uDaneCM7.dane.chWykonajPolecenie = POL_ZAPISZ_PWM_NAPEDU;
		uDaneCM7.dane.sAdres = PK_ZAPISZ_WYSTER_NAPEDU;	//fejkowy adres wysyłany w celu uzyskania potwierdzenia zapisu
		uDaneCM7.dane.chRozmiar = chRozmDanych;
		Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
		break;

	case PK_ZAPISZ_TRYB_REG:
#ifdef TESTY
		assert(chRozmDanych == LICZBA_REG_PARAM);
#endif
		for (n=0; n<chRozmDanych; n++)
			uDaneCM7.dane.uRozne.U8[n] = chDane[n];
		uDaneCM7.dane.chWykonajPolecenie = POL_ZAPISZ_TRYB_REG;
		uDaneCM7.dane.sAdres = PK_ZAPISZ_TRYB_REG;	//fejkowy adres wysyłany w celu uzyskania potwierdzenia zapisu
		uDaneCM7.dane.chRozmiar = chRozmDanych;
		Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
		break;

	case PK_RESETUJ_CM4:	//resetuj rdzeń CM4, zwykle po zmianie konfiguracji
		if ((uDaneCM4.dane.chFlagiLotu & FL_SILN_UZBROJONE) != FL_SILN_UZBROJONE)	//nie pozwalaj na reset gdy silniki są uzbrojone
		{
			Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
			HAL_Delay(10);
			NVIC_SystemReset();
		}
		else
			Wyslij_KodBledu(BLAD_ODMOWA_WYKONANIA, chPolecenie, chInterfejs);
		break;

	case PK_WSTRZYMAJ_TELEMETRIE:	//wznów lub tymczasowo wstrzymaj wysyłnie telemetrii na czas transmisji innych danych
		chWstrzymajTelemetrie  = chDane[0];		//wartość niezerowa tymczasowo wstrzymuje działanie telemetrii
		Wyslij_KodBledu(BLAD_OK, chPolecenie, chInterfejs);
		break;

	}
    return chErr;
}
