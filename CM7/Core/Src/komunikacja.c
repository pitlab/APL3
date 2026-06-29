//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł obsługi komunikacji ze światem przy pomocy protokołu komunikacyjnego
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <AnalizaDrgan.h>
#include <FFT.h>
#include <Kamera.h>
#include <Komunikacja.h>
#include <KonfigFram.h>
#include <LCD/ILI9488.h>
#include <Telemetria.h>
#include "FlashKonfig.h"
#include "FlashNOR.h"
#include "ProtokolKomunikacyjny.h"
#include "PoleceniaKomunikacyjne.h"
#include "WymianaCM7.h"
#include "Ekran.h"
#include "cmsis_os.h"

uint32_t nOffsetDanych;
int16_t sSzerZdjecia, sWysZdjecia;
uint16_t sAdres;
uint8_t cStatusZdjecia;		//status gotowości wykonania zdjęcia
static unia8_32_t un8_32;
//static un8_16_t un8_16;
extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
extern stKonfKam_t stKonfKam;
extern uint16_t sOkresTelemetrii[LICZBA_RAMEK_TELEMETR * OKRESOW_TELEMETRII_W_RAMCE];	//zmienna definiujaca okres wysyłania telemetrii dla wszystkich zmiennych
extern  uint8_t cPolecenie;
extern  uint8_t chRozmDanych;
extern  uint8_t cDane[ROZMIAR_DANYCH_KOMUNIKACJI];
extern uint16_t sBuforSektoraFlash[ROZMIAR16_BUF_SEKT];	//Bufor sektora Flash NOR umieszczony w AXI-SRAM
extern uint8_t chTrybPracy;
extern uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) sBuforKamery[SZER_ZDJECIA * WYS_ZDJECIA];
extern uint16_t sWskBufSektora;	//wskazuje na poziom zapełnienia bufora
extern stBSP_ID_t stBSP_ID;	//struktura zawierajaca adres i nazwę BSP
extern uint8_t chStatusPolaczenia;
extern uint8_t chStatusTelemetrii;		//określa czy i jaki rodzaj telemetrii ma być wysyłany
extern stFFT_t stKonfigFFT;
extern stIdentyfikacjaSilnikow_t stIdentSiln;
extern float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fWynikFFT[LICZBA_TESTOW_FFT][LICZBA_ZMIENNYCH_FFT][FFT_MAX_ROZMIAR / 2];	//wartość sygnału wyjściowego


////////////////////////////////////////////////////////////////////////////////
// Funkcja wykonuje zadania zdefiniowane dla wszystkich poleceń komunikacyjnych
// Parametry:
// [we] cPolecenie - indeks poelcenia komunikacyjnego
// [we/wy] *cDane - wskaźnik na strukturę danych wejściowych lub wyjściowych z danymi do poleceń
// [we] chRozmDanych - rozmiar danych z parametrami dla poleceń
// [we] cInterfejs - wskazuje na interfejs komunikacyjny którym przyszło polecenie. Odpowiedź ma pójść tą samą drogą
// [we] cAdresZdalny - identyfikuje urządzenie wysyłajace dane
// Zwraca: kod będu
////////////////////////////////////////////////////////////////////////////////
uint8_t UruchomPolecenie(uint8_t cPolecenie, uint8_t *cDane, uint8_t chRozmDanych, uint8_t cInterfejs, uint8_t cAdresZdalny)
{
	uint8_t n, cBłąd = BLAD_OK;
	uint8_t chRozmiar;
	uint16_t sPrzesuniecie;

	switch (cPolecenie)
	{
	case PK_KOD_BLEDU:	cBłąd = Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);		break;	//odeslij kod błędu: OK

	case PK_ZROB_ZDJECIE:		//polecenie wykonania zdjęcia.
		cStatusZdjecia = SGZ_GOTOWE;
		cBłąd = ZrobZdjecie(sBuforKamery, DISP_X_SIZE * DISP_Y_SIZE / 2);
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_POB_STAT_ZDJECIA:	//pobierz status gotowości zdjęcia - trzeba dopracować metodę ustawiania stautus po wykonaniu zdjecia w jakims callbacku
		cDane[0] = cStatusZdjecia;
		cBłąd = WyslijRamke(cAdresZdalny, PK_POB_STAT_ZDJECIA, 1, cDane, cInterfejs);
		for (uint16_t y=0; y<320; y++)
		{
			for (uint16_t x=0; x<480; x++)
			{
				if (x == 5)
					sBuforKamery[y*480 + x] = ZOLTY;
				if (y == 5)
					sBuforKamery[y*480 + x] = CZERWONY;
			}
		}
		break;

	case PK_POBIERZ_ZDJECIE:		//polecenie przesłania fragmentu zdjecia. We: [0..3] - wskaźnik na pozycje bufora, [4] - rozmiar danych do przesłania
		for (n=0; n<4; n++)
			un8_32.dane8[n] = cDane[n];
		nOffsetDanych = un8_32.dane32;
		cBłąd = WyslijRamke(cAdresZdalny, PK_POBIERZ_ZDJECIE, cDane[4], (uint8_t*)(sBuforKamery + nOffsetDanych),  cInterfejs);
		break;

	case PK_USTAW_BSP:		//ustawia identyfikator/adres urządzenia
		stBSP_ID.chAdres = cDane[0];
		for (n=0; n<DLUGOSC_NAZWY; n++)
			stBSP_ID.chNazwa[n] = cDane[n+1];
		for (n=0; n<4; n++)
			stBSP_ID.chAdrIP[n] = cDane[n+DLUGOSC_NAZWY+1];
		cBłąd = ZapiszPaczkeKonfigu(FKON_NAZWA_ID_BSP, (uint8_t*)&stBSP_ID);
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_POBIERZ_BSP:		//pobiera identyfikator/adres urządzenia
		cDane[0] = stBSP_ID.chAdres;
		for (n=0; n<DLUGOSC_NAZWY; n++)
			cDane[n+1] = stBSP_ID.chNazwa[n];
		for (n=0; n<4; n++)
			cDane[n+DLUGOSC_NAZWY+1] = stBSP_ID.chAdrIP[n];
		cBłąd = WyslijRamke(cAdresZdalny, PK_POBIERZ_BSP, DLUGOSC_NAZWY+5, cDane, cInterfejs);
		//polecenie PK_POBIERZ_BSP otwiera połączenie UART, więc zmień stan na otwarty
		if (cInterfejs == INTERF_UART)
		{
			chStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_UART);
			chStatusPolaczenia |= (STAT_POL_OTWARTY << STAT_POL_UART);
		}
		break;

	case PK_UST_TR_PRACY:	//ustaw tryb pracy
		chTrybPracy = cDane[0];
		cBłąd = Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_POB_PAR_KAMERY:	//pobierz parametry pracy kamery
		cDane[0] = stKonfKam.chSzerWy;
		cDane[1] = stKonfKam.chWysWy;
		cDane[2] = stKonfKam.chSzerWe;
		cDane[3] = stKonfKam.chWysWe;
		cDane[4] = stKonfKam.chPrzesWyPoz;
		cDane[5] = stKonfKam.chPrzesWyPio;
		cDane[6] = stKonfKam.chObracanieObrazu;
		cDane[7] = stKonfKam.chFormatObrazu;
		cDane[8] = (uint8_t)(stKonfKam.sWzmocnienieR >> 8);
		cDane[9] = (uint8_t)(stKonfKam.sWzmocnienieR & 0xFF);
		cDane[10] = (uint8_t)(stKonfKam.sWzmocnienieG >> 8);
		cDane[11] = (uint8_t)(stKonfKam.sWzmocnienieG & 0xFF);
		cDane[12] = (uint8_t)(stKonfKam.sWzmocnienieB >> 8);
		cDane[13] = (uint8_t)(stKonfKam.sWzmocnienieB & 0xFF);
		cDane[14] = stKonfKam.chKontrBalBieli;
		cDane[15] = stKonfKam.chNasycenie;
		cDane[16] = (uint8_t)(stKonfKam.sCzasEkspozycji >> 8);		//AEC Long Channel Exposure [19:0]: 0x3500..02
		cDane[17] = (uint8_t)(stKonfKam.sCzasEkspozycji & 0xFF);
		cDane[18] = stKonfKam.chKontrolaExpo;
		cDane[19] = stKonfKam.chTrybyEkspozycji;
		cDane[20] = (uint8_t)(stKonfKam.sAGCLong >> 8);
		cDane[21] = (uint8_t)(stKonfKam.sAGCLong & 0xFF);
		cDane[22] = (uint8_t)(stKonfKam.sAGCVTS >> 8);
		cDane[23] = (uint8_t)(stKonfKam.sAGCVTS & 0xFF);
		cDane[24] = stKonfKam.chKontrolaISP0;		//0x5000
		cDane[25] = stKonfKam.chKontrolaISP1;		//0x50001
		cDane[26] = stKonfKam.chProgUsuwania;		//0x5080 Even CTRL 00 Treshold for even odd  cancelling
		cDane[27] = (uint8_t)(stKonfKam.sAGCAdjust >> 8);
		cDane[28] = (uint8_t)(stKonfKam.sAGCAdjust & 0xFF);
		cDane[29] = stKonfKam.chPoziomEkspozycji;
		cBłąd = WyslijRamke(cAdresZdalny, PK_POB_PAR_KAMERY, 30, cDane, cInterfejs);
		break;

	case PK_UST_PAR_KAMERY:	//ustaw parametry pracy kamery
		stKonfKam.chSzerWy = cDane[0];
		stKonfKam.chWysWy = cDane[1];
		stKonfKam.chSzerWe = cDane[2];
		stKonfKam.chWysWe = cDane[3];
		stKonfKam.chPrzesWyPoz = cDane[4];
		stKonfKam.chPrzesWyPio = cDane[5];
		stKonfKam.chObracanieObrazu = cDane[6];
		stKonfKam.chFormatObrazu = cDane[7];
		stKonfKam.sWzmocnienieR = ((uint16_t)cDane[8] << 8) + cDane[9];
		stKonfKam.sWzmocnienieG = ((uint16_t)cDane[10] << 8) + cDane[11];
		stKonfKam.sWzmocnienieB = ((uint16_t)cDane[12] << 8) + cDane[13];
		stKonfKam.chKontrBalBieli = cDane[14];
		stKonfKam.chNasycenie = cDane[15];
		stKonfKam.sCzasEkspozycji = ((uint16_t)cDane[16] << 8) + cDane[17];
		stKonfKam.chKontrolaExpo = cDane[18];
		stKonfKam.chTrybyEkspozycji = cDane[19];
		stKonfKam.sAGCLong = ((uint16_t)cDane[20] << 8) + cDane[21];
		stKonfKam.sAGCVTS = ((uint16_t)cDane[22] << 8) + cDane[23];
		stKonfKam.chKontrolaISP0 = cDane[24];		//0x5000
		stKonfKam.chKontrolaISP1 = cDane[25];		//0x50001
		stKonfKam.chProgUsuwania = cDane[26];		//0x5080 Even CTRL 00 Treshold for even odd  cancelling
		stKonfKam.sAGCAdjust = ((uint16_t)cDane[27] << 8) + cDane[28];
		stKonfKam.chPoziomEkspozycji = cDane[29];
		cBłąd = UstawKamere(&stKonfKam);	//wersja z rejestrami osobno
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_UST_PAR_KAMERY_GRUP:	//ustaw parametry pracy kamery grupowo
		stKonfKam.chSzerWy = cDane[0];
		stKonfKam.chWysWy = cDane[1];
		stKonfKam.chSzerWe = cDane[2];
		stKonfKam.chWysWe = cDane[3];
		stKonfKam.chPrzesWyPoz = cDane[4];
		stKonfKam.chPrzesWyPio = cDane[5];
		stKonfKam.chObracanieObrazu = cDane[6];
		stKonfKam.chFormatObrazu = cDane[7];
		stKonfKam.sWzmocnienieR = ((uint16_t)cDane[8] << 8) + cDane[9];
		stKonfKam.sWzmocnienieG = ((uint16_t)cDane[10] << 8) + cDane[11];
		stKonfKam.sWzmocnienieB = ((uint16_t)cDane[12] << 8) + cDane[13];
		stKonfKam.chKontrBalBieli = cDane[14];
		stKonfKam.chNasycenie = cDane[15];
		stKonfKam.sCzasEkspozycji = ((uint16_t)cDane[16] << 8) + cDane[17];
		stKonfKam.chKontrolaExpo = cDane[18];
		stKonfKam.chTrybyEkspozycji = cDane[19];
		stKonfKam.sAGCLong = ((uint16_t)cDane[20] << 8) + cDane[21];
		stKonfKam.sAGCVTS = ((uint16_t)cDane[22] << 8) + cDane[23];
		stKonfKam.chKontrolaISP0 = cDane[24];		//0x5000
		stKonfKam.chKontrolaISP1 = cDane[25];		//0x50001
		stKonfKam.chProgUsuwania = cDane[26];		//0x5080 Even CTRL 00 Treshold for even odd  cancelling
		stKonfKam.sAGCAdjust = ((uint16_t)cDane[27] << 8) + cDane[28];
		stKonfKam.chPoziomEkspozycji = cDane[29];
		cBłąd = UstawKamere2(&stKonfKam);	//wersja z grupami rejestrów
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_RESETUJ_KAMERE:
		cBłąd = InicjujKamere();
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_ZAPISZ_BUFOR:
		un8_32.dane8[0] = cDane[0];
		un8_32.dane8[1] = cDane[1];
		sWskBufSektora = un8_32.dane16[0];	//adres bezwzględny bufora
		//przepisz dane 8-bitowe  i zapisz po konwersji w buforze 16-bitowym
		for (uint8_t n=0; n<cDane[2]; n++)	//cDane[2] - rozmiar wyrażony w słowach
		{
			un8_32.dane8[0] = cDane[2*n+3];
			un8_32.dane8[1] = cDane[2*n+4];
			sBuforSektoraFlash[sWskBufSektora + n] = un8_32.dane16[0];
		}
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_ZAPISZ_FLASH: 	//zapisz bufor 256 słów do sektora flash NOR o przekazanym adresie
		for (n=0; n<4; n++)
			un8_32.dane8[n] = cDane[n];	//adres sektora
		cBłąd = ZapiszDaneFlashNOR(un8_32.dane32, sBuforSektoraFlash, ROZMIAR16_BUF_SEKT);
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		sWskBufSektora = 0;
		break;

	case PK_KASUJ_FLASH:	//kasuj sektor 128kB flash
		for (n=0; n<4; n++)
			un8_32.dane8[n] = cDane[n];
		cBłąd = KasujSektorFlashNOR(un8_32.dane32);
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_CZYTAJ_FLASH:	//odczytaj zawartość Flash
		for (uint8_t n=0; n<4; n++)
			un8_32.dane8[n] = cDane[n];	//adres sektora

		if (cDane[4] > ROZMIAR16_BUF_SEKT)	//jeżeli zażądano odczytu więcej niż pomieści bufor sektora to zwróc błąd
		{
			cBłąd = BLAD_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
			break;
		}
		if (2* cDane[4] > ROZMIAR_DANYCH_KOMUNIKACJI)	//jeżeli zażądano odczytu więcej niż pomieści ramka komunikacyjna to zwróc błąd
		{
			cBłąd = BLAD_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
			break;
		}

		CzytajDaneFlashNOR(un8_32.dane32, sBuforSektoraFlash, cDane[4]);
		cBłąd = WyslijRamke(cAdresZdalny, PK_CZYTAJ_FLASH, 2*cDane[4], (uint8_t*)sBuforSektoraFlash, cInterfejs);
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_CZYTAJ_OKRES_TELE:	//odczytaj a APL3 okresy telemetrii: cDane[0] == liczba pozycji okresu telemetrii  do odczytania, cDane[1] == numer zmiennej względen początku
		chRozmiar = cDane[0];
		sPrzesuniecie = cDane[1] + cDane[2] * 0x100;	//indeks pierwszej zmiennej, młodszy przodem
		for (uint8_t n=0; n<chRozmiar; n++)
		{
			cDane[2*n+0] = (uint8_t)(sOkresTelemetrii[sPrzesuniecie+n] & 0x00FF);	//młodszy przodem
			cDane[2*n+1] = (uint8_t)(sOkresTelemetrii[sPrzesuniecie+n] >> 8);
		}
		cBłąd = WyslijRamke(cAdresZdalny, PK_CZYTAJ_OKRES_TELE, 2*chRozmiar, cDane, cInterfejs);
		break;

	case PK_ZAPISZ_OKRES_TELE:	//zapisz okresy telemetrii
		sPrzesuniecie = cDane[0] + cDane[1] * 0x100;	//indeks pierwszej zmiennej, młodszy przodem
		chRozmiar = (chRozmDanych - 2) / 2;
		for (uint8_t n=0; n<chRozmiar; n++)
			sOkresTelemetrii[n + sPrzesuniecie] = cDane[2*n+2] + cDane[2*n+3] * 0x100;	//kolejnne okresy telemetrii, młodszy przodem
		cBłąd = ZapiszKonfiguracjeTelemetrii(sPrzesuniecie);
		if (cBłąd == BLAD_OK)
			InicjalizacjaTelemetrii();
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_ZAPISZ_FRAM_U8:	//zapisuje bajty do FRAM
		if (cDane[0] > ROZMIAR_ROZNE_CHAR)	//liczba danych uint8_t
		{
			cBłąd = BLAD_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
			break;
		}
		un8_32.dane8[0] = cDane[1];	//adres zapisu
		un8_32.dane8[1] = cDane[2];
		sAdres = un8_32.dane16[0];						//zapamietaj adres do sprawdzenia czy już się zapisało
		uDaneCM7.dane.sAdres = un8_32.dane16[0];		//adres zapisu bloku liczb
		uDaneCM7.dane.chWykonajPolecenie = POL7_ZAPISZ_FRAM_U8;
		uDaneCM7.dane.chRozmiar = cDane[0];		//ilość liczb uint8_t
		for (n=0; n<cDane[0]; n++)
			uDaneCM7.dane.uRozne.U8[n] = cDane[3+n];
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_ZAPISZ_FRAM_FLOAT:				//Wysyła dane typu float do zapisu we FRAM w rdzeniu CM4 o rozmiarze ROZMIAR_ROZNE_FLOAT
		if (cDane[0] > ROZMIAR_ROZNE_FLOAT)	//liczba danych float (nie uint8_t)
		{
			cBłąd = BLAD_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
			break;
		}
		un8_32.dane8[0] = cDane[1];	//adres zapisu
		un8_32.dane8[1] = cDane[2];
		sAdres = un8_32.dane16[0];					//zapamietaj adres do sprawdzenia czy już się zapisało
		uDaneCM7.dane.sAdres = un8_32.dane16[0];	//adres zapisu bloku liczb
		uDaneCM7.dane.chRozmiar = cDane[0];			//ilość liczb float
		for (n=0; n<cDane[0]; n++)
		{
			for (uint8_t i=0; i<4; i++)
				un8_32.dane8[i] = cDane[3+n*4+i];
			uDaneCM7.dane.uRozne.f32[n] = un8_32.daneFloat;
		}
		uDaneCM7.dane.chWykonajPolecenie = POL7_ZAPISZ_FRAM_FLOAT;
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_WYSLIJ_POTW_ZAPISU:	//jeżeli dane się zapisały to odeslij BLAD_OK. jeeli jeszcze nie to ERR_PROCES_TRWA
		if ((uDaneCM4.dane.chPotwierdzenieWykonania != POL7_ZAPISZ_FRAM_FLOAT) || (uDaneCM4.dane.sAdres != sAdres))
		{
			cBłąd = BLAD_PROCES_TRWA;		//dane jeszcze nie przyszły
			Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
			break;
		}
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		uDaneCM7.dane.chWykonajPolecenie = POL7_NIC;	//wyłącz wykonywanie polecenia POL7_ZAPISZ_FRAM_FLOAT
		break;

	case PK_CZYTAJ_FRAM_U8:
		if (cDane[0] > ROZMIAR_ROZNE_CHAR)
		{
			cBłąd = BLAD_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
			break;
		}
		un8_32.dane8[0] = cDane[1];	//adres do odczytu
		un8_32.dane8[1] = cDane[2];
		sAdres = un8_32.dane16[0];			//zapamiętaj adres
		uDaneCM7.dane.sAdres = un8_32.dane16[0];
		uDaneCM7.dane.chRozmiar = cDane[0];
		uDaneCM7.dane.chWykonajPolecenie = POL7_CZYTAJ_FRAM_U8;
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_CZYTAJ_FRAM_FLOAT:			//odczytaj i wstaw do bufora fRozne[] zawartość FRAM spod podanego adresu w cDane[1..2] o rozmiarze podanym w cDane[0]
		if (cDane[0] > ROZMIAR_ROZNE_FLOAT)
		{
			cBłąd = BLAD_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
			break;
		}
		un8_32.dane8[0] = cDane[1];	//adres do odczytu
		un8_32.dane8[1] = cDane[2];
		sAdres = un8_32.dane16[0];			//zapamiętaj adres
		uDaneCM7.dane.sAdres = un8_32.dane16[0];
		uDaneCM7.dane.chRozmiar = cDane[0];
		uDaneCM7.dane.chWykonajPolecenie = POL7_CZYTAJ_FRAM_FLOAT;
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_WYSLIJ_ODCZYT_FRAM:	//wysyła odczytane wcześniej dane o rozmiarze podanym w cDane[0]
		uDaneCM7.dane.chWykonajPolecenie = POL7_NIC;	//wyłącz wykonywanie polecenia PK_WYSLIJ_ODCZYT_FRAM
		if (uDaneCM4.dane.sAdres != sAdres)
		{
			cBłąd = BLAD_PROCES_TRWA;		//dane jeszcze nie przyszły
			Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
			osDelay(1);	//przełącz wątek aby pozwolić CM4 wykonać polecenie
			break;
		}
		chRozmiar = cDane[0];	//zapamiętaj w zmiennej, bo dane będą nadpisane;
		for (n=0; n<chRozmiar; n++)
			cDane[n] = uDaneCM4.dane.uRozne.U8[n];
		cBłąd = WyslijRamke(cAdresZdalny, PK_WYSLIJ_ODCZYT_FRAM, chRozmiar, cDane, cInterfejs);
		break;

	/*case PK_REKONFIG_SERWA_RC:	//wykonuje ponowną konfigurację wejść i wyjść RC po zmianie zawartosci FRAM
		uDaneCM7.dane.chWykonajPolecenie = POL7_REKONFIG_SERWA_RC;
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;*/

	case PK_ZAMKNIJ_POLACZENIE:	// zamyka połączenie UART, więc zmień stan na STAT_POL_GOTOWY
		if (cInterfejs == INTERF_UART)
		{
			chStatusPolaczenia &= ~(STAT_POL_MASKA_GOT << STAT_POL_UART);
		}
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_ZAPISZ_KONFIG_PID:			//Wysyła dane do zapisu we FRAM w rdzeniu CM4 oraz zapisania do zmienych regulatora
		if ((cDane[0] >= LICZBA_PID) || (chRozmDanych > ROZMIAR_ROZNE_CHAR))	//indeks kanału regulatora nie powinien przekraczać liczby regulatorów i nie powinna zostać przepełniona struktura danych przekazywanych
		{
			cBłąd = BLAD_ZLA_ILOSC_DANYCH;
			Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
			break;
		}
#ifdef TESTY
		assert(chRozmDanych == ROZMIAR_REG_PID - 1);	//przesyłane są liczby float + 1 początkowy bajt będący indeksem regulatora
#endif
		uDaneCM7.dane.chWykonajPolecenie = POL7_ZAPISZ_KONFIG_PID;
		uDaneCM7.dane.chRozmiar = (chRozmDanych - 1) / sizeof(float);		//ilość zmiennych regulatora typu float
		uDaneCM7.dane.sAdres = cDane[0];	//indeks regulatora
		for (n=0; n<uDaneCM7.dane.chRozmiar; n++)
		{
			for (uint8_t i=0; i<sizeof(float); i++)
				un8_32.dane8[i] = cDane[1 + n*sizeof(float) + i];
			uDaneCM7.dane.uRozne.f32[n] = un8_32.daneFloat;
		}
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_ZBIERAJ_EKSTREMA_RC:	//rozpoczyna zbieranie ekstremalnych wartości kanałów obu odbiorników RC
		uDaneCM7.dane.chWykonajPolecenie = POL7_ZBIERAJ_EKSTREMA_RC;
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_ZAPISZ_EKSTREMA_RC:		//kończy zbieranie ekstremalnych wartości kanałów obu odbiorników RC i zapisuje wyniki do FRAM
		uDaneCM7.dane.chWykonajPolecenie = POL7_ZAPISZ_EKSTREMA_RC;
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_ZAPISZ_WYSTER_NAPEDU:		//zapisuje znormalizowane nastawy wysterowania napędu dla wartości minimalnej, zawisu i maksymalnej
#ifdef TESTY
		assert(chRozmDanych == 2*LICZBA_DANYCH_NAPEDU);	//Rozmiar danych jest liczbą przeslanych bajtów a mamy zapisać 3 słowa 16-bitowe
#endif
		for (n=0; n<chRozmDanych / 2; n++)
		{
			for (uint8_t i=0; i<2; i++)
				un8_32.dane8[i] = cDane[n*2+i];
			uDaneCM7.dane.uRozne.U16[n] = un8_32.dane16[0];
		}
		uDaneCM7.dane.chWykonajPolecenie = POL7_ZAPISZ_PWM_NAPEDU;
		uDaneCM7.dane.sAdres = PK_ZAPISZ_WYSTER_NAPEDU;	//fejkowy adres wysyłany w celu uzyskania potwierdzenia zapisu
		uDaneCM7.dane.chRozmiar = chRozmDanych / 2;
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_ZAPISZ_TRYB_REG:
#ifdef TESTY
		assert(chRozmDanych == LICZBA_REG_PARAM);
#endif
		for (n=0; n<chRozmDanych; n++)
			uDaneCM7.dane.uRozne.U8[n] = cDane[n];
		uDaneCM7.dane.chWykonajPolecenie = POL7_ZAPISZ_TRYB_REG;
		uDaneCM7.dane.sAdres = PK_ZAPISZ_TRYB_REG;	//fejkowy adres wysyłany w celu uzyskania potwierdzenia zapisu
		uDaneCM7.dane.chRozmiar = chRozmDanych;
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_RESETUJ_CM4:	//resetuj rdzeń CM4, zwykle po zmianie konfiguracji
		if ((uDaneCM4.dane.chTrybLotu & BTR_UZBROJONY) != BTR_UZBROJONY)	//nie pozwalaj na reset gdy silniki są uzbrojone
		{
			Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
			HAL_Delay(10);
			NVIC_SystemReset();
		}
		else
			Wyslij_KodBledu(BLAD_ODMOWA_WYKONANIA, cPolecenie, cInterfejs);
		break;

	case PK_WSTRZYMAJ_TELEMETRIE:	//wznów lub tymczasowo wstrzymaj wysyłnie telemetrii na czas transmisji innych danych
		chStatusTelemetrii  = cDane[0];		//wartość niezerowa tymczasowo wstrzymuje działanie telemetrii
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_WYLACZ_POLECENIE_CM4:	//wykonuje polecenie POL7_NIC wyłączajac wykonywanie poprzedniego polecenia
		uDaneCM7.dane.chWykonajPolecenie = POL7_NIC;
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_PRZELADUJ_WSKAZN_LED:	//odczytaj konfigurację wskaźników LED z pamieci FRAM i załaduj do zmiennych aby wprowadzona zmiana stała się widoczna
		uDaneCM7.dane.chWykonajPolecenie = POL7_PRZELADUJ_WSKAZN_LED;
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_CZYTAJ_PARAMETRY_FFT:		//odczytaj parametry pracy FFT z APL3
		cDane[0] = stKonfigFFT.chWykladnikPotegi;
		cDane[1] = stKonfigFFT.chRodzajOkna;
		cDane[2] = stKonfigFFT.chAktywnSilniki;
		cDane[3] = stKonfigFFT.chMaxWysterowanie;
		cBłąd = WyslijRamke(cAdresZdalny, PK_CZYTAJ_PARAMETRY_FFT, 4, cDane, cInterfejs);
		break;

	case PK_ZAPISZ_PARAMETRY_FFT:	//zapisz parametry pracy FFT
		stKonfigFFT.chWykladnikPotegi =  cDane[0];
		stKonfigFFT.chRodzajOkna = cDane[1];
		stKonfigFFT.chAktywnSilniki = cDane[2];
		stKonfigFFT.chMaxWysterowanie = cDane[3];
		cBłąd = ZapiszKonfiguracjejFFT();
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_CZYTAJ_WYNIKI_FFT:	//odczytaj z pamiędci DRAM wyniki serii testów FFT dla akcelerometrów i żyroskopów. Prześlij je jako float o połowie precyzji
		uint8_t chRozmiar = cDane[0];				//liczba wyników FFT typu float w ramce (32)
		uint8_t chIndeksRamkiWyniku = cDane[1];	//wskazuje na ramkę danych w ramach jednego FFT [0..63] =[(32/32)-1..(2048/32)-1]
		uint8_t chIndeksZmiennej = cDane[2];		//indeks kolejnej zmiennej [0..5]
		uint8_t chLiniaWodospadu = cDane[3];		//numer kolejnego FFT tworzącego wodospad [0..99]
		uint16_t sIndeksWyniku = (uint16_t)chRozmiar * chIndeksRamkiWyniku;

		for (uint8_t n=0; n<chRozmiar; n++)
			Float2Char16(fWynikFFT[chLiniaWodospadu][chIndeksZmiennej][sIndeksWyniku + n], &cDane[n * 2]);	//konwertuj liczbę float na liczbę o połowie precyzji i zapisz w 2 bajtach
		cBłąd = WyslijRamke(cAdresZdalny, PK_CZYTAJ_PARAMETRY_FFT, chRozmiar * 2, cDane, cInterfejs);
		break;

	case PK_ROZP_ANALIZE_DRGAN:
		RysujProstokatWypelniony(0, 0, DISP_X_SIZE, DISP_Y_SIZE, CZARNY);	//czyści ekran
		cBłąd = RozpocznijAnalizęDrgań(&stKonfigFFT, &chTrybPracy);
		Wyslij_KodBledu(cBłąd, cPolecenie, cInterfejs);
		break;

	case PK_ZATRZYMAJ_SILNIKI:	//zatrzymuje silniki w trakcie testu FFT
		uDaneCM7.dane.uRozne.U8[0] = LICZBA_TESTOW_FFT;	//bieżące etap badania: 0..LICZBA_TESTOW_FFT
		uDaneCM7.dane.uRozne.U8[1] = 0;		//stKonfigFFT->chAktywnSilniki: każdy bit zmiennej odpowiada silnikowi. Wartość 0 zatrzymuje wszystko
		uDaneCM7.dane.uRozne.U8[2] = 0;		//stKonfigFFT->chMaxWysterowanie;
		uDaneCM7.dane.chWykonajPolecenie = POL7_WYSTERUJ_SILNIKI_AD;
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_PRZELADUJ_KONF_PID:	//przeładuj konfigurację PID po zmianie
		uDaneCM7.dane.chWykonajPolecenie = POL7_PRZELADUJ_PID;
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_REKONFIG_WEJSCIA_RC:	//wykonuje ponowną konfigurację wejść RC po zmianie konfiguracji we FRAM
		uDaneCM7.dane.chWykonajPolecenie = POL7_REKONFIG_WEJSCIA_RC;
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_REKONFIG_WYJSCIA_RC:	//wykonuje ponowną konfigurację wyjść RC po zmianie konfiguracji we FRAM
		uDaneCM7.dane.chWykonajPolecenie = POL7_REKONFIG_WYJSCIA_RC;
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	case PK_URUCHOM_INDENT_SILN:	//uruchamia proces identyfikacji silników, kręcąc kolejno każdym z nich
		for (uint8_t n=0; n<4; n++)	//dane 0..1 zawierają wartość wysterowania silników, dane 2..3 czas pracy
			uDaneCM7.dane.uRozne.U8[n] = cDane[n];
		stIdentSiln.sWysterowanie = uDaneCM7.dane.uRozne.U16[0];
		stIdentSiln.sCzasIdent = uDaneCM7.dane.uRozne.U16[1];
		cBłąd = RozpocznijIdentyfikacjęSilników(&stIdentSiln, &chTrybPracy);
		Wyslij_KodBledu(BLAD_OK, cPolecenie, cInterfejs);
		break;

	}
    return cBłąd;
}
