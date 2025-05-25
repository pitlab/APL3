//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł osługi telemetrii
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "telemetria.h"
#include "wymiana_CM7.h"
#include "czas.h"
#include "flash_konfig.h"
#include "protokol_kom.h"
#include "polecenia_komunikacyjne.h"

// Dane telemetryczne są wysyłane w zbiorczej ramce. Na początku ramki znajdują się słowa identyfikujące rodzaj przesyłanych danych, gdzie kolejne bity
// określają rodzaj przesyłanych zmiennych. Każda zmienna może mieć zdefiniowany inny okres wysyłania będący wielokrotnością KWANT_CZASU_TELEMETRII
// dla KWANT_CZASU_TELEMETRII == 10ms daje to max 100Hz, min 0,4Hz
// Maksymalnie w ramce można przesłać 112 zmiennych 16-bitowych + 14 bajtów informacji o zmiennych


ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM4")))	chRamkaTelemetrii[2][ROZMIAR_RAMKI_UART]);	//ramki telemetryczne: przygotowywana i wysyłana
uint8_t chOkresTelem[LICZBA_ZMIENNYCH_TELEMETRYCZNYCH];	//zmienna definiujaca okres wysyłania telemetrii dla wszystkich zmiennych
uint8_t chLicznikTelem[LICZBA_ZMIENNYCH_TELEMETRYCZNYCH];
uint8_t chIndeksNapelnRamki;	//okresla ktora tablica ramki telemetrycznej jest napełniania

extern unia_wymianyCM4_t uDaneCM4;
extern uint8_t chAdresZdalny[ILOSC_INTERF_KOM];	//adres sieciowy strony zdalnej
extern stBSP_t stBSP;						//własny adres sieciowy
extern UART_HandleTypeDef hlpuart1;
extern un8_16_t un8_16;	//unia do konwersji między danymi 16 i 8 bit
extern volatile uint8_t chUartKomunikacjiZajety;

uint8_t chOdczytano, chDoOdczytu = LICZBA_ZMIENNYCH_TELEMETRYCZNYCH;

////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjalizuje zmienne używane do obsługi telemetrii
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjalizacjaTelemetrii(void)
{
	uint8_t chPaczka[ROZMIAR_PACZKI_KONF8];
	uint8_t chOdczytano;
	uint8_t chDoOdczytu = LICZBA_ZMIENNYCH_TELEMETRYCZNYCH;
	uint8_t chIndeksPaczki = 0;
	uint8_t chProbOdczytu = 5;

	//Inicjuj zmienne wartościa wyłączoną gdyby nie dało się odczytać konfiguracji.
	//Lepiej jest mieć telemetrię wyłaczoną niż zapchaną pracujacą na 100%
	for (uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
		chOkresTelem[n] = TEMETETRIA_WYLACZONA;

	while (chDoOdczytu && chProbOdczytu)		//czytaj kolejne paczki aż skompletuje tyle danych ile potrzeba
	{
		chOdczytano = CzytajPaczkeKonfigu(chPaczka, FKON_OKRES_TELEMETRI1 + chIndeksPaczki);		//odczytaj 30 bajtów danych + identyfikator i CRC
		if (chOdczytano == ROZMIAR_PACZKI_KONF8)
		{
			for (uint16_t n=0; n<ROZMIAR_PACZKI_KONF8 - 2; n++)
			{
				if (chDoOdczytu)	//nie czytaj wiecej niż trzeba zby nie przepelnić zmiennej
				{
					chOkresTelem[n + chIndeksPaczki * 30] = chPaczka[n+2];
					chDoOdczytu--;
				}
			}
			chIndeksPaczki++;
		}
		chProbOdczytu--;
	}

	//inicjuj licznik okresem wysyłania
	for (uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
		chLicznikTelem[n] = chOkresTelem[n];
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja obsługuje przygotownie i wysyłkę zmiennych telemetrycznych na zewnatrz
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObslugaTelemetrii(uint8_t chInterfejs)
{
	//uint64_t lIdnetyfikatorZmiennej, lListaZmiennych = 0;
	uint8_t chLicznikZmienych = 0;
	uint8_t chRozmiarRamki = 0;
	float fZmienna;

	//wyczyść pola w ramce na bity identyfikujące zmienne, bo kolejne bity będą OR-owane z wartoscią początkową, któa musi być zerowa
	for(uint16_t n=0; n<LICZBA_BAJTOW_ID_TELEMETRII; n++)
		chRamkaTelemetrii[chIndeksNapelnRamki][ROZMIAR_NAGLOWKA + n] = 0;

	for(uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
	{
		if (chLicznikTelem[n] != 0xFF)	//wartość oznaczająca żeby nie wysyłać danych
			chLicznikTelem[n]--;

		if (chLicznikTelem[n] == 0)
		{
			chLicznikTelem[n] = chOkresTelem[n];
			fZmienna = PobierzZmiennaTele(n);
			if (chRozmiarRamki < (ROZMIAR_RAMKI_UART - ROZMIAR_CRC - 2))	//sprawdź czy dane mieszczą się w ramce
			{
				chRozmiarRamki = WstawDaneDoRamkiTele(chIndeksNapelnRamki, chLicznikZmienych, n, fZmienna);
				chLicznikZmienych++;
			}
		}
	}

	if (chLicznikZmienych)	//jeżeli jest coś do wysłania
	{
		chRozmiarRamki = PrzygotujRamkeTele(chIndeksNapelnRamki, chAdresZdalny[chInterfejs], stBSP.chAdres, chLicznikZmienych);	//utwórz ciało ramki gotowe do wysyłk
		chUartKomunikacjiZajety = 1;
		HAL_UART_Transmit_DMA(&hlpuart1, &chRamkaTelemetrii[chIndeksNapelnRamki][0], (uint16_t)chRozmiarRamki);	//wyślij ramkę

		//wskaż na następną ramkę
		chIndeksNapelnRamki++;
		chIndeksNapelnRamki &= 0x01;
	}
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja wstawia do bieżącego bufora telemetrii liczbę do wysłania
// Parametry: fDane - liczba do wysłania
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
float PobierzZmiennaTele(uint64_t lZmienna)
{
	float fZmiennaTelem;

	switch (lZmienna)
	{
	case TELEID_AKCEL1X:	fZmiennaTelem = uDaneCM4.dane.fAkcel1[0];		break;
	case TELEID_AKCEL1Y:	fZmiennaTelem = uDaneCM4.dane.fAkcel1[1];		break;
	case TELEID_AKCEL1Z:	fZmiennaTelem = uDaneCM4.dane.fAkcel1[2];		break;
	case TELEID_AKCEL2X:	fZmiennaTelem = uDaneCM4.dane.fAkcel2[0];		break;
	case TELEID_AKCEL2Y:	fZmiennaTelem = uDaneCM4.dane.fAkcel2[1];		break;
	case TELEID_AKCEL2Z:	fZmiennaTelem = uDaneCM4.dane.fAkcel2[2];		break;
	case TELEID_ZYRO1P:		fZmiennaTelem = uDaneCM4.dane.fKatZyro1[0];		break;
	case TELEID_ZYRO1Q:		fZmiennaTelem = uDaneCM4.dane.fKatZyro1[1];		break;
	case TELEID_ZYRO1R:		fZmiennaTelem = uDaneCM4.dane.fKatZyro1[2];		break;
	case TELEID_ZYRO2P:		fZmiennaTelem = uDaneCM4.dane.fKatZyro2[0];		break;
	case TELEID_ZYRO2Q:		fZmiennaTelem = uDaneCM4.dane.fKatZyro2[1];		break;
	case TELEID_ZYRO2R:		fZmiennaTelem = uDaneCM4.dane.fKatZyro2[2];		break;
	case TELEID_MAGNE1X:	fZmiennaTelem = uDaneCM4.dane.fMagne1[0];		break;
	case TELEID_MAGNE1Y:	fZmiennaTelem = uDaneCM4.dane.fMagne1[1];		break;
	case TELEID_MAGNE1Z:	fZmiennaTelem = uDaneCM4.dane.fMagne1[2];		break;
	case TELEID_MAGNE2X:	fZmiennaTelem = uDaneCM4.dane.fMagne2[0];		break;
	case TELEID_MAGNE2Y:	fZmiennaTelem = uDaneCM4.dane.fMagne2[1];		break;
	case TELEID_MAGNE2Z:	fZmiennaTelem = uDaneCM4.dane.fMagne2[2];		break;
	case TELEID_MAGNE3X:	fZmiennaTelem = uDaneCM4.dane.fMagne3[0];		break;
	case TELEID_MAGNE3Y:	fZmiennaTelem = uDaneCM4.dane.fMagne3[1];		break;
	case TELEID_MAGNE3Z:	fZmiennaTelem = uDaneCM4.dane.fMagne3[2];		break;
	case TELEID_TEMPIMU1:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_IMU1];		break;
	case TELEID_TEMPIMU2:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_IMU1];		break;

	//zmienne AHRS
	case TELEID_KAT_IMU1X:	fZmiennaTelem = uDaneCM4.dane.fKatIMU1[0];		break;
	case TELEID_KAT_IMU1Y:	fZmiennaTelem = uDaneCM4.dane.fKatIMU1[1];		break;
	case TELEID_KAT_IMU1Z:	fZmiennaTelem = uDaneCM4.dane.fKatIMU1[2];		break;
	case TELEID_KAT_IMU2X:	fZmiennaTelem = uDaneCM4.dane.fKatIMU2[0];		break;
	case TELEID_KAT_IMU2Y:	fZmiennaTelem = uDaneCM4.dane.fKatIMU2[1];		break;
	case TELEID_KAT_IMU2Z:	fZmiennaTelem = uDaneCM4.dane.fKatIMU2[2];		break;
	case TELEID_KAT_ZYRO1X:	fZmiennaTelem = uDaneCM4.dane.fKatZyro1[0];		break;
	case TELEID_KAT_ZYRO1Y:	fZmiennaTelem = uDaneCM4.dane.fKatZyro1[1];		break;
	case TELEID_KAT_ZYRO1Z:	fZmiennaTelem = uDaneCM4.dane.fKatZyro1[2];		break;
	case TELEID_KAT_AKCELX:	fZmiennaTelem = uDaneCM4.dane.fKatAkcel1[0];	break;
	case TELEID_KAT_AKCELY:	fZmiennaTelem = uDaneCM4.dane.fKatAkcel1[1];	break;
	case TELEID_KAT_AKCELZ:	fZmiennaTelem = uDaneCM4.dane.fKatAkcel1[2];	break;

	//zmienne barametryczne
	case TELEID_CISBEZW1:	fZmiennaTelem = uDaneCM4.dane.fCisnie[0];		break;
	case TELEID_CISBEZW2:	fZmiennaTelem = uDaneCM4.dane.fCisnie[1];		break;
	case TELEID_WYSOKOSC1:	fZmiennaTelem = uDaneCM4.dane.fWysoko[0];		break;
	case TELEID_WYSOKOSC2:	fZmiennaTelem = uDaneCM4.dane.fWysoko[1];		break;
	case TELEID_CISROZN1:	fZmiennaTelem = uDaneCM4.dane.fCisnRozn[0];		break;
	case TELEID_CISROZN2:	fZmiennaTelem = uDaneCM4.dane.fCisnRozn[1];		break;
	case TELEID_PREDIAS1:	fZmiennaTelem = uDaneCM4.dane.fPredkosc[0];		break;
	case TELEID_PREDIAS2:	fZmiennaTelem = uDaneCM4.dane.fPredkosc[1];		break;
	case TELEID_TEMPCISB1:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_BARO1];		break;
	case TELEID_TEMPCISB2:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_BARO2];		break;
	case TELEID_TEMPCISR1:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_CISR1];		break;
	case TELEID_TEMPCISR2:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_CISR2];		break;

	default: fZmiennaTelem = -1.0;
	}
	return fZmiennaTelem;
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja wstawia do bieżącej ramki telemetrii liczbę do wysłania
// Parametry:
// chIndNapRam - Indeks napełnianej ramki (numer pierwszej tablicy)
// chPozycja - kolejny nymer zmiennej w ramce
// chIdZmiennej - identyfikator typu zmiennej
// fDane - liczba do wysłania
// Zwraca: rozmiar ramki
////////////////////////////////////////////////////////////////////////////////
uint8_t WstawDaneDoRamkiTele(uint8_t chIndNapRam, uint8_t chPozycja, uint8_t chIdZmiennej, float fDane)
{
	uint8_t chDane[2];
	uint8_t chRozmiar;
	uint8_t chBajtBitu;	//numer bajtu w ktorym jest bit identyfikacyjny

	Float2Char16(fDane, chDane);	//konwertuj liczbę float na liczbę o połowie precyzji i zapisz w 2 bajtach

	//wstaw dane
	chRozmiar = ROZMIAR_NAGLOWKA + LICZBA_BAJTOW_ID_TELEMETRII + 2 * chPozycja;
	chRamkaTelemetrii[chIndNapRam][chRozmiar + 0] = chDane[0];
    chRamkaTelemetrii[chIndNapRam][chRozmiar + 1] = chDane[1];

    //wstaw bit identyfikatora zmiennej
    chBajtBitu = chIdZmiennej / 8;
    chRamkaTelemetrii[chIndNapRam][ROZMIAR_NAGLOWKA + chBajtBitu] |= 1 << (chIdZmiennej - (chBajtBitu * 8));
    return chRozmiar + 2;
}



///////////////////////////////////////////////////////////////////////////////
// Tworzy nagłówek ramki telemetrii. Inicjuje CRC
// Parametry: chIndNapRam - indeks napełnianiej ramki (jedna jest napełniana, druga sie wysyła)
// chAdrZdalny - adres urządzenia do którego wysyłamy
// chAdrLokalny - nasz adres
// lListaZmiennych - zmienna z polam bitowymi okreslającymi przesyłane zmienne
// chRozmDanych - liczba zmiennych telemetrycznych do wysłania w ramce
// Zwraca: rozmiar ramki
////////////////////////////////////////////////////////////////////////////////
uint8_t PrzygotujRamkeTele(uint8_t chIndNapRam, uint8_t chAdrZdalny, uint8_t chAdrLokalny, uint8_t chRozmDanych)
{
	uint32_t nCzasSystemowy = PobierzCzasT6();

	InicjujCRC16(0, WIELOMIAN_CRC);
	chRamkaTelemetrii[chIndNapRam][0] = NAGLOWEK;
	chRamkaTelemetrii[chIndNapRam][1] = CRC->DR = chAdrZdalny;
	chRamkaTelemetrii[chIndNapRam][2] = CRC->DR = chAdrLokalny;
	chRamkaTelemetrii[chIndNapRam][3] = CRC->DR = (nCzasSystemowy / 10) & 0xFF;
	chRamkaTelemetrii[chIndNapRam][4] = CRC->DR = PK_TELEMETRIA;
	chRamkaTelemetrii[chIndNapRam][5] = CRC->DR = chRozmDanych * 2 + LICZBA_BAJTOW_ID_TELEMETRII;

	//policz CRC z danych i listy zmiennych
	for(uint16_t n=0; n < (chRozmDanych * 2 + LICZBA_BAJTOW_ID_TELEMETRII); n++)
		CRC->DR = chRamkaTelemetrii[chIndNapRam][ROZMIAR_NAGLOWKA + n];

	un8_16.dane16 = (uint16_t)CRC->DR;
	chRamkaTelemetrii[chIndNapRam][ROZMIAR_NAGLOWKA + LICZBA_BAJTOW_ID_TELEMETRII + chRozmDanych * 2 + 0] =  un8_16.dane8[1];	//starszy
	chRamkaTelemetrii[chIndNapRam][ROZMIAR_NAGLOWKA + LICZBA_BAJTOW_ID_TELEMETRII + chRozmDanych * 2 + 1] =  un8_16.dane8[0];	//młodszy

	return chRozmDanych * 2 + LICZBA_BAJTOW_ID_TELEMETRII + ROZMIAR_NAGLOWKA + ROZMIAR_CRC;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje konwersję z float na tablicę 2 znaków (float o połowie precyzji)
// znaczenie bitów obu formatów float, 32 bitowego pojedyńczej precyzji i 16 bitowego połowy precyzji
// gdzie z=znak, c=cecha, m=mantysa
// float32: zccccccc cmmmmmmm mmmmmmmm mmmmmmmm	(1+8+23)
// float16: zcccccmm mmmmmmmm (1+5+10)
// Parametry:
// [i] fData - zmienna float do konwersji
// [o] *chData - wskaźnik na tablicę znaków
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void Float2Char16(float fData, uint8_t* chData)
{
    typedef union
    {
    	uint8_t array[sizeof(float)];
	float fuData;
    } fUnion;
    fUnion temp;
    volatile uint8_t chCecha;

    temp.fuData = fData;
    *(chData+1) = temp.array[3] & 0x80;   //znak liczby
    chCecha = ((temp.array[3] & 0x7F)<<1) + ((temp.array[2] & 0x80)>>7);
    chCecha -= (127-15);
    if (chCecha > 127)
      chCecha = 1;          //gdy cecha poza zakresem
    else
      chCecha &= 0x1F;
    *(chData+1) += chCecha<<2;
    *(chData+1) += (temp.array[2] & 0x60)>>5;   //mantysa

    *(chData+0) =  (temp.array[2] & 0x1F)<<3;
    *(chData+0) += (temp.array[1] & 0xE0)>>5;
 }



////////////////////////////////////////////////////////////////////////////////
// Zapisuje do FLASH okres wysyłania kolenych zmiennych telemetrycznych
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszKonfiguracjeTelemetrii(void)
{
	//uint8_t chPaczka[ROZMIAR_PACZKI_KONF8];
	uint8_t chDoZapisu = LICZBA_ZMIENNYCH_TELEMETRYCZNYCH;
	uint8_t chIndeksPaczki = 0;
	uint8_t chProbZapisu = 5;
	uint8_t chErr;

	while (chDoZapisu && chProbZapisu)		//czytaj kolejne paczki aż skompletuje tyle danych ile potrzeba
	{
		chErr = ZapiszPaczkeKonfigu(FKON_OKRES_TELEMETRI1 + chIndeksPaczki, &chOkresTelem[chIndeksPaczki * ROZMIAR_DANYCH_WPACZCE]);
		if (chErr == ERR_OK)
		{
			chIndeksPaczki++;
			if (chDoZapisu > ROZMIAR_DANYCH_WPACZCE)
				chDoZapisu -= ROZMIAR_DANYCH_WPACZCE;
			else
				chDoZapisu = 0;
		}
		chProbZapisu--;
	}
	return chErr;
}
