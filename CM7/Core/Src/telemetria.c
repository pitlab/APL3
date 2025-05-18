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
extern uint8_t chAdresLokalny;						//własny adres sieciowy
extern UART_HandleTypeDef hlpuart1;
extern un8_16_t un8_16;	//unia do konwersji między danymi 16 i 8 bit
extern volatile uint8_t chLPUartZajety;


////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjalizuje zmienne używane do obsługi telemetrii
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjalizacjaTelemetrii(void)
{
	uint8_t chPaczka[ROZMIAR_PACZKI_KONF8];
	uint8_t chOdczytano, chDoOdczytu = LICZBA_ZMIENNYCH_TELEMETRYCZNYCH;
	uint8_t chIndeksPaczki = 0;
	uint8_t chProbOdczytu = 5;

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

	for (uint16_t n=0; n<3; n++)
		chOkresTelem[n] = 2;

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
	uint64_t lIdnetyfikatorZmiennej, lListaZmiennych = 0;
	uint8_t chLicznikZmienych = 0;
	uint8_t chRozmiarRamki = 0;
	float fZmienna;

	for(uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
	{
		if (chLicznikTelem[n] != 0xFF)	//wartość oznaczająca żeby nie wysyłać danych
			chLicznikTelem[n]--;

		if (chLicznikTelem[n] == 0)
		{
			chLicznikTelem[n] = chOkresTelem[n];
			lIdnetyfikatorZmiennej = 0x01 << n;
			fZmienna = PobierzZmiennaTele(lIdnetyfikatorZmiennej);
			if (chRozmiarRamki < (ROZMIAR_RAMKI_UART - ROZMIAR_CRC - 2))	//sprawdź czy dane mieszczą się w ramce
			{
				lListaZmiennych |= lIdnetyfikatorZmiennej;
				chRozmiarRamki = WstawDoRamkiTele(chIndeksNapelnRamki, chLicznikZmienych, fZmienna);
				chLicznikZmienych++;
			}
		}
	}

	if (lListaZmiennych)	//jeżeli jest coś do wysłania
	{
		chRozmiarRamki = PrzygotujRamkeTele(chIndeksNapelnRamki, chAdresZdalny[chInterfejs], chAdresLokalny, lListaZmiennych, chLicznikZmienych);	//utwórz ciało ramki gotowe do wysyłk
		chLPUartZajety = 1;
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
	case TELEM1_AKCEL1X:	fZmiennaTelem = uDaneCM4.dane.fAkcel1[0];		break;
	case TELEM1_AKCEL1Y:	fZmiennaTelem = uDaneCM4.dane.fAkcel1[1];		break;
	case TELEM1_AKCEL1Z:	fZmiennaTelem = uDaneCM4.dane.fAkcel1[2];		break;
	case TELEM1_AKCEL2X:	fZmiennaTelem = uDaneCM4.dane.fAkcel2[0];		break;
	case TELEM1_AKCEL2Y:	fZmiennaTelem = uDaneCM4.dane.fAkcel2[1];		break;
	case TELEM1_AKCEL2Z:	fZmiennaTelem = uDaneCM4.dane.fAkcel2[2];		break;
	case TELEM1_ZYRO1P:		fZmiennaTelem = uDaneCM4.dane.fKatZyro1[0];		break;
	case TELEM1_ZYRO1Q:		fZmiennaTelem = uDaneCM4.dane.fKatZyro1[1];		break;
	case TELEM1_ZYRO1R:		fZmiennaTelem = uDaneCM4.dane.fKatZyro1[2];		break;
	case TELEM1_ZYRO2P:		fZmiennaTelem = uDaneCM4.dane.fKatZyro2[0];		break;
	case TELEM1_ZYRO2Q:		fZmiennaTelem = uDaneCM4.dane.fKatZyro2[1];		break;
	case TELEM1_ZYRO2R:		fZmiennaTelem = uDaneCM4.dane.fKatZyro2[2];		break;
	case TELEM1_MAGNE1X:	fZmiennaTelem = uDaneCM4.dane.fMagne1[0];		break;
	case TELEM1_MAGNE1Y:	fZmiennaTelem = uDaneCM4.dane.fMagne1[1];		break;
	case TELEM1_MAGNE1Z:	fZmiennaTelem = uDaneCM4.dane.fMagne1[2];		break;
	case TELEM1_MAGNE2X:	fZmiennaTelem = uDaneCM4.dane.fMagne2[0];		break;
	case TELEM1_MAGNE2Y:	fZmiennaTelem = uDaneCM4.dane.fMagne2[1];		break;
	case TELEM1_MAGNE2Z:	fZmiennaTelem = uDaneCM4.dane.fMagne2[2];		break;
	case TELEM1_MAGNE3X:	fZmiennaTelem = uDaneCM4.dane.fMagne3[0];		break;
	case TELEM1_MAGNE3Y:	fZmiennaTelem = uDaneCM4.dane.fMagne3[1];		break;
	case TELEM1_MAGNE3Z:	fZmiennaTelem = uDaneCM4.dane.fMagne3[2];		break;
	case TELEM1_KAT_IMU1X:	fZmiennaTelem = uDaneCM4.dane.fKatIMU1[0];		break;
	case TELEM1_KAT_IMU1Y:	fZmiennaTelem = uDaneCM4.dane.fKatIMU1[1];		break;
	case TELEM1_KAT_IMU1Z:	fZmiennaTelem = uDaneCM4.dane.fKatIMU1[2];		break;
	case TELEM1_KAT_IMU2X:	fZmiennaTelem = uDaneCM4.dane.fKatIMU2[0];		break;
	case TELEM1_KAT_IMU2Y:	fZmiennaTelem = uDaneCM4.dane.fKatIMU2[1];		break;
	case TELEM1_KAT_IMU2Z:	fZmiennaTelem = uDaneCM4.dane.fKatIMU2[2];		break;
	case TELEM1_KAT_ZYRO1X:	fZmiennaTelem = uDaneCM4.dane.fKatZyro1[0];		break;
	case TELEM1_KAT_ZYRO1Y:	fZmiennaTelem = uDaneCM4.dane.fKatZyro1[1];		break;
	case TELEM1_KAT_ZYRO1Z:	fZmiennaTelem = uDaneCM4.dane.fKatZyro1[2];		break;

	case TELEM2_CISBEZW1:	fZmiennaTelem = uDaneCM4.dane.fCisnie[0];		break;
	case TELEM2_CISBEZW2:	fZmiennaTelem = uDaneCM4.dane.fCisnie[1];		break;
	case TELEM2_WYSOKOSC1:	fZmiennaTelem = uDaneCM4.dane.fWysoko[0];		break;
	case TELEM2_WYSOKOSC2:	fZmiennaTelem = uDaneCM4.dane.fWysoko[1];		break;
	case TELEM2_CISROZN1:	fZmiennaTelem = uDaneCM4.dane.fCisnRozn[0];		break;
	case TELEM2_CISROZN2:	fZmiennaTelem = uDaneCM4.dane.fCisnRozn[1];		break;
	case TELEM2_PREDIAS1:	fZmiennaTelem = uDaneCM4.dane.fPredkosc[0];		break;
	case TELEM2_PREDIAS2:	fZmiennaTelem = uDaneCM4.dane.fPredkosc[1];		break;
	case TELEM2_TEMPCIS1:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_BARO1];		break;
	case TELEM2_TEMPCIS2:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_BARO2];		break;
	case TELEM2_TEMPIMU1:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_IMU1];		break;
	case TELEM2_TEMPIMU2:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_IMU1];		break;
	case TELEM2_TEMPCISR1:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_CISR1];		break;
	case TELEM2_TEMPCISR2:	fZmiennaTelem = uDaneCM4.dane.fTemper[TEMP_CISR2];		break;

	default: fZmiennaTelem = -1.0;
	}
	return fZmiennaTelem;
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja wstawia do bieżącej ramki telemetrii liczbę do wysłania
// Parametry: fDane - liczba do wysłania
// Zwraca: rozmiar ramki
////////////////////////////////////////////////////////////////////////////////
uint8_t WstawDoRamkiTele(uint8_t chIndNapRam, uint8_t chPozycja, float fDane)
{
	uint8_t chDane[2];
	uint8_t chRozmiar;

	Float2Char16(fDane, chDane);	//konwertuj liczbę float na liczbę o połowie precyzji i zapisz w 2 bajtach

	chRozmiar = ROZMIAR_NAGLOWKA + LICZBA_BAJTOW_ID_TELEMETRII + 2 * chPozycja;
	chRamkaTelemetrii[chIndNapRam][chRozmiar + 0] = chDane[0];
    chRamkaTelemetrii[chIndNapRam][chRozmiar + 1] = chDane[1];
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
uint8_t PrzygotujRamkeTele(uint8_t chIndNapRam, uint8_t chAdrZdalny, uint8_t chAdrLokalny, uint64_t lListaZmiennych, uint8_t chRozmDanych)
{
	uint32_t nCzasSystemowy = PobierzCzasT6();

	InicjujCRC16(0, WIELOMIAN_CRC);
	chRamkaTelemetrii[chIndNapRam][0] = NAGLOWEK;
	chRamkaTelemetrii[chIndNapRam][1] = CRC->DR = chAdrZdalny;
	chRamkaTelemetrii[chIndNapRam][2] = CRC->DR = chAdrLokalny;
	chRamkaTelemetrii[chIndNapRam][3] = CRC->DR = (nCzasSystemowy / 10) & 0xFF;
	chRamkaTelemetrii[chIndNapRam][4] = CRC->DR = PK_TELEMETRIA;
	chRamkaTelemetrii[chIndNapRam][5] = CRC->DR = chRozmDanych * 2 + LICZBA_BAJTOW_ID_TELEMETRII;

	//wstaw listę zmiennych na początku pola danych
	for(uint16_t n=0; n<LICZBA_BAJTOW_ID_TELEMETRII; n++)
		chRamkaTelemetrii[chIndeksNapelnRamki][ROZMIAR_NAGLOWKA + n] = (lListaZmiennych >> (n*8)) & 0xFF;

	//policz CRC z danych i listy zmiennych
	for(uint16_t n=0; n < (chRozmDanych * 2 + LICZBA_BAJTOW_ID_TELEMETRII); n++)
		CRC->DR = chRamkaTelemetrii[chIndeksNapelnRamki][ROZMIAR_NAGLOWKA + n];

	un8_16.dane16 = (uint16_t)CRC->DR;
	chRamkaTelemetrii[chIndeksNapelnRamki][ROZMIAR_NAGLOWKA + LICZBA_BAJTOW_ID_TELEMETRII + chRozmDanych * 2 + 0] =  un8_16.dane8[1];	//starszy
	chRamkaTelemetrii[chIndeksNapelnRamki][ROZMIAR_NAGLOWKA + LICZBA_BAJTOW_ID_TELEMETRII + chRozmDanych * 2 + 1] =  un8_16.dane8[0];	//młodszy

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
// Zapisuje do FLASH częstotliość wysyłania kolenych zmiennych telemetrycznych
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszKonfiguracjeTelemetrii(void)
{
	uint8_t chPaczka[ROZMIAR_PACZKI_KONF8];
	uint8_t chDoZapisu = LICZBA_ZMIENNYCH_TELEMETRYCZNYCH;
	uint8_t chIndeksPaczki = 0;
	uint8_t chProbZapisu = 5;
	uint8_t chErr;

	while (chDoZapisu && chProbZapisu)		//czytaj kolejne paczki aż skompletuje tyle danych ile potrzeba
	{
		chPaczka[0] = FKON_OKRES_TELEMETRI1 + chIndeksPaczki;
		for (uint16_t n=0; n<ROZMIAR_PACZKI_KONF8 - 2; n++)
		{
			chPaczka[n+2] = chOkresTelem[n + chIndeksPaczki * ROZMIAR_DANYCH_WPACZCE];

			if (chDoZapisu)	//nie zapisuj wiecej niż trzeba zby nie przepelnić zmiennej
				chDoZapisu--;
		}
		chErr = ZapiszPaczkeKonfigu(chPaczka);
		if (chErr == ERR_OK)
			chIndeksPaczki++;
		chProbZapisu--;
	}
	return chErr;
}
