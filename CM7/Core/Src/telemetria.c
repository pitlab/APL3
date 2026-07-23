//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł obsługi telemetrii
//
// (c) PitLab 2025-26
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <Czas.h>
#include <Dotyk.h>
#include <FFT.h>
#include <Telemetria.h>
#include "WymianaCM7.h"
#include "FlashKonfig.h"
#include "ProtokolKomunikacyjny.h"
#include "PoleceniaKomunikacyjne.h"

// Dane telemetryczne są wysyłane w zbiorczej ramce mogącej pomieścić MAX_ZMIENNYCH_TELEMETR_W_RAMCE (115). Dane są z puli adresowej obejmującej MAX_INDEKSOW_TELEMETR_W_RAMCE (128) zmiennych.
// Ponieważ danych może być więcej, przewodziano 2 lub wiecej rodzajów ramek telemetrii
// Na początku ramki znajduje się wielobajtowe słowo (LICZBA_BAJTOW_ID_TELEMETRII), gdzie kolejne bity są identyfikatorami przesyłanych zmiennych.
// Każda zmienna może mieć zdefiniowany inny okres wysyłania będący wielokrotnością KWANT_CZASU_TELEMETRII
// dla KWANT_CZASU_TELEMETRII == 10ms daje to max 100 Hz, min 0,025 Hz (40s)
// Każda ramka ma 2 kopie, gdzie jedna ramka jest napełniana a druga wysyłana.

// Ramka szybka jest specyficznym rodzajem telemetri, gdzie przesyłane są dane produkowane w ilościach większychch niż można było by przesłać normalną telemetrią. Dotyczy to FFT z
// żyroskopów i akcelerometrów, które produkowane są w ilości 6 x szerokośćFFT/2 na każdy obieg pętli. Dane są gromadzone w buforze a szybka ramka opróźnia bufor.
// Zamiast wielobajtowego słowa określającego bity zmiennych są zawarte 16-bitwy indeks numeru próby FFT oraz indeks numeru wyniku w ramach FFT

// LPUART korzysta z BDMA a ono ma dostęp tylko do SRAM4, więc ramka telemetrii musi być zdefiniowana w SRAM4
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4_CM7")))	cRamkaTelemetrii[2 * LICZBA_RAMEK_TELEMETR][ROZMIAR_RAMKI_KOMUNIKACYJNEJ];	//ramki telemetryczne: przygotowywana i wysyłana dla 16-bitowych zmiennych 0..127 oraz 128..255
uint16_t sOkresTelemetrii[LICZBA_ZMIENNYCH_TELEMETRYCZNYCH];	//zmienna definiujaca okres wysyłania telemetrii dla wszystkich zmiennych
uint16_t sLicznikTelemetrii[LICZBA_ZMIENNYCH_TELEMETRYCZNYCH];
uint8_t cStatusTelemetrii;		//określa czy i jaki rodzaj telemetrii ma być wysyłany
extern unia_wymianyCM4_t uDaneCM4;
extern uint8_t cAdresZdalny[ILOSC_INTERF_KOM];	//adres sieciowy strony zdalnej
extern UART_HandleTypeDef hlpuart1;
static unia8_32_t un8_32;		//unia do konwersji między danymi 16 i 8 bit
extern volatile uint8_t cDoWyslania[1 + LICZBA_RAMEK_TELEMETR];	//lista rzeczy do wysłania po zakończeniu bieżącej transmisji: ramka poleceń i ramki telemetryczne
extern stBSP_ID_t stBSP_ID;	//struktura zawierajaca adresy i nazwę BSP
extern volatile st_ZajetośćLPUART_t st_ZajetośćLPUART;
extern stStatusDotyku_t stStatusDotyku;
extern RTC_TimeTypeDef stTime;
extern RTC_DateTypeDef stDate;
extern float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fWynikFFT[LICZBA_TESTOW_FFT][LICZBA_ZMIENNYCH_FFT][FFT_MAX_ROZMIAR / 2];	//wartość sygnału wyjściowego
extern uint16_t sIndeksWysyłkiFFT;		//wskazuje na na numer próbki FFT przesyłany telemetrią
extern uint8_t cIndeksWysyłkiTestuFFT;	//wskazuje na nume testu FFT obecnie wysyłanego telemetrią
extern stFFT_t stKonfigFFT;


////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjalizuje zmienne używane do obsługi telemetrii. Dane są zapisane w porządku cienkokońcówkowym
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjalizacjaTelemetrii(void)
{
	uint8_t cPaczka[ROZMIAR_PACZKI_KONFIGU];
	uint8_t chOdczytano;
	uint8_t cDoOdczytu = LICZBA_ZMIENNYCH_TELEMETRYCZNYCH;
	uint8_t cIndeksPaczki = 0;
	uint8_t cProbOdczytu = PROB_ODCZYTU_TELEMETRII;
	uint16_t sOkres;

	//Inicjuj zmienne wartościa wyłączoną gdyby nie dało się odczytać konfiguracji.
	//Lepiej jest mieć telemetrię wyłaczoną niż zapchaną pracujacą na 100%
	for (uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
		sOkresTelemetrii[n] = TELEMETRIA_WYLACZONA;

	while (cDoOdczytu && cProbOdczytu)		//czytaj kolejne paczki aż skompletuje tyle danych ile potrzeba
	{
		chOdczytano = CzytajPaczkeKonfigu(cPaczka, FKON_OKRES_TELEMETRI1 + cIndeksPaczki);		//odczytaj 30 bajtów danych + identyfikator i CRC
		if (chOdczytano == ROZMIAR_PACZKI_KONFIGU)
		{
			for (uint16_t n=0; n<((ROZMIAR_PACZKI_KONFIGU - 2) / 2); n++)
			{
				if (cDoOdczytu)	//nie czytaj wiecej niż trzeba aby nie przepełnić zmiennej
				{
					sOkres = cPaczka[2*n+2] + cPaczka[2*n+3] * 0x100;
					sOkresTelemetrii[n + cIndeksPaczki * ROZMIAR_DANYCH_WPACZCE /2] = sOkres;
					cDoOdczytu--;
				}
			}
			cIndeksPaczki++;
			cProbOdczytu = PROB_ODCZYTU_TELEMETRII;
		}
		cProbOdczytu--;
	}

	//inicjuj licznik okresem wysyłania
	for (uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
		sLicznikTelemetrii[n] = sOkresTelemetrii[n];
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja przygotowuje i rozpoczyna wysyłkę ramek telemetrycznych
// Parametry: cInterfejs - interfejs komunikacyjny (na razie tylko LPUART)
// Zwraca: ilość wysyłanych danych
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaTelemetrii(uint8_t cInterfejs)
{
	uint8_t cIloscDanych[LICZBA_RAMEK_TELEMETR] = {0, 0};
	uint8_t cIndeksAdresow;	//okresla indeks puli adresowej rakmi. Zmienne 0..127 idą w ramce 0, zmienne 128..255 w ramce 1, itd
	uint8_t cTypRamki;
	uint8_t cWysyłamTyleDanych = 0;	//parametr zwrotny funkcji
	float fZmienna;
	extern uint8_t cStatusPolaczenia;

	if (cStatusTelemetrii == TELEM_WSTRZYMAJ)
		return cWysyłamTyleDanych;

	//szybka telemetria wykorzystuje tylko ramkę RAMKA_TELE1, czyli chIloscDanych[0] oraz cRamkaTelemetrii[0 i 1]
	if (cStatusTelemetrii == TELEM_SZYBKA)
	{
		if (stKonfigFFT.cIndeksTestu > cIndeksWysyłkiTestuFFT)	//czekaj z wysyłką na wyprodukowanie danych przez FFT
		{
			if ((st_ZajetośćLPUART.sDoWysłania[RAMKA_TELE1] == 0) && (st_ZajetośćLPUART.cZajętyPrzez == (int8_t)LPUART_WOLNY))	//dodaj nowe dane tylko do pustej kolejki
			{
				//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);		//serwo kanał 7
				cIloscDanych[0] = WstawDaneDoSzybkiejRamkiTele(st_ZajetośćLPUART.cIndeksNapełnianejRamki[RAMKA_TELE1], cIndeksWysyłkiTestuFFT, &sIndeksWysyłkiFFT);

				//testuj warunek zakończenia procesu wysyłki FFT
				if (sIndeksWysyłkiFFT >= stKonfigFFT.sLiczbaProbek / 2)
				{
					sIndeksWysyłkiFFT = 0;
					cIndeksWysyłkiTestuFFT++;
					if (cIndeksWysyłkiTestuFFT == LICZBA_TESTOW_FFT)
						cStatusTelemetrii = TELEM_NORMALNA;	//po wysłaniu wszystkich wyników FFT wróc do normalnej tlemetrii
				}
				cTypRamki = TELEM_SZYBKA;
			}
		}
	}
	else		//normalna telemetria
	{
		//wyczyść w ramkach pola na bity identyfikujące zmienne, bo kolejne bity będą OR-owane z wartością początkową, więc musi ona na początku być zerem
		for (uint8_t r=0; r<LICZBA_RAMEK_TELEMETR; r++)
		{
			for(uint8_t n=0; n<LICZBA_BAJTOW_ID_TELEMETRII; n++)
				cRamkaTelemetrii[st_ZajetośćLPUART.cIndeksNapełnianejRamki[r+1] + 2 * r][ROZMIAR_NAGLOWKA + n] = 0;
			cIloscDanych[r] = LICZBA_BAJTOW_ID_TELEMETRII;
		}

		for(uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
		{
			if (sLicznikTelemetrii[n] != TELEMETRIA_WYLACZONA)	//wartość oznaczająca aby nie wysyłać danych
			{
				sLicznikTelemetrii[n]--;
				if (sLicznikTelemetrii[n] == 0)		//licznik zmiennej doszedł do 0 więc trzeba ją wysłać
				{
					sLicznikTelemetrii[n] = sOkresTelemetrii[n];		//przeładuj licznik nowym okresem
					fZmienna = PobierzZmiennaTele(n, &uDaneCM4.dane);
					cIndeksAdresow = n >> 7;
					if (cIloscDanych[cIndeksAdresow] < (ROZMIAR_RAMKI_KOMUNIKACYJNEJ - ROZMIAR_CRC - 2))	//sprawdź czy dane mieszczą się w ramce
					{
						WstawDaneDoRamkiTele(st_ZajetośćLPUART.cIndeksNapełnianejRamki[cIndeksAdresow + 1], cIndeksAdresow, cIloscDanych[cIndeksAdresow], n, fZmienna);
						cIloscDanych[cIndeksAdresow] += 2;
					}
				}
			}
		}
		cTypRamki = TELEM_NORMALNA;
	}

	//przygotuj ramki wypełnione danymi  do wysłania dodając nagłówek, taki sam dla każdego typu ramki
	for (uint8_t r=0; r<LICZBA_RAMEK_TELEMETR; r++)
	{
		if (cIloscDanych[r] > LICZBA_BAJTOW_ID_TELEMETRII)	//jeżeli jest coś do wysłania
		{
			PrzygotujRamkeTele(st_ZajetośćLPUART.cIndeksNapełnianejRamki[r+1] + r * LICZBA_RAMEK_TELEMETR, cTypRamki, cAdresZdalny[cInterfejs], stBSP_ID.cAdres, cIloscDanych[r]);	//utwórz ramkę gotową do wysyłki
			st_ZajetośćLPUART.sDoWysłania[r+1] = cIloscDanych[r] + ROZM_CIALA_RAMKI;
		}
	}

	//rozpocznij fizyczną transmisję danych
	__disable_irq();	//sekcja krytyczna wykonywana przy wyłączonych przerwaniach
	if (st_ZajetośćLPUART.cZajętyPrzez == (int8_t)LPUART_WOLNY)	//jeżeli LPUART nie jest zajęty to wyślij telemetrię
	{
		for (uint8_t r=0; r<LICZBA_RAMEK_TELEMETR; r++)
		{
			if (st_ZajetośćLPUART.sDoWysłania[r+1])
			{
				cStatusPolaczenia |= (STAT_POL_PRZESYLA << STAT_POL_UART);		//sygnalizuj transfer danych
				st_ZajetośćLPUART.cZajętyPrzez = RAMKA_TELE1 + r;
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//serwo kanał 1 - LPUART_ZAJETY
				HAL_UART_Transmit_DMA(&hlpuart1, &cRamkaTelemetrii[st_ZajetośćLPUART.cIndeksNapełnianejRamki[r+1] + r * LICZBA_RAMEK_TELEMETR][0], st_ZajetośćLPUART.sDoWysłania[r+1]);	//wyślij ramkę - Uwaga, nie wyśle 2 ramek na raz, zrobić kolejkę wysyłania
				cWysyłamTyleDanych = st_ZajetośćLPUART.sDoWysłania[r+1];
				st_ZajetośćLPUART.sDoWysłania[r+1] = 0;	//wysłano więc zdejmij z kolejki i zezwól na ponowne napełnienie bufora
				st_ZajetośćLPUART.cIndeksNapełnianejRamki[r+1]++;	//przełacz indeks podwójnego buforowania aby można było napełniać drugi bufor
				st_ZajetośćLPUART.cIndeksNapełnianejRamki[r+1] &= 0x01;
				//zdjęcie flagi zajetości LPUART następuje w HAL_UART_TxCpltCallback() po fizycznym zakończeniu wysyłki
				break;	//wyjdź z pętli po wysłaniu pierwszej ramki
			}
		}
	}
	__enable_irq();		//koniec sekcji krytycznej
	return cWysyłamTyleDanych;
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja wstawia do bieżącej ramki telemetrii liczbę do wysłania
// Parametry:
// 	chIndNapRam - Indeks napełnianych ramek (ramki o przeciwnej parzystości są w tym czasie opróżniane)
// 	chIndAdresow - indeks puli adresowej zmiennych telemetrycznych. Adresy 0..127 mają indeks 0, adresy 128..255 indeks 1, itp
// 	chPozycja - miejsce zmiennej w ramce bez uwzględnienia nagłówna
// 	sIdZmiennej - identyfikator typu zmiennej
// 	fDane - liczba do wysłania
// Zwraca: rozmiar ramki
////////////////////////////////////////////////////////////////////////////////
uint8_t WstawDaneDoRamkiTele(uint8_t cIndNapRam, uint8_t cIndAdresow, uint8_t cPozycja, uint16_t sIdZmiennej, float fDane)
{
	uint8_t cDane[2];
	uint8_t cRozmiar;
	uint8_t cIdZmiennej = sIdZmiennej & 0x7F;
	uint8_t cBajtBitu;	//numer bajtu w ktorym jest bit identyfikacyjny

	//wstaw dane
	cRozmiar = ROZMIAR_NAGLOWKA + cPozycja;
	Float2Char16(fDane, cDane);	//konwertuj liczbę float na liczbę o połowie precyzji i zapisz w 2 bajtach
	cRamkaTelemetrii[cIndNapRam + cIndAdresow * LICZBA_RAMEK_TELEMETR][cRozmiar + 0] = cDane[0];
    cRamkaTelemetrii[cIndNapRam + cIndAdresow * LICZBA_RAMEK_TELEMETR][cRozmiar + 1] = cDane[1];

    //wstaw bit identyfikatora zmiennej
    cBajtBitu = cIdZmiennej / 8;
    cRamkaTelemetrii[cIndNapRam + cIndAdresow * LICZBA_RAMEK_TELEMETR][ROZMIAR_NAGLOWKA + cBajtBitu] |= 1 << (cIdZmiennej - (cBajtBitu * 8));
    return cRozmiar + 2;
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja wstawia dane do szybkiej ramki telemetrii. Szybka ramka zawiera tylko 1 rodzaj danych pobierany z bufora.
// Na obecnym etapie dotyczy przesyłania wyników FFT z akcelerometrów i żyroskopów. W ramce są dane ze wszystkich 3 osi obu czujników
// Parametry:
// 	chIndNapRam - Indeks napełnianych ramek (ramki o przeciwnej parzystości są w tym czasie opróżniane)
// 	chIndeksTestu - indeks kolejnego FFT [0..99]
// 	*sIndeksFFT - wskaźnik na indeks próbki w ramach FFT [0..2047]
// Zwraca: rozmiar ramki
////////////////////////////////////////////////////////////////////////////////
uint8_t WstawDaneDoSzybkiejRamkiTele(uint8_t cIndNapRam, uint8_t cIndeksTestu, uint16_t *sIndeksFFT)
{
	uint8_t cBityZmiennej = AKCEL_X | AKCEL_Y | AKCEL_Z | ZYRO_X | ZYRO_Y | ZYRO_Z;
	uint8_t cLiczbaBajtówRamki = 4;

	//pierwsza część danych ramki zawiera 4 bajtową identyfikację danych
	cRamkaTelemetrii[cIndNapRam][ROZMIAR_NAGLOWKA + 0] = cBityZmiennej;
	cRamkaTelemetrii[cIndNapRam][ROZMIAR_NAGLOWKA + 1] = cIndeksTestu;
	un8_32.dane16[0] = *sIndeksFFT;
	cRamkaTelemetrii[cIndNapRam][ROZMIAR_NAGLOWKA + 2] = un8_32.dane8[0];
	cRamkaTelemetrii[cIndNapRam][ROZMIAR_NAGLOWKA + 3] = un8_32.dane8[1];

	//wstaw tyle danych FFT aby wypełnić ramkę, ale dane mają pochodzić tylko z jednego FFT, bo indeks testu idzie na początku danych ramki
	for (uint16_t n=0; n<((ROZMIAR_DANYCH_KOMUNIKACJI - 4) / 12); n++)	//dla uproszczenia zakładam że ramka ma komplet 6 danych 16-bit
	{
		cLiczbaBajtówRamki += PobierzWynikiFFT(&cRamkaTelemetrii[cIndNapRam][n * 12 + 4 + ROZMIAR_NAGLOWKA], cBityZmiennej, cIndeksTestu, *sIndeksFFT);
		(*sIndeksFFT)++;
		if (*sIndeksFFT >= stKonfigFFT.sLiczbaProbek / 2)	//koniec danych nie musi być równoważny z pełną ramką, więc...
			break;										//...przerwij napełnianie kiedy zostały przygotowane wszystkie dane
	}
	return cLiczbaBajtówRamki;
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja wstawia do bieżącego bufora telemetrii liczbę do wysłania
// Parametry: fDane - liczba do wysłania
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
float PobierzZmiennaTele(uint16_t sZmienna, stWymianyCM4_t *stDane)
{
	float fZmiennaTele;

	switch (sZmienna)
	{
	case TID_AKCEL1X:		fZmiennaTele = stDane->fAkcel1[0];			break;
	case TID_AKCEL1Y:		fZmiennaTele = stDane->fAkcel1[1];			break;
	case TID_AKCEL1Z:		fZmiennaTele = stDane->fAkcel1[2];			break;
	case TID_AKCEL2X:		fZmiennaTele = stDane->fAkcel2[0];			break;
	case TID_AKCEL2Y:		fZmiennaTele = stDane->fAkcel2[1];			break;
	case TID_AKCEL2Z:		fZmiennaTele = stDane->fAkcel2[2];			break;
	case TID_ZYRO1P:		fZmiennaTele = stDane->fZyroKal1[0];		break;
	case TID_ZYRO1Q:		fZmiennaTele = stDane->fZyroKal1[1];		break;
	case TID_ZYRO1R:		fZmiennaTele = stDane->fZyroKal1[2];		break;
	case TID_ZYRO2P:		fZmiennaTele = stDane->fZyroKal2[0];		break;
	case TID_ZYRO2Q:		fZmiennaTele = stDane->fZyroKal2[1];		break;
	case TID_ZYRO2R:		fZmiennaTele = stDane->fZyroKal2[2];		break;
	case TID_MAGNE1X:		fZmiennaTele = stDane->fMagne1[0];			break;
	case TID_MAGNE1Y:		fZmiennaTele = stDane->fMagne1[1];			break;
	case TID_MAGNE1Z:		fZmiennaTele = stDane->fMagne1[2];			break;
	case TID_MAGNE2X:		fZmiennaTele = stDane->fMagne2[0];			break;
	case TID_MAGNE2Y:		fZmiennaTele = stDane->fMagne2[1];			break;
	case TID_MAGNE2Z:		fZmiennaTele = stDane->fMagne2[2];			break;
	case TID_MAGNE3X:		fZmiennaTele = stDane->fMagne3[0];			break;
	case TID_MAGNE3Y:		fZmiennaTele = stDane->fMagne3[1];			break;
	case TID_MAGNE3Z:		fZmiennaTele = stDane->fMagne3[2];			break;
	case TID_TEMPIMU1:		fZmiennaTele = stDane->fTemper[TEMP_IMU1];	break;
	case TID_TEMPIMU2:		fZmiennaTele = stDane->fTemper[TEMP_IMU1];	break;

	//zmienne AHRS
	case TID_KAT_IMU1X:		fZmiennaTele = stDane->fKatIMU1[0];			break;
	case TID_KAT_IMU1Y:		fZmiennaTele = stDane->fKatIMU1[1];			break;
	case TID_KAT_IMU1Z:		fZmiennaTele = stDane->fKatIMU1[2];			break;
	case TID_KAT_IMU2X:		fZmiennaTele = stDane->fKatIMU2[0];			break;
	case TID_KAT_IMU2Y:		fZmiennaTele = stDane->fKatIMU2[1];			break;
	case TID_KAT_IMU2Z:		fZmiennaTele = stDane->fKatIMU2[2];			break;
	case TID_KAT_ZYRO1X:	fZmiennaTele = stDane->fKatZyro1[0];		break;
	case TID_KAT_ZYRO1Y:	fZmiennaTele = stDane->fKatZyro1[1];		break;
	case TID_KAT_ZYRO1Z:	fZmiennaTele = stDane->fKatZyro1[2];		break;
	case TID_KAT_AKCELX:	fZmiennaTele = stDane->fKatAkcel1[0];		break;
	case TID_KAT_AKCELY:	fZmiennaTele = stDane->fKatAkcel1[1];		break;
	case TID_KAT_AKCELZ:	fZmiennaTele = stDane->fKatAkcel1[2];		break;

	//zmienne barametryczne
	case TID_CISBEZW1:		fZmiennaTele = stDane->fCisnieBzw[0];		break;
	case TID_CISBEZW2:		fZmiennaTele = stDane->fCisnieBzw[1];		break;
	case TID_WYSOKOSC_AGL1:	fZmiennaTele = stDane->fWysokoAGL[0];		break;	//Wysokość nad poziomem startu [m]
	case TID_WYSOKOSC_AGL2:	fZmiennaTele = stDane->fWysokoAGL[1];		break;
	case TID_WYSOKOSC_MSL1:	fZmiennaTele = stDane->fWysokoMSL[0];		break;	//Wysokość nad poziomem morza [m]
	case TID_WYSOKOSC_MSL2:	fZmiennaTele = stDane->fWysokoMSL[1];		break;
	case TID_CISROZN1:		fZmiennaTele = stDane->fCisnRozn[0];		break;
	case TID_CISROZN2:		fZmiennaTele = stDane->fCisnRozn[1];		break;
	case TID_PREDIAS1:		fZmiennaTele = stDane->fPredkosc[0];		break;
	case TID_PREDIAS2:		fZmiennaTele = stDane->fPredkosc[1];		break;
	case TID_WARIO1:		fZmiennaTele = stDane->fWariometr[0];		break;
	case TID_WARIO2:		fZmiennaTele = stDane->fWariometr[1];		break;
	case TID_TEMPCISB1:		fZmiennaTele = stDane->fTemper[TEMP_BARO1];	break;
	case TID_TEMPCISB2:		fZmiennaTele = stDane->fTemper[TEMP_BARO2];	break;
	case TID_TEMPCISR1:		fZmiennaTele = stDane->fTemper[TEMP_CISR1];	break;
	case TID_TEMPCISR2:		fZmiennaTele = stDane->fTemper[TEMP_CISR2];	break;

	//odbiorniki RC
	case TID_RC_KAN1:
	case TID_RC_KAN2:
	case TID_RC_KAN3:
	case TID_RC_KAN4:
	case TID_RC_KAN5:
	case TID_RC_KAN6:
	case TID_RC_KAN7:
	case TID_RC_KAN8:
	case TID_RC_KAN9:
	case TID_RC_KAN10:
	case TID_RC_KAN11:
	case TID_RC_KAN12:
	case TID_RC_KAN13:
	case TID_RC_KAN14:
	case TID_RC_KAN15:	//aby nie generować zbyt dużo kodu stosuję wzór do obliczenia indeksu zmiennej dla wszystkich kanałów
	case TID_RC_KAN16:	fZmiennaTele = stDane->sKanalRC[sZmienna - TID_RC_KAN1];	break;

	//Serwa
	case TID_SERWO1:
	case TID_SERWO2:
	case TID_SERWO3:
	case TID_SERWO4:
	case TID_SERWO5:
	case TID_SERWO6:
	case TID_SERWO7:
	case TID_SERWO8:
	case TID_SERWO9:
	case TID_SERWO10:
	case TID_SERWO11:
	case TID_SERWO12:
	case TID_SERWO13:
	case TID_SERWO14:
	case TID_SERWO15:		//aby nie generować zbyt dużo kodu stosuję wzór do obliczenia indeksu zmiennej dla wszystkich kanałów
	case TID_SERWO16:		fZmiennaTele = stDane->sWyjscieRC[sZmienna - TID_SERWO1];	break;

	case TID_BSP_AKCELX:	fZmiennaTele = stDane->stBSP.fAkcel[0];		break;
	case TID_BSP_AKCELY:	fZmiennaTele = stDane->stBSP.fAkcel[1];		break;
	case TID_BSP_AKCELZ:	fZmiennaTele = stDane->stBSP.fAkcel[2];		break;
	case TID_BSP_ZYROP:		fZmiennaTele = stDane->stBSP.fZyro[0];		break;
	case TID_BSP_ZYROQ:		fZmiennaTele = stDane->stBSP.fZyro[1];		break;
	case TID_BSP_ZYROR:		fZmiennaTele = stDane->stBSP.fZyro[2];		break;
	case TID_BSP_MAGNEX:	fZmiennaTele = stDane->stBSP.fMagne[0];		break;
	case TID_BSP_MAGNEY:	fZmiennaTele = stDane->stBSP.fMagne[1];		break;
	case TID_BSP_MAGNEZ:	fZmiennaTele = stDane->stBSP.fMagne[2];		break;

	case TID_BSP_IMUX:		fZmiennaTele = stDane->stBSP.fKatIMU[0];	break;
	case TID_BSP_IMUY:		fZmiennaTele = stDane->stBSP.fKatIMU[1];	break;
	case TID_BSP_IMUZ:		fZmiennaTele = stDane->stBSP.fKatIMU[2];	break;
	case TID_BSP_AGL:		fZmiennaTele = stDane->stBSP.fWysokoscAGL;	break;
	case TID_BSP_AMSL:		fZmiennaTele = stDane->stBSP.fWysokoscMSL;	break;
	case TID_BSP_IAS:		fZmiennaTele = stDane->stBSP.fIAS;			break;
	case TID_BSP_KURS:		fZmiennaTele = stDane->stBSP.fKursGeo;		break;

	case TID_BSP_PRED_POLN:	fZmiennaTele = stDane->stBSP.fPredkoscN;	break;
	case TID_BSP_PRED_WSCH:	fZmiennaTele = stDane->stBSP.fPredkoscE;	break;
	case TID_BSP_PRED_WDOL:	fZmiennaTele = stDane->stBSP.fPredkoscD;	break;
	case TID_BSP_SZER_GEO:	fZmiennaTele = stDane->stBSP.dSzerokoscGeo;	break;
	case TID_BSP_DLUG_GEO:	fZmiennaTele = stDane->stBSP.dDlugoscGeo;	break;

	case TID_DOTYK_ADC0:		fZmiennaTele = stStatusDotyku.sAdc[0];		break;
	case TID_DOTYK_ADC1:		fZmiennaTele = stStatusDotyku.sAdc[1];		break;
	case TID_DOTYK_ADC2:		fZmiennaTele = stStatusDotyku.sAdc[2];		break;
	case TID_CZAS_PETLI: 		fZmiennaTele = stDane->ndT;					break;
	case TID_JAKOSC_UP_RC1:		fZmiennaTele = stDane->cJakoscUpLinkuRC1;	break;
	case TID_JAKOSC_UP_RC2:		fZmiennaTele = stDane->cJakoscUpLinkuRC2;	break;
	case TID_JAKOSC_DOWN_RC:	fZmiennaTele = stDane->cJakoscDnLinkuRC;	break;

	//pomiary analogowe
	case TID_ADC1_1:			fZmiennaTele = stDane->fNapCzujZewn[0];	break;
	case TID_ADC1_2:			fZmiennaTele = stDane->fNapCzujZewn[1];	break;
	case TID_ADC2_1:			fZmiennaTele = stDane->fNapCzujZewn[2];	break;
	case TID_ADC2_2:			fZmiennaTele = stDane->fNapCzujZewn[3];	break;
	case TID_BAT1_NAPIECIE:		fZmiennaTele = stDane->fNapiecieAku[0];	break;
	case TID_BAT1_PRAD:			fZmiennaTele = stDane->fPradAku[0];		break;
	case TID_BAT1_ENERGIA:		fZmiennaTele = stDane->fEnergiaPobr[0];	break;
	case TID_BAT2_NAPIECIE:		fZmiennaTele = stDane->fNapiecieAku[1];	break;
	case TID_BAT2_PRAD:			fZmiennaTele = stDane->fPradAku[1];		break;
	case TID_BAT2_ENERGIA:		fZmiennaTele = stDane->fEnergiaPobr[1];	break;
	case TID_BAT_RTC_NAPIECIE:	fZmiennaTele = stDane->fNapiecieBatRTC;	break;
	case TID_TEMPERATURA_CPU:	fZmiennaTele = stDane->fTemperCPU;		break;
	case TID_NAPIECIE_WE1:		fZmiennaTele = stDane->fNapiecieWej[0];	break;
	case TID_NAPIECIE_WE2:		fZmiennaTele = stDane->fNapiecieWej[1];	break;
	case TID_NAPIECIE_SERW:		fZmiennaTele = stDane->fNapiecieSerw;	break;
	case TID_NAPIECIE_USB:		fZmiennaTele = stDane->fNapiecieUSB;	break;

	case TID_FFT_ZYRO_AKCEL:	break;	//wyniki transformaty fouriera przesyłane w specyficznej szybkiej ramce

	//--- zmienne telemetryczne w ramce 2 -----------------------------------------------
	case TID_PID_PRZE_WZAD:		fZmiennaTele = stDane->stPID[PID_KĄTA_PRZE].fZadana;		break;	//wartość zadana regulatora sterowania przechyleniem
	case TID_PID_PRZE_FZAD:		fZmiennaTele = stDane->stPID[PID_KĄTA_PRZE].fFiltrWZad;		break;	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
	case TID_PID_PRZE_FWEJ:		fZmiennaTele = stDane->stPID[PID_KĄTA_PRZE].fFiltrWWej;		break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_PRZE_FROZ:		fZmiennaTele = stDane->stPID[PID_KĄTA_PRZE].fFiltrRóżn;		break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_PRZE_WY_P:		fZmiennaTele = stDane->stPID[PID_KĄTA_PRZE].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_PRZE_WY_I:		fZmiennaTele = stDane->stPID[PID_KĄTA_PRZE].fWyjscieI;		break;	//wyjście członu I
	case TID_PID_PRZE_WY_D:		fZmiennaTele = stDane->stPID[PID_KĄTA_PRZE].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_PRZE_WYPRZ:	fZmiennaTele = stDane->stPID[PID_KĄTA_PRZE].fWyjscieWyprz;	break;	//wyjście członu wyprzedzającego
	case TID_PID_PRZE_WYJ:		fZmiennaTele = stDane->stPID[PID_KĄTA_PRZE].fWyjsciePID;	break;	//wyjście regulatora sterowania przechyleniem
	case TID_PID_PK_PRZE_WZAD:	fZmiennaTele = stDane->stPID[PID_PRED_PRZE].fZadana;		break;	//wartość zadana regulatora sterowania prędkością kątową przechylenia
	case TID_PID_PK_PRZE_FZAD:	fZmiennaTele = stDane->stPID[PID_PRED_PRZE].fFiltrWZad;		break;	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
	case TID_PID_PK_PRZE_FWEJ:	fZmiennaTele = stDane->stPID[PID_PRED_PRZE].fFiltrWWej;		break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_PK_PRZE_FROZ:	fZmiennaTele = stDane->stPID[PID_KĄTA_PRZE].fFiltrRóżn;		break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_PK_PRZE_WY_P:	fZmiennaTele = stDane->stPID[PID_PRED_PRZE].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_PK_PRZE_WY_D:	fZmiennaTele = stDane->stPID[PID_PRED_PRZE].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_PK_PRZE_WYPRZ:	fZmiennaTele = stDane->stPID[PID_PRED_PRZE].fWyjscieWyprz;	break;	//wyjście członu wyprzedzającego
	case TID_PID_PK_PRZE_WYJ:	fZmiennaTele = stDane->stPID[PID_PRED_PRZE].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością kątową przechylenia

	case TID_PID_POCH_WZAD:		fZmiennaTele = stDane->stPID[PID_KĄTA_POCH].fZadana;		break;	//wartość zadana regulatora sterowania pochyleniem
	case TID_PID_POCH_FZAD:		fZmiennaTele = stDane->stPID[PID_KĄTA_POCH].fFiltrWZad;		break;	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
	case TID_PID_POCH_FWEJ:		fZmiennaTele = stDane->stPID[PID_KĄTA_POCH].fFiltrWWej;		break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_POCH_FROZ:		fZmiennaTele = stDane->stPID[PID_KĄTA_POCH].fFiltrRóżn;		break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_POCH_WY_P:		fZmiennaTele = stDane->stPID[PID_KĄTA_POCH].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_POCH_WY_I:		fZmiennaTele = stDane->stPID[PID_KĄTA_POCH].fWyjscieI;		break;	//wyjście członu I
	case TID_PID_POCH_WY_D:		fZmiennaTele = stDane->stPID[PID_KĄTA_POCH].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_POCH_WYPRZ:	fZmiennaTele = stDane->stPID[PID_KĄTA_POCH].fWyjscieWyprz;	break;	//wyjście członu wyprzedzającego
	case TID_PID_POCH_WYJ:		fZmiennaTele = stDane->stPID[PID_KĄTA_POCH].fWyjsciePID;	break;	//wyjście regulatora sterowania pochyleniem
	case TID_PID_PK_POCH_WZAD:	fZmiennaTele = stDane->stPID[PID_PRED_POCH].fZadana;		break;	//wartość zadana
	case TID_PID_PK_POCH_FZAD:	fZmiennaTele = stDane->stPID[PID_PRED_POCH].fFiltrWZad;		break;	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
	case TID_PID_PK_POCH_FWEJ:	fZmiennaTele = stDane->stPID[PID_PRED_POCH].fFiltrWWej;		break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_PK_POCH_FROZ:	fZmiennaTele = stDane->stPID[PID_PRED_POCH].fFiltrRóżn;		break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_PK_POCH_WY_P:	fZmiennaTele = stDane->stPID[PID_PRED_POCH].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_PK_POCH_WY_D:	fZmiennaTele = stDane->stPID[PID_PRED_POCH].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_PK_POCH_WYPRZ:	fZmiennaTele = stDane->stPID[PID_PRED_POCH].fWyjscieWyprz;	break;	//wyjście członu wyprzedzającego
	case TID_PID_PK_POCH_WYJ:	fZmiennaTele = stDane->stPID[PID_PRED_POCH].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością kątową pochylenia

	case TID_PID_ODCH_WZAD:		fZmiennaTele = stDane->stPID[PID_KĄTA_ODCH].fZadana;		break;	//wartość zadana regulatora sterowania odchyleniem
	case TID_PID_ODCH_FZAD:		fZmiennaTele = stDane->stPID[PID_KĄTA_ODCH].fFiltrWZad;		break;	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
	case TID_PID_ODCH_FWEJ:		fZmiennaTele = stDane->stPID[PID_KĄTA_ODCH].fFiltrWWej;		break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_ODCH_FROZ:		fZmiennaTele = stDane->stPID[PID_KĄTA_ODCH].fFiltrRóżn;		break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_ODCH_WY_P:		fZmiennaTele = stDane->stPID[PID_KĄTA_ODCH].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_ODCH_WY_I:		fZmiennaTele = stDane->stPID[PID_KĄTA_ODCH].fWyjscieI;		break;	//wyjście członu I
	case TID_PID_ODCH_WY_D:		fZmiennaTele = stDane->stPID[PID_KĄTA_ODCH].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_ODCH_WYPRZ:	fZmiennaTele = stDane->stPID[PID_KĄTA_ODCH].fWyjscieWyprz;	break;	//wyjście członu wyprzedzającego
	case TID_PID_ODCH_WYJ:		fZmiennaTele = stDane->stPID[PID_KĄTA_ODCH].fWyjsciePID;	break;	//wyjście regulatora sterowania odchyleniem
	case TID_PID_PK_ODCH_WZAD:	fZmiennaTele = stDane->stPID[PID_PRED_ODCH].fZadana;		break;	//wartość zadana
	case TID_PID_PK_ODCH_FZAD:	fZmiennaTele = stDane->stPID[PID_PRED_ODCH].fFiltrWZad;		break;	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
	case TID_PID_PK_ODCH_FWEJ:	fZmiennaTele = stDane->stPID[PID_PRED_ODCH].fFiltrWWej;		break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_PK_ODCH_FROZ:	fZmiennaTele = stDane->stPID[PID_PRED_ODCH].fFiltrRóżn;		break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_PK_ODCH_WY_P:	fZmiennaTele = stDane->stPID[PID_PRED_ODCH].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_PK_ODCH_WY_D:	fZmiennaTele = stDane->stPID[PID_PRED_ODCH].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_PK_ODCH_WYPRZ:	fZmiennaTele = stDane->stPID[PID_PRED_ODCH].fWyjscieWyprz;	break;	//wyjście członu wyprzedzającego
	case TID_PID_PK_ODCH_WYJ:	fZmiennaTele = stDane->stPID[PID_PRED_ODCH].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością kątową odchylenia

	case TID_PID_WYSO_WZAD:		fZmiennaTele = stDane->stPID[PID_WYSOKOSCI].fZadana;		break;	//wartość zadana regulatora sterowania wysokością
	case TID_PID_WYSO_FZAD:		fZmiennaTele = stDane->stPID[PID_WYSOKOSCI].fFiltrWZad;		break;	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
	case TID_PID_WYSO_FWEJ:		fZmiennaTele = stDane->stPID[PID_WYSOKOSCI].fFiltrWWej;		break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_WYSO_FROZ:		fZmiennaTele = stDane->stPID[PID_WYSOKOSCI].fFiltrRóżn;		break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_WYSO_WY_P:		fZmiennaTele = stDane->stPID[PID_WYSOKOSCI].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_WYSO_WY_I:		fZmiennaTele = stDane->stPID[PID_WYSOKOSCI].fWyjscieI;		break;	//wyjście członu I
	case TID_PID_WYSO_WY_D:		fZmiennaTele = stDane->stPID[PID_WYSOKOSCI].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_WYSO_WYPRZ:	fZmiennaTele = stDane->stPID[PID_WYSOKOSCI].fWyjscieWyprz;	break;	//wyjście członu wyprzedzającego
	case TID_PID_WYSO_WYJ:		fZmiennaTele = stDane->stPID[PID_WYSOKOSCI].fWyjsciePID;	break;	//wyjście regulatora sterowania odchyleniem
	case TID_PID_PR_WYSO_WZAD:	fZmiennaTele = stDane->stPID[PID_PRED_ZWYS].fZadana;		break;	//wartość zadana
	case TID_PID_PR_WYSO_FZAD:	fZmiennaTele = stDane->stPID[PID_PRED_ZWYS].fFiltrWZad;		break;	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
	case TID_PID_PR_WYSO_FWEJ:	fZmiennaTele = stDane->stPID[PID_PRED_ZWYS].fFiltrWWej;		break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_PK_WYSO_FROZ:	fZmiennaTele = stDane->stPID[PID_PRED_ZWYS].fFiltrRóżn;		break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_PR_WYSO_WY_P:	fZmiennaTele = stDane->stPID[PID_PRED_ZWYS].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_PR_WYSO_WY_D:	fZmiennaTele = stDane->stPID[PID_PRED_ZWYS].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_PR_WYSO_WYPRZ:	fZmiennaTele = stDane->stPID[PID_PRED_ZWYS].fWyjscieWyprz;	break;	//wyjście członu wyprzedzającego
	case TID_PID_PR_WYSO_WYJ:	fZmiennaTele = stDane->stPID[PID_PRED_ZWYS].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością zmiany wysokości

	case TID_PID_NAWN_WZAD:		fZmiennaTele = stDane->stPID[PID_NAWIG_PÓŁN].fZadana;		break;	//wartość zadana regulatora sterowania nawigacją w kierunku północnym
	case TID_PID_NAWN_FWEJ:		fZmiennaTele = stDane->stPID[PID_NAWIG_PÓŁN].fFiltrWWej;	break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_NAWN_FROZ:		fZmiennaTele = stDane->stPID[PID_NAWIG_PÓŁN].fFiltrRóżn;	break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_NAWN_WY_P:		fZmiennaTele = stDane->stPID[PID_NAWIG_PÓŁN].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_NAWN_WY_I:		fZmiennaTele = stDane->stPID[PID_NAWIG_PÓŁN].fWyjscieI;		break;	//wyjście członu I
	case TID_PID_NAWN_WY_D:		fZmiennaTele = stDane->stPID[PID_NAWIG_PÓŁN].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_NAWN_WYJ:		fZmiennaTele = stDane->stPID[PID_NAWIG_PÓŁN].fWyjsciePID;	break;	//wyjście regulatora sterowania nawigacją w kierunku północnym

	case TID_PID_PR_NAWN_WZAD:	fZmiennaTele = stDane->stPID[PID_PRED_PÓŁN].fZadana;		break;	//wartość zadana
	case TID_PID_PR_NAWN_FWEJ:	fZmiennaTele = stDane->stPID[PID_PRED_PÓŁN].fFiltrWWej;		break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_PR_NAWN_FROZ:	fZmiennaTele = stDane->stPID[PID_PRED_PÓŁN].fFiltrRóżn;		break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_PR_NAWN_WY_P:	fZmiennaTele = stDane->stPID[PID_PRED_PÓŁN].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_PR_NAWN_WY_I:	fZmiennaTele = stDane->stPID[PID_PRED_PÓŁN].fWyjscieI;		break;	//wyjście członu I
	case TID_PID_PR_NAWN_WY_D:	fZmiennaTele = stDane->stPID[PID_PRED_PÓŁN].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_PR_NAWN_WYJ:	fZmiennaTele = stDane->stPID[PID_PRED_PÓŁN].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością w kierunku północnym

	case TID_PID_NAWE_WZAD:		fZmiennaTele = stDane->stPID[PID_NAWIG_WSCH].fZadana;		break;	//wartość zadana regulatora sterowania nawigacją w kierunku wschodnim
	case TID_PID_NAWE_FWEJ:		fZmiennaTele = stDane->stPID[PID_NAWIG_WSCH].fFiltrWWej;	break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_NAWE_FROZ:		fZmiennaTele = stDane->stPID[PID_NAWIG_WSCH].fFiltrRóżn;	break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_NAWE_WY_P:		fZmiennaTele = stDane->stPID[PID_NAWIG_WSCH].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_NAWE_WY_I:		fZmiennaTele = stDane->stPID[PID_NAWIG_WSCH].fWyjscieI;		break;	//wyjście członu I
	case TID_PID_NAWE_WY_D:		fZmiennaTele = stDane->stPID[PID_NAWIG_WSCH].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_NAWE_WYJ:		fZmiennaTele = stDane->stPID[PID_NAWIG_WSCH].fWyjsciePID;	break;	//wyjście regulatora sterowania nawigacją w kierunku północnym

	case TID_PID_PR_NAWE_WZAD:	fZmiennaTele = stDane->stPID[PID_PRED_WSCH].fZadana;		break;	//wartość zadana
	case TID_PID_PR_NAWE_FWEJ:	fZmiennaTele = stDane->stPID[PID_PRED_WSCH].fFiltrWWej;		break;	//przefiltrowana (0..31) wartość wejściowa dla wszystkich członów
	case TID_PID_PR_NAWE_FROZ:	fZmiennaTele = stDane->stPID[PID_PRED_WSCH].fFiltrRóżn;		break;	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
	case TID_PID_PR_NAWE_WY_P:	fZmiennaTele = stDane->stPID[PID_PRED_WSCH].fWyjscieP;		break;	//wyjście członu P
	case TID_PID_PR_NAWE_WY_I:	fZmiennaTele = stDane->stPID[PID_PRED_WSCH].fWyjscieI;		break;	//wyjście członu I
	case TID_PID_PR_NAWE_WY_D:	fZmiennaTele = stDane->stPID[PID_PRED_WSCH].fWyjscieD;		break;	//wyjście członu D
	case TID_PID_PR_NAWE_WYJ:	fZmiennaTele = stDane->stPID[PID_PRED_WSCH].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością w kierunku wschodnim

	case TID_PID_STROJENIE1:	fZmiennaTele = stDane->fStrojenie[0];						break;	//wartość parametru strojącego 1
	case TID_PID_STROJENIE2:	fZmiennaTele = stDane->fStrojenie[1];						break;	//wartość parametru strojącego 2

	default:	fZmiennaTele = -1.0f;
	}
	return fZmiennaTele;
}



///////////////////////////////////////////////////////////////////////////////
// Tworzy nagłówek ramki telemetrii. Inicjuje CRC
// Parametry:
//  cIndeksRamki - indeks napełnianiej ramki uwzględniajacy numer ramki i podwójne buforowanie (jedna jest napełniana, druga sie wysyła)
//  cTypRamki - okresla czy to jest normalna ramka telemetryczna, czy szybka
//  cAdrZdalny - adres urządzenia do którego wysyłamy
//  cAdrLokalny - nasz adres
//  cRozmDanych - liczba bajtw danych uwzględniająca pole bitowe zmiennych w typowej ramce oraz nagłówek w ramce szybkiej
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PrzygotujRamkeTele(uint8_t cIndeksRamki, uint8_t cTypRamki, uint8_t cAdrZdalny, uint8_t cAdrLokalny, uint8_t cRozmDanych)
{
	PobierzDateCzas(&stDate, &stTime);

	InicjujCRC16(0, WIELOMIAN_CRC);
	cRamkaTelemetrii[cIndeksRamki][0] = NAGLOWEK;
	cRamkaTelemetrii[cIndeksRamki][1] = CRC->DR = cAdrZdalny;
	cRamkaTelemetrii[cIndeksRamki][2] = CRC->DR = cAdrLokalny;
	cRamkaTelemetrii[cIndeksRamki][3] = CRC->DR = (uint8_t)stTime.SubSeconds;	//1/256 cześć sekundy czasu RTC synchronizowanego z GPS
	if (cTypRamki == TELEM_NORMALNA)
		cRamkaTelemetrii[cIndeksRamki][4] = CRC->DR = PK_TELEMETRIA1 + cIndeksRamki / 2;	//indeks ramki wskazuje na zakres zmiennych a to determinuje numer polecenia
	else
		cRamkaTelemetrii[cIndeksRamki][4] = CRC->DR = PK_TELEM_SZYBKA;	//ramka szybk jest na razie tylko jedna
	cRamkaTelemetrii[cIndeksRamki][5] = CRC->DR = cRozmDanych;

	//policz CRC z danych i listy zmiennych
	for(uint16_t n=0; n < cRozmDanych; n++)
		CRC->DR = cRamkaTelemetrii[cIndeksRamki][ROZMIAR_NAGLOWKA + n];

	un8_32.dane16[0] = (uint16_t)CRC->DR;
	cRamkaTelemetrii[cIndeksRamki][ROZMIAR_NAGLOWKA + cRozmDanych + 0] =  un8_32.dane8[0];	//młodszy przodem
	cRamkaTelemetrii[cIndeksRamki][ROZMIAR_NAGLOWKA + cRozmDanych + 1] =  un8_32.dane8[1];	//starszy

	//return chRozmDanych * 2 + LICZBA_BAJTOW_ID_TELEMETRII + ROZMIAR_NAGLOWKA + ROZMIAR_CRC;
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
void Float2Char16(float fData, uint8_t* cData)
{
    typedef union
    {
    	uint8_t array[sizeof(float)];
	float fuData;
    } fUnion;
    fUnion temp;
    volatile uint8_t cCecha;

    temp.fuData = fData;
    *(cData+1) = temp.array[3] & 0x80;   //znak liczby
    cCecha = ((temp.array[3] & 0x7F)<<1) + ((temp.array[2] & 0x80)>>7);
    cCecha -= (127-15);
    if (cCecha > 127)
      cCecha = 1;          //gdy cecha poza zakresem
    else
      cCecha &= 0x1F;
    *(cData+1) += cCecha<<2;
    *(cData+1) += (temp.array[2] & 0x60)>>5;   //mantysa

    *(cData+0) =  (temp.array[2] & 0x1F)<<3;
    *(cData+0) += (temp.array[1] & 0xE0)>>5;
 }



////////////////////////////////////////////////////////////////////////////////
// Zapisuje do FLASH okres wysyłania kolenych zmiennych telemetrycznych w porządku młodszy przodem
// Parametry: sPrzesuniecie - przesuniecie danych względem początku, numer zmiennej od ktorego dane mają być zapisane
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszKonfiguracjeTelemetrii(uint16_t sPrzesuniecie)
{
	uint8_t cDoZapisu = OKRESOW_TELEMETRII_W_RAMCE;
	uint16_t sIndeksPaczki = (sPrzesuniecie * 2) / ROZMIAR_DANYCH_WPACZCE;
	uint8_t cProbZapisu = 3;
	uint8_t cBłąd;

	while (cDoZapisu && cProbZapisu)		//czytaj kolejne paczki aż skompletuje tyle danych ile potrzeba
	{
		cBłąd = ZapiszPaczkeKonfigu(FKON_OKRES_TELEMETRI1 + sIndeksPaczki, (uint8_t*)&sOkresTelemetrii[sIndeksPaczki * ROZMIAR_DANYCH_WPACZCE / 2]);
		if (cBłąd == BLAD_OK)
		{
			sIndeksPaczki++;
			cProbZapisu = 3;
			if (cDoZapisu > ROZMIAR_DANYCH_WPACZCE / 2)
				cDoZapisu -= ROZMIAR_DANYCH_WPACZCE / 2;
			else
				cDoZapisu = 0;
		}
		cProbZapisu--;
	}
	return cBłąd;
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja włącza lub wyłącza telemetrię w tym telemetrię szybką
// Parametry: chOperacja - polecenie do ustawienia w zmiennej statusu telemetrii
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WłączTelemetrię(uint8_t cOperacja)
{
	cStatusTelemetrii = cOperacja;
	sIndeksWysyłkiFFT = cIndeksWysyłkiTestuFFT = 0;	//resetuj parametry szybkiej telemetrii
}
