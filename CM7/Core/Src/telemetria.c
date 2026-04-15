//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł obsługi telemetrii
//
// (c) PitLab 2025-26
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "telemetria.h"
#include "wymiana_CM7.h"
#include "czas.h"
#include "flash_konfig.h"
#include "protokol_kom.h"
#include "polecenia_komunikacyjne.h"
#include "dotyk.h"
#include "fft.h"

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
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4_CM7")))	chRamkaTelemetrii[2 * LICZBA_RAMEK_TELEMETR][ROZMIAR_RAMKI_KOMUNIKACYJNEJ];	//ramki telemetryczne: przygotowywana i wysyłana dla 16-bitowych zmiennych 0..127 oraz 128..255
uint16_t sOkresTelemetrii[LICZBA_ZMIENNYCH_TELEMETRYCZNYCH];	//zmienna definiujaca okres wysyłania telemetrii dla wszystkich zmiennych
uint16_t sLicznikTelemetrii[LICZBA_ZMIENNYCH_TELEMETRYCZNYCH];
uint8_t chStatusTelemetrii;		//określa czy i jaki rodzaj telemetrii ma być wysyłany
extern unia_wymianyCM4_t uDaneCM4;
extern uint8_t chAdresZdalny[ILOSC_INTERF_KOM];	//adres sieciowy strony zdalnej
extern UART_HandleTypeDef hlpuart1;
static unia8_32_t un8_32;		//unia do konwersji między danymi 16 i 8 bit
extern volatile uint8_t chDoWyslania[1 + LICZBA_RAMEK_TELEMETR];	//lista rzeczy do wysłania po zakończeniu bieżącej transmisji: ramka poleceń i ramki telemetryczne
extern stBSP_ID_t stBSP_ID;	//struktura zawierajaca adresy i nazwę BSP
extern volatile st_ZajetośćLPUART_t st_ZajetośćLPUART;
extern struct _statusDotyku statusDotyku;
extern RTC_TimeTypeDef stTime;
extern RTC_DateTypeDef stDate;
extern float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fWynikFFT[LICZBA_TESTOW_FFT][LICZBA_ZMIENNYCH_FFT][FFT_MAX_ROZMIAR / 2];	//wartość sygnału wyjściowego
extern uint16_t sIndeksWysyłkiFFT;		//wskazuje na na numer próbki FFT przesyłany telemetrią
extern uint8_t chIndeksWysyłkiTestuFFT;	//wskazuje na nume testu FFT obecnie wysyłanego telemetrią
extern stFFT_t stKonfigFFT;


////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjalizuje zmienne używane do obsługi telemetrii. Dane są zapisane w porządku cienkokońcówkowym
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjalizacjaTelemetrii(void)
{
	uint8_t chPaczka[ROZMIAR_PACZKI_KONFIGU];
	uint8_t chOdczytano;
	uint8_t chDoOdczytu = LICZBA_ZMIENNYCH_TELEMETRYCZNYCH;
	uint8_t chIndeksPaczki = 0;
	uint8_t chProbOdczytu = PROB_ODCZYTU_TELEMETRII;
	uint16_t sOkres;

	//Inicjuj zmienne wartościa wyłączoną gdyby nie dało się odczytać konfiguracji.
	//Lepiej jest mieć telemetrię wyłaczoną niż zapchaną pracujacą na 100%
	for (uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
		sOkresTelemetrii[n] = TELEMETRIA_WYLACZONA;

	while (chDoOdczytu && chProbOdczytu)		//czytaj kolejne paczki aż skompletuje tyle danych ile potrzeba
	{
		chOdczytano = CzytajPaczkeKonfigu(chPaczka, FKON_OKRES_TELEMETRI1 + chIndeksPaczki);		//odczytaj 30 bajtów danych + identyfikator i CRC
		if (chOdczytano == ROZMIAR_PACZKI_KONFIGU)
		{
			for (uint16_t n=0; n<((ROZMIAR_PACZKI_KONFIGU - 2) / 2); n++)
			{
				if (chDoOdczytu)	//nie czytaj wiecej niż trzeba aby nie przepełnić zmiennej
				{
					sOkres = chPaczka[2*n+2] + chPaczka[2*n+3] * 0x100;
					sOkresTelemetrii[n + chIndeksPaczki * ROZMIAR_DANYCH_WPACZCE /2] = sOkres;
					chDoOdczytu--;
				}
			}
			chIndeksPaczki++;
			chProbOdczytu = PROB_ODCZYTU_TELEMETRII;
		}
		chProbOdczytu--;
	}

	//inicjuj licznik okresem wysyłania
	for (uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
		sLicznikTelemetrii[n] = sOkresTelemetrii[n];
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja przygotowuje i rozpoczyna wysyłkę ramek telemetrycznych
// Parametry: chInterfejs - interfejs komunikacyjny (na razie tylko LPUART)
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObslugaTelemetrii(uint8_t chInterfejs)
{
	uint8_t chIloscDanych[LICZBA_RAMEK_TELEMETR] = {0, 0};
	uint8_t chIndeksAdresow;	//okresla indeks puli adresowej rakmi. Zmienne 0..127 idą w ramce 0, zmienne 128..255 w ramce 1, itd
	uint8_t chTypRamki;
	float fZmienna;
	extern uint8_t chStatusPolaczenia;

	if (chStatusTelemetrii == TELEM_WSTRZYMAJ)
		return;

	//szybka telemetria wykorzystuje tylko ramkę RAMKA_TELE1, czyli chIloscDanych[0] oraz chRamkaTelemetrii[0 i 1]
	if (chStatusTelemetrii == TELEM_SZYBKA)
	{
		if (stKonfigFFT.chIndeksTestu > chIndeksWysyłkiTestuFFT)	//czekaj z wysyłką na wyprodukowanie danych przez FFT
		{
			if ((st_ZajetośćLPUART.sDoWysłania[RAMKA_TELE1] == 0) && (st_ZajetośćLPUART.chZajętyPrzez == (int8_t)LPUART_WOLNY))	//dodaj nowe dane tylko do pustej kolejki
			{
				//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);		//serwo kanał 7
				chIloscDanych[0] = WstawDaneDoSzybkiejRamkiTele(st_ZajetośćLPUART.chIndeksNapełnianejRamki[RAMKA_TELE1], chIndeksWysyłkiTestuFFT, &sIndeksWysyłkiFFT);

				//testuj warunek zakończenia procesu wysyłki FFT
				if (sIndeksWysyłkiFFT >= stKonfigFFT.sLiczbaProbek / 2)
				{
					sIndeksWysyłkiFFT = 0;
					chIndeksWysyłkiTestuFFT++;
					if (chIndeksWysyłkiTestuFFT == LICZBA_TESTOW_FFT)
						chStatusTelemetrii = TELEM_NORMALNA;	//po wysłaniu wszystkich wyników FFT wróc do normalnej tlemetrii
				}
				chTypRamki = TELEM_SZYBKA;
			}
		}
	}
	else		//normalna telemetria
	{
		//wyczyść w ramkach pola na bity identyfikujące zmienne, bo kolejne bity będą OR-owane z wartością początkową, więc musi ona na początku być zerem
		for (uint8_t r=0; r<LICZBA_RAMEK_TELEMETR; r++)
		{
			for(uint8_t n=0; n<LICZBA_BAJTOW_ID_TELEMETRII; n++)
				chRamkaTelemetrii[st_ZajetośćLPUART.chIndeksNapełnianejRamki[r+1] + 2 * r][ROZMIAR_NAGLOWKA + n] = 0;
			chIloscDanych[r] = LICZBA_BAJTOW_ID_TELEMETRII;
		}

		for(uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
		{
			if (sLicznikTelemetrii[n] != TELEMETRIA_WYLACZONA)	//wartość oznaczająca aby nie wysyłać danych
			{
				sLicznikTelemetrii[n]--;
				if (sLicznikTelemetrii[n] == 0)		//licznik zmiennej doszedł do 0 więc trzeba ją wysłać
				{
					sLicznikTelemetrii[n] = sOkresTelemetrii[n];		//przeładuj licznik nowym okresem
					fZmienna = PobierzZmiennaTele(n);
					chIndeksAdresow = n >> 7;
					if (chIloscDanych[chIndeksAdresow] < (ROZMIAR_RAMKI_KOMUNIKACYJNEJ - ROZMIAR_CRC - 2))	//sprawdź czy dane mieszczą się w ramce
					{
						WstawDaneDoRamkiTele(st_ZajetośćLPUART.chIndeksNapełnianejRamki[chIndeksAdresow + 1], chIndeksAdresow, chIloscDanych[chIndeksAdresow], n, fZmienna);
						chIloscDanych[chIndeksAdresow] += 2;
					}
				}
			}
		}
		chTypRamki = TELEM_NORMALNA;
	}

	//przygotuj ramki wypełnione danymi  do wysłania dodając nagłówek, taki sam dla każdego typu ramki
	for (uint8_t r=0; r<LICZBA_RAMEK_TELEMETR; r++)
	{
		if (chIloscDanych[r] > LICZBA_BAJTOW_ID_TELEMETRII)	//jeżeli jest coś do wysłania
		{
			PrzygotujRamkeTele(st_ZajetośćLPUART.chIndeksNapełnianejRamki[r+1] + r * LICZBA_RAMEK_TELEMETR, chTypRamki, chAdresZdalny[chInterfejs], stBSP_ID.chAdres, chIloscDanych[r]);	//utwórz ramkę gotową do wysyłki
			st_ZajetośćLPUART.sDoWysłania[r+1] = chIloscDanych[r] + ROZM_CIALA_RAMKI;
		}
	}

	//rozpocznij fizyczną transmisję danych
	__disable_irq();	//sekcja krytyczna wykonywana przy wyłączonych przerwaniach
	if (st_ZajetośćLPUART.chZajętyPrzez == (int8_t)LPUART_WOLNY)	//jeżeli LPUART nie jest zajęty to wyślij telemetrię
	{
		for (uint8_t r=0; r<LICZBA_RAMEK_TELEMETR; r++)
		{
			if (st_ZajetośćLPUART.sDoWysłania[r+1])
			{
				chStatusPolaczenia |= (STAT_POL_PRZESYLA << STAT_POL_UART);		//sygnalizuj transfer danych
				st_ZajetośćLPUART.chZajętyPrzez = RAMKA_TELE1 + r;
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//serwo kanał 1 - LPUART_ZAJETY
				HAL_UART_Transmit_DMA(&hlpuart1, &chRamkaTelemetrii[st_ZajetośćLPUART.chIndeksNapełnianejRamki[r+1] + r * LICZBA_RAMEK_TELEMETR][0], st_ZajetośćLPUART.sDoWysłania[r+1]);	//wyślij ramkę - Uwaga, nie wyśle 2 ramek na raz, zrobić kolejkę wysyłania
				st_ZajetośćLPUART.sDoWysłania[r+1] = 0;	//wysłano więc zdejmij z kolejki i zezwól na ponowne napełnienie bufora
				st_ZajetośćLPUART.chIndeksNapełnianejRamki[r+1]++;	//przełacz indeks podwójnego buforowania aby można było napełniać drugi bufor
				st_ZajetośćLPUART.chIndeksNapełnianejRamki[r+1] &= 0x01;
				//zdjęcie flagi zajetości LPUART następuje w HAL_UART_TxCpltCallback() po fizycznym zakończeniu wysyłki
				break;	//wyjdź z pętli po wysłaniu pierwszej ramki
			}
		}
	}
	__enable_irq();		//koniec sekcji krytycznej
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
uint8_t WstawDaneDoRamkiTele(uint8_t chIndNapRam, uint8_t chIndAdresow, uint8_t chPozycja, uint16_t sIdZmiennej, float fDane)
{
	uint8_t chDane[2];
	uint8_t chRozmiar;
	uint8_t chIdZmiennej = sIdZmiennej & 0x7F;
	uint8_t chBajtBitu;	//numer bajtu w ktorym jest bit identyfikacyjny

	//wstaw dane
	chRozmiar = ROZMIAR_NAGLOWKA + chPozycja;
	Float2Char16(fDane, chDane);	//konwertuj liczbę float na liczbę o połowie precyzji i zapisz w 2 bajtach
	chRamkaTelemetrii[chIndNapRam + chIndAdresow * LICZBA_RAMEK_TELEMETR][chRozmiar + 0] = chDane[0];
    chRamkaTelemetrii[chIndNapRam + chIndAdresow * LICZBA_RAMEK_TELEMETR][chRozmiar + 1] = chDane[1];

    //wstaw bit identyfikatora zmiennej
    chBajtBitu = chIdZmiennej / 8;
    chRamkaTelemetrii[chIndNapRam + chIndAdresow * LICZBA_RAMEK_TELEMETR][ROZMIAR_NAGLOWKA + chBajtBitu] |= 1 << (chIdZmiennej - (chBajtBitu * 8));
    return chRozmiar + 2;
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
uint8_t WstawDaneDoSzybkiejRamkiTele(uint8_t chIndNapRam, uint8_t chIndeksTestu, uint16_t *sIndeksFFT)
{
	uint8_t chBityZmiennej = AKCEL_X | AKCEL_Y | AKCEL_Z | ZYRO_X | ZYRO_Y | ZYRO_Z;
	uint8_t chLiczbaBajtówRamki = 4;

	//pierwsza część danych ramki zawiera 4 bajtową identyfikację danych
	chRamkaTelemetrii[chIndNapRam][ROZMIAR_NAGLOWKA + 0] = chBityZmiennej;
	chRamkaTelemetrii[chIndNapRam][ROZMIAR_NAGLOWKA + 1] = chIndeksTestu;
	un8_32.dane16[0] = *sIndeksFFT;
	chRamkaTelemetrii[chIndNapRam][ROZMIAR_NAGLOWKA + 2] = un8_32.dane8[0];
	chRamkaTelemetrii[chIndNapRam][ROZMIAR_NAGLOWKA + 3] = un8_32.dane8[1];

	//wstaw tyle danych FFT aby wypełnić ramkę, ale dane mają pochodzić tylko z jednego FFT, bo indeks testu idzie na początku danych ramki
	for (uint16_t n=0; n<((ROZMIAR_DANYCH_KOMUNIKACJI - 4) / 12); n++)	//dla uproszczenia zakładam że ramka ma komplet 6 danych 16-bit
	{
		chLiczbaBajtówRamki += PobierzWynikiFFT(&chRamkaTelemetrii[chIndNapRam][n * 12 + 4 + ROZMIAR_NAGLOWKA], chBityZmiennej, chIndeksTestu, *sIndeksFFT);
		(*sIndeksFFT)++;
		if (*sIndeksFFT >= stKonfigFFT.sLiczbaProbek / 2)	//koniec danych nie musi być równoważny z pełną ramką, więc...
			break;										//...przerwij napełnianie kiedy zostały przygotowane wszystkie dane
	}
	return chLiczbaBajtówRamki;
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja wstawia do bieżącego bufora telemetrii liczbę do wysłania
// Parametry: fDane - liczba do wysłania
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
float PobierzZmiennaTele(uint16_t sZmienna)
{
	float fZmiennaTele;

	switch (sZmienna)
	{
	case TELEID_AKCEL1X:	fZmiennaTele = uDaneCM4.dane.fAkcel1[0];		break;
	case TELEID_AKCEL1Y:	fZmiennaTele = uDaneCM4.dane.fAkcel1[1];		break;
	case TELEID_AKCEL1Z:	fZmiennaTele = uDaneCM4.dane.fAkcel1[2];		break;
	case TELEID_AKCEL2X:	fZmiennaTele = uDaneCM4.dane.fAkcel2[0];		break;
	case TELEID_AKCEL2Y:	fZmiennaTele = uDaneCM4.dane.fAkcel2[1];		break;
	case TELEID_AKCEL2Z:	fZmiennaTele = uDaneCM4.dane.fAkcel2[2];		break;
	case TELEID_ZYRO1P:		fZmiennaTele = uDaneCM4.dane.fZyroKal1[0];		break;
	case TELEID_ZYRO1Q:		fZmiennaTele = uDaneCM4.dane.fZyroKal1[1];		break;
	case TELEID_ZYRO1R:		fZmiennaTele = uDaneCM4.dane.fZyroKal1[2];		break;
	case TELEID_ZYRO2P:		fZmiennaTele = uDaneCM4.dane.fZyroKal2[0];		break;
	case TELEID_ZYRO2Q:		fZmiennaTele = uDaneCM4.dane.fZyroKal2[1];		break;
	case TELEID_ZYRO2R:		fZmiennaTele = uDaneCM4.dane.fZyroKal2[2];		break;
	case TELEID_MAGNE1X:	fZmiennaTele = uDaneCM4.dane.fMagne1[0];		break;
	case TELEID_MAGNE1Y:	fZmiennaTele = uDaneCM4.dane.fMagne1[1];		break;
	case TELEID_MAGNE1Z:	fZmiennaTele = uDaneCM4.dane.fMagne1[2];		break;
	case TELEID_MAGNE2X:	fZmiennaTele = uDaneCM4.dane.fMagne2[0];		break;
	case TELEID_MAGNE2Y:	fZmiennaTele = uDaneCM4.dane.fMagne2[1];		break;
	case TELEID_MAGNE2Z:	fZmiennaTele = uDaneCM4.dane.fMagne2[2];		break;
	case TELEID_MAGNE3X:	fZmiennaTele = uDaneCM4.dane.fMagne3[0];		break;
	case TELEID_MAGNE3Y:	fZmiennaTele = uDaneCM4.dane.fMagne3[1];		break;
	case TELEID_MAGNE3Z:	fZmiennaTele = uDaneCM4.dane.fMagne3[2];		break;
	case TELEID_TEMPIMU1:	fZmiennaTele = uDaneCM4.dane.fTemper[TEMP_IMU1];		break;
	case TELEID_TEMPIMU2:	fZmiennaTele = uDaneCM4.dane.fTemper[TEMP_IMU1];		break;

	//zmienne AHRS
	case TELEID_KAT_IMU1X:	fZmiennaTele = uDaneCM4.dane.fKatIMU1[0];		break;
	case TELEID_KAT_IMU1Y:	fZmiennaTele = uDaneCM4.dane.fKatIMU1[1];		break;
	case TELEID_KAT_IMU1Z:	fZmiennaTele = uDaneCM4.dane.fKatIMU1[2];		break;
	case TELEID_KAT_IMU2X:	fZmiennaTele = uDaneCM4.dane.fKatIMU2[0];		break;
	case TELEID_KAT_IMU2Y:	fZmiennaTele = uDaneCM4.dane.fKatIMU2[1];		break;
	case TELEID_KAT_IMU2Z:	fZmiennaTele = uDaneCM4.dane.fKatIMU2[2];		break;
	case TELEID_KAT_ZYRO1X:	fZmiennaTele = uDaneCM4.dane.fKatZyro1[0];		break;
	case TELEID_KAT_ZYRO1Y:	fZmiennaTele = uDaneCM4.dane.fKatZyro1[1];		break;
	case TELEID_KAT_ZYRO1Z:	fZmiennaTele = uDaneCM4.dane.fKatZyro1[2];		break;
	case TELEID_KAT_AKCELX:	fZmiennaTele = uDaneCM4.dane.fKatAkcel1[0];		break;
	case TELEID_KAT_AKCELY:	fZmiennaTele = uDaneCM4.dane.fKatAkcel1[1];		break;
	case TELEID_KAT_AKCELZ:	fZmiennaTele = uDaneCM4.dane.fKatAkcel1[2];		break;

	//zmienne barametryczne
	case TELEID_CISBEZW1:	fZmiennaTele = uDaneCM4.dane.fCisnieBzw[0];		break;
	case TELEID_CISBEZW2:	fZmiennaTele = uDaneCM4.dane.fCisnieBzw[1];		break;
	case TELEID_WYSOKOSC1:	fZmiennaTele = uDaneCM4.dane.fWysokoMSL[0];		break;
	case TELEID_WYSOKOSC2:	fZmiennaTele = uDaneCM4.dane.fWysokoMSL[1];		break;
	case TELEID_CISROZN1:	fZmiennaTele = uDaneCM4.dane.fCisnRozn[0];		break;
	case TELEID_CISROZN2:	fZmiennaTele = uDaneCM4.dane.fCisnRozn[1];		break;
	case TELEID_PREDIAS1:	fZmiennaTele = uDaneCM4.dane.fPredkosc[0];		break;
	case TELEID_PREDIAS2:	fZmiennaTele = uDaneCM4.dane.fPredkosc[1];		break;
	case TELEID_TEMPCISB1:	fZmiennaTele = uDaneCM4.dane.fTemper[TEMP_BARO1];		break;
	case TELEID_TEMPCISB2:	fZmiennaTele = uDaneCM4.dane.fTemper[TEMP_BARO2];		break;
	case TELEID_TEMPCISR1:	fZmiennaTele = uDaneCM4.dane.fTemper[TEMP_CISR1];		break;
	case TELEID_TEMPCISR2:	fZmiennaTele = uDaneCM4.dane.fTemper[TEMP_CISR2];		break;

	//odbiorniki RC
	case TELEID_RC_KAN1:
	case TELEID_RC_KAN2:
	case TELEID_RC_KAN3:
	case TELEID_RC_KAN4:
	case TELEID_RC_KAN5:
	case TELEID_RC_KAN6:
	case TELEID_RC_KAN7:
	case TELEID_RC_KAN8:
	case TELEID_RC_KAN9:
	case TELEID_RC_KAN10:
	case TELEID_RC_KAN11:
	case TELEID_RC_KAN12:
	case TELEID_RC_KAN13:
	case TELEID_RC_KAN14:
	case TELEID_RC_KAN15:	//aby nie generować zbyt dużo kodu stosuję wzór do obliczenia indeksu zmiennej dla wszystkich kanałów
	case TELEID_RC_KAN16:	fZmiennaTele = uDaneCM4.dane.sKanalRC[sZmienna - TELEID_RC_KAN1];	break;

	//Serwa
	case TELEID_SERWO1:
	case TELEID_SERWO2:
	case TELEID_SERWO3:
	case TELEID_SERWO4:
	case TELEID_SERWO5:
	case TELEID_SERWO6:
	case TELEID_SERWO7:
	case TELEID_SERWO8:
	case TELEID_SERWO9:
	case TELEID_SERWO10:
	case TELEID_SERWO11:
	case TELEID_SERWO12:
	case TELEID_SERWO13:
	case TELEID_SERWO14:
	case TELEID_SERWO15:	//aby nie generować zbyt dużo kodu stosuję wzór do obliczenia indeksu zmiennej dla wszystkich kanałów
	case TELEID_SERWO16:	fZmiennaTele = uDaneCM4.dane.sWyjscieRC[sZmienna - TELEID_SERWO1];	break;

	case TELEID_DOTYK_ADC0:	fZmiennaTele = statusDotyku.sAdc[0];		break;
	case TELEID_DOTYK_ADC1:	fZmiennaTele = statusDotyku.sAdc[1];		break;
	case TELEID_DOTYK_ADC2:	fZmiennaTele = statusDotyku.sAdc[2];		break;
	case TELEID_CZAS_PETLI: fZmiennaTele = uDaneCM4.dane.ndT;			break;

	case TELEID_FFT_ZYRO_AKCEL:		break;	//wyniki transformaty fouriera przesyłane w specyficznej szybkiej ramce

	case TELEID_PID_PRZE_WZAD:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRZE].fZadana;		break;	//wartość zadana regulatora sterowania przechyleniem
	case TELEID_PID_PRZE_WYJ:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRZE].fWyjsciePID;	break;	//wyjście regulatora sterowania przechyleniem
	case TELEID_PID_PRZE_WY_P:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRZE].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_PRZE_WY_I:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRZE].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_PRZE_WY_D:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRZE].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_PK_PRZE_WYJ:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_PRZE].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością kątową przechylenia
	case TELEID_PID_PK_PRZE_WY_P:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_PRZE].fWyjscieP;	break;	//wyjście członu P
	case TELEID_PID_PK_PRZE_WY_I:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_PRZE].fWyjscieI;	break;	//wyjście członu I
	case TELEID_PID_PK_PRZE_WY_D:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_PRZE].fWyjscieD;	break;	//wyjście członu D

	case TELEID_PID_POCH_WZAD:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_POCH].fZadana;		break;	//wartość zadana regulatora sterowania pochyleniem
	case TELEID_PID_POCH_WYJ:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_POCH].fWyjsciePID;	break;	//wyjście regulatora sterowania pochyleniem
	case TELEID_PID_POCH_WY_P:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_POCH].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_POCH_WY_I:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_POCH].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_POCH_WY_D:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_POCH].fWyjscieD;		break;	//wyjście członu D
	case TELEID_PID_PK_POCH_WYJ:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_POCH].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością kątową pochylenia
	case TELEID_PID_PK_POCH_WY_P:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_POCH].fWyjscieP;	break;	//wyjście członu P
	case TELEID_PID_PK_POCH_WY_I:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_POCH].fWyjscieI;	break;	//wyjście członu I
	case TELEID_PID_PK_POCH_WY_D:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_POCH].fWyjscieD;	break;	//wyjście członu D

	case TELEID_PID_ODCH_WZAD:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_ODCH].fZadana;		break;	//wartość zadana regulatora sterowania odchyleniem
	case TELEID_PID_ODCH_WYJ:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_ODCH].fWyjsciePID;	break;	//wyjście regulatora sterowania odchyleniem
	case TELEID_PID_ODCH_WY_P:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_ODCH].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_ODCH_WY_I:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_ODCH].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_ODCH_WY_D:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_ODCH].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_PK_ODCH_WYJ:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_ODCH].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością kątową odchylenia
	case TELEID_PID_PK_ODCH_WY_P:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_ODCH].fWyjscieP;	break;	//wyjście członu P
	case TELEID_PID_PK_ODCH_WY_I:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_ODCH].fWyjscieI;	break;	//wyjście członu I
	case TELEID_PID_PK_ODCH_WY_D:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_ODCH].fWyjscieD;	break;	//wyjście członu D

	case TELEID_PID_WYSO_WZAD:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WYSO].fZadana;		break;	//wartość zadana regulatora sterowania wysokością
	case TELEID_PID_WYSO_WYJ:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WYSO].fWyjsciePID;	break;	//wyjście regulatora sterowania odchyleniem
	case TELEID_PID_WYSO_WY_P:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WYSO].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_WYSO_WY_I:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WYSO].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_WYSO_WY_D:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WYSO].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_PR_WYSO_WYJ:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WARIO].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością zmiany wysokości
	case TELEID_PID_PR_WYSO_WY_P:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WARIO].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_PR_WYSO_WY_I:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WARIO].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_PR_WYSO_WY_D:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WARIO].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_NAWN_WZAD:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_N].fZadana;		break;	//wartość zadana regulatora sterowania nawigacją w kierunku północnym
	case TELEID_PID_NAWN_WYJ:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_N].fWyjsciePID;	break;	//wyjście regulatora sterowania nawigacją w kierunku północnym
	case TELEID_PID_NAWN_WY_P:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_N].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_NAWN_WY_I:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_N].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_NAWN_WY_D:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_N].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_PR_NAWN_WYJ:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_N].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością w kierunku północnym
	case TELEID_PID_PR_NAWN_WY_P:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_N].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_PR_NAWN_WY_I:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_N].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_PR_NAWN_WY_D:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_N].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_NAWE_WZAD:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_E].fZadana;		break;	//wartość zadana regulatora sterowania nawigacją w kierunku wschodnim
	case TELEID_PID_NAWE_WYJ:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_E].fWyjsciePID;	break;	//wyjście regulatora sterowania nawigacją w kierunku północnym
	case TELEID_PID_NAWE_WY_P:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_E].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_NAWE_WY_I:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_E].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_NAWE_WY_D:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_E].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_PR_NAWE_WYJ:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_E].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością w kierunku wschodnim
	case TELEID_PID_PR_NAWE_WY_P:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_E].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_PR_NAWE_WY_I:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_E].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_PR_NAWE_WY_D:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_E].fWyjscieD;		break;	//wyjście członu D

	default:	fZmiennaTele = -1.0f;
	}
	return fZmiennaTele;
}



///////////////////////////////////////////////////////////////////////////////
// Tworzy nagłówek ramki telemetrii. Inicjuje CRC
// Parametry:
//  chIndeksRamki - indeks napełnianiej ramki uwzględniajacy numer ramki i podwójne buforowanie (jedna jest napełniana, druga sie wysyła)
//  chTypRamki - okresla czy to jest normalna ramka telemetryczna, czy szybka
//  chAdrZdalny - adres urządzenia do którego wysyłamy
//  chAdrLokalny - nasz adres
//  chRozmDanych - liczba bajtw danych uwzględniająca pole bitowe zmiennych w typowej ramce oraz nagłówek w ramce szybkiej
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PrzygotujRamkeTele(uint8_t chIndeksRamki, uint8_t chTypRamki, uint8_t chAdrZdalny, uint8_t chAdrLokalny, uint8_t chRozmDanych)
{
	PobierzDateCzas(&stDate, &stTime);

	InicjujCRC16(0, WIELOMIAN_CRC);
	chRamkaTelemetrii[chIndeksRamki][0] = NAGLOWEK;
	chRamkaTelemetrii[chIndeksRamki][1] = CRC->DR = chAdrZdalny;
	chRamkaTelemetrii[chIndeksRamki][2] = CRC->DR = chAdrLokalny;
	chRamkaTelemetrii[chIndeksRamki][3] = CRC->DR = (uint8_t)stTime.SubSeconds;	//1/256 cześć sekundy czasu RTC synchronizowanego z GPS
	if (chTypRamki == TELEM_NORMALNA)
		chRamkaTelemetrii[chIndeksRamki][4] = CRC->DR = PK_TELEMETRIA1 + chIndeksRamki / 2;	//indeks ramki wskazuje na zakres zmiennych a to determinuje numer polecenia
	else
		chRamkaTelemetrii[chIndeksRamki][4] = CRC->DR = PK_TELEM_SZYBKA;	//ramka szybk jest na razie tylko jedna
	chRamkaTelemetrii[chIndeksRamki][5] = CRC->DR = chRozmDanych;

	//policz CRC z danych i listy zmiennych
	for(uint16_t n=0; n < chRozmDanych; n++)
		CRC->DR = chRamkaTelemetrii[chIndeksRamki][ROZMIAR_NAGLOWKA + n];

	un8_32.dane16[0] = (uint16_t)CRC->DR;
	chRamkaTelemetrii[chIndeksRamki][ROZMIAR_NAGLOWKA + chRozmDanych + 0] =  un8_32.dane8[0];	//młodszy przodem
	chRamkaTelemetrii[chIndeksRamki][ROZMIAR_NAGLOWKA + chRozmDanych + 1] =  un8_32.dane8[1];	//starszy

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
// Zapisuje do FLASH okres wysyłania kolenych zmiennych telemetrycznych w porządku młodszy przodem
// Parametry: sPrzesuniecie - przesuniecie danych względem początku, numer zmiennej od ktorego dane mają być zapisane
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszKonfiguracjeTelemetrii(uint16_t sPrzesuniecie)
{
	uint8_t chDoZapisu = OKRESOW_TELEMETRII_W_RAMCE;
	uint16_t sIndeksPaczki = (sPrzesuniecie * 2) / ROZMIAR_DANYCH_WPACZCE;
	uint8_t chProbZapisu = 3;
	uint8_t chErr;

	while (chDoZapisu && chProbZapisu)		//czytaj kolejne paczki aż skompletuje tyle danych ile potrzeba
	{
		chErr = ZapiszPaczkeKonfigu(FKON_OKRES_TELEMETRI1 + sIndeksPaczki, (uint8_t*)&sOkresTelemetrii[sIndeksPaczki * ROZMIAR_DANYCH_WPACZCE / 2]);
		if (chErr == BLAD_OK)
		{
			sIndeksPaczki++;
			chProbZapisu = 3;
			if (chDoZapisu > ROZMIAR_DANYCH_WPACZCE / 2)
				chDoZapisu -= ROZMIAR_DANYCH_WPACZCE / 2;
			else
				chDoZapisu = 0;
		}
		chProbZapisu--;
	}
	return chErr;
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja włącza lub wyłącza telemetrię w tym telemetrię szybką
// Parametry: chOperacja - polecenie do ustawienia w zmiennej statusu telemetrii
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WłączTelemetrię(uint8_t chOperacja)
{
	chStatusTelemetrii = chOperacja;
	sIndeksWysyłkiFFT = chIndeksWysyłkiTestuFFT = 0;	//resetuj parametry szybkiej telemetrii
}
