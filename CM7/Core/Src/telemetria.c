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

// Dane telemetryczne są wysyłane w zbiorczej ramce mogącej pomieścić MAX_ZMIENNYCH_TELEMETR_W_RAMCE (115). Dane są z puli adresowej obejmującej MAX_INDEKSOW_TELEMETR_W_RAMCE (128) zmiennych.
// Ponieważ danych może być więcej, przewodziano 2 rodzaje lub nawet wiecej ramek telemetrii
// Na początku ramki znajdują się słowa identyfikujące rodzaj przesyłanych danych, gdzie kolejne bity określają rodzaj przesyłanych zmiennych.
// Każda zmienna może mieć zdefiniowany inny okres wysyłania będący wielokrotnością KWANT_CZASU_TELEMETRII
// dla KWANT_CZASU_TELEMETRII == 10ms daje to max 100 Hz, min 0,025 Hz (40s)
// Każda ramka ma 2 kopie, gdzie jedna ramka jest napełniana a druga wysyłana.

// LPUART korzysta z BDMA a ono ma dostęp tylko do SRAM4, więc ramka telemetrii musi być zdefiniowana w SRAM4
ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM4")))	chRamkaTelemetrii[2*LICZBA_RAMEK_TELEMETR][ROZMIAR_RAMKI_UART]);	//ramki telemetryczne: przygotowywana i wysyłana dla zmiennych 0..127 oraz 128..255
uint16_t sOkresTelemetrii[LICZBA_RAMEK_TELEMETR * MAX_INDEKSOW_TELEMETR_W_RAMCE];	//zmienna definiujaca okres wysyłania telemetrii dla wszystkich zmiennych
uint16_t sLicznikTelemetrii[LICZBA_RAMEK_TELEMETR * MAX_INDEKSOW_TELEMETR_W_RAMCE];
uint8_t chIndeksNapelnRamki;	//okresla ktora tablica ramki telemetrycznej jest napełniania

extern unia_wymianyCM4_t uDaneCM4;
extern uint8_t chAdresZdalny[ILOSC_INTERF_KOM];	//adres sieciowy strony zdalnej
extern UART_HandleTypeDef hlpuart1;
static un8_16_t un8_16;		//unia do konwersji między danymi 16 i 8 bit
//extern volatile uint8_t chUartKomunikacjiZajety;
extern volatile uint8_t chDoWyslania[1 + LICZBA_RAMEK_TELEMETR];	//lista rzeczy do wysłania po zakończeniu bieżącej transmisji: ramka poleceń i ramki telemetryczne
extern stBSP_t stBSP;	//struktura zawierajaca adresy i nazwę BSP
extern volatile st_ZajetoscLPUART_t st_ZajetoscLPUART;
uint8_t chOdczytano, chDoOdczytu = MAX_INDEKSOW_TELEMETR_W_RAMCE;

////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjalizuje zmienne używane do obsługi telemetrii. Dane są zapisane w porządku cienkokońcówkowym
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjalizacjaTelemetrii(void)
{
	uint8_t chPaczka[ROZMIAR_PACZKI_KONFIG];
	uint8_t chOdczytano;
	uint8_t chDoOdczytu = MAX_INDEKSOW_TELEMETR_W_RAMCE;
	uint8_t chIndeksPaczki = 0;
	uint8_t chProbOdczytu = 5;
	uint16_t sOkres;

	//Inicjuj zmienne wartościa wyłączoną gdyby nie dało się odczytać konfiguracji.
	//Lepiej jest mieć telemetrię wyłaczoną niż zapchaną pracujacą na 100%
	for (uint16_t n=0; n<MAX_INDEKSOW_TELEMETR_W_RAMCE; n++)
		sOkresTelemetrii[n] = TELEMETRIA_WYLACZONA;

	while (chDoOdczytu && chProbOdczytu)		//czytaj kolejne paczki aż skompletuje tyle danych ile potrzeba
	{
		chOdczytano = CzytajPaczkeKonfigu(chPaczka, FKON_OKRES_TELEMETRI1 + chIndeksPaczki);		//odczytaj 30 bajtów danych + identyfikator i CRC
		if (chOdczytano == ROZMIAR_PACZKI_KONFIG)
		{
			for (uint16_t n=0; n<((ROZMIAR_PACZKI_KONFIG - 2) / 2); n++)
			{
				if (chDoOdczytu)	//nie czytaj wiecej niż trzeba zby nie przepelnić zmiennej
				{
					sOkres = chPaczka[2*n+2] + chPaczka[2*n+3] * 0x100;
					sOkresTelemetrii[n + chIndeksPaczki * ROZMIAR_DANYCH_WPACZCE /2] = sOkres;
					chDoOdczytu--;
				}
			}
			chIndeksPaczki++;
		}
		chProbOdczytu--;
	}

	//inicjuj licznik okresem wysyłania
	for (uint16_t n=0; n<MAX_INDEKSOW_TELEMETR_W_RAMCE; n++)
		sLicznikTelemetrii[n] = sOkresTelemetrii[n];
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja przygotowuje i wysyła zmienne telemetryczne
// Parametry: chInterfejs - interfejs komunikacyjny (na razie tylko LPUART)
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObslugaTelemetrii(uint8_t chInterfejs)
{
	uint8_t chLicznikZmienych = 0;
	uint8_t chIloscDanych[LICZBA_RAMEK_TELEMETR] = {0, 0};
	uint8_t chNrRamki;
	//uint16_t sRozmiarRamki;
	float fZmienna;

	//wyczyść w ramkach pola na bity identyfikujące zmienne, bo kolejne bity będą OR-owane z wartością początkową, więc musi ona na początku być zerem
	for (uint8_t r=0; r<LICZBA_RAMEK_TELEMETR; r++)
	{
		for(uint8_t n=0; n<LICZBA_BAJTOW_ID_TELEMETRII; n++)
			chRamkaTelemetrii[chIndeksNapelnRamki + 2 * r][ROZMIAR_NAGLOWKA + n] = 0;
	}

	for(uint16_t n=0; n<LICZBA_ZMIENNYCH_TELEMETRYCZNYCH; n++)
	{
		if (sLicznikTelemetrii[n] != TELEMETRIA_WYLACZONA)	//wartość oznaczająca aby nie wysyłać danych
			sLicznikTelemetrii[n]--;

		if (sLicznikTelemetrii[n] == 0)		//licznik zmiennej doszedł do 0 więc trzeba ją wysłać
		{
			sLicznikTelemetrii[n] = sOkresTelemetrii[n];		//przeładuj licznik nowym okresem
			fZmienna = PobierzZmiennaTele(n);
			chNrRamki = n >> 7;
			if (chIloscDanych[chNrRamki] < (ROZMIAR_RAMKI_UART - ROZMIAR_CRC - 2))	//sprawdź czy dane mieszczą się w ramce
			{

				WstawDaneDoRamkiTele(chIndeksNapelnRamki, chLicznikZmienych, n, fZmienna);
				chIloscDanych[chNrRamki]++;
				//chLicznikZmienych++;
			}
		}
	}

	//przygotuj ramki do wysłania
	for (uint8_t r=0; r<LICZBA_RAMEK_TELEMETR; r++)
	{
		if (chIloscDanych[r] > 0)	//jeżeli jest coś do wysłania
		{
			PrzygotujRamkeTele(chIndeksNapelnRamki, chAdresZdalny[chInterfejs], stBSP.chAdres, chLicznikZmienych);	//utwórz ramkę gotową do wysyłki
			st_ZajetoscLPUART.sDoWyslania[r+1] = chIloscDanych[r] * 2 + LICZBA_BAJTOW_ID_TELEMETRII + ROZM_CIALA_RAMKI;
		}
	}

	if (st_ZajetoscLPUART.chZajetyPrzez == 0)	//jeżeli LPUART nie jest zajęty to wyślij telemetrię
	{
		for (uint8_t r=0; r<LICZBA_RAMEK_TELEMETR; r++)
		{
			if (st_ZajetoscLPUART.sDoWyslania[r+1])
			{
				st_ZajetoscLPUART.chZajetyPrzez = RAMKA_TELE1 + r;
				HAL_UART_Transmit_DMA(&hlpuart1, &chRamkaTelemetrii[chIndeksNapelnRamki][0], st_ZajetoscLPUART.sDoWyslania[r+1]);	//wyślij ramkę - Uwaga, nie wyśle 2 ramek na raz, zrobić kolejkę wysyłania
				break;
			}
		}
	}

	//wskaż na następną ramkę
	chIndeksNapelnRamki++;
	chIndeksNapelnRamki &= 0x01;
}



///////////////////////////////////////////////////////////////////////////////
// Funkcja wstawia do bieżącej ramki telemetrii liczbę do wysłania
// Parametry:
// chIndNapRam - Indeks napełnianych ramek (ramki o przeciwnej parzystości są w tym czasie opróżniane)
// chPozycja - kolejny numer zmiennej w ramce
// sIdZmiennej - identyfikator typu zmiennej
// fDane - liczba do wysłania
// Zwraca: rozmiar ramki
////////////////////////////////////////////////////////////////////////////////
uint8_t WstawDaneDoRamkiTele(uint8_t chIndNapRam, uint8_t chPozycja, uint16_t sIdZmiennej, float fDane)
{
	uint8_t chDane[2];
	uint8_t chRozmiar;
	uint8_t chIdZmiennej = sIdZmiennej & 0x7F;
	uint8_t chBajtBitu;	//numer bajtu w ktorym jest bit identyfikacyjny

	chIndNapRam &= 0x01;	//obetnij nadmiar danych. To co przychodzi wskazuje tylko na ramkę napełnianą (jedną z dwóch)
	chIndNapRam *= 2 * (sIdZmiennej >> 7);	//zmienne o numerach będących wielokrotnością 128 mają być umieszczone w kolejnych parach tablicy

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
	case TELEID_ZYRO1P:		fZmiennaTele = uDaneCM4.dane.fKatZyro1[0];		break;
	case TELEID_ZYRO1Q:		fZmiennaTele = uDaneCM4.dane.fKatZyro1[1];		break;
	case TELEID_ZYRO1R:		fZmiennaTele = uDaneCM4.dane.fKatZyro1[2];		break;
	case TELEID_ZYRO2P:		fZmiennaTele = uDaneCM4.dane.fKatZyro2[0];		break;
	case TELEID_ZYRO2Q:		fZmiennaTele = uDaneCM4.dane.fKatZyro2[1];		break;
	case TELEID_ZYRO2R:		fZmiennaTele = uDaneCM4.dane.fKatZyro2[2];		break;
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
	case TELEID_SERWO16:	fZmiennaTele = uDaneCM4.dane.sSerwo[sZmienna - TELEID_SERWO1];	break;

	case TELEID_PID_PRZE_WZAD:		break;	//wartość zadana regulatora sterowania przechyleniem
	case TELEID_PID_PRZE_WYJ:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRZE].fWyjsciePID;	break;	//wyjście regulatora sterowania przechyleniem
	case TELEID_PID_PRZE_WY_P:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRZE].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_PRZE_WY_I:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRZE].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_PRZE_WY_D:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRZE].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_PK_PRZE_WYJ:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_PRZE].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością kątową przechylenia
	case TELEID_PID_PK_PRZE_WY_P:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_PRZE].fWyjscieP;	break;	//wyjście członu P
	case TELEID_PID_PK_PRZE_WY_I:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_PRZE].fWyjscieI;	break;	//wyjście członu I
	case TELEID_PID_PK_PRZE_WY_D:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_PRZE].fWyjscieD;	break;	//wyjście członu D

	case TELEID_PID_POCH_WZAD:	break;	//wartość zadana regulatora sterowania pochyleniem
	case TELEID_PID_POCH_WYJ:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_POCH].fWyjsciePID;	break;	//wyjście regulatora sterowania pochyleniem
	case TELEID_PID_POCH_WY_P:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_POCH].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_POCH_WY_I:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_POCH].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_POCH_WY_D:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_POCH].fWyjscieD;		break;	//wyjście członu D
	case TELEID_PID_PK_POCH_WYJ:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_POCH].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością kątową pochylenia
	case TELEID_PID_PK_POCH_WY_P:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_POCH].fWyjscieP;	break;	//wyjście członu P
	case TELEID_PID_PK_POCH_WY_I:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_POCH].fWyjscieI;	break;	//wyjście członu I
	case TELEID_PID_PK_POCH_WY_D:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_POCH].fWyjscieD;	break;	//wyjście członu D

	case TELEID_PID_ODCH_WZAD:	break;	//wartość zadana regulatora sterowania odchyleniem
	case TELEID_PID_ODCH_WYJ:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_ODCH].fWyjsciePID;	break;	//wyjście regulatora sterowania odchyleniem
	case TELEID_PID_ODCH_WY_P:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_ODCH].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_ODCH_WY_I:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_ODCH].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_ODCH_WY_D:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_ODCH].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_PK_ODCH_WYJ:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_ODCH].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością kątową odchylenia
	case TELEID_PID_PK_ODCH_WY_P:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_ODCH].fWyjscieP;	break;	//wyjście członu P
	case TELEID_PID_PK_ODCH_WY_I:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_ODCH].fWyjscieI;	break;	//wyjście członu I
	case TELEID_PID_PK_ODCH_WY_D:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PK_ODCH].fWyjscieD;	break;	//wyjście członu D

	case TELEID_PID_WYSO_WZAD:	break;	//wartość zadana regulatora sterowania wysokością
	case TELEID_PID_WYSO_WYJ:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WYSO].fWyjsciePID;	break;	//wyjście regulatora sterowania odchyleniem
	case TELEID_PID_WYSO_WY_P:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WYSO].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_WYSO_WY_I:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WYSO].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_WYSO_WY_D:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WYSO].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_PR_WYSO_WYJ:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WARIO].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością zmiany wysokości
	case TELEID_PID_PR_WYSO_WY_P:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WARIO].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_PR_WYSO_WY_I:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WARIO].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_PR_WYSO_WY_D:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_WARIO].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_NAWN_WZAD:	break;	//wartość zadana regulatora sterowania nawigacją w kierunku północnym
	case TELEID_PID_NAWN_WYJ:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_N].fWyjsciePID;	break;	//wyjście regulatora sterowania nawigacją w kierunku północnym
	case TELEID_PID_NAWN_WY_P:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_N].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_NAWN_WY_I:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_N].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_NAWN_WY_D:		fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_NAW_N].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_PR_NAWN_WYJ:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_N].fWyjsciePID;	break;	//wyjście regulatora sterowania prędkością w kierunku północnym
	case TELEID_PID_PR_NAWN_WY_P:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_N].fWyjscieP;		break;	//wyjście członu P
	case TELEID_PID_PR_NAWN_WY_I:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_N].fWyjscieI;		break;	//wyjście członu I
	case TELEID_PID_PR_NAWN_WY_D:	fZmiennaTele = uDaneCM4.dane.stWyjPID[PID_PRE_N].fWyjscieD;		break;	//wyjście członu D

	case TELEID_PID_NAWE_WZAD:	break;	//wartość zadana regulatora sterowania nawigacją w kierunku wschodnim
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
// Parametry: chIndNapRam - indeks napełnianiej ramki (jedna jest napełniana, druga sie wysyła)
// chAdrZdalny - adres urządzenia do którego wysyłamy
// chAdrLokalny - nasz adres
// lListaZmiennych - zmienna z polam bitowymi okreslającymi przesyłane zmienne
// chRozmDanych - liczba zmiennych telemetrycznych do wysłania w ramce
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PrzygotujRamkeTele(uint8_t chIndNapRam, uint8_t chAdrZdalny, uint8_t chAdrLokalny, uint8_t chRozmDanych)
{
	uint32_t nCzasSystemowy = PobierzCzasT6();

	InicjujCRC16(0, WIELOMIAN_CRC);
	chRamkaTelemetrii[chIndNapRam][0] = NAGLOWEK;
	chRamkaTelemetrii[chIndNapRam][1] = CRC->DR = chAdrZdalny;
	chRamkaTelemetrii[chIndNapRam][2] = CRC->DR = chAdrLokalny;
	chRamkaTelemetrii[chIndNapRam][3] = CRC->DR = (nCzasSystemowy / 10) & 0xFF;
	chRamkaTelemetrii[chIndNapRam][4] = CRC->DR = PK_TELEMETRIA1 + chIndNapRam / 2;	//indeks ramki wskazuje na zakres zmiennych a to determinuje numer polecenia
	chRamkaTelemetrii[chIndNapRam][5] = CRC->DR = chRozmDanych * 2 + LICZBA_BAJTOW_ID_TELEMETRII;

	//policz CRC z danych i listy zmiennych
	for(uint16_t n=0; n < (chRozmDanych * 2 + LICZBA_BAJTOW_ID_TELEMETRII); n++)
		CRC->DR = chRamkaTelemetrii[chIndNapRam][ROZMIAR_NAGLOWKA + n];

	un8_16.dane16 = (uint16_t)CRC->DR;
	chRamkaTelemetrii[chIndNapRam][ROZMIAR_NAGLOWKA + LICZBA_BAJTOW_ID_TELEMETRII + chRozmDanych * 2 + 0] =  un8_16.dane8[0];	//młodszy przodem
	chRamkaTelemetrii[chIndNapRam][ROZMIAR_NAGLOWKA + LICZBA_BAJTOW_ID_TELEMETRII + chRozmDanych * 2 + 1] =  un8_16.dane8[1];	//starszy

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
	uint8_t chIndeksPaczki = (sPrzesuniecie * 2) / ROZMIAR_DANYCH_WPACZCE;
	uint8_t chProbZapisu = 3;
	uint8_t chErr;

	while (chDoZapisu && chProbZapisu)		//czytaj kolejne paczki aż skompletuje tyle danych ile potrzeba
	{
		chErr = ZapiszPaczkeKonfigu(FKON_OKRES_TELEMETRI1 + chIndeksPaczki, (uint8_t*)&sOkresTelemetrii[chIndeksPaczki * ROZMIAR_DANYCH_WPACZCE / 2]);
		if (chErr == ERR_OK)
		{
			chIndeksPaczki++;
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
