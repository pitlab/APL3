//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł protokołu komunikacyjnego
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

/* Struktura ramki komunikacyjnej
 * NAGLOWEK = 0xAA Strumień danych wejściowych analizujemy pod kątem obecności nagłówka.
 * ADRES ODBIORCY - 0x00 stacja naziemna, 0xFF broadcast, pozostałe to numery BSP w roju.Podczas odbioru w tym miejscu przestajemy analizować jeżeli ramka nas nie dotyczy.Od tego miejsca może zaczynać się szyfrowanie
 * ADERS NADAWCY - 0x01..0xFE aby można było zidentyfikować od kogo pochodzi ramka
 * ZNACZNIK CZASU - licznik setnych części sekundy po to aby można było poskładać we właściwej kolejności dane przesyłane w wielu ramkach
 * POLECENIE - kod polecenia do wykonania.
 * ROZMIAR - liczba bajtów danych ramki
 * DANE - opcjonalne dane
 * CRC16 - suma kontrolna ramki od nagłówka do CRC16. młodszy przodem */

#include <Czas.h>
#include <Komunikacja.h>
#include "cmsis_os.h"
#include "ProtokolKomunikacyjny.h"
#include <stdio.h>
#include "WymianaCM7.h"
#include "FlashKonfig.h"
#include <string.h>
#include <Telemetria.h>

//definicje zmiennych
static uint8_t cStanProtokolu[ILOSC_INTERF_KOM];
uint8_t cAdresZdalny[ILOSC_INTERF_KOM];	//adres sieciowy strony zdalnej
static uint8_t cAdresPrzychodzacy;	//może to być adres BSP lub broadcast
static uint8_t cLicznikDanych[ILOSC_INTERF_KOM];
static uint8_t cZnakCzasu[ILOSC_INTERF_KOM];
static uint16_t sCrc16We;
static uint8_t cPolecenie;
static uint8_t cRozmDanych;
static uint8_t cDaneRamkiKom[ROZMIAR_DANYCH_KOMUNIKACJI];
static unia8_32_t un8_32;
stBSP_ID_t stBSP_ID;	//struktura zawierajaca adresy i nazwę BSP
const char* cNazwaSierotki = {"Sierotka Wronia"};	//domyślna nazwa nienazwanego BSP

//ponieważ BDMA nie potrafi komunikować się z pamiecią AXI, więc jego bufory musza być w SRAM4
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4_CM7")))	cBuforNadDMA[ROZMIAR_RAMKI_KOMUNIKACYJNEJ];
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4_CM7")))	cBuforOdbDMA[ROZMIAR_BUF_ODB_DMA];
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4_CM7")))	cRamkaTelemetrii[2*LICZBA_RAMEK_TELEMETR][ROZMIAR_RAMKI_KOMUNIKACYJNEJ];	//ramki telemetryczne: przygotowywana i wysyłana dla zmiennych 0..127 oraz 128..255
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".Bufory_SRAM3"))) cBuforNadRamkiKomTCP[ROZMIAR_RAMKI_KOMUNIKACYJNEJ];
uint8_t cBuforKomOdb[ROZMIAR_BUF_ANALIZY_ODB];
volatile uint16_t sWskNap; 		//wskaźnik napełniania bufora kołowego cBuforKomOdb[]
volatile uint16_t sWskOpr;		//wskaźnik opróżniania bufora kołowego cBuforKomOdb[]
//volatile uint8_t chUartKomunikacjiZajety;	//wskazuje na wysyłanie ramki z poniższej listy
//volatile uint8_t chDoWyslaniaLPUART[1 + LICZBA_RAMEK_TELEMETR];	//lista rzeczy do wysłania po zakończeniu bieżącej transmisji: ramka poleceń i ramki telemetryczne
volatile st_ZajetośćLPUART_t st_ZajetośćLPUART;
uint8_t cTimeoutOdbioru;

//deklaracje zmiennych zewnętrznych
extern uint16_t sBuforKamery[ROZM_BUF16_KAM];
extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef hdma_lpuart1_tx;
extern UART_HandleTypeDef huart7;
extern volatile uint8_t cCzasSwieceniaLED[LICZBA_LED];	//czas świecenia liczony w kwantach 0,1s jest zmniejszany w przerwaniu TIM17_IRQHandler
extern void CzytajPamiecObrazu(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t* bufor);
extern void Error_Handler(void);
extern uint8_t cRozmiarRamkiNadTCP;		//rozmiar ramki nadawczej TCP. Jest zerowany po wysłaniu i ustawioany gdy gotowy do wysyłki


////////////////////////////////////////////////////////////////////////////////
// Inicjalizacja protokołu komunikacyjnego UART
// Parametry: nrak
//Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujProtokol(void)
{
	uint8_t cPaczka[ROZMIAR_PACZKI_KONFIGU];
	uint8_t cOdczytano;

	//odczytaj z konfiguracji i ustaw własny adres sieciowy
	cOdczytano = CzytajPaczkeKonfigu(cPaczka, FKON_NAZWA_ID_BSP);
	if (cOdczytano == ROZMIAR_PACZKI_KONFIGU)
	{
		stBSP_ID.cAdres = cPaczka[2];
		for (int16_t n=0; n<DLUGOSC_NAZWY; n++)
			stBSP_ID.cNazwa[n] = cPaczka[n+3];
		for (int16_t n=0; n<4; n++)
			stBSP_ID.cAdrIP[n] = cPaczka[n+3+DLUGOSC_NAZWY];
	}

	//jeżeli adres jest nieważny to ustaw ostatni możliwy i daj specyficaną nazwę
	if ((stBSP_ID.cAdres == 0) || (stBSP_ID.cAdres == 0xFF))
	{
		stBSP_ID.cAdres = 0xFE;
		for (int16_t n=0; n<strlen(cNazwaSierotki); n++)
			stBSP_ID.cNazwa[n] = cNazwaSierotki[n];
	}

	for (int16_t n=0; n<ROZMIAR_BUF_ODB_DMA+8; n++)
		cBuforOdbDMA[n] = 0x55;	//wypełnij wzorcem do analizy
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjalizuje zmienne w wątku odbiorczym danych komunikacyjnych po LPUART1
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjalizacjaWatkuOdbiorczegoLPUART1(void)
{
	sWskNap = sWskOpr = 0;
	st_ZajetośćLPUART.cZajętyPrzez = (int8_t)LPUART_WOLNY;
	return HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, cBuforOdbDMA, ROZMIAR_BUF_ODB_DMA);
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja uruchamiana w wątku odbiorczym danych komunikacyjnych po LPUART1
// Analizuje dane z bufora odebranych komunikatów napełnianego w callbacku przychodzących danych
// Parametry: nic
// Zwraca: kod błędu informujący również o tym że nie ma nic do wysłania
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaWatkuOdbiorczegoLPUART1(void)
{
	uint8_t cBłąd = BLAD_NIC_DO_ROBOTY;  //domyślny błąd oznaczajacy że nie ma nic do wysłania
	extern uint8_t cStatusPolaczenia;

	while (sWskNap != sWskOpr)
	{
		cStatusPolaczenia |= (STAT_POL_PRZESYLA << STAT_POL_UART);		//sygnalizuj transfer danych
		cBłąd = AnalizujDaneKom(cBuforKomOdb[sWskOpr], INTERF_UART);
		if (cBłąd)
			cCzasSwieceniaLED[LED_CZER] = 5;

		sWskOpr++;
		//zapętlenie wskaźnika bufora kołowego
		if (sWskOpr >= ROZMIAR_BUF_ANALIZY_ODB)
			sWskOpr = 0;
		cTimeoutOdbioru = 5;	//x osDelay(2); [ms] w głównym wątku
		cStatusPolaczenia &= ~(STAT_POL_MASKA_OTW << STAT_POL_UART);	//sygnalizuj powrót do stanu otwartości
	}

	//po upływie timeoutu resetuj stan protokołu aby następną ramkę zaczął dekodować od nagłówka
	if (cTimeoutOdbioru)
	{
		cTimeoutOdbioru--;
		if (!cTimeoutOdbioru)
		{
			cStanProtokolu[INTERF_UART] = PR_ODBIOR_NAGL;
		}
	}
	//sprawdź czy jest właczony bit zezwolenia na przerwanie Idle, bo po wystąpienie błędów potrafi się wyłączyć co uniemożliwia odbiór
	if ((hlpuart1.Instance->CR1 & USART_CR1_IDLEIE) == 0)
	{
		InicjalizacjaWatkuOdbiorczegoLPUART1();
		cCzasSwieceniaLED[LED_CZER] = 5;
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wątek odbiorczy
// Parametry: argument* ?
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WatekOdbiorczyLPUART1(void *argument)
{
	uint32_t nCzasTele, nCzasPoprzedniTele;
	extern volatile st_ZajetośćLPUART_t st_ZajetośćLPUART;
	uint8_t cBłąd;
	uint8_t chDanychDoWysłania;
	uint8_t chCzasDrzemki;
	extern uint8_t cStatusPolaczenia;
	uint8_t cStatusUART;

	cBłąd = InicjalizacjaWatkuOdbiorczegoLPUART1();
	InicjalizacjaTelemetrii();
	nCzasPoprzedniTele = PobierzCzasT6();

	if (cBłąd)
	{
		cStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_UART);
		cStatusPolaczenia |= (STAT_POL_NIEAKTYWNY << STAT_POL_UART);	//a jeżeli nie to do stanu gotowosci
	}
	else
	while(1)
	{
		chDanychDoWysłania = 0;
		//w pierwszej kolejności obsłuż protokół komunikacyjny
		if (st_ZajetośćLPUART.cZajętyPrzez == (int8_t)LPUART_WOLNY)
		{
			cBłąd = ObslugaWatkuOdbiorczegoLPUART1();
			if (cBłąd != BLAD_NIC_DO_ROBOTY)
				osDelay(2);	//czas na obsługę ramki tylko wtedy gdy jest coś do wysłania
		}

		//w drugiej kolejności obsłuż telemetrię
		//pełna ramka na 115,2kbps wysyła się 21,7ms (46Hz), na 57,6kbps wysyła się  43,4ms (23Hz)
		nCzasTele = MinalCzas(nCzasPoprzedniTele);	//czas w mikrosekundach
		if ((nCzasTele >= KWANT_CZASU_TELEMETRII * 1000) && (st_ZajetośćLPUART.cZajętyPrzez == (int8_t)LPUART_WOLNY))
		{
			chDanychDoWysłania = ObslugaTelemetrii(INTERF_UART);
			nCzasPoprzedniTele += KWANT_CZASU_TELEMETRII * 1000;
			if (chDanychDoWysłania)
			{
				chCzasDrzemki = (chDanychDoWysłania * 92) / 1000; 	//1 bajt na 115200 bps wysyła się 86,8us, więc nie wracaj wcześniej aż sie wyśle. Po optymalizacji potrzebna jest trochę większa wartość
				if (chCzasDrzemki < 10)
					chCzasDrzemki = 10;		//kwant czasu telemetrii 100Hz to 10ms
			}
			else
				chCzasDrzemki = 10;		//kwant czasu telemetrii 100Hz to 10ms
			osDelay(chCzasDrzemki);
		}
		cStatusUART |= cStatusPolaczenia & (STAT_POL_MASKA << STAT_POL_UART);
		cStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_UART);
		if (cStatusUART == (STAT_POL_OTWARTY << STAT_POL_UART))	//jeżeli był ustawiony bit transmisji lub otwartości
			cStatusPolaczenia |= (STAT_POL_OTWARTY << STAT_POL_UART);	//to wróć do stanu otwartego łącza
		else
			cStatusPolaczenia |= (STAT_POL_GOTOWY << STAT_POL_UART);	//a jeżeli nie to do stanu gotowosci
	}
}



////////////////////////////////////////////////////////////////////////////////
// Callback przerwania UARTA pracujacego w trybie DMA z obsługą detekcji IDLE.
// Przepisuje odebrane dane z małego bufora odbiorczego DMA do większego bufora kołowego analizy protokołu
// Parametry:
// *huart - uchwyt uarta
// sOdebrano - liczba odebranych znaków
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == LPUART1)
	{
		if ((huart->RxEventType == HAL_UART_RXEVENT_TC) || (huart->RxEventType == HAL_UART_RXEVENT_IDLE))
		{
			for (uint16_t n=0; n<Size; n++)
			{
				cBuforKomOdb[sWskNap] = cBuforOdbDMA[n];
				sWskNap++;
				//zapętlenie wskaźnika bufora kołowego
				if (sWskNap >= ROZMIAR_BUF_ANALIZY_ODB)
					sWskNap = 0;
			}

	#ifdef DEBUG
			if (Size > ROZMIAR_BUF_ODB_DMA)	//czy bufor nie jest za mały
				Error_Handler();
	#endif
			//ponownie włącz odbiór
			HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, cBuforOdbDMA, ROZMIAR_BUF_ODB_DMA);	//ponieważ przerwanie przychodzi od UART_DMARxHalfCplt więc ustaw dwukrotnie większy rozmiar aby całą ramkę odebrać na przerwanu od połowy danych
		}
	}
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == LPUART1)
	{
		__disable_irq();	//sekcja krytyczna wykonywana przy wyłączonych przerwaniach
		st_ZajetośćLPUART.cZajętyPrzez = (int8_t)LPUART_WOLNY;		//skończyło się wysyłanie, wyczyść flagę zajetości
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);	//serwo kanał 1 - LPUART_WOLNY

		//port jest wolny, można wysłać rzeczy zaległe
		for (uint8_t n=0; n< ROZMIAR_KOLEJKI_LPUART; n++)
		{
			if (st_ZajetośćLPUART.sDoWysłania[n])
			{
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//serwo kanał 1 - LPUART_ZAJETY
				switch (n)
				{
				case RAMKA_POLECEN:	HAL_UART_Transmit_DMA(&hlpuart1, &cBuforNadDMA[0], st_ZajetośćLPUART.sDoWysłania[n]);		 break;
				case RAMKA_TELE1:	HAL_UART_Transmit_DMA(&hlpuart1, &cRamkaTelemetrii[0 + st_ZajetośćLPUART.cIndeksNapełnianejRamki[n]][0], st_ZajetośćLPUART.sDoWysłania[n]);		break;
				case RAMKA_TELE2:	HAL_UART_Transmit_DMA(&hlpuart1, &cRamkaTelemetrii[2 + st_ZajetośćLPUART.cIndeksNapełnianejRamki[n]][0], st_ZajetośćLPUART.sDoWysłania[n]);		break;
				}
				st_ZajetośćLPUART.cZajętyPrzez = n;
				st_ZajetośćLPUART.sDoWysłania[n] = 0;	//wysłano więc zdejmij z kolejki i zezwól na ponowne napełnienie bufora
				st_ZajetośćLPUART.cIndeksNapełnianejRamki[n]++;	//przełacz indeks podwójnego buforowania aby można było napełniać drugi bufor
				st_ZajetośćLPUART.cIndeksNapełnianejRamki[n] &= 0x01;
				break;	//zakończ wykonanie petli for po wysłaniu pierwszej transmisji
			}
		}
		__enable_irq();		//koniec sekcji krytycznej
	}
}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == LPUART1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, cBuforOdbDMA, ROZMIAR_BUF_ODB_DMA);
		//HAL_UART_Receive_DMA(&hlpuart1, cBuforOdbDMA, ILOSC_ODBIORU_DMA);
	}
	cCzasSwieceniaLED[LED_CZER] = 5;
}


////////////////////////////////////////////////////////////////////////////////
// Analizuje dane przychodzące z interfejsów komunikacyjnych w trybie: pytanie - odpowiedź
// Parametry:
// chWe - odbierany bajt
// chInterfejs - identyfikator interfejsu odbierająceg znak
//Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AnalizujDaneKom(uint8_t cWe, uint8_t cInterfejs)
{
    uint8_t cBłąd;

    cBłąd = DekodujRamke(cWe, &cAdresZdalny[cInterfejs], &cZnakCzasu[cInterfejs], &cPolecenie, &cRozmDanych, cDaneRamkiKom, cInterfejs);
    if (cBłąd == BLAD_GOTOWE)
    	cBłąd = UruchomPolecenie(cPolecenie, cDaneRamkiKom, cRozmDanych, cInterfejs, cAdresZdalny[cInterfejs]);

    return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Dekoduje ramki przychodzące z interfejsu komunikacyjnego
// Parametry:
// chWe - odbierany bajt
// *chAdresNad - adres urządzenia nadajacego ramkę
// *cPolecenie - numer polecenia
// *cZnakCzasu - znacznik czasu ramki
// *chNrRamki - numer kolejny ramki
// *chData - wskaźnik na dane do polecenia
// *chDataSize - ilość danych do polecenia
//Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t DekodujRamke(uint8_t cWe, uint8_t *chAdrZdalny, uint8_t *cZnakCzasu, uint8_t *cPolecenie, uint8_t *cRozmDanych, uint8_t *cDane, uint8_t cInterfejs)
{
	uint8_t n, cBłąd = BLAD_OK;
	uint16_t sCrc16Obl;

    switch (cStanProtokolu[cInterfejs])
    {
    case PR_ODBIOR_NAGL:	//testuj czy odebrano nagłówek
		if (cWe == NAGLOWEK)
			cStanProtokolu[cInterfejs] = PR_ADRES_ODB;
		else
		{
			cBłąd = BLAD_ZLY_NAGL;
			printf("n");
		}
		break;

    case PR_ADRES_ODB:
    	if ((cWe == stBSP_ID.cAdres) || (cWe == ADRES_BROADCAST))				//czy odebraliśmy własny adres sieciowy lub adres rozgłoszeniowy
    	{
    		cAdresPrzychodzacy = cWe;	//zachowaj do liczenia CRC
    		cStanProtokolu[cInterfejs] = PR_ADRES_NAD;
    	}
    	else
    		cStanProtokolu[cInterfejs] = PR_ODBIOR_NAGL;
    	break;

    case PR_ADRES_NAD:			//adres sieciowy strony zdalnej
    	*chAdrZdalny = cWe;
    	cStanProtokolu[cInterfejs] = PR_ZNAK_CZASU;
    	break;

    case PR_ZNAK_CZASU:   //odbierz znacznik czasu
        *cZnakCzasu = cWe;
        cStanProtokolu[cInterfejs] = PR_POLECENIE;
        break;

    case PR_POLECENIE:
    	*cPolecenie = cWe;
    	cStanProtokolu[cInterfejs] = PR_ROZM_DANYCH;
    	printf("P:%d,", cWe);
    	break;

    case PR_ROZM_DANYCH:	//odebrano rozmiar danych
    	*cRozmDanych = cWe;
    	cLicznikDanych[cInterfejs] = 0;
    	if (*cRozmDanych > 0)
    		cStanProtokolu[cInterfejs] = PR_DANE;
    	else
    		cStanProtokolu[cInterfejs] = PR_CRC16_1;
    	break;

    case PR_DANE:
    	*(cDane + cLicznikDanych[cInterfejs]) = cWe;
    	cLicznikDanych[cInterfejs]++;
    	if (cLicznikDanych[cInterfejs] == *cRozmDanych)
    	{
    		cStanProtokolu[cInterfejs] = PR_CRC16_1;
    	}
    	break;

    case PR_CRC16_1:
    	sCrc16We = cWe;	//młodszy przodem
    	cStanProtokolu[cInterfejs] = PR_CRC16_2;
    	break;

    case PR_CRC16_2:
    	sCrc16We += cWe * 0x100;
		cStanProtokolu[cInterfejs] = PR_ODBIOR_NAGL;
		//dodać blokadę zasobu CRC
		InicjujCRC16(0, WIELOMIAN_CRC);
		*((volatile uint8_t *)&CRC->DR) = cAdresPrzychodzacy;
		*((volatile uint8_t *)&CRC->DR) = *chAdrZdalny;
		*((volatile uint8_t *)&CRC->DR) = *cZnakCzasu;
		*((volatile uint8_t *)&CRC->DR) = *cPolecenie;
		*((volatile uint8_t *)&CRC->DR) = *cRozmDanych;
		for (n=0; n<*cRozmDanych; n++)
			*((volatile uint8_t *)&CRC->DR) = *(cDane + n);
		sCrc16Obl = (uint16_t)CRC->DR;
		//zdjąć blokadę zasobu CRC

		if (sCrc16We == sCrc16Obl)
		{
			cBłąd = BLAD_GOTOWE;
			printf("c,");
		}
		else
			cBłąd = BLAD_CRC;
		break;

    default:
    	cStanProtokolu[cInterfejs] = PR_ODBIOR_NAGL;
    	cBłąd = BLAD_ZLY_STAN_PROT;
    	break;
    }

    return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjuje sprzętowy mechanizm liczenia CRC16
// Parametry:
// sInit - wartość inicjująca lizzenie
// sWielomian - wielomian CRC
//Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjujCRC16(uint16_t sInit, uint16_t sWielomian)
{
	CRC->INIT = sInit;
	CRC->POL = sWielomian;
	CRC->CR = CRC_CR_RESET | CRC_CR_POLYSIZE_0;
}



////////////////////////////////////////////////////////////////////////////////
// Liczy sprzętowo CRC16
// Parametry:
// dane - dane wchodzące do odczytu
//Zwraca: obliczone CRC
////////////////////////////////////////////////////////////////////////////////
uint16_t LiczCRC16(uint8_t cDane)
{
	CRC->DR = cDane;
	return (uint16_t)CRC->DR;
}


////////////////////////////////////////////////////////////////////////////////
// Formatuje ramkę, ustawia nagłówek, liczy sumę kontrolną
// Parametry:
// cPolecenie - numer polecenia
// cZnakCzasu - znacznik czasu
// *chDane - wskaźnik na dane do polecenia
// chDlugosc - ilość danych do polecenia
// *chRamka - wskaźnik na ramkę do wysłania
//Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PrzygotujRamke(uint8_t cAdrZdalny, uint8_t cAdrLokalny,  uint8_t cZnakCzasu, uint8_t cPolecenie, uint8_t cRozmDanych, uint8_t *cDane, uint8_t *cRamka)
{
    if (cRozmDanych > ROZMIAR_DANYCH_KOMUNIKACJI)
    	return(BLAD_ZLA_ILOSC_DANYCH);

    if ((cPolecenie & ~0x80) > PK_ILOSC_POLECEN)
    	return(BLAD_ZLE_POLECENIE);

    *(cRamka + 0) = NAGLOWEK;
    *(cRamka + 1) = cAdrZdalny;		//ADRES ODBIORCY
    *(cRamka + 2) = cAdrLokalny;	//ADERS NADAWCY
    *(cRamka + 3) = cZnakCzasu;
    *(cRamka + 4) = cPolecenie;
    *(cRamka + 5) = cRozmDanych;

    //dodać blokadę zasobu CRC
    InicjujCRC16(0, WIELOMIAN_CRC);
    CRC->DR = cAdrZdalny;
	CRC->DR = cAdrLokalny;
	CRC->DR = cZnakCzasu;
	CRC->DR = cPolecenie;
	CRC->DR = cRozmDanych;

    for (uint8_t n=0; n<cRozmDanych; n++)
    	*(cRamka + 6 + n) = CRC->DR =  *(cDane + n);

    un8_32.dane16[0] = (uint16_t)CRC->DR;
    //zdjąć blokadę zasobu CRC

    *(cRamka + cRozmDanych + 6) = un8_32.dane8[0];	//młodszy
    *(cRamka + cRozmDanych + 7) = un8_32.dane8[1];	//starszy
    return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła ramkę przez interfejs komunikacyjny
// W przypadku ramek telemetrycznych znacznik czasu jest 8-bitowym, 100Hz licznikiem czasu
// w przypadku ramek będących odpowiedzią na pytanie odsyłany jest ten sam znacznik czasu który był
// w ramce z pytaniem. W ten sposób jednoznacznie identyfikujemy ramkę odpowiedzi
// Parametry:
// *chFrame - wskaźnik na ramkę do wysłania
// chLen - ilość danych do polecenia
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t WyslijRamke(uint8_t cAdrZdalny, uint8_t cPolecenie, uint8_t cRozmDanych, uint8_t *cDane, uint8_t cInterfejs)
{
	uint8_t cBłąd = BLAD_OK;

	switch (cInterfejs)
	{
	case INTERF_UART:
		cBłąd = PrzygotujRamke(cAdrZdalny, stBSP_ID.cAdres, cZnakCzasu[cInterfejs], cPolecenie, cRozmDanych, cDane, cBuforNadDMA);
		if (cBłąd == BLAD_OK)
		{
			if (cBuforNadDMA[5] == 0xFF)	//rozmiar ???
				break;
			st_ZajetośćLPUART.cZajętyPrzez = RAMKA_POLECEN;
			st_ZajetośćLPUART.sDoWysłania[RAMKA_POLECEN] = (uint16_t)cRozmDanych + ROZM_CIALA_RAMKI;
    		HAL_UART_Transmit_DMA(&hlpuart1, cBuforNadDMA, st_ZajetośćLPUART.sDoWysłania[RAMKA_POLECEN]);
    		st_ZajetośćLPUART.sDoWysłania[RAMKA_POLECEN]  = 0;
    		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//serwo kanał 1 - LPUART_ZAJETY
		}
		break;

	case INTERF_ETH:
		cBłąd = PrzygotujRamke(cAdrZdalny, stBSP_ID.cAdres, cZnakCzasu[cInterfejs], cPolecenie, cRozmDanych, cDane, cBuforNadRamkiKomTCP);
		if (cBłąd == BLAD_OK)
		{
			cRozmiarRamkiNadTCP = cRozmDanych + ROZM_CIALA_RAMKI;		//ustaw rozmiar ramki nadawczej TCP gotowej do wysyłki
		}
    	break;

    case INTERF_USB:	break;
    default: cBłąd = BLAD_ZLY_INTERFEJS;	break;
    }
    return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła ramkę komunikacyjną z kodem błędu
// Parametry:
// [i] chKodBledu - kod błędu
// [i] chParametr  - parametr dodatkowy
// [i] cInterfejs - interfejs komunikacyjny przez który ma być przesłana ramka
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t Wyslij_KodBledu(uint8_t cKodBledu, uint8_t cParametr, uint8_t cInterfejs)
{
	uint8_t cDane[2];
    cDane[0] = cKodBledu;
    cDane[1] = cParametr;

    return WyslijRamke(cAdresZdalny[cInterfejs], PK_KOD_BLEDU, 2, cDane, cInterfejs);
}


uint8_t TestKomunikacjiSTD(void)
{
	uint8_t cBłąd = 0;
	uint16_t sRozmDanych;

	sRozmDanych = sprintf((char*)cBuforNadDMA, "Test komunikacji UART: blokująca\n\r");
	if (sRozmDanych)
	{
		cBłąd = HAL_UART_Transmit(&hlpuart1,  cBuforNadDMA, sRozmDanych, 100);
		//chUartKomunikacjiZajety = 1;
	}
	return cBłąd;
}



uint8_t TestKomunikacjiDMA(void)
{
	uint8_t cBłąd = 0;
	uint16_t sRozmDanych;

	sRozmDanych = sprintf((char*)cBuforNadDMA, "Test komunikacji UART: DMA\n\r");
	if (sRozmDanych)
	{
		cBłąd = HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)cBuforNadDMA, sRozmDanych);
		//chUartKomunikacjiZajety = 1;
	}
	return cBłąd;
}


