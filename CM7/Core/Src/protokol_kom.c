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
 * ADERS NADAWCY - aby można było zidentyfikować od kogo pochodzi ramka
 * ZNACZNIK CZASU - licznik setnych części sekundy po to aby można było poskładać we właściwej kolejności dane przesyłane w wielu ramkach
 * POLECENIE - kod polecenia do wykonania.
 * ROZMIAR - liczba bajtów danych ramki
 * DANE - opcjonalne dane
 * CRC16 - suma kontrolna ramki od nagłówka do CRC16. młodszy przodem */

#include "cmsis_os.h"
#include "protokol_kom.h"
#include <stdio.h>
#include "wymiana_CM7.h"
#include "telemetria.h"
#include "komunikacja.h"
#include "flash_konfig.h"
#include <string.h>
#include "czas.h"

//definicje zmiennych
static uint8_t chStanProtokolu[ILOSC_INTERF_KOM];
uint8_t chAdresZdalny[ILOSC_INTERF_KOM];	//adres sieciowy strony zdalnej
static uint8_t chAdresPrzychodzacy;	//może to być adres BSP lub broadcast
static uint8_t chLicznikDanych[ILOSC_INTERF_KOM];
static uint8_t chZnakCzasu[ILOSC_INTERF_KOM];
static uint16_t sCrc16We;
//volatile uint32_t nCzasSystemowy;
static uint8_t chPolecenie;
static uint8_t chRozmDanych;
static uint8_t chDaneRamkiKom[ROZMIAR_DANYCH_KOMUNIKACJI];
static un8_16_t un8_16;
stBSP_t stBSP;	//struktura zawierajaca adresy i nazwę BSP
const char* chNazwaSierotki = {"Sierotka Wronia"};

//ponieważ BDMA nie potrafi komunikować się z pamiecią AXI, więc jego bufory musza być w SRAM4
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4")))	chBuforNadDMA[ROZMIAR_RAMKI_KOMUNIKACYJNEJ];
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4")))	chBuforOdbDMA[ROZMIAR_BUF_ODB_DMA+8];
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4")))	chRamkaTelemetrii[2*LICZBA_RAMEK_TELEMETR][ROZMIAR_RAMKI_KOMUNIKACYJNEJ];	//ramki telemetryczne: przygotowywana i wysyłana dla zmiennych 0..127 oraz 128..255
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".Bufory_SRAM3"))) chBuforNadRamkiKomTCP[ROZMIAR_RAMKI_KOMUNIKACYJNEJ];
extern uint8_t chIndeksNapelnRamki;	//okresla ktora tablica ramki telemetrycznej jest napełniania
uint8_t chWyslaneOK = 1;
uint8_t chBuforKomOdb[ROZMIAR_BUF_ANALIZY_ODB];
volatile uint16_t sWskNap; 		//wskaźnik napełniania bufora kołowego chBuforKomOdb[]
volatile uint16_t sWskOpr;		//wskaźnik opróżniania bufora kołowego chBuforKomOdb[]
//volatile uint8_t chUartKomunikacjiZajety;	//wskazuje na wysyłanie ramki z poniższej listy
//volatile uint8_t chDoWyslaniaLPUART[1 + LICZBA_RAMEK_TELEMETR];	//lista rzeczy do wysłania po zakończeniu bieżącej transmisji: ramka poleceń i ramki telemetryczne
volatile st_ZajetoscLPUART_t st_ZajetoscLPUART;
uint8_t chTimeoutOdbioru;

//deklaracje zmiennych zewnętrznych
extern uint16_t sBuforKamery[ROZM_BUF16_KAM];
extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef hdma_lpuart1_tx;
extern UART_HandleTypeDef huart7;
extern volatile uint8_t chCzasSwieceniaLED[LICZBA_LED];	//czas świecenia liczony w kwantach 0,1s jest zmniejszany w przerwaniu TIM17_IRQHandler
extern void CzytajPamiecObrazu(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t* bufor);
extern void Error_Handler(void);
extern uint8_t chRozmiarRamkiNadTCP;		//rozmiar ramki nadawczej TCP. Jest zerowany po wysłaniu i ustawioany gdy gotowy do wysyłki



////////////////////////////////////////////////////////////////////////////////
// Odbiera dane przychodzące z interfejsów kmunikacyjnych w trybie: pytanie - odpowiedź
// Parametry:
// chIn - odbierany bajt
// chInterfejs - identyfikator interfejsu odbierająceg znak
//Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujProtokol(void)
{
	uint8_t chPaczka[ROZMIAR_PACZKI_KONFIG];
	uint8_t chOdczytano;

	//odczytaj z konfiguracji i ustaw własny adres sieciowy
	chOdczytano = CzytajPaczkeKonfigu(chPaczka, FKON_NAZWA_ID_BSP);
	if (chOdczytano == ROZMIAR_PACZKI_KONFIG)
	{
		stBSP.chAdres = chPaczka[2];
		for (int16_t n=0; n<DLUGOSC_NAZWY; n++)
			stBSP.chNazwa[n] = chPaczka[n+3];
		for (int16_t n=0; n<4; n++)
			stBSP.chAdrIP[n] = chPaczka[n+3+DLUGOSC_NAZWY];
	}

	//jeżeli adres jest nieważny to ustaw ostatni możliwy i daj specyficaną nazwę
	if ((stBSP.chAdres == 0) || (stBSP.chAdres == 0xFF))
	{
		stBSP.chAdres = 0xFE;
		for (int16_t n=0; n<strlen(chNazwaSierotki); n++)
			stBSP.chNazwa[n] = chNazwaSierotki[n];
	}

	for (int16_t n=0; n<ROZMIAR_BUF_ODB_DMA+8; n++)
		chBuforOdbDMA[n] = 0x55;	//wypełnij wzorcem do analizy
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
	st_ZajetoscLPUART.chZajetyPrzez = LPUART_WOLNY;
	return HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, chBuforOdbDMA, ROZMIAR_BUF_ODB_DMA);
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja uruchamiana w wątku odbiorczym danych komunikacyjnych po LPUART1
// Analizuje dane z bufora odebranych komunikatów napełnianego w callbacku przychodzących danych
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObslugaWatkuOdbiorczegoLPUART1(void)
{
	uint8_t chErr;
	extern uint8_t chStatusPolaczenia;

	while (sWskNap != sWskOpr)
	{
		chStatusPolaczenia |= (STAT_POL_PRZESYLA << STAT_POL_UART);		//sygnalizuj transfer danych
		chErr = AnalizujDaneKom(chBuforKomOdb[sWskOpr], INTERF_UART);
		if (chErr)
			chCzasSwieceniaLED[LED_CZER] = 5;

		sWskOpr++;
		//zapętlenie wskaźnika bufora kołowego
		if (sWskOpr >= ROZMIAR_BUF_ANALIZY_ODB)
			sWskOpr = 0;
		chTimeoutOdbioru = 5;	//x osDelay(2); [ms] w głównym wątku
		chStatusPolaczenia &= ~(STAT_POL_MASKA_OTW << STAT_POL_UART);	//sygnalizuj powrót do stanu otwartości
	}

	//po upływie timeoutu resetuj stan protokołu aby następną ramkę zaczął dekodować od nagłówka
	if (chTimeoutOdbioru)
	{
		chTimeoutOdbioru--;
		if (!chTimeoutOdbioru)
		{
			chStanProtokolu[INTERF_UART] = PR_ODBIOR_NAGL;
		}
	}
	//sprawdź czy jest właczony bit zezwolenia na przerwanie Idle, bo po wystąpienie błędów potrafi się wyłączyć co uniemożliwia odbiór
	if ((hlpuart1.Instance->CR1 & USART_CR1_IDLEIE) == 0)
	{
		InicjalizacjaWatkuOdbiorczegoLPUART1();
		chCzasSwieceniaLED[LED_CZER] = 5;
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
				chBuforKomOdb[sWskNap] = chBuforOdbDMA[n];
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
			HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, chBuforOdbDMA, ROZMIAR_BUF_ODB_DMA);	//ponieważ przerwanie przychodzi od UART_DMARxHalfCplt więc ustaw dwukrotnie większy rozmiar aby całą ramkę odebrać na przerwanu od połowy danych
		}
	}
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == LPUART1)
	{
		//skończyło się wysyłanie, wyczyść flagę zajetości i liczbę rzeczy do wysłania
		if (st_ZajetoscLPUART.chZajetyPrzez != LPUART_WOLNY)
			st_ZajetoscLPUART.chZajetyPrzez = LPUART_WOLNY;

		//port jest wolny, można wysłać rzeczy zaległe
		for (uint8_t n=0; n< ROZMIAR_KOLEJKI_LPUART; n++)
		{
			if (st_ZajetoscLPUART.sDoWyslania[n])
			{
				st_ZajetoscLPUART.chZajetyPrzez = n;
				switch (n)
				{
				case RAMKA_POLECEN:
					HAL_UART_Transmit_DMA(&hlpuart1, chBuforNadDMA, st_ZajetoscLPUART.sDoWyslania[n]);
					break;

				case RAMKA_TELE1:	//ramkę telemetryczną 1
					HAL_UART_Transmit_DMA(&hlpuart1, &chRamkaTelemetrii[0 + chIndeksNapelnRamki][0], st_ZajetoscLPUART.sDoWyslania[n]);
					st_ZajetoscLPUART.sDoWyslania[n] = 0;	//nie ma nic wiecej do wysłania
					break;

				case RAMKA_TELE2:	//ramkę telemetryczną 2
					HAL_UART_Transmit_DMA(&hlpuart1, &chRamkaTelemetrii[2 + chIndeksNapelnRamki][0], st_ZajetoscLPUART.sDoWyslania[n]);
					st_ZajetoscLPUART.sDoWyslania[n] = 0;	//nie ma nic wiecej do wysłania
					break;

				default:
					assert(1);	//tutaj program nigdy nie powinien wskoczyć
					break;
				}
				break;	//zakończ wykonanie petli for po wysłaniu pierwszej transmisji
			}
		}
	}
}


/*
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == LPUART1);
}





void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == LPUART1)
	{
		for (uint16_t n=0; n<ILOSC_ODBIORU_DMA; n++)
		{
			chBuforKomOdb[sWskNap] = chBuforOdbDMA[n];
			sWskNap++;
			//zapętlenie wskaźnika bufora kołowego
			if (sWskNap >= ROZMIAR_BUF_ANALIZY_ODB)
				sWskNap = 0;
		}
		//ponownie włącz odbiór
		HAL_UART_Receive_DMA(&hlpuart1, chBuforOdbDMA, ILOSC_ODBIORU_DMA);	//ponieważ przerwanie przychodzi od UART_DMARxHalfCplt więc ustaw dwukrotnie większy rozmiar aby całą ramkę odebrać na przerwanu od połowy danych
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == LPUART1)
	{
		for (uint16_t n=0; n<ILOSC_ODBIORU_DMA; n++)
		{
			chBuforKomOdb[sWskNap] = chBuforOdbDMA[n];
			sWskNap++;
			//zapętlenie wskaźnika bufora kołowego
			if (sWskNap >= ROZMIAR_BUF_ANALIZY_ODB)
				sWskNap = 0;
		}
		//ponownie włącz odbiór
		HAL_UART_Receive_DMA(&hlpuart1, chBuforOdbDMA, ILOSC_ODBIORU_DMA);	//ponieważ przerwanie przychodzi od UART_DMARxHalfCplt więc ustaw dwukrotnie większy rozmiar aby całą ramkę odebrać na przerwanu od połowy danych
	}
}

*/

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == LPUART1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, chBuforOdbDMA, ROZMIAR_BUF_ODB_DMA);	//ponieważ przerwanie przychodzi od UART_DMARxHalfCplt więc ustaw dwukrotnie większy rozmiar aby całą ramkę odebrać na przerwanu od połowy danych
		//HAL_UART_Receive_DMA(&hlpuart1, chBuforOdbDMA, ILOSC_ODBIORU_DMA);
	}
	chCzasSwieceniaLED[LED_CZER] = 5;
}


////////////////////////////////////////////////////////////////////////////////
// Analizuje dane przychodzące z interfejsów komunikacyjnych w trybie: pytanie - odpowiedź
// Parametry:
// chWe - odbierany bajt
// chInterfejs - identyfikator interfejsu odbierająceg znak
//Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AnalizujDaneKom(uint8_t chWe, uint8_t chInterfejs)
{
    uint8_t chErr;

    chErr = DekodujRamke(chWe, &chAdresZdalny[chInterfejs], &chZnakCzasu[chInterfejs], &chPolecenie, &chRozmDanych, chDaneRamkiKom, chInterfejs);
    if (chErr == ERR_RAMKA_GOTOWA)
    	chErr = UruchomPolecenie(chPolecenie, chDaneRamkiKom, chRozmDanych, chInterfejs, chAdresZdalny[chInterfejs]);

    return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Dekoduje ramki przychodzące z interfejsu komunikacyjnego
// Parametry:
// chWe - odbierany bajt
// *chAdresNad - adres urządzenia nadajacego ramkę
// *chPolecenie - numer polecenia
// *chZnakCzasu - znacznik czasu ramki
// *chNrRamki - numer kolejny ramki
// *chData - wskaźnik na dane do polecenia
// *chDataSize - ilość danych do polecenia
//Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t DekodujRamke(uint8_t chWe, uint8_t *chAdrZdalny, uint8_t *chZnakCzasu, uint8_t *chPolecenie, uint8_t *chRozmDanych, uint8_t *chDane, uint8_t chInterfejs)
{
	uint8_t n, chErr = BLAD_OK;
	uint16_t sCrc16Obl;

    switch (chStanProtokolu[chInterfejs])
    {
    case PR_ODBIOR_NAGL:	//testuj czy odebrano nagłówek
		if (chWe == NAGLOWEK)
			chStanProtokolu[chInterfejs] = PR_ADRES_ODB;
		else
		{
			chErr = ERR_ZLY_NAGL;
			printf("n");
		}
		break;

    case PR_ADRES_ODB:
    	if ((chWe == stBSP.chAdres) || (chWe == ADRES_BROADCAST))				//czy odebraliśmy własny adres sieciowy lub adres rozgłoszeniowy
    	{
    		chAdresPrzychodzacy = chWe;	//zachowaj do liczenia CRC
    		chStanProtokolu[chInterfejs] = PR_ADRES_NAD;
    	}
    	else
    		chStanProtokolu[chInterfejs] = PR_ODBIOR_NAGL;
    	break;

    case PR_ADRES_NAD:			//adres sieciowy strony zdalnej
    	*chAdrZdalny = chWe;
    	chStanProtokolu[chInterfejs] = PR_ZNAK_CZASU;
    	break;

    case PR_ZNAK_CZASU:   //odbierz znacznik czasu
        *chZnakCzasu = chWe;
        chStanProtokolu[chInterfejs] = PR_POLECENIE;
        break;

    case PR_POLECENIE:
    	*chPolecenie = chWe;
    	chStanProtokolu[chInterfejs] = PR_ROZM_DANYCH;
    	printf("P:%d,", chWe);
    	break;

    case PR_ROZM_DANYCH:	//odebrano rozmiar danych
    	*chRozmDanych = chWe;
    	chLicznikDanych[chInterfejs] = 0;
    	if (*chRozmDanych > 0)
    		chStanProtokolu[chInterfejs] = PR_DANE;
    	else
    		chStanProtokolu[chInterfejs] = PR_CRC16_1;
    	break;

    case PR_DANE:
    	*(chDane + chLicznikDanych[chInterfejs]) = chWe;
    	chLicznikDanych[chInterfejs]++;
    	if (chLicznikDanych[chInterfejs] == *chRozmDanych)
    	{
    		chStanProtokolu[chInterfejs] = PR_CRC16_1;
    	}
    	break;

    case PR_CRC16_1:
    	sCrc16We = chWe;	//młodszy przodem
    	chStanProtokolu[chInterfejs] = PR_CRC16_2;
    	break;

    case PR_CRC16_2:
    	sCrc16We += chWe * 0x100;
		chStanProtokolu[chInterfejs] = PR_ODBIOR_NAGL;
		//dodać blokadę zasobu CRC
		InicjujCRC16(0, WIELOMIAN_CRC);
		*((volatile uint8_t *)&CRC->DR) = chAdresPrzychodzacy;
		*((volatile uint8_t *)&CRC->DR) = *chAdrZdalny;
		*((volatile uint8_t *)&CRC->DR) = *chZnakCzasu;
		*((volatile uint8_t *)&CRC->DR) = *chPolecenie;
		*((volatile uint8_t *)&CRC->DR) = *chRozmDanych;
		for (n=0; n<*chRozmDanych; n++)
			*((volatile uint8_t *)&CRC->DR) = *(chDane + n);
		sCrc16Obl = (uint16_t)CRC->DR;
		//zdjąć blokadę zasobu CRC

		if (sCrc16We == sCrc16Obl)
		{
			chErr = ERR_RAMKA_GOTOWA;
			printf("c,");
		}
		else
			chErr = ERR_CRC;
		break;

    default:
    	chStanProtokolu[chInterfejs] = PR_ODBIOR_NAGL;
    	chErr = ERR_ZLY_STAN_PROT;
    	break;
    }

    return chErr;
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
uint16_t LiczCRC16(uint8_t chDane)
{
	CRC->DR = chDane;
	return (uint16_t)CRC->DR;
}


////////////////////////////////////////////////////////////////////////////////
// Formatuje ramkę, ustawia nagłówek, liczy sumę kontrolną
// Parametry:
// chPolecenie - numer polecenia
// chZnakCzasu - znacznik czasu
// *chDane - wskaźnik na dane do polecenia
// chDlugosc - ilość danych do polecenia
// *chRamka - wskaźnik na ramkę do wysłania
//Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PrzygotujRamke(uint8_t chAdrZdalny, uint8_t chAdrLokalny,  uint8_t chZnakCzasu, uint8_t chPolecenie, uint8_t chRozmDanych, uint8_t *chDane, uint8_t *chRamka)
{
    if (chRozmDanych > ROZMIAR_DANYCH_KOMUNIKACJI)
    	return(ERR_ZLA_ILOSC_DANYCH);

    if ((chPolecenie & ~0x80) > PK_ILOSC_POLECEN)
    	return(ERR_ZLE_POLECENIE);

    *(chRamka++) = NAGLOWEK;
    *(chRamka++) = chAdrZdalny;		//ADRES ODBIORCY
    *(chRamka++) = chAdrLokalny;	//ADERS NADAWCY
    *(chRamka++) = chZnakCzasu;
    *(chRamka++) = chPolecenie;
    *(chRamka++) = chRozmDanych;

    //dodać blokadę zasobu CRC
    InicjujCRC16(0, WIELOMIAN_CRC);
    CRC->DR = chAdrZdalny;
	CRC->DR = chAdrLokalny;
	CRC->DR = chZnakCzasu;
	CRC->DR = chPolecenie;
	CRC->DR = chRozmDanych;

    for (uint8_t n=0; n<chRozmDanych; n++)
    	*(chRamka++) = CRC->DR =  *(chDane + n);

    un8_16.dane16 = (uint16_t)CRC->DR;
    //zdjąć blokadę zasobu CRC

    *(chRamka++) = un8_16.dane8[0];	//młodszy
    *(chRamka++) = un8_16.dane8[1];	//starszy

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
uint8_t WyslijRamke(uint8_t chAdrZdalny, uint8_t chPolecenie, uint8_t chRozmDanych, uint8_t *chDane, uint8_t chInterfejs)
{
	uint8_t chErr = BLAD_OK;

	switch (chInterfejs)
	{
	case INTERF_UART:
		chErr = PrzygotujRamke(chAdrZdalny, stBSP.chAdres, chZnakCzasu[chInterfejs], chPolecenie, chRozmDanych, chDane, chBuforNadDMA);
		if (chErr == BLAD_OK)
		{
    		st_ZajetoscLPUART.chZajetyPrzez = RAMKA_POLECEN;
    		st_ZajetoscLPUART.sDoWyslania[RAMKA_POLECEN - 1] = (uint16_t)chRozmDanych + ROZM_CIALA_RAMKI;
    		HAL_UART_Transmit_DMA(&hlpuart1, chBuforNadDMA, (uint16_t)chRozmDanych + ROZM_CIALA_RAMKI);
		}
		break;

	case INTERF_ETH:
		chErr = PrzygotujRamke(chAdrZdalny, stBSP.chAdres, chZnakCzasu[chInterfejs], chPolecenie, chRozmDanych, chDane, chBuforNadRamkiKomTCP);
		if (chErr == BLAD_OK)
		{
			chRozmiarRamkiNadTCP = chRozmDanych + ROZM_CIALA_RAMKI;		//ustaw rozmiar ramki nadawczej TCP gotowej do wysyłki
		}
    	break;

    case INTERF_USB:	break;
    default: chErr = ERR_ZLY_INTERFEJS;	break;
    }
    return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła ramkę komunikacyjną z kodem OK
// Parametry:
// [i] chInterfejs - interfejs komunikacyjny przez który ma być przesłana ramka
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t Wyslij_OK(uint8_t chParametr1, uint8_t chParametr2, uint8_t chInterfejs)
{
	uint8_t chDane[2];
    chDane[0] = chParametr1;
    chDane[1] = chParametr2;

    return WyslijRamke(chAdresZdalny[chInterfejs], PK_OK, 2, chDane, chInterfejs);
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła ramkę komunikacyjną z kodem błędu
// Parametry:
// [i] chKodBledu - kod błędu
// [i] chParametr  - parametr dodatkowy
// [i] chInterfejs - interfejs komunikacyjny przez który ma być przesłana ramka
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t Wyslij_ERR(uint8_t chKodBledu, uint8_t chParametr, uint8_t chInterfejs)
{
	uint8_t chDane[2];
    chDane[0] = chKodBledu;
    chDane[1] = chParametr;

    return WyslijRamke(chAdresZdalny[chInterfejs], PK_BLAD, 2, chDane, chInterfejs);
}


uint8_t TestKomunikacjiSTD(void)
{
	uint8_t chErr = 0;
	uint16_t sRozmDanych;
	/*extern char chBuforNapisowCM4[ROZMIAR_BUF_NAPISOW_CM4];
	extern uint8_t chWskNapBufNapisowCM4;
	extern uint8_t chWskOprBufNapisowCM4;

	//jeżeli z rdzenia CM4 przyszły jakieś napisy dla konsoli to wyświetl je
	for (uint8_t n=0; n<ROZMIAR_BUF_NAD_DMA; n++)
	{
		sRozmDanych = n;
		if (chWskNapBufNapisowCM4 != chWskOprBufNapisowCM4)
		{
			chBuforNadDMA[n] = chBuforNapisowCM4[chWskOprBufNapisowCM4];
			chWskOprBufNapisowCM4++;
			if (chWskOprBufNapisowCM4 == ROZMIAR_BUF_NAPISOW_CM4)
				chWskOprBufNapisowCM4 = 0;
		}
		else
			break;
	}*/

	sRozmDanych = sprintf((char*)chBuforNadDMA, "Test komunikacji UART: blokująca\n\r");
	if (sRozmDanych)
	{
		chErr = HAL_UART_Transmit(&hlpuart1,  chBuforNadDMA, sRozmDanych, 100);
		//chUartKomunikacjiZajety = 1;
	}
	return chErr;
}



uint8_t TestKomunikacjiDMA(void)
{
	uint8_t chErr = 0;
	uint16_t sRozmDanych;

	sRozmDanych = sprintf((char*)chBuforNadDMA, "Test komunikacji UART: DMA\n\r");
	if (sRozmDanych)
	{
		chErr = HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)chBuforNadDMA, sRozmDanych);
		//chUartKomunikacjiZajety = 1;
	}
	return chErr;
}


