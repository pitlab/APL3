//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł protokołu komunikacyjnego
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

/* Struktura ramki komunikacyjnej
 * 0xAA - Nagłówek.Strumień danych wejściowych analizujemy pod kątem obecności nagłówka.
 * ADRES ODBIORCY - 0x00 stacja naziemna, 0xFF broadcast, pozostałe to numery BSP w roju.Podczas odbioru w tym miejscu przestajemy analizować jeżeli ramka nas nie dotyczy.Od tego miejsca może zaczynać się szyfrowanie
 * ADERS NADAWCY - aby można było zidentyfikować od kogo pochodzi ramka
 * ZNACZNIK CZASU - licznik setnych części sekundy po to aby można było poskładać we właściwej kolejności dane przesyłane w wielu ramkach
 * POLECENIE - kod polecenia do wykonania.
 * ROZMIAR - liczba bajtów danych ramki
 * DANE - opcjonalne dane
 * CRC16 - suma kontrolna ramki od nagłówka do CRC16. Starszy przodem */

#include "cmsis_os.h"
#include "protokol_kom.h"
#include "kamera.h"
#include <stdio.h>
#include "wymiana_CM7.h"
#include "flash_nor.h"

union _un8_16		//unia do konwersji między danymi 16 i 8 bit
{
	uint16_t dane16;
	uint8_t dane8[2];
} un8_16;

union _un8_32		//unia do konwersji między danymi 32 i 8 bit
{
	uint32_t dane32;
	uint8_t dane8[4];
} un8_32;

//definicje zmiennych
uint8_t chStanProtokolu[ILOSC_INTERF_KOM];
uint8_t chAdresZdalny[ILOSC_INTERF_KOM];	//adres sieciowy strony zdalnej
uint8_t chAdresLokalny;						//własny adres sieciowy
uint8_t chLicznikDanych[ILOSC_INTERF_KOM];
uint8_t chZnakCzasu[ILOSC_INTERF_KOM];
uint16_t sCrc16We;
uint8_t chRamkaWyj[ROZMIAR_RAMKI_UART];
volatile uint8_t chWskNap, chWskOpr;		//wskaźniki napełniania i opróżniania bufora kołowego
volatile uint32_t nCzasSystemowy;
int16_t sSzerZdjecia, sWysZdjecia;
uint8_t chStatusZdjecia;		//status gotowości wykonania zdjęcia

//ponieważ BDMA nie potrafi komunikować się z pamiecią AXI, więc jego bufory musza być w SRAM4
uint8_t chBuforNadDMA[ROZMIAR_BUF_NAD_DMA]  __attribute__((section(".Bufory_SRAM4")));
uint8_t chBuforOdbDMA[ROZMIAR_BUF_ODB_DMA]  __attribute__((section(".Bufory_SRAM4")));
uint8_t chWyslaneOK = 1;
uint8_t chBuforKomOdb[256];
uint8_t chTimeoutOdbioru;

//deklaracje zmiennych zewnętrznych
extern uint32_t nBuforKamery[ROZM_BUF32_KAM];
extern uint8_t chTrybPracy;
extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef hdma_lpuart1_tx;
extern uint16_t sBuforD1[ROZMIAR16_BUFORA]  __attribute__((section(".Bufory_SRAM1")));
//extern struct st_KonfKam KonfKam;

////////////////////////////////////////////////////////////////////////////////
// Odbiera dane przychodzące z interfejsów kmunikacyjnych w trybie: pytanie - odpowiedź
// Parametry:
// chIn - odbierany bajt
// chInterfejs - identyfikator interfejsu odbierająceg znak
//Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InitProtokol(void)
{
	//odczytaj z konfiguracji i ustaw własny adres sieciowy
	chAdresLokalny = 2;
	return ERR_OK;
}




////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjalizuje zmienne w wątku odbiorczym danych komunikacyjnych po LPUART1
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjalizacjaWatkuOdbiorczegoLPUART1(void)
{
	chWskNap = chWskOpr = 0;
	//HAL_UARTEx_ReceiveToIdle_DMA (&hlpuart1, &chBuforKomOdb, 9);
	//HAL_UART_Receive_DMA (&hlpuart1, &chBuforKomOdb, 1);	//odbieraj do bufora
	//HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, chBuforOdbDMA, 9);
	HAL_UART_Receive_IT (&hlpuart1, chBuforOdbDMA, 1);	//odbieraj do bufora
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja uruchamiana w wątku odbiorczym danych komunikacyjnych po LPUART1
// Analizuje dane z bufora odebranych komunikatów napełnianego w callbacku przychodzących danych
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObslugaWatkuOdbiorczegoLPUART1(void)
{
	//if (chWskNap != chWskOpr)
	while (chWskNap != chWskOpr)
	{
		AnalizujDaneKom(chBuforKomOdb[chWskOpr], INTERF_UART);
		chWskOpr++;
		chWskOpr &= 0xFF;	//zapętlenie wskaźnika bufora kołowego
		chTimeoutOdbioru = 50;	//x osDelay(5); [ms] w głównym wątku
	}

	//po upływie timeoutu resetuj stan protokołu aby następną ramkę zaczął dekodować od nagłówka
	if (chTimeoutOdbioru)
	{
		chTimeoutOdbioru--;
		if (!chTimeoutOdbioru)
			chStanProtokolu[INTERF_UART] = PR_ODBIOR_NAGL;
	}
}



/*void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	//uint8_t chErr;
	uint16_t n;

	if (huart->Instance == LPUART1)
	{
		for (n=0; n<Size; n++)
		{
			chBuforKomOdb[chWskNap] = chBuforOdbDMA[n];
			chWskNap++;
			chWskNap &= 0xF;	//zapętlenie wskaźnika bufora kołowego
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, chBuforOdbDMA, 9);	//ponownie włącz odbiór
		chErr = HAL_UART_Receive_DMA (&hlpuart1, &chBuforKomOdb, 1);	//odbieraj do bufora
	}
}*/



////////////////////////////////////////////////////////////////////////////////
// Callback przerwania UARTA. Przepisuje odebrane dane z małego bufora do większego bufora kołowego analizy protokołu
// Parametry:
// *huart - uchwyt uarta
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == LPUART1)
	{
		chBuforKomOdb[chWskNap] = chBuforOdbDMA[0];
		chWskNap++;
		chWskNap &= 0xFF;	//zapętlenie wskaźnika bufora kołowego
		HAL_UART_Receive_IT (&hlpuart1, chBuforOdbDMA, 1);	//odbieraj do bufora
	}
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
    uint8_t n, chErr;
    uint16_t sPix;
    uint32_t nOffsetDanych;
    static uint8_t chPolecenie;
    static uint8_t chRozmDanych;
    static uint8_t chDane[ROZM_DANYCH_UART];
    extern struct st_KonfKam strKonfKam;


    chErr = DekodujRamke(chWe, &chAdresZdalny[chInterfejs], &chZnakCzasu[chInterfejs], &chPolecenie, &chRozmDanych, chDane, chInterfejs);
    if (chErr == ERR_RAMKA_GOTOWA)
    {
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
			chStatusZdjecia = SGZ_BLAD;		//dopóki nie ma kamery niech zgłasza bład
			chErr = Wyslij_OK(PK_ZROB_ZDJECIE, 0, chInterfejs);
			break;

		case PK_POB_STAT_ZDJECIA:	//pobierz status gotowości zdjęcia
			//generuj testową strukturę obrazu
			if (chStatusZdjecia == SGZ_GOTOWE)
			{
				for (int x=0; x<ROZM_BUF32_KAM; x++)
				{
					sPix = (x*2) & 0xFFFF;
					nBuforKamery[x] = (sPix+1)*0x10000 + sPix;
				}
			}
			chDane[0] = chStatusZdjecia;
			chErr = WyslijRamke(chAdresZdalny[chInterfejs], PK_POB_STAT_ZDJECIA, 1, chDane, chInterfejs);
			break;

		case PK_POBIERZ_ZDJECIE:		//polecenie przesłania fragmentu zdjecia. We: [0..3] - wskaźnik na pozycje bufora, [4] - rozmiar danych do przesłania
			for (n=0; n<4; n++)
				un8_32.dane8[n] = chDane[n];
			nOffsetDanych = un8_32.dane32;
			WyslijRamke(chAdresZdalny[chInterfejs], PK_POBIERZ_ZDJECIE, chDane[4], (uint8_t*)(nBuforKamery + nOffsetDanych),  chInterfejs);
			break;

		case PK_USTAW_ID:		//ustawia identyfikator/adres urządzenia
			chAdresLokalny = chDane[0];
			break;

		case PK_POBIERZ_ID:		//pobiera identyfikator/adres urządzenia
			chDane[0] = chAdresLokalny;
			chErr = WyslijRamke(chAdresZdalny[chInterfejs], PK_POBIERZ_ID, 1, chDane, chInterfejs);
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
			chErr = WyslijRamke(chAdresZdalny[chInterfejs], PK_POB_PAR_KAMERY, 6, chDane, chInterfejs);
			break;

		case PK_UST_PAR_KAMERY:	//ustaw parametry pracy kamery
			strKonfKam.sSzerWy = chDane[0] * SKALA_ROZDZ_KAM;
			strKonfKam.sWysWy = chDane[1] * SKALA_ROZDZ_KAM;
			strKonfKam.sSzerWe = chDane[2] * SKALA_ROZDZ_KAM;
			strKonfKam.sWysWe = chDane[3] * SKALA_ROZDZ_KAM;
			strKonfKam.chTrybDiagn = chDane[4];
			strKonfKam.chFlagi = chDane[5];
			chErr = Wyslij_OK(PK_UST_PAR_KAMERY, 0, chInterfejs);
			break;

		case PK_ZAPISZ_FLASH: 	//zapisz stronę 32 bajtów flash
			for (uint8_t n=0; n<4; n++)
				un8_32.dane8[n] = chDane[n];
			//przepisz dane 8-bitowe  i zapisz po konwersji w buforze 16-bitowym
			for (uint8_t n=0; n<((chDane[4]-5)>>1); n++)
			{
				un8_16.dane8[0] = chDane[n+5];
				un8_16.dane8[1] = chDane[n+6];
				sBuforD1[n] = un8_16.dane16;
			}
			chErr = ZapiszDaneFlashNOR(un8_32.dane32, sBuforD1, (chDane[4]-5)>>1);
			if (chErr == ERR_OK)
				chErr = Wyslij_OK(PK_ZAPISZ_FLASH, 0, chInterfejs);
			else
				chErr = Wyslij_ERR(chErr, 0, chInterfejs);
			break;

		case PK_KASUJ_FLASH:	//kasuj sektor 128kB flash
			for (uint8_t n=0; n<4; n++)
				un8_32.dane8[n] = chDane[n];
			chErr = KasujSektorFlashNOR(un8_32.dane32);

			//na potwierdzenie wyślij ramkę OK lub ramkę z kodem błędu
			if (chErr == ERR_OK)
				chErr = Wyslij_OK(PK_KASUJ_FLASH, 0, chInterfejs);
			else
				chErr = Wyslij_ERR(chErr, 0, chInterfejs);
			break;

		}
    }
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
	uint8_t n, chErr = ERR_OK;
	uint16_t sCrc16Obl;
    //jeżeli minał czas odbioru danych ustalony podczas odbioru nagłówka to resetuj stan protokołu
    //zmienna chProtoTimeout jest dekrementowana co 1ms w procedurze obsługi przerwania timera RIT
    //if (chProtoTimeout[chInterfejs] == 0)
   // {
     //   chProtoState[chInterfejs] = PR_ODB_NAGL;
    //}

    switch (chStanProtokolu[chInterfejs])
    {
    case PR_ODBIOR_NAGL:	//testuj czy odebrano nagłówek
	if (chWe == NAGLOWEK)
	{
		chStanProtokolu[chInterfejs] = PR_ADRES_ODB;
            //NVIC_ICER0 =  (1 << ((unsigned int)(RIT_IRQn) & 0x1F));     //wyłącz przerwanie RIT
            //chProtoTimeout[chInterfejs] = 30; //30ms
            //NVIC_ISER0 =  (1 << ((unsigned int)(RIT_IRQn) & 0x1F));     //włącz przerwanie RIT
	}
	else
	    chErr = ERR_ZLY_NAGL;
	break;

    case PR_ADRES_ODB:
    	if (chWe == chAdresLokalny)				//czy odebraliśmy własny adres sieciowy
    		chStanProtokolu[chInterfejs] = PR_ADRES_NAD;
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
    	sCrc16We = chWe * 0x100;
    	chStanProtokolu[chInterfejs] = PR_CRC16_2;
    	break;

    case PR_CRC16_2:
    	sCrc16We += chWe;
		chStanProtokolu[chInterfejs] = PR_ODBIOR_NAGL;
		//dodać blokadę zasobu CRC
		InicjujCRC16(0, WIELOMIAN_CRC);
		*((volatile uint8_t *)&CRC->DR) = chAdresLokalny;
		*((volatile uint8_t *)&CRC->DR) = *chAdrZdalny;
		*((volatile uint8_t *)&CRC->DR) = *chZnakCzasu;
		*((volatile uint8_t *)&CRC->DR) = *chPolecenie;
		*((volatile uint8_t *)&CRC->DR) = *chRozmDanych;
		for (n=0; n<*chRozmDanych; n++)
			*((volatile uint8_t *)&CRC->DR) = *(chDane + n);
		sCrc16Obl = (uint16_t)CRC->DR;
		//zdjąć blokadę zasobu CRC

		if (sCrc16We == sCrc16Obl)
			chErr = ERR_RAMKA_GOTOWA;
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
	//CRC->CR = CRC_CR_RESET | CRC_CR_POLYSIZE_0 | CRC_CR_REV_IN_0;
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
    if (chRozmDanych > ROZM_DANYCH_UART)
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

    *(chRamka++) = un8_16.dane8[1];	//starszy
    *(chRamka++) = un8_16.dane8[0];	//młodszy
    return ERR_OK;
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
	uint8_t chErr;
	uint8_t chLokalnyZnakCzasu = (nCzasSystemowy / 10) & 0xFF;

    if (chPolecenie & 0x80)
    	chErr = PrzygotujRamke(chAdrZdalny, chAdresLokalny,  chLokalnyZnakCzasu, chPolecenie, chRozmDanych, chDane, chRamkaWyj);	//ramka telemetryczna
    else
    	chErr = PrzygotujRamke(chAdrZdalny, chAdresLokalny,  chZnakCzasu[chInterfejs], chPolecenie, chRozmDanych, chDane, chRamkaWyj);	//ramka odpowiedzi

    if (chErr == ERR_OK)
    {
    	switch (chInterfejs)
    	{
    	case INTERF_UART:	HAL_UART_Transmit(&hlpuart1,  chRamkaWyj, chRozmDanych + ROZM_CIALA_RAMKI, 100);	break;
    						//HAL_UART_Transmit_DMA(&hlpuart1, chRamkaWyj, (uint16_t)chRozmDanych + ROZM_CIALA_RAMKI);
    	case INTERF_ETH:	break;
    	case INTERF_USB:	break;
    	default: chErr = ERR_ZLY_INTERFEJS;	break;
    	}
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


uint8_t TestKomunikacji(void)
{
	uint16_t sRozmDanych;
	uint8_t chErr;
	extern char chBuforNapisowCM4[ROZMIAR_BUF_NAPISOW_CM4];
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
	}

	//sRozmDanych = sprintf((char*)chBuforNadDMA, "Test komunikacji UART\n\r");
	if (sRozmDanych)
		chErr = HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)chBuforNadDMA, sRozmDanych);
	//osDelay(1);
	return chErr;
}




