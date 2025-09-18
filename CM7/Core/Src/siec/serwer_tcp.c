//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi serwera TCP/IP
// Serwer pracuje w wątku WatekSerweraTCP znajdującym się w pliku main.c
// Używany jest jako alternatywny kanał przesyłania poleceń komunikacyjnych
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <siec/serwer_tcp.h>
#include "protokol_kom.h"
#include "lwip/api.h"
#include "polecenia_komunikacyjne.h"


extern volatile uint8_t chCzasSwieceniaLED[LICZBA_LED];	//czas świecenia liczony w kwantach 0,1s jest zmniejszany w przerwaniu TIM17_IRQHandler
struct netconn *DeskryptorPolaczeniaPasywnego, *DeskryptorPolaczeniaAktywnego, *DeskryptorNadawczy;	//połączenia serwera i klienta
struct netbuf *bufor;
err_t nErr;
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".Bufory_SRAM3"))) chBuforNadRamkiKomTCP[ROZMIAR_RAMKI_KOMUNIKACYJNEJ];
uint8_t chRozmiarRamkiNadTCP;		//rozmiar ramki nadawczej TCP. Jest zerowany po wysłaniu i ustawioany gdy gotowy do wysyłki
extern uint8_t chStatusPolaczenia;



////////////////////////////////////////////////////////////////////////////////
// Wykonuje otwarcie pasywne portu serwera
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t OtworzPortSertweraTCP(void)
{
	err_t nErr;

	chStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_TCP);
	DeskryptorPolaczeniaPasywnego = netconn_new(NETCONN_TCP);
	if (DeskryptorPolaczeniaPasywnego == NULL)
		return BLAD_OTWARCIA_GNIAZDA;

	nErr = netconn_bind(DeskryptorPolaczeniaPasywnego, IP_ADDR_ANY, 4000);
	if (nErr)
	{
		netconn_delete(DeskryptorPolaczeniaPasywnego);
		return (uint8_t)(nErr & 0xFF);
	}

	nErr = netconn_listen(DeskryptorPolaczeniaPasywnego);
	if (nErr)
	{
		netconn_delete(DeskryptorPolaczeniaPasywnego);
	}

	chStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_TCP);
	chStatusPolaczenia |= (STAT_POL_GOTOWY << STAT_POL_TCP);
	return (uint8_t)(nErr & 0xFF);
}



////////////////////////////////////////////////////////////////////////////////
// Obsługa pętli głównej serwera TCP służącego do komunikacji z aplikacją NSK
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaSerweraTCP(void)
{
	err_t nErr;
	uint8_t chErr;
	nErr = netconn_accept(DeskryptorPolaczeniaPasywnego, &DeskryptorPolaczeniaAktywnego);
	if (nErr == ERR_OK)
	{
		chStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_TCP);
		chStatusPolaczenia |= STAT_POL_OTWARTY << STAT_POL_TCP;
		while ((nErr = netconn_recv(DeskryptorPolaczeniaAktywnego, &bufor)) == ERR_OK)
		{
			void* chDane;
			u16_t sDlugosc;
			netbuf_data(bufor, &chDane, &sDlugosc);
			if (sDlugosc > 0)
			{
				for (uint16_t n=0; n<sDlugosc; n++)
				{
					chErr = AnalizujDaneKom(*((uint8_t*)chDane + n), INTERF_ETH);
					if (chErr)
						chCzasSwieceniaLED[LED_CZER] = 5;	//ewentualne problemy komunikacyjne sygnalizuj czerwonym LED-em przez wielokrotność 100ms

					if (chRozmiarRamkiNadTCP > 0)	//czy jest odpowiedź do odesłania
					{
						chStatusPolaczenia |= STAT_POL_PRZESYLA << STAT_POL_TCP;
						nErr = netconn_write(DeskryptorPolaczeniaAktywnego, chBuforNadRamkiKomTCP, chRozmiarRamkiNadTCP, NETCONN_COPY);
						chRozmiarRamkiNadTCP = 0;
						chStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_TCP);
						chStatusPolaczenia |= STAT_POL_OTWARTY << STAT_POL_TCP;
					}
				}
			}
			netbuf_delete(bufor);
		}
		netconn_delete(DeskryptorPolaczeniaAktywnego);
		chStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_TCP);
		chStatusPolaczenia |= STAT_POL_GOTOWY << STAT_POL_TCP;
	}
	else
		chErr = (uint8_t)(nErr & 0xFF);

	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła ramkę polecenia komunikcyjnego w pakiecie TCP
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t WyslijDanePrzezTCP(uint8_t *Dane, uint8_t chRozmiar)
{
	err_t nErr;

	nErr = netconn_accept(DeskryptorPolaczeniaPasywnego, &DeskryptorNadawczy);
	if (nErr == ERR_OK)
	{
		nErr = netconn_write(DeskryptorNadawczy, Dane, (size_t)chRozmiar, NETCONN_COPY);
	}
	netconn_delete(DeskryptorNadawczy);
	return (uint8_t)nErr & 0xFF;
}
