//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi serwera TCP/IP
// Serwer pracuje w wątku WatekSerweraTCP znajdującym się w pliku main.c
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <lwip/altcp.h>
#include <lwip/tcp.h>
#include <siec/serwer_tcp.h>

uint8_t chWiadomoscGG[DLUGOSC_WIADOMOSCI_GG];	//bufor na odebraną wiadomość
uint8_t chNowaWiadomoscGG;	//informuje o pojawieniu się nowej wiadomości



////////////////////////////////////////////////////////////////////////////////
// Obsługa serwer do wypisywania komunikatów na ekranie
// Parametry: *dane - wskaźnik na bufor z komunikatem
//  sDlugosc - rozmiar danych w buforze
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AnalizaKomunikatuTCP(uint8_t* dane, uint16_t sDlugosc)
{
	uint8_t chErr = BLAD_OK;

	if (sDlugosc > DLUGOSC_WIADOMOSCI_GG)
		sDlugosc = DLUGOSC_WIADOMOSCI_GG;

	if (sDlugosc > 0)
	{
		for (uint16_t n=0; n<sDlugosc; n++)
			chWiadomoscGG[n] = *(dane + n);
		chWiadomoscGG[sDlugosc] = 0;	//zero terminujące string
		chNowaWiadomoscGG = 1;
	}
	return chErr;
}
