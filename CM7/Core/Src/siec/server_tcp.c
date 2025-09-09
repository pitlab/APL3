//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi serwera TCP/IP
// Serwer pracuje w wątku WatekSerweraTCP znajdującym się w pliku main.c
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "siec/server_tcp.h"
#include "siec/makra.h"
#include <lwip/altcp.h>
#include <lwip/tcp.h>



////////////////////////////////////////////////////////////////////////////////
// Obsługa serwer do wypisywania komunikatów na ekranie
// Parametry: *dane - wskaźnik na bufor z komunikatem
//  sDlugosc - rozmiar danych w buforze
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AnalizaKomunikatuTCP(void* dane, uint16_t sDlugosc)
{
	uint8_t chErr = BLAD_OK;

	return chErr;
}
