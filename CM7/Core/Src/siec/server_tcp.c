//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi serwera TCP/IP
//
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
// Startuje testowy serwer do wypisywania komunikatów na ekranie
// Parametry: sPort - numer portu na jakim uruchamiana jest usługa
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ServerTcpKomunikatow(uint16_t sPort)
{
	struct tcp_pcb *pcb, *listen_pcb;
	uint8_t chErr = ERR_OK;
	IRQ_DECL_PROTECT(x);

	IRQ_PROTECT(x, LWIP_IRQ_PRIO);
	pcb = tcp_new();
	IRQ_UNPROTECT(x);
	if (pcb == NULL)
		return -1;

	IRQ_PROTECT(x, LWIP_IRQ_PRIO);
	chErr = tcp_bind(pcb, IP_ADDR_ANY, sPort);
	IRQ_UNPROTECT(x);
	if (chErr != ERR_OK)
	{
		IRQ_PROTECT(x, LWIP_IRQ_PRIO);
		// Trzeba zwolnić pamięć, poprzednio było tcp_abandon(pcb, 0);
		tcp_close(pcb);
		IRQ_UNPROTECT(x);
		return -1;
	}

	// Między wywołaniem tcp_listen a tcp_accept stos lwIP nie może odbierać połączeń.
	IRQ_PROTECT(x, LWIP_IRQ_PRIO);
	listen_pcb = tcp_listen(pcb);
	if (listen_pcb)
	//tcp_accept(listen_pcb, accept_callback);
	tcp_accept(listen_pcb, accept);

	IRQ_UNPROTECT(x);

	if (listen_pcb == NULL)
	{
		IRQ_PROTECT(x, LWIP_IRQ_PRIO);
		//Trzeba zwolnić pamięć, poprzednio było tcp_abandon(pcb, 0);
		tcp_close(pcb);
		IRQ_UNPROTECT(x);
		return -1;
	}
	return chErr;
}
