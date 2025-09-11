//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi serwera RTSP
// Serwer pracuje w wątku WatekSerweraRTSP znajdującym się w pliku main.c
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "siec/serwerRTSP.h"
#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/sockets.h"

//podsłuchowanie w wireshark: tcp.port == 8554 oraz udp.port >= 5000

/*RTSP działa na porcie 554/TCP (czasem UDP).
Klient (np. VLC, ffmpeg, kamera IP) wysyła komendy typu:
OPTIONS – jakie metody wspierasz,
DESCRIBE – opisz strumień (SDP),
SETUP – ustaw transport (UDP/TCP, porty RTP/RTCP),
PLAY – zacznij strumieniować,
TEARDOWN – zatrzymaj.

Sama transmisja danych to RTP (Real-time Transport Protocol) – idzie równolegle na ustalonym porcie.
Parsowanie komend RTSP
Odebrany tekst jest podobny do HTTP.

Przykład:
DESCRIBE rtsp://192.168.1.100/stream1 RTSP/1.0
CSeq: 2
Accept: application/sdp

Musisz:
rozpoznać metodę (DESCRIBE, SETUP, PLAY…),
zapamiętać numer sekwencji CSeq,
wysłać odpowiedź (RTSP/1.0 200 OK + nagłówki).

Przygotowanie SDP (Session Description Protocol)

W odpowiedzi na DESCRIBE serwer musi wysłać opis strumienia, np.:
v=0
o=- 0 0 IN IP4 192.168.1.100
s=STM32 RTSP Stream
m=video 5004 RTP/AVP 96
a=rtpmap:96 H264/90000

To mówi klientowi (np. VLC), że dane wideo będą na UDP port 5004, w formacie H.264.

Start RTP
Po SETUP klient wskaże porty UDP, na które ma lecieć RTP i RTCP.
Po PLAY musisz wysyłać pakiety RTP – czyli:
nagłówek RTP (12 bajtów),
fragment ramki wideo/audio.
Klienci oczekują np. H.264 (Annex B → NAL units w RTP).

Zegar i synchronizacja
RTP wymaga timestampów i sequence number.
Możesz używać TIM w STM32 do odmierzania 90 kHz (dla H.264) albo 8 kHz/48 kHz (dla audio).
*/

static int client_rtp_port = PORT_RTP;   // port docelowy RTP (ustawiany przez SETUP)
static struct sockaddr_in client_addr;   // adres klienta
int sockfd, connfd;
struct sockaddr_in servaddr, cliaddr;
socklen_t len;


////////////////////////////////////////////////////////////////////////////////
// Inicjalizacja serwera RTSP
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
err_t OtworzPolaczenieSerweraRTSP(void)
{
	err_t nErr;

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) vTaskDelete(NULL);

	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = INADDR_ANY;
	servaddr.sin_port = htons(PORT_RTSP);

	nErr = bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr));
	if (nErr)
		return nErr;
	nErr = listen(sockfd, 2);
	return nErr;
}



////////////////////////////////////////////////////////////////////////////////
// Obsługa serwera RTSP
// Parametry: *dane - wskaźnik na bufor z komunikatem
//  sDlugosc - rozmiar danych w buforze
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void ObslugaSerweraRTSP(void)
{
	char buffer[1024];
	int len;
	int cseq = 1;

	while ((len = recv(connfd, buffer, sizeof(buffer)-1, 0)) > 0)
	{
		buffer[len] = 0;

		// znajdź CSeq
		char *cseq_ptr = strstr(buffer, "CSeq:");
		if (cseq_ptr)
		{
			sscanf(cseq_ptr, "CSeq: %d", &cseq);
		}

		if (strstr(buffer, "OPTIONS"))
		{
			char reply[256];
			snprintf(reply, sizeof(reply), "RTSP/1.0 200 OK\r\nCSeq: %d\r\nPublic: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n\r\n", cseq);
			send(connfd, reply, strlen(reply), 0);
		}
		else if (strstr(buffer, "DESCRIBE"))
		{
			char sdp[256];
			snprintf(sdp, sizeof(sdp),
					 "v=0\r\n"
					 "o=- 0 0 IN IP4 192.168.1.100\r\n"
					 "s=STM32 RTSP Stream\r\n"
					 "m=video %d RTP/AVP 96\r\n"
					 "a=rtpmap:96 H264/90000\r\n", PORT_RTP);

			char reply[512];
			snprintf(reply, sizeof(reply), "RTSP/1.0 200 OK\r\nCSeq: %d\r\nContent-Type: application/sdp\r\nContent-Length: %d\r\n\r\n%s", cseq, (int)strlen(sdp), sdp);
			send(connfd, reply, strlen(reply), 0);
		}
		else if (strstr(buffer, "SETUP"))
		{
			// zakładamy UDP, klient poda w Transport: client_port=xxxx
			char *tp = strstr(buffer, "client_port=");
			if (tp)
			{
				sscanf(tp, "client_port=%d", &client_rtp_port);
			}

			// ustaw adres klienta
			struct sockaddr_in addr;
			socklen_t addrlen = sizeof(addr);
			getpeername(connfd, (struct sockaddr*)&addr, &addrlen);
			client_addr.sin_family = AF_INET;
			client_addr.sin_port = htons(client_rtp_port);
			client_addr.sin_addr = addr.sin_addr;

			char reply[256];
			snprintf(reply, sizeof(reply), "RTSP/1.0 200 OK\r\nCSeq: %d\r\nTransport: RTP/AVP;unicast;client_port=%d-%d\r\nSession: 12345678\r\n\r\n", cseq, client_rtp_port, client_rtp_port+1);
			send(connfd, reply, strlen(reply), 0);
		}
		else if (strstr(buffer, "PLAY")) {
			char reply[256];
			snprintf(reply, sizeof(reply), "RTSP/1.0 200 OK\r\nCSeq: %d\r\nSession: 12345678\r\n\r\n", cseq);
			send(connfd, reply, strlen(reply), 0);

			// Start RTP stream w osobnym tasku
			xTaskCreate(WatekStreamujacyRTP, "rtp", 1024, NULL, osPriorityNormal, NULL);
		}
		else if (strstr(buffer, "TEARDOWN"))
		{
			char reply[128];
			snprintf(reply, sizeof(reply), "RTSP/1.0 200 OK\r\nCSeq: %d\r\nSession: 12345678\r\n\r\n", cseq);
			send(connfd, reply, strlen(reply), 0);
			break;
		}
	}
	closesocket(connfd);
}



////////////////////////////////////////////////////////////////////////////////
// Wątek RTP generujący losowe dane jako "wideo"
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WatekStreamujacyRTP(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
        vTaskDelete(NULL);

    uint8_t rtp_packet[1400];
    uint16_t seq = 0;
    uint32_t timestamp = 0;

    while (1)
    {
        // RTP header (12 bajtów)
        rtp_packet[0] = 0x80;      // V=2, P=0, X=0, CC=0
        rtp_packet[1] = 96;        // PT=96 (dynamic, np. H264)
        rtp_packet[2] = seq >> 8;
        rtp_packet[3] = seq & 0xFF;
        rtp_packet[4] = timestamp >> 24;
        rtp_packet[5] = timestamp >> 16;
        rtp_packet[6] = timestamp >> 8;
        rtp_packet[7] = timestamp & 0xFF;
        rtp_packet[8] = 0x12;      // SSRC (stały na próbę)
        rtp_packet[9] = 0x34;
        rtp_packet[10] = 0x56;
        rtp_packet[11] = 0x78;

        // Payload (losowe bajty)
        for (int i = 12; i < 400; i++)
        {
            rtp_packet[i] = rand() & 0xFF;
        }

        // Wysłanie do klienta
        sendto(sock, rtp_packet, 400, 0, (struct sockaddr*)&client_addr, sizeof(client_addr));
        seq++;
        timestamp += 3600; // udawana prędkość ~40fps

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

