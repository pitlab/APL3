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
#include "czas.h"

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
//int nDeskrGniazdaPolaczenia, nDeskrGniazdaOdbioru;
struct sockaddr_in AdresSerwera, AdresKlienta;
uint8_t chTrwaStreamRTP;
extern const unsigned char oko480x320[152318];
extern uint8_t chStatusPolaczenia;


////////////////////////////////////////////////////////////////////////////////
// Inicjalizacja serwera RTSP
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
err_t OtworzPolaczenieSerweraRTSP(int *nGniazdoPolaczenia)
{
	err_t nErr;

	*nGniazdoPolaczenia = socket(AF_INET, SOCK_STREAM, 0);
	if (*nGniazdoPolaczenia < 0) vTaskDelete(NULL);

	memset(&AdresSerwera, 0, sizeof(AdresSerwera));
	AdresSerwera.sin_family = AF_INET;				//rodzina portów, dla IP4 zawsze AF_INET
	AdresSerwera.sin_addr.s_addr = INADDR_ANY;		//adres IP
	AdresSerwera.sin_port = htons(PORT_RTSP);		//numer portu na którym nasłuchuje serwer (big endian)

	nErr = bind(*nGniazdoPolaczenia, (struct sockaddr*)&AdresSerwera, sizeof(AdresSerwera));
	if (nErr)
		return nErr;
	nErr = listen(*nGniazdoPolaczenia, 2);

	chStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_RTSP);
	chStatusPolaczenia |= (STAT_POL_GOTOWY << STAT_POL_RTSP);
	return nErr;
}



////////////////////////////////////////////////////////////////////////////////
// Obsługa serwera RTSP
// Parametry: *dane - wskaźnik na bufor z komunikatem
//  sDlugosc - rozmiar danych w buforze
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void ObslugaSerweraRTSP(int nGniazdoPolaczenia)
{
	char buffer[1024];
	ssize_t nRozmiar;
	int cseq = 1;
	socklen_t nRozmiarGniazda;
	int nDeskrGniazdaOdbioru;

	nRozmiarGniazda = sizeof(AdresKlienta);
	nDeskrGniazdaOdbioru = accept(nGniazdoPolaczenia, (struct sockaddr*)&AdresKlienta, &nRozmiarGniazda);
	if (nDeskrGniazdaOdbioru >= 0)
	{
		chStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_RTSP);
		chStatusPolaczenia |= (STAT_POL_OTWARTY << STAT_POL_RTSP);
		while ((nRozmiar = recv(nDeskrGniazdaOdbioru, buffer, sizeof(buffer)-1, 0)) > 0)
		//nRozmiar = recv(nDeskrGniazdaOdbioru, buffer, sizeof(buffer)-1, 0);
		//while (nRozmiar > 0)
		{
			buffer[nRozmiar] = 0;	//utnij string na końcu, aby nie były analizowane śmieci w buforze
			char *cseq_ptr = strstr(buffer, "CSeq:");	// znajdź CSeq
			if (cseq_ptr)
				sscanf(cseq_ptr, "CSeq: %d", &cseq);

			if (strstr(buffer, "OPTIONS"))
			{
				char reply[256];
				snprintf(reply, sizeof(reply), "RTSP/1.0 200 OK\r\nCSeq: %d\r\nPublic: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n\r\n", cseq);
				send(nDeskrGniazdaOdbioru, reply, strlen(reply), 0);
			}
			else if (strstr(buffer, "DESCRIBE"))
			{
				char sdp[256];
				snprintf(sdp, sizeof(sdp),
						 "v=0\r\n"
						 "o=- 0 0 IN IP4 192.168.1.101\r\n"
						 "s=STM32 Camera Stream\r\n"
						 "t=0 0\r\n"
						 "m=video %d RTP/AVP 26\r\n"
						 "a=rtpmap:26 JPEG/90000\r\n", PORT_RTP);

				char reply[512];
				snprintf(reply, sizeof(reply), "RTSP/1.0 200 OK\r\nCSeq: %d\r\nContent-Type: application/sdp\r\nContent-Length: %d\r\n\r\n%s", cseq, (int)strlen(sdp), sdp);
				send(nDeskrGniazdaOdbioru, reply, strlen(reply), 0);
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
				getpeername(nDeskrGniazdaOdbioru, (struct sockaddr*)&addr, &addrlen);
				client_addr.sin_family = AF_INET;
				client_addr.sin_port = htons(client_rtp_port);
				client_addr.sin_addr = addr.sin_addr;

				char reply[256];
				snprintf(reply, sizeof(reply), "RTSP/1.0 200 OK\r\nCSeq: %d\r\nTransport: RTP/AVP;unicast;client_port=%d-%d\r\nSession: 12345678\r\n\r\n", cseq, client_rtp_port, client_rtp_port+1);
				send(nDeskrGniazdaOdbioru, reply, strlen(reply), 0);
			}
			else if (strstr(buffer, "PLAY"))
			{
				char reply[256];
				snprintf(reply, sizeof(reply), "RTSP/1.0 200 OK\r\nCSeq: %d\r\nSession: 12345678\r\n\r\n", cseq);
				send(nDeskrGniazdaOdbioru, reply, strlen(reply), 0);

				// Start RTP stream w osobnym tasku
				chTrwaStreamRTP = 1;
				xTaskCreate(WatekStreamujacyRTP, "rtp", 1024, NULL, osPriorityNormal, NULL);
			}
			else if (strstr(buffer, "TEARDOWN"))
			{
				char reply[128];
				snprintf(reply, sizeof(reply), "RTSP/1.0 200 OK\r\nCSeq: %d\r\nSession: 12345678\r\n\r\n", cseq);
				send(nDeskrGniazdaOdbioru, reply, strlen(reply), 0);
				chTrwaStreamRTP = 0;
				break;
			}
		}
		closesocket(nDeskrGniazdaOdbioru);
	}
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

    uint8_t chPakiet_RTP[ROZMIAR_PAKIETU_RTP];
    uint16_t sNumerPakietu = 0;
    uint32_t nTimeStamp = PobierzCzasT6();
    uint32_t nOffsetObrazu, nRozmiarObrazu, nPorcjaObrazu;

    while (chTrwaStreamRTP)
    {
    	chStatusPolaczenia |= (STAT_POL_PRZESYLA << STAT_POL_RTSP);

    	//początek obrazu
    	nRozmiarObrazu = sizeof(oko480x320);
    	nOffsetObrazu = 0;
    	while (nOffsetObrazu < nRozmiarObrazu)
		{
			//Nagłówek RTP (12 bajtów)
			chPakiet_RTP[0] = 0x00;      // V(wersja)=2, P(padding)=0, X(extension)=0, CC(CRSC count)=0
			chPakiet_RTP[1] = 26;        //Payload Type: PT=96 (dynamic, np. H264), PT=26 (JPEG), PT=32 (MPEG2)
			chPakiet_RTP[2] = sNumerPakietu >> 8;	//Sequence number
			chPakiet_RTP[3] = sNumerPakietu & 0xFF;
			chPakiet_RTP[4] = nTimeStamp >> 24;
			chPakiet_RTP[5] = nTimeStamp >> 16;
			chPakiet_RTP[6] = nTimeStamp >> 8;
			chPakiet_RTP[7] = nTimeStamp & 0xFF;
			chPakiet_RTP[8] = 0x12;     //SSRC (Synchronizacja źródła): 32-bitowy identyfikator unikalny dla każdej aktywnej aplikacji lub urządzenia w ramach sesji RTP,
			chPakiet_RTP[9] = 0x34;		//   służący do synchronizacji strumieni pochodzących z różnych źródeł.
			chPakiet_RTP[10] = 0x56;
			chPakiet_RTP[11] = 0x78;

			//Nagłówek JPEG  (8 bajtów)
			chPakiet_RTP[12] = 0; // Type-specific
			chPakiet_RTP[13] = (nOffsetObrazu >> 16) & 0xFF;
			chPakiet_RTP[14] = (nOffsetObrazu >> 8) & 0xFF;
			chPakiet_RTP[15] = nOffsetObrazu & 0xFF;
			chPakiet_RTP[16] = 1;   // Type = baseline JPEG
			chPakiet_RTP[17] = 255; // Q=255 tablice kwantyzacji są w JPEG
			chPakiet_RTP[18] = 480/8; // width/8
			chPakiet_RTP[19] = 320/8; // height/8

			//przepisz kolejny kawałek obrazu do ramki
			nPorcjaObrazu = nRozmiarObrazu - nOffsetObrazu;
			if (nPorcjaObrazu > ROZMIAR_PAKIETU_RTP - 20)
				nPorcjaObrazu = ROZMIAR_PAKIETU_RTP - 20;
			memcpy(&chPakiet_RTP[20], oko480x320 + nOffsetObrazu, nPorcjaObrazu);

			//w ostatnim segmencie daj znacznik
	        if ((nOffsetObrazu + nPorcjaObrazu) >= nRozmiarObrazu)
	        {
	        	chPakiet_RTP[1] |= 0x80; // M=1
	        }

			// Wysłanie do klienta
			sendto(sock, chPakiet_RTP, 20 + nPorcjaObrazu, 0, (struct sockaddr*)&client_addr, sizeof(client_addr));
			sNumerPakietu++;
			nOffsetObrazu += nPorcjaObrazu;
		}
    	nTimeStamp += 90000 / 25; // np. 25 fps
    	vTaskDelay(pdMS_TO_TICKS(40));
    }
    chStatusPolaczenia &= ~(STAT_POL_MASKA << STAT_POL_RTSP);
    chStatusPolaczenia |= (STAT_POL_GOTOWY << STAT_POL_RTSP);
}

