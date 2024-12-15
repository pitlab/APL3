//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi odbiornika GNSS na UART8
//
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "GNSS.h"
#include "uBlox.h"
#include <stdio.h>

uint8_t chBuforAnalizyGNSS[ROZMIAR_BUF_ANALIZY_GNSS];
uint8_t chBuforOdbioruGNSS[ROZMIAR_BUF_ODBIORU_GNSS];
extern UART_HandleTypeDef huart8;
extern uint32_t nZainicjowanoCM4[2];		//flagi inicjalizacji sprzętu
volatile uint8_t chWskNapBoGNSS, chWskOprBoGNSS;		//wskaźniki napełniania i opróżniania bufora kołowego analizy danych GNSS
uint16_t sCzasInicjalizacjiGNSS = 0;	//licznik czasu	inicjalizacji wyrażony w obiegach pętli 1/200Hz = 5ms

////////////////////////////////////////////////////////////////////////////////
// Inicjuje pracę odbiornika GNSS do pracy na 5Hz z ograniczoną liczbą komunikatów
// Parametry: brak
// Zwraca: kod zakończenia inicjalizacji: ERR_DONE = zakończono, ERR_OK - w trakcie
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujGNSS(void)
{
	uint8_t n, chRozmiar = 0;
    uint8_t chDane[55];
    uint8_t chErr = ERR_OK;

    switch (sCzasInicjalizacjiGNSS++)	//zlicza kolejne uruchomienia co 1 obieg pętli głównej
    {
    case 0:	chWskNapBoGNSS = chWskOprBoGNSS = 0;	break;
    case 1:	//ustaw domyślną prędkość
    	huart8.Init.BaudRate = 9600;
    	chErr = UART_SetConfig(&huart8);
    	break;

    case 60:	//konfiguracja dla nowego modułu ustawionego domyślnie na 9600. Trwa ok. 20ms
		//chRozmiar  = sprintf((char*)hDane, "$PMTK251,9600*17\r\n");  //baudrate 9600
		//chRozmiar  = sprintf((char*)chDane, " $PMTK251,19200*22\r\n");  //baudrate 19200
		chRozmiar  = sprintf((char*)chDane, "$PMTK251,38400*27\r\n");  //baudrate 38400
		//chRozmiar  = sprintf((char*)chDane, "$PMTK251,57600*2C\r\n");  //baudrate 57600
		//chRozmiar  = sprintf((char*)chDane, "$PMTK251,115200*1F\r\n");  //baudrate 115200
		break;

    //case 6:	//ustaw Ublox: UART1 38400 in=(uUBX+NMEA+RTCM) out=(NMEA)	Trwa ok 30ms
/*    case 100:	//ustaw Ublox: UART1 38400 in=(uUBX+NMEA+RTCM) out=(NMEA)	Trwa ok 30ms
    	for (n=0; n<CFG_PRT_LEN; n++)
			chDane[n] = CFG_PRT_LIST[1][n];
		chRozmiar = CFG_PRT_LEN;
		break; */

    //case 12:
    case 110:
    	//huart8.Init.BaudRate = 38400;
    	huart8.Init.BaudRate = 38400;
    	chErr = UART_SetConfig(&huart8);
    	chRozmiar=0;
    	break;

    //case 13: chRozmiar  = sprintf((char*)chDane, "$PMTK286,1*23\r\n"); break; //Active Interference Cancelation
    case 112: chRozmiar  = sprintf((char*)chDane, "$PMTK286,1*23\r\n"); break; //Active Interference Cancelation

    //case 14: chRozmiar  = sprintf(chDane, "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"); break; //wysyłaj RMC, GGA, GSA
    //case 14: chRozmiar  = sprintf(chDane, "$PMTK314,0,1,0,2,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2E\r\n"); break; //wysyłaj RMC, GGA/2, GSA/5
    case 120: chRozmiar  = sprintf((char*)chDane, "$PMTK314,0,1,10,2,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0*1F\r\n"); break; //zebrane z Reverse Engineering -
                                                                                                            //wysyłaj RMC, GGA/2, GSA/5, VTG/10
    /* RAMKI:
     *  - RMT - dane nawigacyjne
     *  - GGA - wysokość n.p.m. oraz liczba satelitów
     *  - GSA - PDOP,HDOP i GDOP
     *  - VTG - biegun magnetyczny i właściwy
     */

    //case 16: chRozmiar  = sprintf((char*)chDane, "$PMTK313,1*2E\r\n"); break; //Enable to search a SBAS satellite
    case 130: chRozmiar  = sprintf((char*)chDane, "$PMTK313,1*2E\r\n"); break; //Enable to search a SBAS satellite
    //case 17: chRozmiar  = sprintf((char*)chDane, "$PMTK301,2*2E\r\n"); break; //DGPS data source mode: WAAS
    case 140: chRozmiar  = sprintf((char*)chDane, "$PMTK301,2*2E\r\n"); break; //DGPS data source mode: WAAS

    //case 18: chRozmiar  = sprintf((char*)chDane, "$PMTK220,500*2B\r\n"); break; //2Hz
    //case 18: chRozmiar  = sprintf((char*)chDane, "$PMTK220,250*29\r\n"); break; //4Hz
    case 150: chRozmiar  = sprintf((char*)chDane, "$PMTK220,200*2C\r\n"); break; //5Hz
    //case 18: chRozmiar  = sprintf((char*)chDane, "$PMTK220,100*2F\r\n");  break; //10Hz

    //Od tej pory wysyłaj polecenia konfiguracji uBlox, które są nieczytelne, wiec bezkolizyjne dla MTK
    case 190:	// CFG-DAT (ustaw standard daty)
    	for (n=0; n<CFG_DAT_LEN; n++)
    		chDane[n] = CFG_DAT[n];
    	chRozmiar = CFG_DAT_LEN;
    	break;

    default:
      /*  if ((sCzasInicjalizacjiGNSS >= 20) & (sCzasInicjalizacjiGNSS < 2*CFG_MSG_COMMANDS+20))	//wyślij CFG_MSG_COMMANDS co dwa obiegi pętli
        {
        	if (sCzasInicjalizacjiGNSS & 0x01)
        		break;		//nie wysyłaj na nieparzystych obiegach

        	//za do drugim wywołaniem co 10ms wyślij jedno polecenie
			for (n=0; n<CFG_MSG_PREAMB_LEN; n++)	//wyślij preambułę
				chDane[n] = CFG_MSG_PREAMB[n];

			for (n=0; n<CFG_MSG_LEN; n++)			//następnie właściwe polecenie
				chDane[CFG_MSG_PREAMB_LEN+n] = CFG_MSG_LIST[(sCzasInicjalizacjiGNSS-20)/2][n];
			chRozmiar = CFG_MSG_PREAMB_LEN + CFG_MSG_LEN;
			break;
        }

        if ((sCzasInicjalizacjiGNSS >= 120) & (sCzasInicjalizacjiGNSS < 4*CFG_PRT_COMMANDS+120))	//wyślij CFG_PRT_COMMANDS
        {
        	if (sCzasInicjalizacjiGNSS & 0x03)
        		break;		//nie wysyłaj na nieczwórkowych obiegach

        	for (n=0; n<CFG_PRT_LEN; n++)
				chDane[n] = CFG_PRT_LIST[(sCzasInicjalizacjiGNSS-120)/4][n];
			chRozmiar = CFG_PRT_LEN;
			break;
        }*/

        //if (sCzasInicjalizacjiGNSS > 4*CFG_PRT_COMMANDS+120)
    	if (sCzasInicjalizacjiGNSS > 200)
        {
			nZainicjowanoCM4[0] |= INIT1_GNSS;   //ustaw flagę inicjalizacji GNSS
			sCzasInicjalizacjiGNSS = 0;   //jest później używany do indeksowania bufora
			chErr = ERR_DONE;
			break;
        }
    }

    if (chRozmiar)
    {
        //chErr = HAL_UART_Transmit(&huart8, chDane, chRozmiar, HAL_MAX_DELAY);	 //wyślij dane do GPS
    	chErr = HAL_UART_Transmit_DMA(&huart8, chDane, chRozmiar);	 //wyślij dane do GPS
    }

    /*/licz sumę kontrolną ramek, po to aby wstawić właściwą sumę do polecenia
    uint8_t x, ch;
    volatile uint8_t chSum = 0;

    //chRozmiar  = sprintf(chDane, "$PMTK220,200*2C\r\n");
    //sprintf(chDane, "$PMTK314,0,1,0,2,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2E\r\n");
    //chRozmiar = sprintf(chDane, "$PMTK314,0,1,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2D\r\n");   //OK
    chRozmiar = sprintf(chDane, "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2F\r\n\r\n");
    for (x=1; x<chRozmiar-5;x++)
    {
        ch = chDane[x];
        chSum ^= ch;
        if (ch == '*')
            break;
    }*/
    return chErr;  //inicjalizacja nie zakończona
}




////////////////////////////////////////////////////////////////////////////////
// Callback odbiorczy UART
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	if (huart->Instance == UART8)
	{
		//przepisz dane odebrane do większego bufora kołowego analizy danych
		for (uint8_t n=0; n<Size; n++)
		{
			chBuforAnalizyGNSS[chWskNapBoGNSS] = chBuforOdbioruGNSS[n];
			chWskNapBoGNSS++;
			chWskNapBoGNSS &= MASKA_ROZM_BUF_ODB_GNSS;
		}

		//wznów odbiór
		HAL_UARTEx_ReceiveToIdle_DMA(&huart8, chBuforOdbioruGNSS, ROZMIAR_BUF_ODBIORU_GNSS);
	}
}
