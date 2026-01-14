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
#include "wymiana.h"
#include "WeWyRC.h"
#include "petla_glowna.h"

// Potrzebne informacje znajdują się w następujaących ramkach. Na początek wysarczymi GGA i RMC
//Ramka	[GGA]	GLL	GSA	GSV	[RMC]	VTG	GRS	GST	ZDA	GBS	PUBX00	PUBX03 PuBX04
//Lon	+		+			+							+
//Lat	+		+			+							+
//Alt	+
//CoG						+		+					+
//SoG						+		+					+
//SvN	+												+		+
//Time	+		+			+			+	+	+	+	+				+
//Date						+					+						+
//HDOP	+			+									+
//VDOP				+									+
//PDOP				+
//TDOP												+


uint8_t chBuforNadawczyGNSS[ROZMIAR_BUF_NAD_GNSS];
uint8_t chBuforOdbioruGNSS[ROZMIAR_BUF_ODB_GNSS];
uint8_t chBuforAnalizyGNSS[ROZMIAR_BUF_ANA_GNSS];
extern uint8_t chBuforAnalizySBus1[ROZMIAR_BUF_ANA_SBUS];
extern uint8_t chBuforAnalizySBus2[ROZMIAR_BUF_ANA_SBUS];
extern uint8_t chBuforOdbioruSBus1[ROZM_BUF_ODB_SBUS];
extern uint8_t chBuforOdbioruSBus2[ROZM_BUF_ODB_SBUS];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart8;
volatile uint8_t chWskNapBaGNSS, chWskOprBaGNSS;		//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych GNSS
extern volatile uint8_t chWskNapBufAnaSBus1, chWskOprBufAnaSBus1; //wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych S-Bus1
uint16_t sCzasInicjalizacjiGNSS = 0;	//licznik czasu	inicjalizacji wyrażony w obiegach pętli 1/200Hz = 5ms
extern volatile unia_wymianyCM4_t uDaneCM4;



////////////////////////////////////////////////////////////////////////////////
// Inicjuje pracę odbiornika GNSS do pracy na 5Hz z ograniczoną liczbą komunikatów
// Każde kolejne uruchomienie realizuje fragment procesu rociągniętego w czasie. Trzeba uruchamiać go dotąd, aż zwróci ERR_DONE
// Pierwsze 2 sekundy czeka na prezentację modułu na prędkości 9600 i jezeli go wykryje to zaczyna jego inicjalizację.
// Jeżeli nie wykryje to wymusza prędkosć docelową i wywyła konfigurację, bo zakłada że moduł jest podtrzymany bateryjnie trzyma swoją konfigurację i prędkość transmisji
// Parametry: brak
// Zwraca: kod zakończenia inicjalizacji: ERR_DONE = zakończono, BLAD_OK - w trakcie
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujGNSS(void)
{
	uint8_t n, chRozmiar = 0;
    uint8_t chErr = BLAD_OK;

    switch (sCzasInicjalizacjiGNSS)	//zlicza kolejne uruchomienia co 1 obieg pętli głównej
    {
    case 1:
    	chWskNapBaGNSS = chWskOprBaGNSS = 0;	//inicjuj wskaźniki napełniania i opróżniania buforma kołowego analizy danych z GNSS
		break;

	//od tego czasu co 50ms wykonaj serię zapisów zmian prędkości  transmitowanych na 9600, 19200, 38400, 75600 i 115200
    case CYKL_STARTU_ZMIAN_PREDK + 0:	//ustaw 9600bps
    	huart8.Init.BaudRate = 9600;
    	chErr = UART_SetConfig(&huart8);
    	HAL_UART_Receive_DMA(&huart8, chBuforOdbioruGNSS, ROZMIAR_BUF_ODB_GNSS);
    	break;

    case CYKL_STARTU_ZMIAN_PREDK + 1:	//wyślij polecenie zmiany prędkości dla GPS
    	//chRozmiar  = sprintf((char*)hDane, "$PMTK251,9600*17\r\n");  //baudrate 9600
		//chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, " $PMTK251,19200*22\r\n");  //baudrate 19200
		//chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK251,38400*27\r\n");  //baudrate 38400
		chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK251,57600*2C\r\n");  //baudrate 57600
		//chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK251,115200*1F\r\n");  //baudrate 115200
    	break;

    case CYKL_STARTU_ZMIAN_PREDK + 10:	//wyślij polecenie zmiany prędkości dla u-Blox
		//chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PUBX,41,1,0003,0002,19200,0*20\r\n");  //baudrate 19200
		//chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PUBX,41,1,0003,0002,38400,0*25\r\n");  //baudrate 38400
		chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PUBX,41,1,0003,0002,57600,0*2E\r\n");  //baudrate 57600
		break;

    case CYKL_STARTU_ZMIAN_PREDK + 20:	//ustaw 19200bps
       	huart8.Init.BaudRate = 19200;
       	chErr = UART_SetConfig(&huart8);
       	HAL_UART_Receive_DMA(&huart8, chBuforOdbioruGNSS, ROZMIAR_BUF_ODB_GNSS);
       	break;

    case CYKL_STARTU_ZMIAN_PREDK + 21:	//wyślij polecenie zmiany prędkości dla GPS
       	chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK251,57600*2C\r\n");  //baudrate 57600
       	break;

    case CYKL_STARTU_ZMIAN_PREDK + 30:
		chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PUBX,41,1,0003,0002,57600,0*2E\r\n");  //baudrate 57600
		break;

    case CYKL_STARTU_ZMIAN_PREDK + 40:
       	huart8.Init.BaudRate = 38400;
       	chErr = UART_SetConfig(&huart8);
       	HAL_UART_Receive_DMA(&huart8, chBuforOdbioruGNSS, ROZMIAR_BUF_ODB_GNSS);
       	break;

    case CYKL_STARTU_ZMIAN_PREDK + 41:
       	chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK251,57600*2C\r\n");  //baudrate 57600
       	break;

    case CYKL_STARTU_ZMIAN_PREDK + 50:
		chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PUBX,41,1,0003,0002,57600,0*2E\r\n");  //baudrate 57600
		break;

    case CYKL_STARTU_ZMIAN_PREDK + 60:
       	huart8.Init.BaudRate = 57600;
       	chErr = UART_SetConfig(&huart8);
       	HAL_UART_Receive_DMA(&huart8, chBuforOdbioruGNSS, ROZMIAR_BUF_ODB_GNSS);
       	break;

    case CYKL_STARTU_ZMIAN_PREDK + 61:
       	chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK251,57600*2C\r\n");  //baudrate 57600
       	break;

    case CYKL_STARTU_ZMIAN_PREDK + 70:
		chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PUBX,41,1,0003,0002,57600,0*2E\r\n");  //baudrate 57600
		break;

    case CYKL_STARTU_ZMIAN_PREDK + 80:
       	huart8.Init.BaudRate = 115200;
       	chErr = UART_SetConfig(&huart8);
       	HAL_UART_Receive_DMA(&huart8, chBuforOdbioruGNSS, ROZMIAR_BUF_ODB_GNSS);
       	break;

    case CYKL_STARTU_ZMIAN_PREDK + 81:
       	HAL_UART_Receive_DMA(&huart8, chBuforOdbioruGNSS, ROZMIAR_BUF_ODB_GNSS);
       	chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK251,57600*2C\r\n");  //baudrate 57600
       	break;

    case CYKL_STARTU_ZMIAN_PREDK + 90:
		chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PUBX,41,1,0003,0002,57600,0*2E\r\n");  //baudrate 57600
		break;


    case 950:							//po konfguracji uBlox
    case CYKL_STARTU_INI_MTK + 0:		//po konfiguracji MTK
    	huart8.Init.BaudRate = 57600;
    	chErr = UART_SetConfig(&huart8);
    	HAL_UART_Receive_DMA(&huart8, chBuforOdbioruGNSS, ROZMIAR_BUF_ODB_GNSS);
    	chRozmiar=0;
    	break;

    //case 13: chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK286,1*23\r\n"); break; //Active Interference Cancelation
    case CYKL_STARTU_INI_MTK + 20:
		chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK286,1*23\r\n");  //Active Interference Cancelation
		break;

    case CYKL_STARTU_INI_MTK + 30:
		chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK314,0,1,10,2,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0*1F\r\n");  //wysyłaj RMC, GGA/2, GSA/5, VTG/10
		//chRozmiar  = sprintf(chBuforNadawczyGNSS, "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");  //wysyłaj RMC, GGA, GSA
		//chRozmiar  = sprintf(chBuforNadawczyGNSS, "$PMTK314,0,1,0,2,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2E\r\n"); break; //wysyłaj RMC, GGA/2, GSA/5
		break;

    /* RAMKI:
     *  - RMTc - dane nawigacyjne
     *  - GGA - wysokość n.p.m. oraz liczba satelitów
     *  - GSA - PDOP,HDOP i GDOP
     *  - VTG - biegun magnetyczny i właściwy
     */


    case CYKL_STARTU_INI_MTK + 40:
		chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK313,1*2E\r\n");  //Enable to search a SBAS satellite
		break;

    case CYKL_STARTU_INI_MTK + 50:
		chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK301,2*2E\r\n");  //DGPS data source mode: WAAS
		break;

    case CYKL_STARTU_INI_MTK + 60:
		//chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK220,500*2B\r\n");  //2Hz
		//chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK220,250*29\r\n");  //4Hz
		chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK220,200*2C\r\n");  //5Hz
		//chRozmiar  = sprintf((char*)chBuforNadawczyGNSS, "$PMTK220,100*2F\r\n");  //10Hz
		break;

		//U-BLOX
	case CYKL_STARTU_INI_UBLOX-1:
		huart8.Init.BaudRate = 9600;		//ustaw domyślną prędkość, gdyż wcześniej została podniesiona dla MTK
		chErr = UART_SetConfig(&huart8);
		HAL_UART_Receive_DMA(&huart8, chBuforOdbioruGNSS, ROZMIAR_BUF_ODB_GNSS);
		break;

	case CYKL_STARTU_INI_UBLOX + 200:
		for (n=0; n<CFG_SBAS_LEN; n++)
			chBuforNadawczyGNSS[n] = CFG_SBAS[n];	// --- CFG-SBAS ---
		chRozmiar = CFG_SBAS_LEN;
		break;

	case CYKL_STARTU_INI_UBLOX + 210:
		for (n=0; n<CFG_NAV_LEN; n++)
			chBuforNadawczyGNSS[n] = CFG_NAV[n];	// --- CFG-NAV ---
		chRozmiar = CFG_NAV_LEN;
		break;

	case CYKL_STARTU_INI_UBLOX + 230:
		for (n=0; n<CFG_NAV2_LEN; n++)
			chBuforNadawczyGNSS[n] = CFG_NAV2[n];	// --- CFG-NAV2 ---
		chRozmiar = CFG_NAV2_LEN;
		break;

	case CYKL_STARTU_INI_UBLOX + 250:
		for (n=0; n<CFG_NAV5_LEN; n++)
			chBuforNadawczyGNSS[n] = CFG_NAV5[n];	// --- CFG-NAV5 ---
		chRozmiar = CFG_NAV5_LEN;
		break;

	case CYKL_STARTU_INI_UBLOX + 270:
		for (n=0; n<CFG_NMEA_LEN; n++)
			chBuforNadawczyGNSS[n] = CFG_NMEA[n];	// --- CFG-NMEA --- - ustawienie wersji protokołu
		chRozmiar = CFG_NMEA_LEN;
		break;

	case CYKL_STARTU_INI_UBLOX + 280:
		for (n=0; n<CFG_RATE_LEN; n++)
			chBuforNadawczyGNSS[n] = CFG_RATE[n];	// --- CFG-RATE --- - pomiar 5 Hz
		chRozmiar = CFG_RATE_LEN;
		break;

	case CYKL_STARTU_INI_UBLOX + 290:
		for (n=0; n<CFG_RXM_LEN; n++)
			chBuforNadawczyGNSS[n] = CFG_RXM[n];	// --- CFG-RXM --- - Performance Mode
		chRozmiar = CFG_RXM_LEN;
		break;

    default:
        if ((sCzasInicjalizacjiGNSS >= CYKL_STARTU_INI_UBLOX) && (sCzasInicjalizacjiGNSS < CYKL_STARTU_INI_UBLOX + 4*CFG_MSG_COMMANDS))	//wyślij CFG_MSG_COMMANDS co dwa obiegi pętli
        {
        	if (sCzasInicjalizacjiGNSS & 0x03)
        		break;		//nie wysyłaj na nieparzystych obiegach

        	//za do drugim wywołaniem co 10ms wyślij jedno polecenie
			for (n=0; n<CFG_MSG_PREAMB_LEN; n++)	//wyślij preambułę
				chBuforNadawczyGNSS[n] = CFG_MSG_PREAMB[n];
			for (n=0; n<CFG_MSG_LEN; n++)			//następnie właściwe polecenie
				chBuforNadawczyGNSS[CFG_MSG_PREAMB_LEN+n] = CFG_MSG_LIST[(sCzasInicjalizacjiGNSS - CYKL_STARTU_INI_UBLOX)/4][n];
			chRozmiar = CFG_MSG_PREAMB_LEN + CFG_MSG_LEN;
			break;
        }

        if ((sCzasInicjalizacjiGNSS >= CYKL_STARTU_INI_UBLOX + 300) && (sCzasInicjalizacjiGNSS < CYKL_STARTU_INI_UBLOX + 300 + 8*CFG_PRT_COMMANDS))	//wyślij CFG_PRT_COMMANDS
        {
        	if (sCzasInicjalizacjiGNSS & 0x07)
        		break;		//nie wysyłaj na nieczwórkowych obiegach

        	for (n=0; n<CFG_PRT_LEN; n++)
				chBuforNadawczyGNSS[n] = CFG_PRT_LIST[(sCzasInicjalizacjiGNSS - (CYKL_STARTU_INI_UBLOX + 300))/8][n];
			chRozmiar = CFG_PRT_LEN;
			break;
        }

		if ((uDaneCM4.dane.nZainicjowano == INIT_WYKR_MTK) && (sCzasInicjalizacjiGNSS < CYKL_STARTU_INI_MTK))
			sCzasInicjalizacjiGNSS = CYKL_STARTU_INI_MTK;		//jeżeli wykryto komunikaty charakterystyczne dla chipsetu MTK to przyspiesz inicjalizację

		if ((uDaneCM4.dane.nZainicjowano == INIT_WYKR_UBLOX) && (sCzasInicjalizacjiGNSS < CYKL_STARTU_INI_UBLOX))
			sCzasInicjalizacjiGNSS = CYKL_STARTU_INI_UBLOX;		//jeżeli wykryto komunikaty charakterystyczne dla chipsetu uBlox to przyspiesz inicjalizację

		if (sCzasInicjalizacjiGNSS > 1000)
		{
			uDaneCM4.dane.nZainicjowano |= INIT_GNSS_GOTOWY;	//nie ciągnij inicjalizacji w nieskończoność
			sCzasInicjalizacjiGNSS = 0;		//inicjuj aby można było ponownie wystartować
		}
    }
    sCzasInicjalizacjiGNSS++;

    if (chRozmiar)
    {
    	chErr = HAL_UART_Transmit_DMA(&huart8, chBuforNadawczyGNSS, chRozmiar);	 //wyślij dane do modułu GNSS
    }

    /*/licz sumę kontrolną ramek, po to aby wstawić właściwą sumę do polecenia
    uint8_t x, ch;
    volatile uint8_t chSum = 0;

    //chRozmiar  = sprintf(chBuforNadawczyGNSS, "$PMTK220,200*2C\r\n");
    //sprintf(chBuforNadawczyGNSS, "$PMTK314,0,1,0,2,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2E\r\n");
    //chRozmiar = sprintf(chBuforNadawczyGNSS, "$PMTK314,0,1,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2D\r\n");   //OK
    chRozmiar = sprintf(chBuforNadawczyGNSS, "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2F\r\n\r\n");
    for (x=1; x<chRozmiar-5;x++)
    {
        ch = chBuforNadawczyGNSS[x];
        chSum ^= ch;
        if (ch == '*')
            break;
    }*/
    return chErr;  //inicjalizacja nie zakończona
}



////////////////////////////////////////////////////////////////////////////////
// Callback odbiorczy UART - obecnie nie używane
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	//odbiór GPS
	if (huart->Instance == UART8)
	{
		//przepisz dane odebrane do większego bufora kołowego analizy danych
		for (uint8_t n=0; n<Size; n++)
		{
			chBuforAnalizyGNSS[chWskNapBaGNSS] = chBuforOdbioruGNSS[n];
			chWskNapBaGNSS++;
			chWskNapBaGNSS &= MASKA_ROZM_BUF_ANA_GNSS;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart8, chBuforOdbioruGNSS, ROZMIAR_BUF_ODB_GNSS);	//wznów odbiór
	}

	//odbiór SBus2
	if (huart->Instance == USART2)
	{

		//HAL_UARTEx_ReceiveToIdle_DMA(&huart2, chBuforOdbioruSBus2, ROZM_BUF_ODB_SBUS);	//wznów odbiór
		//HAL_UART_Receive_IT(&huart2, chBuforOdbioruSBus2, ROZM_BUF_ODB_SBUS);
	}

	//odbiór SBus1
	if (huart->Instance == UART4)
	{

		//HAL_UARTEx_ReceiveToIdle_DMA(&huart4, chBuforOdbioruSBus1, ROZM_BUF_ODB_SBUS);	//wznów odbiór
		//HAL_UART_Receive_IT(&huart4, chBuforOdbioruSBus1, ROZM_BUF_ODB_SBUS);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Callback przerwania UARTA.
// Parametry:
// *huart - uchwyt uarta
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	extern stRC_t stRC;	//struktura danych odbiorników RC

	//Przepisuje odebrane dane GNSS z małego bufora do większego bufora kołowego analizy protokołu
	if (huart->Instance == UART8)
	{
		for (uint8_t n=0; n<ROZMIAR_BUF_ODB_GNSS; n++)
		{
			chBuforAnalizyGNSS[chWskNapBaGNSS] = chBuforOdbioruGNSS[n];
			chWskNapBaGNSS++;
			chWskNapBaGNSS &= MASKA_ROZM_BUF_ANA_GNSS;	//zapętlenie wskaźnika bufora kołowego
		}
		HAL_UART_Receive_DMA(&huart8, chBuforOdbioruGNSS, ROZMIAR_BUF_ODB_GNSS);
	}

	//dane z SBUS1
	if (huart->Instance == UART4)
	{
		stRC.nCzasWe1 = PobierzCzas();	//czas przyjścia ramki SBus1
		for (uint8_t n=0; n<ROZM_BUF_ODB_SBUS; n++)
		{
			chBuforAnalizySBus1[chWskNapBufAnaSBus1] = chBuforOdbioruSBus1[n];
			chWskNapBufAnaSBus1++;
			chWskNapBufAnaSBus1 &= MASKA_ROZM_BUF_ANA_SBUS;	//zapętlenie wskaźnika bufora kołowego
		}
		HAL_UART_Receive_DMA(&huart4, chBuforOdbioruSBus1, ROZM_BUF_ODB_SBUS);
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);			//kanał serw 2 skonfigurowany jako IO
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);				//kanał serw 4 skonfigurowany jako IO
	}

	if (huart->Instance == USART2)		//dane z SBUS2
	{
		//stRC.nCzasWe2 = PobierzCzas();	//czas przyjścia ramki SBus2
		//HAL_UART_Receive_DMA(&huart2, chBuforOdbioruSBus2, ROZM_BUF_ODB_SBUS);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Liczy sumę kontrolną ramki UBX
// Parametry:
// *chCK_A - wskaźnik na pierwszy bajt sumy
// *chCK_B - wskaźnik na drugi bajt sumy
// *chRamka - wskaźnik na ramkę
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void SumaKontrolnaUBX(uint8_t *chRamka)
{
	uint16_t rozmiar = *(chRamka+5) * 0x100 + *(chRamka+4);
	uint8_t chCK_A = 0;
	uint8_t chCK_B = 0;

	for (uint16_t n=0; n<rozmiar+4; n++)
	{
		chCK_A += chRamka[n+2];
		chCK_B += chCK_A;
	}
	*(chRamka + rozmiar + 6) = chCK_A;
	*(chRamka + rozmiar + 7) = chCK_B;
}
