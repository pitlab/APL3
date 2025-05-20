#pragma once

//definicje portów
#define UART	1
#define ETHS	2	//ethernet jako serwer
#define ETHK	3	//ethernet jako klient
#define USB		4

//#define ROZM_DANYCH_WE_UART	128
//#define ROZM_DANYCH_WY_UART	128
//#define ROZM_DANYCH_WE_ETH	1024
//#define ROZM_DANYCH_WY_ETH	1024


#define ROZM_DANYCH_ETH		1024
#define ROZM_CIALA_RAMKI	8
#define TR_TIMEOUT			250		//timeout ramki w ms
#define TR_PROB_WYSLANIA	1

#define ROZMIAR_RAMKI_UART	254
#define ROZM_DANYCH_UART	(ROZMIAR_RAMKI_UART - ROZM_CIALA_RAMKI)
#define ROZMIAR_NAGLOWKA	6
#define ROZMIAR_CRC			2

//definicje pól ramki
#define PR_ODBIOR_NAGL		0
#define PR_ADRES_NAD		1
#define PR_ADRES_ODB		2
#define PR_ZNAK_CZASU		3
#define PR_POLECENIE		4
#define PR_ROZM_DANYCH		5
#define PR_DANE				6
#define PR_CRC16_1			7
#define PR_CRC16_2			8


#define NAGLOWEK			0xAA
#define ADRES_STACJI		0x00
#define ADRES_BROADCAST		0xFF
#define WIELOMIAN_CRC		0x1021

//nazwy poleceń protokołu
#define PK_OK					0	//akceptacja
#define PK_BLAD					1
#define PK_ZROB_ZDJECIE			2	//polecenie wykonania zdjęcia. We: [0..1] - sSzerokosc zdjecia, [2..3] - wysokość zdjecia
#define PK_POB_STAT_ZDJECIA		3	//pobierz status gotowości zdjęcia
#define PK_POBIERZ_ZDJECIE		4	//polecenie przesłania fragmentu zdjecia. We: [0..3] - wskaźnik na pozycje bufora, [4] - rozmiar danych do przesłania

#define PK_USTAW_ID				5	//ustawia identyfikator/adres urządzenia
#define PK_POBIERZ_ID			6	//pobiera identyfikator/adres urządzenia
#define PK_UST_TR_PRACY			7	//ustaw tryb pracy
#define PK_POB_PAR_KAMERY		8	//pobierz parametry pracy kamery
#define PK_UST_PAR_KAMERY		9	//ustaw parametry pracy kamery

#define PK_ZAPISZ_BUFOR			10	//zapisz dane do bufora 128kB w pamięci RAM
#define PK_ZAPISZ_FLASH			11	//zapisz stronę 32 bajtów flash
#define PK_KASUJ_FLASH			12	//kasuj sektor 128kB flash
#define PK_CZYTAJ_FLASH			13	//odczytaj zawartość Flash 
#define PK_CZYTAJ_OKRES_TELE	14	//odczytaj okresy telemetrii
#define PK_ZAPISZ_OKRES_TELE	15	//zapisz okresy telemetrii

#define PK_TELEMETRIA			99	//ramka telemetryczna

#define PK_ILOSC_POLECEN		17	//liczba poleceń do sprawdzania czy polecenie mieści się w obsługiwanych granicach




//Status gotowośco wykonania zdjęcia
#define SGZ_CZEKA		0		//oczekiwania na wykonanie zdjęcia
#define SGZ_GOTOWE		1		//Zdjecie gotowe, można je pobrać
#define SGZ_BLAD		2		//wystapił błąd wykonania zdjecia

//kamera
#define SKALA_ROZDZ_KAM	16	//proporcja między obrazem zbieranym przez kamerę (HS x VS) a wysyłanym (DVPHO x DVPVO)
#define MAX_SZER_KAM	2592
#define MAX_WYS_KAM		1944 

//Flagi Ustawien Kamery - numery bitów określających funkcjonalność w UstawieniaKamery.cpp
#define FUK1_ZDJ_FILM	0x01	//1 = zdjecie, 0 = film
#define FUK1_OBR_POZ	0x02	//odwróć obraz w poziomie
#define FUK1_OBR_PION	0x04	//odwróć obraz w pionie


//Tryby Diagnostyczne Kamery
#define TDK_PRACA		0		//normalna praca
#define TDK_KRATA_CB	1		//czarnobiała karata
#define TDK_PASKI		2		//7 pionowych pasków



//definicje zmiennych telemetrycznych
#define TELEM1_AKCEL1X		0x000000001
#define TELEM1_AKCEL1Y		0x000000002
#define TELEM1_AKCEL1Z		0x000000004
#define TELEM1_AKCEL2X		0x000000008
#define TELEM1_AKCEL2Y		0x000000010
#define TELEM1_AKCEL2Z		0x000000020
#define TELEM1_ZYRO1P		0x000000040
#define TELEM1_ZYRO1Q		0x000000080
#define TELEM1_ZYRO1R		0x000000100
#define TELEM1_ZYRO2P		0x000000200
#define TELEM1_ZYRO2Q		0x000000400
#define TELEM1_ZYRO2R		0x000000800
#define TELEM1_MAGNE1X		0x000001000
#define TELEM1_MAGNE1Y		0x000002000
#define TELEM1_MAGNE1Z		0x000004000
#define TELEM1_MAGNE2X		0x000008000
#define TELEM1_MAGNE2Y		0x000010000
#define TELEM1_MAGNE2Z		0x000020000
#define TELEM1_MAGNE3X		0x000040000
#define TELEM1_MAGNE3Y		0x000080000
#define TELEM1_MAGNE3Z		0x000100000
#define TELEM1_KAT_IMU1X	0x000200000
#define TELEM1_KAT_IMU1Y	0x000400000
#define TELEM1_KAT_IMU1Z	0x000800000
#define TELEM1_KAT_IMU2X	0x001000000
#define TELEM1_KAT_IMU2Y	0x002000000
#define TELEM1_KAT_IMU2Z	0x004000000
#define TELEM1_KAT_ZYRO1X	0x008000000
#define TELEM1_KAT_ZYRO1Y	0x010000000
#define TELEM1_KAT_ZYRO1Z	0x020000000


#define TELEM2_CISBEZW1		0x100000001
#define TELEM2_CISBEZW2		0x100000002
#define TELEM2_WYSOKOSC1	0x100000004
#define TELEM2_WYSOKOSC2	0x100000008
#define TELEM2_CISROZN1		0x100000010
#define TELEM2_CISROZN2		0x100000020
#define TELEM2_PREDIAS1		0x100000040
#define TELEM2_PREDIAS2		0x100000080
#define TELEM2_TEMPCIS1		0x100000100
#define TELEM2_TEMPCIS2		0x100000200
#define TELEM2_TEMPIMU1		0x100000400
#define TELEM2_TEMPIMU2		0x100000800
#define TELEM2_TEMPCISR1	0x100001000
#define TELEM2_TEMPCISR2	0x100002000

#define LICZBA_ZMIENNYCH_TELEMETRYCZNYCH	(32+14)
#define LICZBA_BAJTOW_ID_TELEMETRII			(LICZBA_ZMIENNYCH_TELEMETRYCZNYCH / 8)	//liczba bajtów w ramce telemetrii identyfikujaca przesyłane zmienne
