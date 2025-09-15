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


//definicje znaczenia zmiennej chZajetyPrzez. Po zakonczeniu transmisji trzeba wysłać:
#define RAMKA_POLECEN	1	//ramkę poleceń
#define RAMKA_TELE1		2	//ramkę telemetryczną 1
#define RAMKA_TELE2		3	//ramke telemetryczną 2
#define ROZMIAR_KOLEJKI_LPUART	3
typedef struct
{
	uint8_t chZajetyPrzez;	//flaga zajętości
	uint16_t sDoWyslania[ROZMIAR_KOLEJKI_LPUART];	//tablica rozmiarów rzeczy do wysłania po zakończeniu bieżącej transmisji: ramka poleceń i ramki telemetryczne

} st_ZajetoscLPUART_t;



#define ROZM_DANYCH_ETH		1024
#define TR_TIMEOUT			250		//timeout ramki w ms
#define TR_PROB_WYSLANIA	1
#define ROZMIAR_NAGLOWKA	6
#define ROZMIAR_CRC			2
#define ROZM_CIALA_RAMKI	(ROZMIAR_NAGLOWKA + ROZMIAR_CRC)

#define ROZMIAR_RAMKI_KOMUNIKACYJNEJ	254
#define ROZMIAR_DANYCH_KOMUNIKACJI		(ROZMIAR_RAMKI_KOMUNIKACYJNEJ - ROZM_CIALA_RAMKI)

//definicje pól ramki
#define PR_ODBIOR_NAGL		0
#define PR_ADRES_ODB		1
#define PR_ADRES_NAD		2
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
#define DLUGOSC_NAZWY		20		//maksymalna długość nazwy BSP, zmiennych telemetrycznych, lub innych nazw

//nazwy poleceń protokołu
#define PK_OK					0	//akceptacja
#define PK_BLAD					1
#define PK_ZROB_ZDJECIE			2	//polecenie wykonania zdjęcia. We: [0..1] - sSzerokosc zdjecia, [2..3] - wysokość zdjecia
#define PK_POB_STAT_ZDJECIA		3	//pobierz status gotowości zdjęcia
#define PK_POBIERZ_ZDJECIE		4	//polecenie przesłania fragmentu zdjecia. We: [0..3] - wskaźnik na pozycje bufora, [4] - rozmiar danych do przesłania
#define PK_USTAW_BSP			5	//ustawia identyfikator/adres urządzenia
#define PK_POBIERZ_BSP			6	//pobiera identyfikator/adres urządzenia
#define PK_UST_TR_PRACY			7	//ustaw tryb pracy
#define PK_POB_PAR_KAMERY		8	//pobierz parametry pracy kamery
#define PK_UST_PAR_KAMERY		9	//ustaw parametry pracy kamery
#define PK_ZAPISZ_BUFOR			10	//zapisz dane do bufora 128kB w pamięci RAM
#define PK_ZAPISZ_FLASH			11	//zapisz stronę 32 bajtów flash
#define PK_KASUJ_FLASH			12	//kasuj sektor 128kB flash
#define PK_CZYTAJ_FLASH			13	//odczytaj zawartość Flash 
#define PK_CZYTAJ_OKRES_TELE	14	//odczytaj okresy telemetrii
#define PK_ZAPISZ_OKRES_TELE	15	//zapisz okresy telemetrii

#define PK_ZAPISZ_FRAM_U8		16	//Wysyła dane typu uint8_t do zapisu we FRAM w rdzeniu CM4 o podanym rozmiarze liczb uint8_t
#define PK_ZAPISZ_FRAM_FLOAT	17	//Wysyła dane typu float do zapisu we FRAM w rdzeniu CM4 o podanym rozmiarze liczb float
#define PK_WYSLIJ_POTW_ZAPISU	18	//wysyła potwierdzenie zapisania danych ERR_OK lub ERR_PROCES_TRWA
#define PK_CZYTAJ_FRAM_U8		19	//Wysyła  polecenie odczytu zawartości FRAM typu uint8_t
#define PK_CZYTAJ_FRAM_FLOAT	20	//wysyła polecenie odczytu zawartości FRAM typu float
#define PK_WYSLIJ_ODCZYT_FRAM	21	//pobiera odczytane wcześniej dane lub zwraca ERR_PROCES_TRWA
#define PK_REKONFIG_SERWA_RC	22	//wykonuje ponowną konfigurację wejść i wyjść RC po zmianie zawartosci FRAM
#define PK_UST_PAR_KAMERY_GRUP	23	//ustaw parametry pracy kamery grupowo
#define PK_RESETUJ_KAMERE		24	//sprzetowo resetuje kamerę i ładuje domyślne ustawienia
#define PK_ZAMKNIJ_POLACZENIE	25	//NSK kończy pracę i zatrzymuje transmisję danych

#define PK_ILOSC_POLECEN		26	//liczba poleceń do sprawdzania czy polecenie mieści się w obsługiwanych granicach

#define PK_TELEMETRIA1			96	//ramka telemetryczna 1
#define PK_TELEMETRIA2			97	//ramka telemetryczna 2
#define PK_TELEMETRIA3			98	//ramka telemetryczna 3 - na razie nie używane
#define PK_TELEMETRIA4			99	//ramka telemetryczna 4 - na razie nie używane


//Status gotowośco wykonania zdjęcia
#define SGZ_CZEKA		0		//oczekiwania na wykonanie zdjęcia
#define SGZ_GOTOWE		1		//Zdjecie gotowe, można je pobrać
#define SGZ_BLAD		2		//wystapił błąd wykonania zdjecia

//kamera
#define KROK_ROZDZ_KAM	16		//najmnijeszy krok zmiany rozmiaru obrazu o tyle pikseli. Umożliwia wysłanie rozmiaru jako liczby 8-bitowej
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
