//////////////////////////////////////////////////////////////////////////////
//
// Definicje zmiennych konfguracyjnych dla CM4 trzymanych w pamięci FRAM FM25V02-G 32k x 8bit
// Zakres adresów do 0x7FFF
//
// (c) PitLab 2024-26
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "SysDefWspolny.h"
//adresy zmiennych konfiguracyjnych w zakresie 0..0x2000  (FA == FRAM Adres)
//Indeksy w komentarzu oznaczaja rozmiar (liczba) i typ (litera) zmiennej
//Typy zmiennych: 
//FAU - zmienna użytkownika, zawiera jego nastawy
//FAS - zmienna systemowa lub dynamiczna, nie dotykać sama się modyfikuje, moze być przenoszona na inne urządzenia
//FAH - zmienna zawierająca indywidualne parametry sprzetu - chronić przed przypadkową zmianą, nie zamieniać między urządzeniami
//
//Formaty liczb. Cyfra przed literą oznacza rozmiar w bajtach:
// U - liczba 8-bitowa całkowita bez znaku.
// S - liczba 8-bitowa całkowita ze znakiem.
// C - znak alfanumeryczny
// F - liczba float

#define FA_USER_VAR	    	0x0000	    //zmienne użytkownika
#define FAU_WE_RC1_MIN		0x0000		//16*2U minimalna wartość każdego kanału z odbiornika RC1
#define FAU_WE_RC1_MAX		0x0020		//16*2U maksymalna wartość każdego kanału z odbiornika RC1
#define FAU_WE_RC2_MIN		0x0040		//16*2U minimalna wartość każdego kanału z odbiornika RC2
#define FAU_WE_RC2_MAX		0x0060		//16*2U maksymalna wartość każdego kanału z odbiornika RC2

#define FAU_80        		0x0080
//wolne 12 bajtów

#define FAU_RC_WY_MIN   	0x0092     	//2U minimalne wysterowanie regulatorów w trakcie lotu w jednostkach standardowych 0-2000
#define FAU_RC_WY_MAX      	0x0094 		//2U maksymalne wysterowanie silników w trakcie lotu w jednostkach standardowych 0-2000
#define FAU_RC_WY_ZAWISU   	0x0096   	//2U wysterowanie regulatorów w zawisie w jednostkach standardowych 0-2000
#define LICZBA_DANYCH_NAPEDU	3

#define FAU_RC_WY_IDENT		0x0098		//2U wysterowanie regulatorów podczas identyfikacji w jednostkach standardowych [0..1999]
#define FAU_CZAS_IDENT		0x009A		//2U czas identyfikacji każdego silnika w milisekundach
#define FAU_9C				0x009C
//wolne 4 bajty

//mikser
#define FAU_MIX_PRZECH		0x00A0		//8*4F współczynnik wpływu przechylenia na dany silnik
#define FAU_MIX_POCHYL  	0x00C0   	//8*4F współczynnik wpływu pochylenia na dany silnik
#define FAU_MIX_ODCHYL    	0x00E0   	//8*4F współczynnik wpływu odchylenia na dany silnik

//regulatory PID
#define FA_USER_PID	    	0x0100
#define FAU_PID_KP          FA_USER_PID+0   //4F wzmocnienienie członu P regulatora
#define FAU_PID_KI          FA_USER_PID+4   //4F wzmocnienienie członu I regulatora
#define FAU_PID_KD          FA_USER_PID+8   //4F wzmocnienienie członu D regulatora
#define FAU_PID_OGR_CALK    FA_USER_PID+12  //4F ograniczenie wartości całki członu I regulatora
#define FAU_PID_MIN_WY		FA_USER_PID+16  //4F minimalna wartość wyjścia
#define FAU_PID_MAX_WY		FA_USER_PID+20  //4F maksymalna wartość wyjścia
#define FAU_PID_MNOZN_WZAD 	FA_USER_PID+24	//4F mnożnik wartości zadanej
#define FAU_PID_PRZES		FA_USER_PID+28 	//4F stała wartość dodawana do wyjścia regulatora (umożliwia lot pod niezerowym kątem)
#define FAU_PID_KW			FA_USER_PID+32	//4F wzmocnienia wyprzedzenia
#define FAU_PID_FLAGI		FA_USER_PID+36	//1U regulator wyłączony (bit 6), Regulator kątowy (bit 7)
#define FAU_PID_FD 			FA_USER_PID+37  //1U Podstawa filtra IIR błędu do liczenia członu różniczkującego
#define FAU_PID_FWZ			FA_USER_PID+38  //1U Podstawa filtra IIR wartości zadanej do liczenia członu wyprzedzajacego
#define FAU_PID_FWE  		FA_USER_PID+39  //1U  Podstawa filtra IIR wartości wejściowej
#define FAU_PID2			FA_USER_PID+40	//1U wolne
#define FAU_PID3			FA_USER_PID+41	//1U wolne
#define ROZMIAR_REG_PID		42
//12 regulatorów zajmuje 504 bajtów - 0x1F8
#define FAU_TRYB_REG	    0x02F8		//6*1U Tryb pracy regulatorów 4 podstawowych wartości przypisanych do drążków i 2 regulatorów pozycji N i E
#define FAU_CRC_PID			0x02FE		//2U CRC konfiguracji regulatorów liczone od 0x0100 do 0x02FE

//konfiguracja odbiorników RC i wyjść serw/ESC zdefiniowane w sys_def_wspolnych.h
#define FAU_KONF_ODB_RC		0x0300		//1U konfiguracja odbiorników RC: Bity 0..3 = RC1, bity 4..7 = RC2: 0=PPM, 1=S-Bus
#define FAU_KONF_SERWA12	0x0301		//1U konfiguracja wyjść: Bity 0..3 = Wyjście 1, bity 4..7 = Wyjście 2
#define FAU_KONF_SERWA34	0x0302		//1U konfiguracja wyjść: Bity 0..3 = Wyjście 3, bity 4..7 = Wyjście 4
#define FAU_KONF_SERWA56	0x0303		//1U konfiguracja wyjść: Bity 0..3 = Wyjście 5, bity 4..7 = Wyjście 6
#define FAU_KONF_SERWA78	0x0304		//1U konfiguracja wyjść: Bity 0..3 = Wyjście 7, bity 4..7 = Wyjście 8
#define FAU_KONF_SERWA916	0x0305		//1U konfiguracja wyjść: 0=wyjścia 9..16 PWM 50Hz, 1=wyjścia 9..12 PWM 100Hz, 2=wyjścia 9..10 PWM 200Hz, 3=wyjście 9 PWM 400Hz
#define FAU_FUNKCJA_WY_RC	0x0306		//16*1U przypisanie funkcji do każdego wyjścia RC
#define FAU_LOW_VOLT_WARN   0x0316  	//4F próg ostrzezenia o niskim napięciu
#define FAU_LOW_VOLT_ALARM  0x031A 		//4F próg alarmu niskiego napięcia
#define FAU_VOLT_DROP_COMP  0x031E		//4F współczynnik kompensacji spadku napięcia pakietu
#define FAU_LANDING_SPD     0x0322		//4F prędkość lądowania
#define FAU_326				0x0326
//wolne 16 bajtów


//definicje wskaźników LED zrobionych z WS8213
#define FAU_WSKLED_TYP_LED	0x0336		//1U typ układu WS8211 lub WS8213, ponieważ maja inne definicje kolorów
#define FAU_WSKLED1			0x0337
#define FAU_WSKLED1_MIN_ZMIENNEJ	FAU_WSKLED1+0	//4F dolny zakres wizualizacji
#define FAU_WSKLED1_MAX_ZMIENNEJ	FAU_WSKLED1+4	//4F górny zakres wizualizacji
#define FAU_WSKLED1_NUM_ZMIENNEJ	FAU_WSKLED1+8	//1U indeks wizualizowanej zmiennej 1
#define FAU_WSKLED1_SZER_WSKAZNIKA	FAU_WSKLED1+9	//1U liczba LED szerokość plamki wskaźnika
#define FAU_WSKLED1_DZIELNNIK_TLA	FAU_WSKLED1+10	//1U wskazuje ile razy tło jest ciemniejsze od plamki
#define FAU_WSKLED1_MIN_CZER		FAU_WSKLED1+11	//1U poziom składowej czerwonej na początku skali1
#define FAU_WSKLED1_MAX_CZER		FAU_WSKLED1+12	//1U poziom składowej czerwonej na końcu skali1
#define FAU_WSKLED1_MIN_ZIEL		FAU_WSKLED1+13	//1U poziom składowej zielonej na początku skali1
#define FAU_WSKLED1_MAX_ZIEL		FAU_WSKLED1+14	//1U poziom składowej zielonej na końcu skali1
#define FAU_WSKLED1_MIN_NIEB		FAU_WSKLED1+15	//1U poziom składowej niebieskiej na początku skali1
#define FAU_WSKLED1_MAX_NIEB		FAU_WSKLED1+16	//1U poziom składowej niebieskiej na końcu skali1
#define FAU_WSKLED1_LICZBA_LED		FAU_WSKLED1+17	//1U liczba LED-ów z których zbudowany jest wskaźnik
#define ROZMIAR_WSKAZNIKA_LED		18
#define LICZBA_WSKAZNIKOW_LED		2

#define FAU_WSKLED2			FAU_WSKLED1 + ROZMIAR_WSKAZNIKA_LED	//0x349
#define FAU_WSKLED2_MIN_ZMIENNEJ	FAU_WSKLED2+0	//4F dolny zakres wizualizacji
#define FAU_WSKLED2_MAX_ZMIENNEJ	FAU_WSKLED2+4	//4F górny zakres wizualizacji
#define FAU_WSKLED2_NUM_ZMIENNEJ	FAU_WSKLED2+8	//1U indeks wizualizowanej zmiennej 2
#define FAU_WSKLED2_SZER_WSKAZNIKA	FAU_WSKLED2+9	//1U liczba LED szerokość plamki wskaźnika
#define FAU_WSKLED2_DZIELNNIK_TLA	FAU_WSKLED2+10	//1U wskazuje ile razy tło jest ciemniejsze od plamki
#define FAU_WSKLED2_MIN_CZER		FAU_WSKLED2+11	//1U definiuje poziom składowej czerwonej na początku skali1
#define FAU_WSKLED2_MAX_CZER		FAU_WSKLED2+12	//1U definiuje poziom składowej czerwonej na końcu skali1
#define FAU_WSKLED2_MIN_ZIEL		FAU_WSKLED2+13	//1U definiuje poziom składowej zielonej na początku skali1
#define FAU_WSKLED2_MAX_ZIEL		FAU_WSKLED2+14	//1U definiuje poziom składowej zielonej na końcu skali1
#define FAU_WSKLED2_MIN_NIEB		FAU_WSKLED2+15	//1U definiuje poziom składowej niebieskiej na początku skali1
#define FAU_WSKLED2_MAX_NIEB		FAU_WSKLED2+16	//1U definiuje poziom składowej niebieskiej na końcu skali1
#define FAU_WSKLED2_LICZBA_LED		FAU_WSKLED2+17	//1U liczba LED-ów z których zbudowany jest wskaźnik

#define FAU_35B						0x035B
//miejsce zarezerwowane na trzeci wskaźnik

#define FAU_KAN_DRAZKA_RC			0x036D		//4*1U Numer kanału przypisany do funkcji drążka aparatury: przechylenia, pochylenia, odchylenia i wysokości
#define FAU_FUNKCJA_KAN_RC			0x0371		//12*1U Numer funkcji przypisanej do kanału RC (5..16)
#define FAU_STROJ1_PARAMETR			0x037D		//1U numer strojonego parametru 1
#define FAU_STROJ2_PARAMETR			0x037E		//1U numer strojonego parametru 2
#define FAU_STROJ1_WART_MIN			0x037F		//4F minimalna wartość parametru 1
#define FAU_STROJ2_WART_MIN			0x0383		//4F minimalna wartość parametru 2
#define FAU_STROJ1_WART_MAX			0x0387		//4F maksymalna wartość parametru 1
#define FAU_STROJ2_WART_MAX			0x038B		//4F maksymalna wartość parametru 2
#define FAU_38F						0x038F
//wolne 369 bajtów




#define FA_HARD_VAR         	0x0500	    //zmienne definiujące parametry sprzętu 0x500 = 1280

#define	FAH_MOD_AKCEL       	FA_HARD_VAR     	//akcelerometry na module inercyjnym dowolnego typu (wspólna konfiguracja)
#define	FAH_AKCEL1X_PRZES   	FAH_MOD_AKCEL+0   	//4F korekcja przesuniecia w osi X akcelerometru 1
#define	FAH_AKCEL1Y_PRZES   	FAH_MOD_AKCEL+4   	//4F korekcja przesuniecia w osi Y akcelerometru 1
#define	FAH_AKCEL1Z_PRZES  		FAH_MOD_AKCEL+8   	//4F korekcja przesuniecia w osi Z akcelerometru 1
#define	FAH_AKCEL1X_WZMOC     	FAH_MOD_AKCEL+12  	//4F korekcja MNOZNIKwanie w osi X akcelerometru 1
#define	FAH_AKCEL1Y_WZMOC     	FAH_MOD_AKCEL+16  	//4F korekcja MNOZNIKwanie w osi Y akcelerometru 1
#define	FAH_AKCEL1Z_WZMOC		FAH_MOD_AKCEL+20  	//4F korekcja MNOZNIKwanie w osi Z akcelerometru 1
#define	FAH_AKCEL2X_PRZES   	FAH_MOD_AKCEL+24   	//4F korekcja przesuniecia w osi X akcelerometru 2
#define	FAH_AKCEL2Y_PRZES   	FAH_MOD_AKCEL+28   	//4F korekcja przesuniecia w osi Y akcelerometru 2
#define	FAH_AKCEL2Z_PRZES  		FAH_MOD_AKCEL+32   	//4F korekcja przesuniecia w osi Z akcelerometru 2
#define	FAH_AKCEL2X_WZMOC     	FAH_MOD_AKCEL+36  	//4F korekcja MNOZNIKwanie w osi X akcelerometru 2
#define	FAH_AKCEL2Y_WZMOC     	FAH_MOD_AKCEL+40  	//4F korekcja MNOZNIKwanie w osi Y akcelerometru 2
#define	FAH_AKCEL2Z_WZMOC		FAH_MOD_AKCEL+44  	//4F korekcja MNOZNIKwanie w osi Z akcelerometru 2

#define	FAH_MOD_ZYRO1      		FAH_MOD_AKCEL+48    //moduł inercyjny dowolnego typu (wspólna konfiguracja)
#define	FAH_ZYRO1_X_PRZ_ZIM 	FAH_MOD_ZYRO1+0  	//4F korekcja przesuniecia w osi X żyroskopu 1 na zimno
#define	FAH_ZYRO1_Y_PRZ_ZIM 	FAH_MOD_ZYRO1+4  	//4F korekcja przesuniecia w osi Y żyroskopu 1 na zimno
#define	FAH_ZYRO1_Z_PRZ_ZIM 	FAH_MOD_ZYRO1+8  	//4F korekcja przesuniecia w osi Z żyroskopu 1 na zimno
#define	FAH_ZYRO1_X_PRZ_POK 	FAH_MOD_ZYRO1+12  	//4F korekcja przesuniecia w osi X żyroskopu 1 w 25°C
#define	FAH_ZYRO1_Y_PRZ_POK 	FAH_MOD_ZYRO1+16  	//4F korekcja przesuniecia w osi Y żyroskopu 1 w 25°C
#define	FAH_ZYRO1_Z_PRZ_POK 	FAH_MOD_ZYRO1+20  	//4F korekcja przesuniecia w osi Z żyroskopu 1 w 25°C
#define	FAH_ZYRO1_X_PRZ_GOR 	FAH_MOD_ZYRO1+24  	//4F korekcja przesuniecia w osi X żyroskopu 1 na gorąco
#define	FAH_ZYRO1_Y_PRZ_GOR 	FAH_MOD_ZYRO1+28  	//4F korekcja przesuniecia w osi Y żyroskopu 1 na gorąco
#define	FAH_ZYRO1_Z_PRZ_GOR 	FAH_MOD_ZYRO1+32  	//4F korekcja przesuniecia w osi Z żyroskopu 1 na gorąco
#define	FAH_ZYRO1P_WZMOC     	FAH_MOD_ZYRO1+36  	//4F korekcja MNOZNIKwanie żyroskopu 1P
#define	FAH_ZYRO1Q_WZMOC     	FAH_MOD_ZYRO1+40  	//4F korekcja MNOZNIKwanie żyroskopu 1Q
#define	FAH_ZYRO1R_WZMOC     	FAH_MOD_ZYRO1+44  	//4F korekcja MNOZNIKwanie żyroskopu 1R

#define	FAH_MOD_ZYRO2      		FAH_MOD_ZYRO1+48    //moduł inercyjny dowolnego typu (wspólna konfiguracja)
#define	FAH_ZYRO2_X_PRZ_ZIM 	FAH_MOD_ZYRO2+0  	//4F korekcja przesuniecia w osi X żyroskopu 2 na zimno
#define	FAH_ZYRO2_Y_PRZ_ZIM 	FAH_MOD_ZYRO2+4  	//4F korekcja przesuniecia w osi Y żyroskopu 2 na zimno
#define	FAH_ZYRO2_Z_PRZ_ZIM 	FAH_MOD_ZYRO2+8  	//4F korekcja przesuniecia w osi Z żyroskopu 2 na zimno
#define	FAH_ZYRO2_X_PRZ_POK 	FAH_MOD_ZYRO2+12  	//4F korekcja przesuniecia w osi X żyroskopu 2 w 25°C
#define	FAH_ZYRO2_Y_PRZ_POK 	FAH_MOD_ZYRO2+16  	//4F korekcja przesuniecia w osi Y żyroskopu 2 w 25°C
#define	FAH_ZYRO2_Z_PRZ_POK 	FAH_MOD_ZYRO2+20  	//4F korekcja przesuniecia w osi Z żyroskopu 2 w 25°C
#define	FAH_ZYRO2_X_PRZ_GOR 	FAH_MOD_ZYRO2+24  	//4F korekcja przesuniecia w osi X żyroskopu 2 na gorąco
#define	FAH_ZYRO2_Y_PRZ_GOR 	FAH_MOD_ZYRO2+28  	//4F korekcja przesuniecia w osi Y żyroskopu 2 na gorąco
#define	FAH_ZYRO2_Z_PRZ_GOR 	FAH_MOD_ZYRO2+32  	//4F korekcja przesuniecia w osi Z żyroskopu 2 na gorąco
#define	FAH_ZYRO2P_WZMOC     	FAH_MOD_ZYRO2+36  	//4F korekcja MNOZNIKwanie żyroskopu 2P
#define	FAH_ZYRO2Q_WZMOC     	FAH_MOD_ZYRO2+40  	//4F korekcja MNOZNIKwanie żyroskopu 2Q
#define	FAH_ZYRO2R_WZMOC     	FAH_MOD_ZYRO2+44  	//4F korekcja MNOZNIKwanie żyroskopu 2R

#define	FAH_ZYRO_TEMP 			FAH_MOD_ZYRO2+48
#define	FAH_ZYRO1_TEMP_ZIM     	FAH_ZYRO_TEMP+0		//4F temperatura kalibracji na zimno żyroskopu 1
#define	FAH_ZYRO1_TEMP_POK     	FAH_ZYRO_TEMP+4		//4F temperatura kalibracji w temperaturze pokojowej żyroskopu 1
#define	FAH_ZYRO1_TEMP_GOR     	FAH_ZYRO_TEMP+8		//4F temperatura kalibracji na gotąco żyroskopu 1
#define	FAH_ZYRO2_TEMP_ZIM     	FAH_ZYRO_TEMP+12	//4F temperatura kalibracji na zimno żyroskopu 2
#define	FAH_ZYRO2_TEMP_POK     	FAH_ZYRO_TEMP+16	//4F temperatura kalibracji w temperaturze pokojowej żyroskopu 2
#define	FAH_ZYRO2_TEMP_GOR     	FAH_ZYRO_TEMP+20	//4F temperatura kalibracji na gotąco żyroskopu 2

#define	FAH_MOD_CIS_ROZ	    	FAH_ZYRO_TEMP+24    //czujniki ciśnienia różnicowego
#define	FAH_CISN_ROZN1_ZIM 		FAH_MOD_CIS_ROZ+0	//4F korekcja czujnika ciśnienia różnicowego 1 na zimno
#define	FAH_CISN_ROZN1_POK 		FAH_MOD_CIS_ROZ+4	//4F korekcja czujnika ciśnienia różnicowego 1 w 25°C
#define	FAH_CISN_ROZN1_GOR 		FAH_MOD_CIS_ROZ+8	//4F korekcja czujnika ciśnienia różnicowego 1 na gorąco
#define	FAH_CISN_ROZN2_ZIM 		FAH_MOD_CIS_ROZ+12	//4F korekcja czujnika ciśnienia różnicowego 2 na zimno
#define	FAH_CISN_ROZN2_POK 		FAH_MOD_CIS_ROZ+16	//4F korekcja czujnika ciśnienia różnicowego 2 w 25°C
#define	FAH_CISN_ROZN2_GOR 		FAH_MOD_CIS_ROZ+20	//4F korekcja czujnika ciśnienia różnicowego 2 na gorąco

//współczynniki kalibracji magnetometru 1 na module inercyjnym
#define	FAH_MAGN1        			FAH_MOD_CIS_ROZ+24
#define	FAH_MAGN1_SKLADNIK_X  		FAH_MAGN1+0 	//4F przesunięcie magnetometru w osi X
#define	FAH_MAGN1_SKLADNIK_Y  		FAH_MAGN1+4 	//4F przesunięcie magnetometru w osi Y
#define	FAH_MAGN1_SKLADNIK_Z  		FAH_MAGN1+8 	//4F przesunięcie magnetometru w osi Z
#define	FAH_MAGN1_MNOZNIK_X  		FAH_MAGN1+12 	//4F mnożnik w osi X
#define	FAH_MAGN1_MNOZNIK_Y  		FAH_MAGN1+16 	//4F mnożnik w osi Y
#define	FAH_MAGN1_MNOZNIK_Z  		FAH_MAGN1+20 	//4F mnożnik w osi Z

//współczynniki kalibracji magnetometru 2 na module inercyjnym
#define	FAH_MAGN2        			FAH_MAGN1+24
#define	FAH_MAGN2_SKLADNIK_X  		FAH_MAGN2+0 	//4F przesunięcie magnetometru w osi X
#define	FAH_MAGN2_SKLADNIK_Y  		FAH_MAGN2+4 	//4F przesunięcie magnetometru w osi Y
#define	FAH_MAGN2_SKLADNIK_Z  		FAH_MAGN2+8 	//4F przesunięcie magnetometru w osi Z
#define	FAH_MAGN2_MNOZNIK_X  		FAH_MAGN2+12 	//4F mnożnik w osi X
#define	FAH_MAGN2_MNOZNIK_Y  		FAH_MAGN2+16 	//4F mnożnik w osi Y
#define	FAH_MAGN2_MNOZNIK_Z  		FAH_MAGN2+20 	//4F mnożnik w osi Z

//współczynniki kalibracji magnetometru 3 na module GNSS
#define	FAH_MAGN3        			FAH_MAGN2+24
#define	FAH_MAGN3_SKLADNIK_X  		FAH_MAGN3+0 	//4F przesunięcie magnetometru w osi X
#define	FAH_MAGN3_SKLADNIK_Y  		FAH_MAGN3+4 	//4F przesunięcie magnetometru w osi Y
#define	FAH_MAGN3_SKLADNIK_Z  		FAH_MAGN3+8 	//4F przesunięcie magnetometru w osi Z
#define	FAH_MAGN3_MNOZNIK_X  		FAH_MAGN3+12 	//4F mnożnik w osi X
#define	FAH_MAGN3_MNOZNIK_Y  		FAH_MAGN3+16 	//4F mnożnik w osi Y
#define	FAH_MAGN3_MNOZNIK_Z  		FAH_MAGN3+20 	//4F mnożnik w osi Z

#define	FAH_MNOZNIK_CISN_BEZWZGL1	FAH_MAGN3+24    //mnożnik ciśnienia bezwzględnego czujnika 1
#define	FAH_MNOZNIK_CISN_BEZWZGL2	FAH_MAGN3+28    //mnożnik ciśnienia bezwzględnego czujnika 2


#define FAG_CZUJ_ZEWN				FAH_MAGN3+32	//zewnętrzne czujniki analogowe
#define FAG_MNOZNIK_CZUJ_ZEWN		FAG_CZUJ_ZEWN+0		//4*4F współczynnik mnożenia analogowego napęcia czujnika zewnętrznego
#define FAG_SKLADNIK_CZUJ_ZEWN		FAG_CZUJ_ZEWN+16	//4*4F współczynnik dodawany do analogowego napęcia czujnika zewnętrznego


#define FAH_TEST					0x0FE0	//4H testowy zapis do pamięci w celu sprawdzenia poprawności

//zmienne systemowe i dynamiczne
#define FA_SYS_VAR	    			0x0FF0
#define	FAS_NUMER_SESJI	  			FA_SYS_VAR		//1U Numer sesji
#define FAS_TSYNC_MON       		FA_SYS_VAR+1   	//1U miesiąc ostatniej synchronizacji
#define FAS_TSYNC_DAY       		FA_SYS_VAR+2	//1U dzień ostatniej synchronizacji
#define FAS_TSYNC_HOU       		FA_SYS_VAR+3	//1U
#define FAS_TSYNC_MIN				FA_SYS_VAR+4
#define FAS_TSYNC_SEC       		FA_SYS_VAR+5
#define FAS_TSYNC_DIF       		FA_SYS_VAR+6	//1U ostatnia różnica czasu synchronizacji
#define	FAS_ENERGIA1	    		FA_SYS_VAR+7    //4D energia pobrana z pakietu 1
#define	FAS_ENERGIA2        		FA_SYS_VAR+11  	//4D energia pobrana z pakietu 2
#define	FAS_CISN_01	    			FA_SYS_VAR+15   //4D ciśnienie zerowe czujnika bezwzględnego 1
#define	FAS_CISN_02	    			FA_SYS_VAR+19   //4D ciśnienie zerowe czujnika bezwzględnego 2




//Waypointy. Każdy zajmuje 12 bajtów. Do końca pamięci jest miejsca na 2389 waypointów
#define FA_USER_WAYPOINTS    0x1000
#define FAU_WP00_


#define FA_END      0x7FFF  //ostatni bajt pamięci konfiguracji (FM25V02 - 256kb)

//////////////////////////////////////////////////////////////////////////////
// Definicje validatorów danych konfiguracyjnych
//////////////////////////////////////////////////////////////////////////////
#define VMIN_PRZES_ACEL		-0.5f	//max wartość odchyłki ujemnej przesunięcia akcelerometrów
#define VMAX_PRZES_ACEL  	0.5f    //j.w. dodatniej
#define VDOM_PRZES_ACEL  	0.0f    //wartość domyślna
#define VMIN_PRZES_ZYRO  	-2.0f   //max wartość odchyłki ujemnej przesunięcia żyroskopów
#define VMAX_PRZES_ZYRO  	2.0f    //j.w. dodatniej
#define VDOM_PRZES_ZYRO  	0.0f    //wartość domyślna

#define VMIN_MNOZNIK_ACEL  	0.8f    //limity wartości odchyłki ujemnej wzmocnienia akcelerometrów
#define VMAX_MNOZNIK_ACEL  	1.4f    //limity wartości odchyłki dodatniej
#define VDOM_MNOZNIK_ACEL  	1.0f    //wartość domyślna
#define VMIN_MNOZNIK_ZYRO  	0.7f    //limity wartości odchyłki ujemnej wzmocnienia żyroskopów ISZ i IDG
#define VMAX_MNOZNIK_ZYRO  	1.3f    //limity wartości odchyłki dodatniej
#define VDOM_MNOZNIK_ZYRO  	1.0f    //wartość domyślna

#define VMIN_MNOZNIK_PABS   0.75f    //dolny limit wartosci MNOZNIKwania czujnika ciśnienia bezwzględnego
#define VMAX_MNOZNIK_PABS   1.25f    //
#define VDOM_MNOZNIK_PAB    1.00f

#define VMIN_SKLADNIK_PDIF	-500.0f    //limity wartości odchyłki offsetu różnicowego czujnika ciśnienia w [Pa]
#define VMAX_SKLADNIK_PDIF	500.0f    //
#define VDOM_SKLADNIK_PDIF	0.0f

#define VMIN_MNOZNIK_MAGN	0.010f		//limity wartości MNOZNIKwania pomiaru magnetometru
#define VMAX_MNOZNIK_MAGN	100.0f
#define VDOM_MNOZNIK_MAGN	1.0f

#define VMIN_SKLADNIK_MAGN	-0.01f		//limity wartości przesunięcia pomiaru magnetometru +/- 10 mT
#define VMAX_SKLADNIK_MAGN	0.01f
#define VDOM_SKLADNIK_MAGN	0.0f

#define VMIN_PID_WZMP    	(float)0.0     //limity wartości wzmocnienienia członu P regulatora
#define VMAX_PID_WZMP    	(float)1000
#define VDOM_PID_WZMP    	(float)1.0

#define VMIN_PID_WZMI    	(float)0.0     //limity wartości wzmocnienienia członu I regulatora
#define VMAX_PID_WZMI    	(float)1000
#define VDOM_PID_WZMI    	(float)0.0

#define VMIN_PID_WZMD    	(float)0.0    //limity wartości wzmocnienienia członu D regulatora
#define VMAX_PID_WZMD    	(float)1000
#define VDOM_PID_WZMD    	(float)0.0

#define VMIN_PID_SLEWR   	(float)1.0     //limity prędkości narastania sygnału regulatora 0 [%/s]
#define VMAX_PID_SLEWR   	(float)10000    //max 100% / 0,01s
#define VDOM_PID_SLEWR   	(float)10000

#define VMIN_PID_ILIM    	(float)0.0     //limit wartości całki członu całkującego regulatora PID
#define VMAX_PID_ILIM    	(float)100     //max 100%
#define VDOM_PID_ILIM    	(float)23

#define VMIN_PID_MINWY   	(float)-100.0    //minimalna wartość wyjścia
#define VMAX_PID_MINWY   	(float)100.0
#define VDOM_PID_MINWY   	(float)-100.0

#define VMIN_PID_MAXWY   	(float)-100.0    //maksymalna wartość wyjścia
#define VMAX_PID_MAXWY   	(float)100.0
#define VDOM_PID_MAXWY   	(float)100.0

#define VMIN_PID_MNOZWZ 	(float)0.0001    //MNOZNIKwanie wartości zadanej
#define VMAX_PID_MNOZWZ 	(float)1000.0
#define VDOM_PID_MNOZWZ 	(float)0.2

#define VMIN_MIX_PRZEPOCH	-1000.0f    //składowe pochylenia i przechylenia długości ramienia koptera w mikserze [mm], max 1m
#define VMAX_MIX_PRZEPOCH   1000.0f
#define VDOM_MIX_PRZEPOCH   0.0f

#define VMIN_MIX_ODCH		-1.0f    //współczynnik wpływu kierunku obrotów silnika na odchylenie
#define VMAX_MIX_ODCH    	1.0f
#define VDOM_MIX_ODCH    	0.0f

#define VMIN_STRPID_MIN 	(float)0.0001    //minimalna wartość parametru do strojenia PID
#define VMAX_STRPID_MIN 	(float)100.0
#define VDOM_STRPID_MIN 	(float)0.01

#define VMIN_STRPID_MAX 	(float)0.001    //maksymalna wartość parametru do strojenia PID
#define VMAX_STRPID_MAX 	(float)1000.0
#define VDOM_STRPID_MAX 	(float)100.0

#define VMIN_STRPID_KRC		4    //numer kanału RC do strojenia PID
#define VMAX_STRPID_KRC 	KANALY_ODB_RC
#define VDOM_STRPID_KRC 	5

#define VMIN_STRPID_PAR		STRP_NIC    //numer strojonego parametru PID
#define VMAX_STRPID_PAR		LICZBA_STROJONYCH_PARAMETROW_PID
#define VDOM_STRPID_PAR		STRP_NIC

/*
#define VALM_SPGAIN       (float)0.0001  //limity wartości  współczynnika wzmocnienia sygnału zadanego z aparatury
#define VALP_SPGAIN       (float)100.0
#define VALD_SPGAIN       (float)1.0    //wartość domyślna*/


#define VMIN_MNOZNIK_WE_ADC		0.0001f   //mnożnik napiecia z czujników analogowych
#define VMAX_MNOZNIK_WE_ADC 	100.0f
#define VDOM_MNOZNIK_WE_ADC 	1.0f

#define VMIN_SKLADNIK_WE_ADC	-1000.0f   //składnik napiecia z czujników analogowych
#define VMAX_SKLADNIK_WE_ADC	1000.0f
#define VDOM_SKLADNIK_WE_ADC	0.0f


#define VMIN_PID_STWYPRZ		-100.0f 		//stała wartość podawana na wejscie wyprzedzające. Domyślnie jest to kąt w radianach stałego pochylenia lub wysokość
#define VMAX_PID_STWYPRZ 		100.0f
#define VDOM_PID_STWYPRZ		0.0f
