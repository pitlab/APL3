//////////////////////////////////////////////////////////////////////////////
//
// Definicje zmiennych konfguracyjnych trzymanych w pamięci FRAM FM25CL64
// Zakres adresów do 0x7FF
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
//adresy zmiennych konfiguracyjnych w zakresie 0..0x2000  (FA == Fram Address)
//Indeksy w komentarzu oznaczaja rozmiar (liczba) i typ (litera) zmiennej
//Typy zmiennych: 
//FAU - zmienna użytkownika, zawiera jego nastawy
//FAS - zmienna systemowa lub dynamiczna, nie dotykać sama się modyfikuje
//FAH - zmienna zawierająca parametry sprzetu - chronić przed przypadkową zmianą
//
//Formaty liczb. Cyfra przed literą oznacza rozmiar w bajtach:
// U - liczba całkowita bez znaku.
// S - liczba całkowita ze znakiem.
// CH - znak alfanumeryczny
// F - liczba float

#define FA_USER_VAR	    	0x0000	    //zmienne użytkownika
#define FAU_WE_RC1_MIN		0x0000		//16*2U minimalna wartość każdego kanału z odbiornika RC1
#define FAU_WE_RC1_MAX		0x0020		//16*2U maksymalna wartość każdego kanału z odbiornika RC1
#define FAU_WE_RC2_MIN		0x0040		//16*2U minimalna wartość każdego kanału z odbiornika RC2
#define FAU_WE_RC2_MAX		0x0060		//16*2U maksymalna wartość każdego kanału z odbiornika RC2

#define FAU_CH6_MIN         0x0080 		//4F minimalna wartość regulowanej zmiennej
#define FAU_CH6_MAX         0x0084      //4F maksymalna wartość regulowanej zmiennej
#define FAU_CH6_FUNCT       0x0088      //1U funkcja kanału 6: rodzaj zmiennej do regulacji
#define FAU_CH7_MIN         0x0089		//4F minimalna wartość regulowanej zmiennej
#define FAU_CH7_MAX         0x008D      //4F maksymalna wartość regulowanej zmiennej
#define FAU_CH7_FUNCT       0x0091      //1U funkcja kanału 7: rodzaj zmiennej do regulacji

#define FAU_PWM_JALOWY      0x0092     	//2U wysterowanie regulatorów na biegu jałowym [us]
#define FAU_PWM_MIN      	0x0094 		//2U minimalne wysterowanie regulatorów w trakcie lotu [us]
#define FAU_WPM_ZAWISU      0x0096   	//2U wysterowanie regulatorów w zawisie [us]
#define FAU_PWM_MAX         0x0098  	//2U maksymalne wysterowanie silników w trakcie lotu [us]

#define FAU_TSYNC_MON       0x009A     	//1U miesiąc ostatniej synchronizacji
#define FAU_TSYNC_DAY       0x009B 		//1U dzień ostatniej synchronizacji
#define FAU_TSYNC_HOU       0x009C		//1U
#define FAU_TSYNC_MIN       0x009D
#define FAU_TSYNC_SEC       0x009E
#define FAU_TSYNC_DIF       0x009F 		//1U ostatnia różnica czasu synchronizacji

//mikser
#define FAU_MIX_PRZECH		0x00A0		//8*4F współczynnik wpływu przechylenia na dany silnik
#define FAU_MIX_POCHYL  	0x00C0   	//8*4F współczynnik wpływu pochylenia na dany silnik
#define FAU_MIX_ODCHYL    	0x00E0   	//8*4F współczynnik wpływu odchylenia na dany silnik

//regulatory PID
#define FA_USER_PID	    	0x0100
#define FAU_PID_P0          FA_USER_PID+0   //4F wzmocnienienie członu P regulatora 0
#define FAU_PID_I0          FA_USER_PID+4   //4F wzmocnienienie członu I regulatora 0
#define FAU_PID_D0          FA_USER_PID+8   //4F wzmocnienienie członu D regulatora 0
#define FAU_PID_OGR_I0      FA_USER_PID+12  //4F ograniczenie wartości całki członu I regulatora 0
#define FAU_PID_MIN_WY0		FA_USER_PID+16  //4F minimalna wartość wyjścia
#define FAU_PID_MAX_WY0		FA_USER_PID+20  //4F maksymalna wartość wyjścia
#define FAU_SKALA_WZADANEJ0 FA_USER_PID+24	//4F skalowanie wartości zadanej
#define FAU_FILTRD_TYP 		FA_USER_PID+28  //1U Stała czasowa filtru członu różniczkującego (bity 0..5), wyłączony (bit 6), Regulator kątowy (bit 7)
#define FAU_PID1	        FA_USER_PID+29  //1U nic
#define FAU_PID2			FA_USER_PID+30	//1U nic
#define FAU_PID3			FA_USER_PID+31	//1U nic
#define ROZMIAR_REG_PID		32

//12 regulatorów zajmuje 336 bajtów - 0x180
#define FAU_TRYB_REG	    0x0280		//6*1U Tryb pracy regulatorów 4 podstawowych wartości przypisanych do drążków i 2 regulatorów pozycji N i E
#define FAU_KAN_DRAZKA_RC	0x0286		//4*1U Numer kanału przypisany do funkcji drążka aparatury: przechylenia, pochylenia, odchylenia i wysokości
#define FAU_FUNKCJA_KAN_RC	0x028A		//12*1U Numer funkcji przypisanej do kanału RC (5..16)

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

//wzmocnienia drążków aparatury dla poszczególnych trybów pracy regulatorów
//#define FAU_ZADANA_AKRO     0x0316		//4x4F wartość zadana z drążków aparatury dla regulatora Akro
//#define FAU_ZADANA_STAB     0x0326		//4x4F wartość zadana z drążków aparatury dla regulatora Stab



#define FA_SYS_VAR	    	0x0400	    //zmienne systemowe i dynamiczne
#define	FAS_NUMER_SESJI	  	0x0400		//1U Numer sesji
#define	FAS_ENERGIA1	    0x0401    	//4D energia pobrana z pakietu 1
#define	FAS_ENERGIA2        0x0405   	//4D energia pobrana z pakietu 2
#define	FAS_CISN_01	    	0x0409    	//4D ciśnienie zerowe czujnika bezwzględnego 1
#define	FAS_CISN_02	    	0x040D    	//4D ciśnienie zerowe czujnika bezwzględnego 2


#define FA_HARD_VAR         	0x0500	    //zmienne definiujące parametry sprzętu 0x500 = 1280

#define	FAH_MOD_AKCEL       	FA_HARD_VAR     	//akcelerometry na module inercyjnym dowolnego typu (wspólna konfiguracja)
#define	FAH_AKCEL1_X_PRZES   	FAH_MOD_AKCEL+0   	//4F korekcja przesuniecia w osi X akcelerometru 1
#define	FAH_AKCEL1_Y_PRZES   	FAH_MOD_AKCEL+4   	//4F korekcja przesuniecia w osi Y akcelerometru 1
#define	FAH_AKCEL1_Z_PRZES  	FAH_MOD_AKCEL+8   	//4F korekcja przesuniecia w osi Z akcelerometru 1
#define	FAH_AKCEL1_X_WZMOC     	FAH_MOD_AKCEL+12  	//4F korekcja skalowanie w osi X akcelerometru 1
#define	FAH_AKCEL1_Y_WZMOC     	FAH_MOD_AKCEL+16  	//4F korekcja skalowanie w osi Y akcelerometru 1
#define	FAH_AKCEL1_Z_WZMOC		FAH_MOD_AKCEL+20  	//4F korekcja skalowanie w osi Z akcelerometru 1
#define	FAH_AKCEL2_X_PRZES   	FAH_MOD_AKCEL+24   	//4F korekcja przesuniecia w osi X akcelerometru 2
#define	FAH_AKCEL2_Y_PRZES   	FAH_MOD_AKCEL+28   	//4F korekcja przesuniecia w osi Y akcelerometru 2
#define	FAH_AKCEL2_Z_PRZES  	FAH_MOD_AKCEL+32   	//4F korekcja przesuniecia w osi Z akcelerometru 2
#define	FAH_AKCEL2_X_WZMOC     	FAH_MOD_AKCEL+36  	//4F korekcja skalowanie w osi X akcelerometru 2
#define	FAH_AKCEL2_Y_WZMOC     	FAH_MOD_AKCEL+40  	//4F korekcja skalowanie w osi Y akcelerometru 2
#define	FAH_AKCEL2_Z_WZMOC		FAH_MOD_AKCEL+44  	//4F korekcja skalowanie w osi Z akcelerometru 2

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
#define	FAH_ZYRO1P_WZMOC     	FAH_MOD_ZYRO1+36  	//4F korekcja skalowanie żyroskopu 1P
#define	FAH_ZYRO1Q_WZMOC     	FAH_MOD_ZYRO1+40  	//4F korekcja skalowanie żyroskopu 1Q
#define	FAH_ZYRO1R_WZMOC     	FAH_MOD_ZYRO1+44  	//4F korekcja skalowanie żyroskopu 1R

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
#define	FAH_ZYRO2P_WZMOC     	FAH_MOD_ZYRO2+36  	//4F korekcja skalowanie żyroskopu 2P
#define	FAH_ZYRO2Q_WZMOC     	FAH_MOD_ZYRO2+40  	//4F korekcja skalowanie żyroskopu 2Q
#define	FAH_ZYRO2R_WZMOC     	FAH_MOD_ZYRO2+44  	//4F korekcja skalowanie żyroskopu 2R

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
#define	FAH_MAGN1        		FAH_MOD_CIS_ROZ+24
#define	FAH_MAGN1_PRZESX  		FAH_MAGN1+0 //4H przesunięcie magnetometru w osi X
#define	FAH_MAGN1_PRZESY  		FAH_MAGN1+4 //4H przesunięcie magnetometru w osi Y
#define	FAH_MAGN1_PRZESZ  		FAH_MAGN1+8 //4H przesunięcie magnetometru w osi Z
#define	FAH_MAGN1_SKALOX  		FAH_MAGN1+12 //4H skalowanie w osi X
#define	FAH_MAGN1_SKALOY  		FAH_MAGN1+16 //4H skalowanie w osi Y
#define	FAH_MAGN1_SKALOZ  		FAH_MAGN1+20 //4H skalowanie w osi Z

//współczynniki kalibracji magnetometru 2 na module inercyjnym
#define	FAH_MAGN2        		FAH_MAGN1+24
#define	FAH_MAGN2_PRZESX  		FAH_MAGN2+0 //4H przesunięcie magnetometru w osi X
#define	FAH_MAGN2_PRZESY  		FAH_MAGN2+4 //4H przesunięcie magnetometru w osi Y
#define	FAH_MAGN2_PRZESZ  		FAH_MAGN2+8 //4H przesunięcie magnetometru w osi Z
#define	FAH_MAGN2_SKALOX  		FAH_MAGN2+12 //4H skalowanie w osi X
#define	FAH_MAGN2_SKALOY  		FAH_MAGN2+16 //4H skalowanie w osi Y
#define	FAH_MAGN2_SKALOZ  		FAH_MAGN2+20 //4H skalowanie w osi Z

//współczynniki kalibracji magnetometru 3 na module GNSS
#define	FAH_MAGN3        		FAH_MAGN2+24
#define	FAH_MAGN3_PRZESX  		FAH_MAGN3+0 //4H przesunięcie magnetometru w osi X
#define	FAH_MAGN3_PRZESY  		FAH_MAGN3+4 //4H przesunięcie magnetometru w osi Y
#define	FAH_MAGN3_PRZESZ  		FAH_MAGN3+8 //4H przesunięcie magnetometru w osi Z
#define	FAH_MAGN3_SKALOX  		FAH_MAGN3+12 //4H skalowanie w osi X
#define	FAH_MAGN3_SKALOY  		FAH_MAGN3+16 //4H skalowanie w osi Y
#define	FAH_MAGN3_SKALOZ  		FAH_MAGN3+20 //4H skalowanie w osi Z

#define	FAH_SKALO_CISN_BEZWZGL1	FAH_MAGN3+24    //współczynnik skalowania ciśnienia bezwzględnego czujnika 1
#define	FAH_SKALO_CISN_BEZWZGL2	FAH_MAGN3+28    //współczynnik skalowania ciśnienia bezwzględnego czujnika 2
#define FAH_TEST				FAH_MAGN3+32	//4H testowy zapis do pamięci w celu sprawdzenia poprawnosci





//Waypointy. Każdy zajmuje 12 bajtów. Do końca pamięci jest miejsca na 2581 waypointów
#define FA_USER_WAYPOINTS    0x0700
#define FAU_WP00_


#define FA_END      0x1FFF  //ostatni bajt pamięci konfiguracji (FM25CL64 - 64kb)

//////////////////////////////////////////////////////////////////////////////
// Definicje validatorów danych konfiguracyjnych
//////////////////////////////////////////////////////////////////////////////
#define VMIN_OFST_ACEL	-0.5f	//max wartość odchyłki ujemnej przesunięcia akcelerometrów
#define VMAX_OFST_ACEL  0.5f    //j.w. dodatniej
#define VDOM_OFST_ACEL  0.0f    //wartość domyślna
#define VMIN_PRZES_ZYRO  -2.0f   //max wartość odchyłki ujemnej przesunięcia żyroskopów
#define VMAX_PRZES_ZYRO  2.0f    //j.w. dodatniej
#define VDOM_PRZES_ZYRO  0.0f    //wartość domyślna

#define VMIN_SKALO_ACEL  	0.8f    //limity wartości odchyłki ujemnej wzmocnienia akcelerometrów
#define VMAX_SKALO_ACEL  	1.4f    //limity wartości odchyłki dodatniej
#define VDOM_SKALO_ACEL  	1.0f    //wartość domyślna
#define VMIN_SKALO_ZYRO  	0.7f    //limity wartości odchyłki ujemnej wzmocnienia żyroskopów ISZ i IDG
#define VMAX_SKALO_ZYRO  	1.3f    //limity wartości odchyłki dodatniej
#define VDOM_SKALO_ZYRO  	1.0f    //wartość domyślna

#define VMIN_SKALO_PABS   0.75f    //dolny limit wartosci skalowania czujnika ciśnienia bezwzględnego
#define VMAX_SKALO_PABS   1.25f    //
#define VDOM_SKALO_PAB    1.00f

#define VMIN_PRZES_PDIF    -500.0f    //limity wartości odchyłki offsetu różnicowego czujnika ciśnienia w [Pa]
#define VMAX_PRZES_PDIF    500.0f    //
#define VDOM_PRZES_PDIF    0.0f

#define VMIN_SKALO_MAGN    	0.010f		//limity wartości skalowania pomiaru magnetometru
#define VMAX_SKALO_MAGN    	100.0f
#define VDOM_SKALO_MAGN    	1.0f

#define VMIN_PRZES_MAGN    	-0.01f		//limity wartości przesunięcia pomiaru magnetometru +/- 10 mT
#define VMAX_PRZES_MAGN    	0.01f
#define VDOM_PRZES_MAGN    	0.0f

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

#define VMIN_PID_SKALAWZ 	(float)0.0001    //skalowanie wartości zadanej
#define VMAX_PID_SKALAWZ 	(float)1000.0
#define VDOM_PID_SKALAWZ 	(float)0.2

#define VMIN_MIX_PRZEPOCH	-1000.0f    //składowe pochylenia i przechylenia długości ramienia koptera w mikserze [mm], max 1m
#define VMAX_MIX_PRZEPOCH   1000.0f
#define VDOM_MIX_PRZEPOCH   0.0f

#define VMIN_MIX_ODCH		-1.0f    //współczynnik wpływu kierunku obrotów silnika na odchylenie
#define VMAX_MIX_ODCH    	1.0f
#define VDOM_MIX_ODCH    	0.0f

/*
#define VALM_SPGAIN       (float)0.0001  //limity wartości  współczynnika wzmocnienia sygnału zadanego z aparatury
#define VALP_SPGAIN       (float)100.0
#define VALD_SPGAIN       (float)1.0    //wartość domyślna*/
