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
//FAS - zmienna systemowa lub dynamiczna, nie dotykać sama się modyfikuje
//FAH - zmienna zawierająca parametry sprzetu - chronić przed przypadkową zmianą
//FAU - zmienna użytkownika, zawiera jego nastawy
//
//Formaty liczb:
// U - liczba całkowita bez znaku. Cyfra przed literą oznacza rozmiar w bajtach
// S - liczba całkowita ze znakiem.
// CH - znak alfanumeryczny
// F - liczba float

#define FA_USER_VAR	    0x0000	    //zmienne użytkownika

#define FAU_TELEMETRY_USB   FA_USER_VAR+104 //64U konfiguracja częstotliwości wysyłania ramek telemetrycznych przez USB
#define FAU_TELEMETRY_MOD   FA_USER_VAR+168 //64U -,,- przez modem
#define FAU_TURBO_FRAME     FA_USER_VAR+232 //2U definicja danych turbo ramki dla USB i modemu
//zostawiam trochę miejsca na ewentualną rozbudowę

#define FA_USER_VAR2	    0x0100	    //zmienne użytkownika 2
#define FAU_RCCH_MIN        FA_USER_VAR2    //2x8 minimalna wartość sygnału z danego kanału odbiornika
#define FAU_RCCH_MAX        FAU_RCCH_MIN+16 //2x8 maksymalna wartość sygnału z danego kanału odbiornika

#define FAU_LOG_CONF1       0x0120          //4I konfiguracja logera 1
#define FAU_LOG_CONF2       FAU_LOG_CONF1+4 //4I konfiguracja logera 2
#define FAU_LOG_CONF3       FAU_LOG_CONF2+4 //4I konfiguracja logera 3
#define FAU_LOG_CONF4       FAU_LOG_CONF3+4 //4I konfiguracja logera 4
#define FAU_LOG_CONF5       FAU_LOG_CONF4+4 //4I konfiguracja logera 5
#define FAU_LOG_CONF6       FAU_LOG_CONF5+4 //4I konfiguracja logera 6
#define FAU_LOG_CONF7       FAU_LOG_CONF6+4 //4I konfiguracja logera 7
#define FAU_LOG_FREQ        FAU_LOG_CONF7+4 //1CH częstotliwość logowania
#define FAU_LOG_NAME        FAU_LOG_FREQ+1  //12CH nazwa modelu w nazwie pliku logu
#define FAU_LANDING_SPD     FAU_LOG_NAME+13 //4F prędkość lądowania
//zostało 182 bajty

#define FA_USER_VAR3	    0x0200                //zmienne użytkownika 3 - mikser
#define FAU_MIX_PITCH       FA_USER_VAR3          //12*4F współczynnik wpływu pochylenia na dany silnik
#define FAU_MIX_ROLL        FAU_MIX_PITCH+48      //12*4F współczynnik wpływu przechylenia na dany silnik
#define FAU_MIX_YAW         FAU_MIX_ROLL+48       //12*4F współczynnik wpływu odchylenia na dany silnik
#define FAU_LOW_VOLT_WARN   FAU_MIX_YAW+48        //4F próg ostrzezenia o niskim napięciu
#define FAU_LOW_VOLT_ALARM  FAU_LOW_VOLT_WARN+4   //4F próg alarmu niskiego napięcia
#define FAU_VOLT_DROP_COMP  FAU_LOW_VOLT_ALARM+4  //4F współczynnik kompensacji spadku napięcia pakietu

#define FAU_CH6_MIN         FAU_VOLT_DROP_COMP+4  //4U minimalna wartość regulowanej zmiennej
#define FAU_CH6_MAX         FAU_CH6_MIN+4         //4U maksymalna wartość regulowanej zmiennej
#define FAU_CH6_FUNCT       FAU_CH6_MAX+4         //1U funkcja kanału 6: rodzaj zmiennej do regulacji
#define FAU_CH7_MIN         FAU_CH6_FUNCT+4       //4U minimalna wartość regulowanej zmiennej
#define FAU_CH7_MAX         FAU_CH7_MIN+4         //4U maksymalna wartość regulowanej zmiennej
#define FAU_CH7_FUNCT       FAU_CH7_MAX+4         //1U funkcja kanału 7: rodzaj zmiennej do regulacji

#define FAU_MODE_CONF       FAU_CH7_FUNCT+1       //1UC tryb pracy automatyki kalibracji
#define FAU_VDROP_COMP      FAU_MODE_CONF+4       //4U kompensacja spadku napięcia na  pakiecie [us/V]
#define FAU_PNE_SEN1_GAIN   FAU_VDROP_COMP+4      //4F wzmocnienie czujnika na module pneumatycznym
#define FAU_PNE_SEN1_OFST   FAU_PNE_SEN1_GAIN+4   //4F offset czujnika na module pneumatycznym
#define FAU_PNE_SEN2_GAIN   FAU_PNE_SEN1_OFST+4   //4F wzmocnienie czujnika na module pneumatycznym
#define FAU_PNE_SEN2_OFST   FAU_PNE_SEN2_GAIN+4   //4F offset czujnika na module pneumatycznym
//zostało 58 bajtów

#define FAU_TSYNC_MON       0x290           //miesiąc ostatniej synchronizacji
#define FAU_TSYNC_DAY       FAU_TSYNC_MON+1 //dzień ostatniej synchronizacji
#define FAU_TSYNC_HOU       FAU_TSYNC_DAY+1
#define FAU_TSYNC_MIN       FAU_TSYNC_HOU+1
#define FAU_TSYNC_SEC       FAU_TSYNC_MIN+1
#define FAU_TSYNC_DIF       FAU_TSYNC_SEC+1 //ostatnia różnica czasu synchronizacji

#define FA_USER_PID	    	0x0300
#define FAU_PID_P0          FA_USER_PID+0   //4U wzmocnienienie członu P regulatora 0
#define FAU_PID_I0          FA_USER_PID+4   //4U wzmocnienienie członu I regulatora 0
#define FAU_PID_D0          FA_USER_PID+8   //4U wzmocnienienie członu D regulatora 0
#define FAU_PID_OGR_I0      FA_USER_PID+12  //4U ograniczenie wartości całki członu I regulatora 0
#define FAU_PID_MIN_WY0		FA_USER_PID+16  //4U minimalna wartość wyjścia
#define FAU_PID_MAX_WY0		FA_USER_PID+20  //4U maksymalna wartość wyjścia
#define FAU_FILTRD_TYP 		FA_USER_PID+24  //1U Stała czasowa filtru członu różniczkującego (bity 0..5), wyłączony (bit 6), Regulator kątowy (bit 7)
#define FAU_PID1	        FA_USER_PID+25  //1U nic
#define FAU_PID2			FA_USER_PID+26	//1U nic
#define FAU_PID3			FA_USER_PID+27	//1U nic
#define ROZMIAR_REG_PID		28
//do adresu 0x3C0 jest miejsce na łącznie 12 regulatorów.





//8 wolnych bajtów
#define FAU_MIN_PWM         0x3D8           //2U minimalne wysterowanie regulatorów w trakcie lotu [us]
#define FAU_HOVER_PWM       FAU_MIN_PWM+2   //2U wysterowanie regulatorów w zawisie [us]
#define FAU_IDLE_PWM        FAU_HOVER_PWM+2 //2U wysterowanie regulatorów na biegu jałowym [us]
#define FAU_MAX_PWM         FAU_IDLE_PWM+2  //2U maksymalne wysterowanie silników w trakcie lotu [us]

//wzmocnienia drążków aparatury dla posczeg�lnych tryb�w pracy regulatorów
#define FAU_SP_GAIN         0x3E0
#define FAU_SPG_ACRO        FAU_SP_GAIN+0   //16U wzmocnienie drążków dla regulatora Acro
#define FAU_SPG_STAB        FAU_SPG_ACRO+16 //16U wzmocnienie drążków dla regulatora Stab
#define FAU_SPG_GSPD        FAU_SPG_STAB+16 //8U wzmocnienie drążków dla regulatora prędkości GPS
#define FAU_SPG_GPOS        FAU_SPG_GSPD+8  //8U wzmocnienie drążków dla regulatora pozycji GPS


#define FA_SYS_VAR	    0x0420	    //zmienne systemowe i dynamiczne
#define	FAS_SESSION_NUMBER  FA_SYS_VAR+0    //1S Numer sesji
#define	FAS_PRESSURE	    FA_SYS_VAR+4    //4D ciśnienie zerowe czujnika bezwzględnego
#define	FAS_ENERGY1	    FA_SYS_VAR+8    //4D energia pobrana z pakietu 1
#define	FAS_ENERGY2         FA_SYS_VAR+12   //4D energia pobrana z pakietu 2




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





/*
#define	FAH_PDVARIO         FAH_ADCIO7_GAIN+4
#define	FAH_PDIF_OFFSET	    FAH_PDVARIO+0   //4H offset napięcia czujnika ciśnienia różnicowego
#define	FAH_VARIO_OFFSET    FAH_PDVARIO+4   //4H offset napięcia członu różniczkującego
#define	FAH_PDIF_GAIN	    FAH_PDVARIO+8   //4H korekcja wzmocnienia napięcia czujnika ciśnienia różnicowego
#define	FAH_VARIO_GAIN      FAH_PDVARIO+12  //4H korekcja wzmocnienia napięcia członu różniczkującego

#define	FAH_POWER           FAH_VARIO_GAIN+4
#define	FAH_CURR1_GAIN      FAH_POWER+0     //4H korekcja wzmocnienia pomiaru prądu czujnikiem 1
#define	FAH_CURR2_GAIN      FAH_POWER+4     //4H korekcja wzmocnienia pomiaru prądu czujnikiem 2
#define	FAH_VOLT0_GAIN      FAH_POWER+8     //4H korekcja wzmocnienia pomiaru napięcia na płycie głównej
#define	FAH_VOLT1_GAIN      FAH_POWER+12    //4H korekcja wzmocnienia pomiaru napięcia czujnikiem 1
#define	FAH_VOLT2_GAIN      FAH_POWER+16    //4H korekcja wzmocnienia pomiaru napięcia czujnikiem 1

#define	FAH_TEMPER          FAH_VOLT2_GAIN+4
#define	FAH_TPNE_OFFSET	    FAH_TEMPER+4    //4H korekcja offsetu temperatury przetwornika A/C modułu pneumatycznego
#define	FAH_ADCIO_TEMP_OFST FAH_TEMPER+8    //4H korekcja offsetu temperatury przetwornika A/C modułu ADCIO
#define	FAH_TCOUP_REF_OFST  FAH_TEMPER+12   //4H korekcja offsetu czujnika temperatury odniesienia termopar
#define	FAH_TALTI_AIO_OFST  FAH_TEMPER+24   //4H korekcja offsetu temperatury wysoko�ciomierza na module All-In-One
*/




//
#define FAH_VARIO_OFFSET_CPL FAH_MAG_HMC+32

//Waypointy. Każdy zajmuje 12 bajtów. Do końca pamięci jest miejsca na 2581 waypointów
#define FA_USER_WAYPOINTS    0x0700
#define FAU_WP00_


#define FA_END      0x1FFF  //ostatni bajt pamięci konfiguracji (FM25CL64 - 64kb)

//////////////////////////////////////////////////////////////////////////////
// Definicje validator�w danych konfiguracyjnych
//////////////////////////////////////////////////////////////////////////////
/*#define VALM_OFST_UNI    (float)-0.3    //max wartość unwersalnej odchyłki ujemnej offsetu
#define VALP_OFST_UNI    (float)0.3     //
#define VALD_OFST_UNI    (float)0.0 
#define VALM_GAIN0_UNI   (float)-0.3    //max wartość unwersalnej odchyłki ujemnej wzmocnienia dla średniej 0
#define VALP_GAIN0_UNI   (float)0.3     //
#define VALD_GAIN0_UNI   (float)1.0
#define VALM_GAIN1_UNI   (float)0.8     //max wartość unwersalnej odchyłki ujemnej wzmocnienia dla średniej 1
#define VALP_GAIN1_UNI   (float)1.2     //
#define VALD_GAIN1_UNI   (float)1.0*/

#define VMIN_OFST_ACEL	-0.5f	//max wartość odchyłki ujemnej przesunięcia akcelerometrów
#define VMAX_OFST_ACEL  0.5f    //j.w. dodatniej
#define VDEF_OFST_ACEL  0.0f    //wartość domyślna
#define VMIN_PRZES_ZYRO  -2.0f   //max wartość odchyłki ujemnej przesunięcia żyroskopów
#define VMAX_PRZES_ZYRO  2.0f    //j.w. dodatniej
#define VDEF_PRZES_ZYRO  0.0f    //wartość domyślna

#define VMIN_SKALO_ACEL  	0.8f    //limity wartości odchyłki ujemnej wzmocnienia akcelerometrów
#define VMAX_SKALO_ACEL  	1.4f    //limity wartości odchyłki dodatniej
#define VDEF_SKALO_ACEL  	1.0f    //wartość domyślna
#define VMIN_SKALO_ZYRO  	0.7f    //limity wartości odchyłki ujemnej wzmocnienia żyroskopów ISZ i IDG
#define VMAX_SKALO_ZYRO  	1.3f    //limity wartości odchyłki dodatniej
#define VDEF_SKALO_ZYRO  	1.0f    //wartość domyślna

/*#define VALM_TCOEF_ACEL   (float)-5.0   //minimalna wartość  współczynnika temperaturowego akcelerometru
#define VALP_TCOEF_ACEL   (float)5.0    //wartość maksymalna
#define VALD_TCOEF_ACEL   (float)1.0    //wartość domyślna

#define VALM_TCOEF_GYRO   (float)-5.0   //minimalna wartość  współczynnika temperaturowego żyroskopu
#define VALP_TCOEF_GYRO   (float)5.0    //wartość maksymalna
#define VALD_TCOEF_GYRO   (float)1.0    //wartość domyślna

#define VALM_TCOEF_MAG   (float)-5.0    //minimalna wartość  współczynnika temperaturowego megnetometru
#define VALP_TCOEF_MAG   (float)5.0     //wartość maksymalna
#define VALD_TCOEF_MAG   (float)1.0     //wartość domyślna*/


#define VMIN_SKALO_PABS   0.75f    //dolny limit wartosci skalowania czujnika ciśnienia bezwzględnego
#define VMAX_SKALO_PABS   1.25f    //
#define VDEF_SKALO_PAB    1.00f


#define VMIN_PRZES_PDIF    -500.0f    //limity wartości odchyłki offsetu różnicowego czujnika ciśnienia w [Pa]
#define VMAX_PRZES_PDIF    500.0f    //
#define VDEF_PRZES_PDIF    0.0f
/*#define VALM_OFST_VARI    (float)-0.8   //limity wartości odchyłki offsetu wariometru
#define VALP_OFST_VARI    (float)0.8    //
#define VALD_OFST_VARI    (float)0.0

#define VALM_GAIN_PDIF    (float)0.8    //limity wartości odchyłki wzmocnienia różnicowego czujnika ciśnienia
#define VALP_GAIN_PDIF    (float)1.2    //
#define VALD_GAIN_PDIF    (float)1.0
#define VALM_GAIN_VARI    (float)0.8    //limity wartości odchyłki wzmocnienia wariometru
#define VALP_GAIN_VARI    (float)1.2    //
#define VALD_GAIN_VARI    (float)1.0

#define VALM_OFST_TEMP    (float)-3.0   //limity wartości odchyłki offsetu temperatury normalnych czujników
#define VALP_OFST_TEMP    (float)3.0    //
#define VALD_OFST_TEMP    (float)0.0 

#define VALM_OFST_TMPM    (float)-25.0  //limity wartości odchyłki offsetu temperatury przetworników A/C
#define VALP_OFST_TMPM    (float)25.0   //
#define VALD_OFST_TMPM    (float)0.0

#define VALM_TCOM_ALTI    (float)-1.2    //limity wartości odchyłki kompensacji temperaturowej wysokościomierzy
#define VALP_TCOM_ALTI    (float)1.2     //
#define VALD_TCOM_ALTI    (float)1.0 */

#define VMIN_SKALO_MAGN    0.010f		//limity wartości skalowania pomiaru magnetometru
#define VMAX_SKALO_MAGN    100.0f
#define VDEF_SKALO_MAGN    1.0f

#define VMIN_PRZES_MAGN    -0.01f		//limity wartości przesunięcia pomiaru magnetometru +/- 10 mT
#define VMAX_PRZES_MAGN    0.01f
#define VDEF_PRZES_MAGN    0.0f

#define VMIN_PID_WZMP    (float)0.0     //limity wartości wzmocnienienia członu P regulatora
#define VMAX_PID_WZMP    (float)1000
#define VDEF_PID_WZMP    (float)1.0

#define VMIN_PID_WZMI    (float)0.0     //limity wartości wzmocnienienia członu I regulatora
#define VMAX_PID_WZMI    (float)1000
#define VDEF_PID_WZMI    (float)0.0

#define VMIN_PID_WZMD    (float)0.0    //limity wartości wzmocnienienia członu D regulatora
#define VMAX_PID_WZMD    (float)1000
#define VDEF_PID_WZMD    (float)0.0

#define VMIN_PID_SLEWR    (float)1.0     //limity prędkości narastania sygnału regulatora 0 [%/s]
#define VMAX_PID_SLEWR    (float)10000    //max 100% / 0,01s
#define VDEF_PID_SLEWR    (float)10000

#define VMIN_PID_ILIM    (float)0.0     //limit wartości całki członu całkującego regulatora PID
#define VMAX_PID_ILIM    (float)100     //max 100%
#define VDEF_PID_ILIM    (float)23 /*

#define VMIN_ADIO_GAIN    (float)0.01   //limity wzmocnienia dla modułu ADCIO
#define VMAX_ADIO_GAIN    (float)100
#define VDEF_ADIO_GAIN    (float)1      //wartość domyślna

#define VALM_ADIO_OFST    (float)-100   //limity offsetu dla modułu ADCIO
#define VALP_ADIO_OFST    (float)100
#define VALD_ADIO_OFST    (float)0      //wartość domyślna

#define VALM_AXX_COEF     (float)80.0    //limity wartości  współczynnika A linearyzacji wysoko�ci
#define VALP_AXX_COEF     (float)180.0 
#define VALD_AXX_COEF     (float)84.3    //wartość domyślna

#define VALM_BXX_COEF     (float)-4000.0 //limity wartości  współczynnika B linearyzacji wysoko�ci
#define VALP_BXX_COEF     (float)0.0
#define VALD_BXX_COEF     (float)-0.8    //wartość domyślna

#define VALM_VDROP_COMP   (float)0.01   //limity wartości  współczynnika B linearyzacji wysoko�ci
#define VALP_VDROP_COMP   (float)100.0
#define VALD_VDROP_COMP   (float)5.8    //wartość domyślna

#define VALM_SPGAIN       (float)0.0001  //limity wartości  współczynnika wzmocnienia sygnału zadanego z aparatury
#define VALP_SPGAIN       (float)100.0
#define VALD_SPGAIN       (float)1.0    //wartość domyślna*/
