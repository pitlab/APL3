//flagi inicjalizacji sprzętu CM4
#define INIT_WYKR_MTK			0x00000001
#define INIT_WYKR_UBLOX			0x00000002
#define INIT_GNSS_GOTOWY		0x00000004
#define INIT_HMC5883			0x00000008
#define INIT_MS5611				0x00000010
#define INIT_BMP581				0x00000020
#define INIT_ICM42688			0x00000040
#define INIT_LSM6DSV			0x00000080
#define INIT_MMC34160			0x00000100
#define INIT_IIS2MDC			0x00000200
#define INIT_ND130				0x00000400
#define INIT_MS4525				0x00000800
#define INIT_P0_MS5611			0x00001000	//ustawiono ciśnienie P0 dla czujnika 1
#define INIT_P0_BMP851			0x00002000	//ustawiono ciśnienie P0 dla czujnika 2
#define INIT_P0_ND140			0x00004000	//wyzerowano prędkość dla ND130
#define INIT_P0_MS4525			0x00008000	//wyzerowano prędkość dla MS4525
#define INIT_TRWA_KAL_ZYRO_ZIM	0x00010000	//trwa kalibracja zera żyroskopu na zimno
#define INIT_TRWA_KAL_ZYRO_POK	0x00020000	//trwa kalibracja zera żyroskopu w temp. pokojowej
#define INIT_TRWA_KAL_ZYRO_GOR	0x00040000	//trwa kalibracja zera żyroskopu na gorąco
#define INIT_WYK_KAL_WZM_ZYRO	0x00080000	//wykonano kalibrację wzmocnienia żyroskopu w danej osi


#define RAD2DEG				(180/M_PI)
#define DEG2RAD				(M_PI/180)
#define KELVIN				273.15f
#define AKCEL1G				9.80665f			//przelicznik z [g] na [m/s^2]
#define NOMINALNE_MAGN		50.0e-6f			//nominalna wartość natężenia pola magnetycznego w Teslach dla Polski centralnej. Źródło: https://www.magnetic-declination.com/ oraz https://www.ncei.noaa.gov/sites/g/files/anmtlf171/files/inline-images/F.jpg
#define INKLINACJA_MAG		(68.f * DEG2RAD)	//inklinacja magnetyczna w radianach. Źródło: https://www.magnetic-declination.com/ lub https://www.ncei.noaa.gov/sites/g/files/anmtlf171/files/inline-images/I.jpg
#define DEKLINACJA_MAG		(6.5f * DEG2RAD)	//deklinacja magnetyczna w radianach. Źródło: https://www.magnetic-declination.com/ lub https://www.ncei.noaa.gov/sites/g/files/anmtlf171/files/inline-images/D.jpg
#define PROMIEN_ZIEMI		6371008.77f			//promień Ziemi w metrach

#define KANALY_SERW		16		//liczba sterowanych kanałów serw
#define KANALY_ODB_RC	16		//liczba odbieranych kanałów na każdym z dwu wejść odbiorników RC
#define KANALY_MIKSERA	8		//liczba kanałów wyjściowych, któe mogą wchodzić do miksera

//definicje kanałów RC
#define KANRC_POCH		0
#define KANRC_PRZE		1
#define KANRC_ODCH		2
#define KANRC_GAZ		3

//przelicznik liczby mikrosekund sygnału PPM na 1 procent wysterowania kanału RC
#define PPM1PROC_UNI   8       //1% = 8us unipolarnie, 0%=1,1us 100%=1,9us
#define PPM1PROC_BIP   4       //1% = 4us bipolarnie, -100%=1,1us 0%=1,5us +100%=1,9us

//czasy impulsów PPM dla założenia że 1% to 4us
#define PPM_MIN			1000    //wartość minimalna sygnału -125%
#define PPM_M100    	1100    //-100%
#define PPM_M90     	1140    //-90%
#define PPM_JALOWY  	1200    //obroty jałowe silników
#define PPM_M75     	1200    //-75%
#define PPM_M50     	1300    //-50%
#define PPM_M25     	1400    //-25%
#define PPM_M20     	1420    //-20%
#define PPM_NEUTR   	1500    //neutrum 1500us
#define PPM_P20     	1580    //+20%
#define PPM_P25     	1600    //+25%
#define PPM_P50     	1700    //+50%
#define PPM_P75     	1800    //+75%
#define PPM_P90     	1860    //+90%
#define PPM_P100    	1900    //+100%
#define PPM_MAX			2000    //wartość maksymalna sygnału +125%

#define OKRES_PWM		2500	//okres kanału PWM w mikrosekundach
#define PRZERWA_PPM		3000	//przerwa między paczkami impulsów PPM odbiornika RC
//#define PPM_SPACE   	20     	//odstęp między kolejnymi impulsami PPM [us]

//definicje typów sygnału z odbiornika RC w konfiguracji FRAM FAU_KONF_ODB_RC
#define ODB_RC_SBUS		1
#define ODB_RC_PPM		2
#define MASKA_TYPU_RC1	0x03	//te bity definiują rodzaj sygnału z odbiornika RC1
#define MASKA_TYPU_RC2	0x30	//te bity definiują rodzaj sygnału z odbiornika RC2

//definicje typów sygnału 2 pierwszych serw
#define SERWO_IO		0	//wyjście skonfigurowane jako wjściowy port IO do debugowania algorytmów
#define SERWO_SBUS		1	//wyjście S-Bus
#define SERWO_PWM400	2	//wyjście PWM 400Hz
#define SERWO_PWM50		3	//wyjście PWM 50Hz



//stany 3-położeniowych przełaczników na kanałach 5, 6 i 7
#define RC_PRZEL_MIN 0	//minus 100%
#define RC_PRZEL_M50 1	//minus 50%
#define RC_PRZEL_NEU 2	//neutrum kanału
#define RC_PRZEL_P50 3	//plus 50%
#define RC_PRZEL_PLU 4	//plus 100%


//definicje temperatur kalibracji żyroskopów
#define TEMP_KAL_ZIMNO		(10.f + KELVIN)
#define TEMP_KAL_POKOJ		(25.f + KELVIN)
#define TEMP_KAL_GORAC		(40.f + KELVIN)
#define TEMP_KAL_ODCHYLKA	5.f
#define OBR_KAL_WZM			3	//liczba obrótów podczas kalibrcji wzmocnienia żyroskopów

//indeksy pola temperatura w uni wymiany CM4
//temepratury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=ND130, 5=MS4525
#define TEMP_BARO1	0
#define TEMP_BARO2	1
#define TEMP_IMU1	2
#define TEMP_IMU2	3
#define TEMP_CISR1	4	//czujnik ciśnienia różnicowego
#define TEMP_CISR2	5	//czujnik ciśnienia różnicowego

#define CZAS_KALIBRACJI		1000	//obiegów pętli głównej po 5ms

//definicje sekwencera kalibracji wzmocnienia żyro
#define SEKW_KAL_WZM_ZYRO_R		0
#define SEKW_KAL_WZM_ZYRO_Q		1
#define SEKW_KAL_WZM_ZYRO_P		2
//#define SEKW_KAL_SKAL_CISN		3


//identyfikatory kalibrowanych magnetometrów
#define MAG1			0x10
#define MAG2			0x20
#define MAG3			0x30
#define KALIBRUJ		0x08	//sygnalizuje trwanie kalibracji w przeciwieństwie do etapu sprawdzenia
#define ZERUJ			0x04	//sygnalizuje potrzebę wyzerowania ekstremów wskazań magnetometru przed pomiarem
#define MASKA_OSI		0x03
#define MASKA_CZUJNIKA	0xF0
//#define NORM_AMPL_MAG	1000		//znormalizowana długość wektora magnetometru


#define WYSOKOSC10PIETER	27.0f	//wysokość w metrach 10 pięter


//definicje trybów lotu
#define TRLOT_BEZPIECZNY	0	//regulatory i nawigacja wyłączone
//do wartości 9 tryby naziemne
#define TRLOT_UZBROJONY		10	//silniki pracują na biegu jałowym
#define TRLOT_WZNOSZENIE	11	//kopter wznosi się na nominalnej prędkości pionowej
#define TRLOT_LOT_RECZNY	12	//lot sterowany przez pilota
#define TRLOT_LOT_AUTO		13	//lot autonomiczny
#define TRLOT_PODEJ_LAD		14	//podejście do lądowania
#define TRLOT_LADOWANIE		15	//lądowanie autonomiczne

//definicje flag trybu lotu dotyczących zmiennej stWymianyCM4.chFlagiLotu
#define WL_TRWA_LOT			0x01
