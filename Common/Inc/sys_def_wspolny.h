//flagi inicjalizacji sprzętu CM4
#define INIT_WYKR_MTK		0x00000001
#define INIT_WYKR_UBLOX		0x00000002
#define INIT_GNSS_GOTOWY	0x00000004
#define INIT_HMC5883		0x00000008
#define INIT_MS5611			0x00000010
#define INIT_BMP581			0x00000020
#define INIT_ICM42688		0x00000040
#define INIT_LSM6DSV		0x00000080
#define INIT_MMC34160		0x00000100
#define INIT_IIS2MDC		0x00000200
#define INIT_ND130			0x00000400
#define INIT_MS4525			0x00000800
#define INIT_P0_MS5611		0x00001000	//ustawiono ciśnienie P0 dla czujnika 1
#define INIT_P0_BMP851		0x00002000	//ustawiono ciśnienie P0 dla czujnika 2
#define INIT_P0_ND140		0x00004000	//wyzerowano prędkość dla ND130
#define INIT_P0_MS4525		0x00008000	//wyzerowano prędkość dla MS4525
#define INIT_TRWA_KALZ_ZYRO	0x00010000	//trwa kalibracj żyroskopu na zimno
#define INIT_TRWA_KALP_ZYRO	0x00020000	//trwa kalibracj żyroskopu w temp. pokojowej
#define INIT_TRWA_KALG_ZYRO	0x00040000	//trwa kalibracj żyroskopu na gorąco


#define KELVIN		273.15f

//definicje temperatur kalibracji żyroskopów
#define TEMP_KAL_ZIMNO		(10.f + KELVIN)
#define TEMP_KAL_POKOJ		(25.f + KELVIN)
#define TEMP_KAL_GORAC		(40.f + KELVIN)
#define TEMP_KAL_ODCHYLKA	5.f

//indeksy pola temperatura w uni wymiany CM4
//temepratury:	0=MS5611, 1=BMP851, 2=ICM42688, 3=LSM6DSV, 4=IIS2MDC, 5=ND130, 6=MS4525
#define TEMP_BARO1	0
#define TEMP_BARO2	1
#define TEMP_IMU1	2
#define TEMP_IMU2	3
#define TEMP_MAG1	4
#define TEMP_CISR1	5	//czujnik ciśnienia różnicowego
#define TEMP_CISR2	6	//czujnik ciśnienia różnicowego

#define CZAS_KALIBRACJI_ZYROSKOPU		1000	//obiegów pętli głównej po 5ms



