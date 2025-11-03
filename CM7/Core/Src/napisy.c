//////////////////////////////////////////////////////////////////////////////
//
// Zestaw napisów wyświetlanych na LCD
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

#include "napisy.h"


#define JEZYK_POLSKI
#ifdef JEZYK_POLSKI


const char *chNapisLcd[MAX_NAPISOW]  = {
"AutoPitLot hv3.0",				//STR_WITAJ_TYTUL		max 17 znaków
"By m%cc mie%c w r%cj Wron%cw na pohybel wra%cym hordom",		//STR_WITAJ_MOTTO: %c będą podmieniane na polskie znaki diakrytyczne
"pl",								//STR_WITAJ_DOMENA		max 3 znaki
"wykryto",							//STR_SPRAWDZ_WYKR
"brakuje",							//STR_SPRAWDZ_BRAK
"Flash NOR 32MB",					//STR_SPRAWDZ_FLASH_NOR
"Flash QSPI 16MB",					//STR_SPRAWDZ_FLASH_QSPI
"Kamera OV5642", 					//STR_SPRAWDZ_KAMERA_OV5642
"nic",								//STR_SPRAWDZ_8
"nic",								//STR_SPRAWDZ_9

"IMU -> MS5611",					//STR_SPRAWDZ_IMU1_MS5611
"IMU -> BMP581",					//STR_SPRAWDZ_IMU1_BMP581
"IMU -> ICM42688",					//STR_SPRAWDZ_IMU1_ICM42688
"IMU -> LSM6DSV", 					//STR_SPRAWDZ_IMU1_LSM6DSV
"IMU -> MMC34160",					//STR_SPRAWDZ_IMU1_MMC34160
"IMU -> IIS2MDC",					//STR_SPRAWDZ_IMU1_IIS2MDC
"IMU -> ND130",						//STR_SPRAWDZ_IMU1_ND130
"HMC5883",							//STR_SPRAWDZ_HMC5883
"Karta SD", 						//STR_SPRAWDZ_KARTA_SD
"GNSS",								//STR_SPRAWDZ_GNSS

"MTK",								//STR_SPRAWDZ_MTK
"Ublox",							//STR_SPRAWDZ_UBLOX
"nic",								//STR_SPRAWDZ_
"Dotyk:",							//STR_TEST_TOUCH
"Menu Glowne",						//STR_MENU_MAIN			max 17 znaków
"nic",								//STR_MENU_PROTOCOLS	max 17 znaków
"nic",								//STR_MENU_MULMETR		max 17 znaków
"nic",								//STR_MENU_TEST			max 17 znaków
"Menu Ustawien",					//STR_MENU_SETINGS
"Menu audio",						//STR_MENU_MEDIA_AUDIO

"Menu Wydajnosc",					//STR_MENU_WYDAJNOSC
"Menu Karta SD",					//STR_MENU_KARTA_SD
"Menu kalibracji IMU",				//STR_MENU_IMU
"Menu Magnetometr",					//STR_MENU_MAGNETOMETR
"Kalibracja",						//STR_KALIBRACJA
"Weryfikacja",						//STR_WERYFIKACJA
"magnetometru",						//STR_MAGNETOMETRU
"Magnetometr",						//STR_MAGNETOMETR
"Ekstrema",							//STR_EKSTREMA
"Magn",								//STR_MAGN - skrótowa nazwa magnetometru
"Kal",								//STR_KAL - skrótowa nazwa kalibracji
"By m%cc zm%cc wra%ce hordy rojem Wron%cw",	//STR_WITAJ_MOTTO2
"Menu kalibracji czujnikow",		//STR_MENU_KALIBRACJE
"Menu pomiarow", 					//STR_MENU_POMIARY
"Menu nastw podsystemow",	 		//STR_MENU_NASTAWY
"Nastawy PID",						//STR_NASTAWY_PID
"przechylenia",						//STR_PRZECHYLENIA
"pochylenia",						//STR_POCHYLENIA
"odchylenia",						//STR_ODCHYLENIA
"wysokosci",						//STR_WYSOKOSCI
"nawigacji N",						//STR_NAWIGACJI_N
"nawigacji E",						//STR_NAWIGACJI_E
"Menu kamera",						//STR_MENU_KAMERA
"Menu Ethernet",					//STR_MENU_ETHERNET
"Dotknij krzyzyk aby skalibrowac ekran",	//STR_DOTKNIJ_ABY_SKALIBROWAC
"Menu OSD",							//STR_MENU_OSD
};


const char *chOpisBledow[MAX_KOMUNIKATOW] = {
"Blad wykonania polecenia!",
"Wdus ekran i trzymaj aby zakonczyc",
"Zbyt niska temeratura zyroskopow wynoszaca %.0f%cC. Musi miescic sie w granicach od %.0f%cC do %.0f%cC",
"Zbyt wysoka temeratura zyroskopow wynoszaca %.0f%cC. Musi miescic sie w granicach od %.0f%cC do %.0f%cC",
};

const char *chNazwyMies3Lit[13]  = {"---", "Sty","Lut", "Mar", "Kwi", "Maj", "Cze", "Lip", "Sie", "Wrz", "Paz", "Lis", "Gru"};

#endif //JEZYK_POLSKI
