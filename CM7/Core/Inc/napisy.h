/*
 * napisy.h
 *
 *  Created on: Nov 19, 2024
 *      Author: PitLab
 */

#ifndef INC_NAPISY_H_
#define INC_NAPISY_H_

#define MAX_NAPISU_WYKRYCIE			21		//do formatowania napisów przy przezentacji wykrywanego sprzętu

//definicje napisów
#define STR_WITAJ_TYTUL				0
#define STR_WITAJ_MOTTO				1
#define STR_WITAJ_DOMENA			2
#define STR_SPRAWDZ_WYKR			3
#define STR_SPRAWDZ_BRAK			4
#define STR_SPRAWDZ_FLASH_NOR		5
#define STR_SPRAWDZ_FLASH_QSPI		6
#define STR_SPRAWDZ_KAMERA_OV5642	7
#define STR_SPRAWDZ_8				8
#define STR_SPRAWDZ_9				9

#define STR_SPRAWDZ_IMU1_MS5611		10
#define STR_SPRAWDZ_IMU1_BMP581		11
#define STR_SPRAWDZ_IMU1_ICM42688	12
#define STR_SPRAWDZ_IMU1_LSM6DSV	13
#define STR_SPRAWDZ_IMU1_MMC34160	14
#define STR_SPRAWDZ_IMU1_IIS2MDC	15
#define STR_SPRAWDZ_IMU1_ND130		16
#define STR_SPRAWDZ_HMC5883			17
#define STR_SPRAWDZ_KARTA_SD		18
#define STR_SPRAWDZ_GNSS			19


#define STR_SPRAWDZ_UBLOX			20
#define STR_SPRAWDZ_MTK				21
#define STR_SPRAWDZ_				22
#define STR_TEST_TOUCH				23
#define STR_MENU_MAIN				24
#define STR_MENU_PROTOCOLS			25
#define STR_MENU_MULMETR			26
#define STR_MENU_TEST				27
#define STR_MENU_SETINGS			28
#define STR_MENU_MULTI_MEDIA		29

#define STR_MENU_WYDAJNOSC			30
#define STR_MENU_KARTA_SD			31
#define STR_MENU_KALIBRACJE			32
#define STR_KONIEC					33
#define MAX_NAPISOW					34	//liczba napisów


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
"Menu MultiMedia",					//STR_MENU_MULTI_MEDIA

"Menu Wydajnosc",					//STR_MENU_WYDAJNOSC
"Menu Karta SD",					//STR_MENU_KARTA_SD
"Menu Kalibracje",					//STR_MENU_KALIBRACJE
"koniec",							//STR_KONIEC
};



//definicje komunikatów o błędach
#define KOMUNIKAT_NAGLOWEK		0
#define KOMUNIKAT_STOPKA		1
#define KOMUNIKAT_ZA_ZIMNO		2
#define KOMUNIKAT_ZA_CIEPLO		3
#define MAX_KOMUNIKATOW			4

const char *chOpisBledow[MAX_KOMUNIKATOW] = {
"Blad wykonania polecenia!",
"Wdus ekran i trzymaj aby zakonczyc",
"Zbyt niska temeratura zyroskopow wynoszaca %.0f%cC. Musi miescic sie w granicach od %.0f%cC do %.0f%cC",
"Zbyt wysoka temeratura zyroskopow wynoszaca %.0f%cC. Musi miescic sie w granicach od %.0f%cC do %.0f%cC",
};


const char *chNazwyMies3Lit[13]  = {"---", "Sty","Lut", "Mar", "Kwi", "Maj", "Cze", "Lip", "Sie", "Wrz", "Paz", "Lis", "Gru"};

#endif //JEZYK_POLSKI
#endif /* INC_NAPISY_H_ */
