/*
 * napisy.h
 *
 *  Created on: Nov 19, 2024
 *      Author: PitLab
 */

#ifndef INC_NAPISY_H_
#define INC_NAPISY_H_


#define STR_WITAJ_TYTUL				0
#define STR_WITAJ_KASZANA			1
#define STR_WITAJ_DOMENA			2
#define STR_SPRAWDZ_WYKR			3
#define STR_SPRAWDZ_BRAK			4
#define STR_SPRAWDZ_FLASH_NOR		5
#define STR_SPRAWDZ_FLASH_QSPI		6

#define STR_SPRAWDZ_KAMERA_OV5642	7
#define STR_SPRAWDZ_MODUL_IMU		8


#define STR_TEST_TOUCH		9
#define STR_LIBEL_TITLE		10

#define STR_MENU_MAIN		11
#define STR_MENU_PROTOCOLS	12
#define STR_MENU_SBUS		13
#define STR_MENU_MULMETR	14
#define STR_MENU_MULTOOL	15
#define STR_MENU_VIBR		16
#define STR_MENU_TEST		17
#define STR_MENU_SETINGS	18

#define MAX_LCD_STR			20

#define JEZYK_POLSKI
#ifdef JEZYK_POLSKI
const char *chNapisLcd[MAX_LCD_STR]  = {
"AutoPitLot v3.0",					//STR_WITAJ_TYTUL		max 17 znaków
"z technologia \"--Kaszana_OFF\"",	//STR_WITAJ_KASZANA
"pl",								//STR_WITAJ_DOMENA		max 3 znaki
"wykryto",							//STR_SPRAWDZ_WYKR
"brakuje",							//STR_SPRAWDZ_BRAK
"Flash NOR 32MB",					//STR_SPRAWDZ_FLASH_NOR
"Flash QSPI 16MB",					//STR_SPRAWDZ_FLASH_QSPI
"Kamera OV5642", 					//STR_SPRAWDZ_KAMERA_OV5642
"Modul IMU",						//STR_SPRAWDZ_MOD_IMU	7

"Dotyk:",							//STR_TEST_TOUCH

"Zapisana kalib.",					//STR_COMPS_STORED
"Menu Glowne",						//STR_MENU_MAIN			max 17 znaków

"Menu Protokoly",					//STR_MENU_PROTOCOLS	max 17 znaków
"Menu S-Bus",						//STR_MENU_SBUS			max 17 znaków
"Menu Multimetr",					//STR_MENU_MULMETR		max 17 znaków
"Menu Niezbednik",					//STR_MENU_MULTOOL		max 17 znaków
"Menu Drgan i ADC",					//STR_MENU_VIBR			max 17 znaków
"Menu Testow",						//STR_MENU_TEST			max 17 znaków
"Menu Ustawien"						//STR_MENU_SETINGS
};

#endif //JEZYK_POLSKI
#endif /* INC_NAPISY_H_ */
