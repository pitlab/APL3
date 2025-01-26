/*
 * napisy.h
 *
 *  Created on: Nov 19, 2024
 *      Author: PitLab
 */

#ifndef INC_NAPISY_H_
#define INC_NAPISY_H_

#define MAX_NAPISU_WYKRYCIE			20		//do formatowania napisów przy przezentacji wykrywanego sprzętu

//definicje napisów
#define STR_WITAJ_TYTUL				0
#define STR_WITAJ_MOTTO				1
#define STR_WITAJ_DOMENA			2
#define STR_SPRAWDZ_WYKR			3
#define STR_SPRAWDZ_BRAK			4
#define STR_SPRAWDZ_FLASH_NOR		5
#define STR_SPRAWDZ_FLASH_QSPI		6
#define STR_SPRAWDZ_KAMERA_OV5642	7
#define STR_SPRAWDZ_IMU1			8
#define STR_SPRAWDZ_GNSS			9
#define STR_SPRAWDZ_KARTA_SD		10
#define STR_SPRAWDZ_1				11
#define STR_SPRAWDZ_2				12
#define STR_SPRAWDZ_3				13
#define STR_SPRAWDZ_4				14
#define STR_SPRAWDZ_5				15
#define STR_SPRAWDZ_6				16
#define STR_SPRAWDZ_7				17
#define STR_SPRAWDZ_8				18
#define STR_TEST_TOUCH				19

#define STR_LIBEL_TITLE				20
#define STR_MENU_MAIN				21
#define STR_MENU_PROTOCOLS			22
#define STR_MENU_SBUS				23
#define STR_MENU_MULMETR			24
#define STR_MENU_MULTOOL			25
#define STR_MENU_VIBR				26
#define STR_MENU_TEST				27
#define STR_MENU_SETINGS			28
#define STR_MENU_MULTI_MEDIA		29

#define STR_MENU_WYDAJNOSC			30
#define STR_KONIEC					31
#define MAX_NAPISOW					32	//liczba napisów


#define JEZYK_POLSKI
#ifdef JEZYK_POLSKI


const char *chNapisLcd[MAX_NAPISOW]  = {
"AutoPitLot hv3.0",				//STR_WITAJ_TYTUL		max 17 znaków
"By m%cc mie%c w r%cj Wron%cw na pohybel wra%cym hordom",		//STR_WITAJ_MOTTO: %c będą podmieniane na polskie znaki
"pl",								//STR_WITAJ_DOMENA		max 3 znaki
"wykryto",							//STR_SPRAWDZ_WYKR
"brakuje",							//STR_SPRAWDZ_BRAK
"Flash NOR 32MB",					//STR_SPRAWDZ_FLASH_NOR
"Flash QSPI 16MB",					//STR_SPRAWDZ_FLASH_QSPI
"Kamera OV5642", 					//STR_SPRAWDZ_KAMERA_OV5642
"Modul IMU",						//STR_SPRAWDZ_IMU1
"Modul GNSS",						//STR_SPRAWDZ_GNSS

"Karta SD", 						//STR_SPRAWDZ_KARTA_SD
"Modul ",  							//STR_SPRAWDZ_1
"Modul ",  							//STR_SPRAWDZ_2
"Modul ",							//STR_SPRAWDZ_3
"Modul ",							//STR_SPRAWDZ_4
"Modul ",							//STR_SPRAWDZ_5
"Modul ",							//STR_SPRAWDZ_6
"Modul ",							//STR_SPRAWDZ_7
"Modul ",							//STR_SPRAWDZ_8
"Dotyk:",							//STR_TEST_TOUCH

"nic",								//STR_LIBEL_TITLE
"Menu Glowne",						//STR_MENU_MAIN			max 17 znaków
"nic",								//STR_MENU_PROTOCOLS	max 17 znaków
"nic",								//STR_MENU_SBUS			max 17 znaków
"nic",								//STR_MENU_MULMETR		max 17 znaków
"nic",								//STR_MENU_MULTOOL		max 17 znaków
"nic",								//STR_MENU_VIBR			max 17 znaków
"nic",								//STR_MENU_TEST			max 17 znaków
"Menu Ustawien",					//STR_MENU_SETINGS
"Menu MultiMedia",					//STR_MENU_MULTI_MEDIA

"Menu Wydajnosc"					//STR_MENU_WYDAJNOSC
"koniec"							//STR_KONIEC
};

const char *chNazwyMies3Lit[13]  = {"---", "Sty","Lut", "Mar", "Kwi", "Maj", "Cze", "Lip", "Sie", "Wrz", "Paz", "Lis", "Gru"};

#endif //JEZYK_POLSKI
#endif /* INC_NAPISY_H_ */
