/*
 * napisy.h
 *
 *  Created on: Nov 19, 2024
 *      Author: PitLab
 */

#ifndef INC_NAPISY_H_
#define INC_NAPISY_H_


#define STR_INVITE_TITE		0
#define STR_INVITE_KASZANA	1
#define STR_INVITE_DOMAIN	2
#define STR_INVITE_ACCEL	3
#define STR_INVITE_MAGNET	4
#define STR_INVITE_IRTHER	5
#define STR_INVITE_PRESENT	6
#define STR_INVITE_ABSENT	7
#define STR_TEST_ENCODER	8
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

#define LANG_POL
#ifdef LANG_POL
const char *chNapisLcd[MAX_LCD_STR]  = {
"Tester modelarski",				//STR_INVITE_TITE		max 17 znaków
"z technologia \"--Kaszana_OFF\"",	//STR_INVITE_KASZANA
"pl",								//STR_INVITE_DOMAIN		max 3 znaki
"Akcelerometr",						//STR_INVITE_ACCEL
"Magnetometr ",						//STR_INVITE_MAGNET
"Termometr IR",						//STR_INVITE_IRTHER
"wykryto",							//STR_INVITE_PRESENT
"brak",								//STR_INVITE_ABSENT
"Enkoder",							//STR_TEST_ENCODER
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

#endif //LANG_POL
#endif /* INC_NAPISY_H_ */
