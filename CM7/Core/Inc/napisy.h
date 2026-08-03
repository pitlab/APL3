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
#define STR_AUDIO					29

#define STR_WYDAJNOSC				30
#define STR_KARTA_SD				31
#define STR_IMU						32
#define STR_MAGNETOMETR				33
#define STR_KALIBRACJA				34
#define STR_WERYFIKACJA				35
#define STR_MAGNETOMETRU			36
#define STR_MENU					37
#define STR_EKSTREMA				38
#define STR_MAGN					39
#define STR_KAL						40
#define STR_WITAJ_MOTTO2			41
#define STR_KALIBRACJE				42
#define STR_POMIARY					43
#define STR_NASTAWY					44
#define STR_NASTAWY_PID				45
#define STR_PRZECHYLENIA			46
#define STR_POCHYLENIA				47
#define STR_ODCHYLENIA				48
#define STR_WYSOKOSCI				49
#define STR_NAWIGACJI_N				50
#define STR_NAWIGACJI_E				51
#define STR_KAMERA					52
#define STR_ETHERNET				53
#define STR_DOTKNIJ_ABY_SKALIBROWAC	54
#define STR_OSD						55
#define STR_PITLAB					56
#define STR_EXIF					57
#define STR_JPEG					58
#define STR_TESTY					59
#define STR_DANE_ODBIORNIKA_RC		60
#define STR_DANE_WYJSC_RC			61
#define STR_AKCELETOMETR			62
#define STR_ZYROSKOP				63
#define STR_TAK						64
#define STR_NIE						65

//#define STR_
#define MAX_NAPISOW					66	//liczba napisów




//definicje komunikatów o błędach
#define KOMUNIKAT_NAGLOWEK			0
#define KOMUNIKAT_DUS_I_TRZYMAJ		1
#define KOMUNIKAT_ZA_ZIMNO			2
#define KOMUNIKAT_ZA_CIEPLO			3
#define MAX_KOMUNIKATOW				4


//definicje nazw pozycji rejestratora. Obecność XD w nazwie oznacza parametr %d, który ma być wypełniony w funkcji sprintf(), XC oznacza parametr %c
#define NREJ_CZAS_GGMMSSSS			0
#define NREJ_CISNIENIE_BZWZGL_XD_PA	1
#define NREJ_WYSOKOSC_MSL_XD_M		2
#define NREJ_WYSOKOSC_AGL_XD_M		3
#define NREJ_WARIOMETR_XD_MS		4
#define NREJ_CISN_ROZNICOWE_XD_PA	5
#define NREJ_PREDK_IAS_XD_MS		6
#define NREJ_TEMP_BARO_XD_K			7
#define NREJ_TEMP_ROZN_XD_K			8
#define NREJ_BAT_XD_NAPIECIE_V		9
#define NREJ_BAT_XD_PRAD_A			10
#define NREJ_BAT_XD_ENER_POBR_MAH	11
#define NREJ_ZASIL_XD_NAPIECIE_V	12
#define NREJ_CZUJ_ZEWN_XD_V			13
#define NREJ_TEMP_CPU_K				14
#define NREJ_SERWA_NAPIECIE_V		15
#define NREJ_ZYRO_SUR_XD_XC_RADS	16
#define NREJ_ZYRO_KAL_XD_XC_RADS	17
#define NREJ_AKCEL_XD_XC_MS2		18
#define NREJ_MAGNETO_XD_XC_GAUSS	19
#define NREJ_TEMP_IMU_XD_K			20
#define NREJ_KAT_KALM_IMU_XC_RAD	21
#define NREJ_KAT_KOMP_IMU_XC_RAD	22
#define NREJ_KAT_KWAT_IMU_XC_RAD	23
#define NREJ_KAT_AKCE_IMU_XC_RAD	24
#define NREJ_KAT_ZYRO_IMU_XC_RAD	25
#define NREJ_SZEROKOSC_GEO_RAD		26
#define NREJ_DLUGOSC_GEO_RAD		27
#define NREJ_WYSOKOSC_GNSS_M		28
#define NREJ_PREDKOSC_WZGL_ZIEMI_MS	29
#define NREJ_KURS_GNSS_RAD			30
#define NREJ_LICZBA_SATELITOW		31
#define NREJ_VDOP_M					32
#define NREJ_HDOP_M					33
#define NREJ_PREDK_GNSS_N_MS		34
#define NREJ_PREDK_GNSS_E_MS		35
#define NREJ_ODBIORNIKRC_KAN_XD		36
#define NREJ_WYJSCIERC_KAN_XD		37
#define NREJ_REG_KATA_PRZE			38
#define NREJ_REG_PRED_PRZE			39
#define NREJ_REG_KATA_POCH			40
#define NREJ_REG_PRED_POCH			41
#define NREJ_REG_KATA_ODCH			42
#define NREJ_REG_PRED_ODCH			43
#define NREJ_REG_WYSOKOSCI			44
#define NREJ_REG_PR_ZM_WYS			45
#define NREJ_REG_KATA_GEON			46
#define NREJ_REG_PRED_GEON			47
#define NREJ_REG_KATA_GEOE			48
#define NREJ_REG_PRED_GEOE			49
#define NREJ_WART_ZADANA			50
#define NREJ_FILTR_WZAD				51
#define NREJ_WART_WEJSCIOWA			52
#define NREJ_FILTR_WWEJ				53
#define NREJ_FILTR_ROZN				54
#define NREJ_WYJ_P					55
#define NREJ_WYJ_I					56
#define NREJ_WYJ_D					57
#define NREJ_WYJ_WYPRZ				58
#define NREJ_WYJSCIE				59


#define LICZBA_NAZW_POZYCJI_REJESTRATORA	60

#endif /* INC_NAPISY_H_ */
