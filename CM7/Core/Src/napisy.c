//////////////////////////////////////////////////////////////////////////////
//
// Zestaw napisów wyświetlanych na LCD
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

#include <Napisy.h>

#define JEZYK_POLSKI
#ifdef JEZYK_POLSKI


const char *cNapisLcd[MAX_NAPISOW]  = {
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
"Glowne",							//STR_MENU_MAIN			max 17 znaków
"nic",								//STR_MENU_PROTOCOLS	max 17 znaków
"nic",								//STR_MENU_MULMETR		max 17 znaków
"nic",								//STR_MENU_TEST			max 17 znaków
"Menu Ustawien",					//STR_MENU_SETINGS
"Audio",							//STR_AUDIO

"Wydajnosc",						//STR_WYDAJNOSC
"Karta SD",							//STR_KARTA_SD
"IMU",								//STR_IMU
"Magnetometr",						//STR_MAGNETOMETR
"Kalibracja",						//STR_KALIBRACJA
"Weryfikacja",						//STR_WERYFIKACJA
"magnetometru",						//STR_MAGNETOMETRU
"Menu",								//STR_MENU
"Ekstrema",							//STR_EKSTREMA
"Magn",								//STR_MAGN - skrótowa nazwa magnetometru
"Kal",								//STR_KAL - skrótowa nazwa kalibracji
"By m%cc zm%cc wra%ce hordy rojem Wron%cw",	//STR_WITAJ_MOTTO2
"kalibracji czujnikow",				//STR_KALIBRACJE
"Pomiary",		 					//STR_POMIARY
"nastwy podsystemow",	 			//STR_NASTAWY
"Nastawy PID",						//STR_NASTAWY_PID
"przechylenia",						//STR_PRZECHYLENIA
"pochylenia",						//STR_POCHYLENIA
"odchylenia",						//STR_ODCHYLENIA
"wysokosci",						//STR_WYSOKOSCI
"nawigacji N",						//STR_NAWIGACJI_N
"nawigacji E",						//STR_NAWIGACJI_E
"Kamera",							//STR_KAMERA
"Ethernet",							//STR_ETHERNET
"Dotknij krzyzyk aby skalibrowac ekran",	//STR_DOTKNIJ_ABY_SKALIBROWAC
"OSD",								//STR_OSD
"PitLab",							//STR_PITLAB
"Exif",								//STR_EXIF
"JPEG",								//STR_JPEG
"Testy",							//STR_TESTY
"Dane odbiornika RC",				//STR_DANE_ODBIORNIKA_RC
"Kanaly ESC i serw",				//STR_DANE_WYJSC_RC
"Akcelerometr",						//STR_AKCELETOMETR
"Zyroskop",							//STR_ZYROSKOP
"Tak",								//STR_TAK
"Nie", 								//STR_NIE
};


const char *cOpisBledow[MAX_KOMUNIKATOW] = {
"Blad wykonania polecenia!",
"Wdus ekran i trzymaj aby zakonczyc",
"Zbyt niska temeratura zyroskopow wynoszaca %.0f%cC. Musi miescic sie w granicach od %.0f%cC do %.0f%cC",
"Zbyt wysoka temeratura zyroskopow wynoszaca %.0f%cC. Musi miescic sie w granicach od %.0f%cC do %.0f%cC",
};

const char *cNazwyMies3Lit[13]  = {"---", "Sty","Lut", "Mar", "Kwi", "Maj", "Cze", "Lip", "Sie", "Wrz", "Paz", "Lis", "Gru"};

//maksymalna długość nazwy to (MAX_ROZMIAR_WPISU_LOGU - 2), musi zmieścić się jeszcze średnik i terminujace zero
const char *cNazwyPozycjiRejestratora[LICZBA_NAZW_POZYCJI_REJESTRATORA] = {
"Czas [g:m:s.ss]",			//NREJ_CZAS_GGMMSSSS
"CisnienieBzw%d [Pa]",		//NREJ_CISNIENIE_BZWZGL_XD_PA
"Wysokosc MSL%d [m]",		//NREJ_WYSOKOSC_MSL_XD_M
"Wysokosc AGL%d [m]",		//NREJ_WYSOKOSC_AGL_XD_M
"Wariometr%d [m/s]",		//NREJ_WARIOMETR_XD_MS
"Cisn.Roznic%d [Pa]",		//NREJ_CISN_ROZNICOWE_XD_PA
"Predk.IAS%d [m/s]",		//NREJ_PREDK_IAS_XD_MS
"Temp.Baro%d [K]",			//NREJ_TEMP_BARO_XD_K
"Temp.CisnRozn%d [K]",		//NREJ_TEMP_ROZN_XD_K
"Bat%d Napiecie [V]",		//NREJ_BAT_XD_NAPIECIE_V
"Bat%d Prad [V]",			//NREJ_BAT_XD_PRAD_A
"Bat%d Ene.Pobr [mAh]",		//NREJ_BAT_XD_ENER_POBR_MAH
"Zasil%d Napiecie [V]",		//NREJ_ZASIL_XD_NAPIECIE_V
"Czujnik Zewn%d [V]",		//NREJ_CZUJ_ZEWN_XD_V
"Temp.CPU [K]",				//NREJ_TEMP_CPU_K
"Serwa.Napiecie [V]",		//NREJ_SERWA_NAPIECIE_V
"ZyroSurowe%d%c [rad/s]",	//NREJ_ZYRO_SUR_XD_XC_RADS
"ZyroKalibr%d%c [rad/s]",	//NREJ_ZYRO_KAL_XD_XC_RADS
"Akcel%d%c [m/s^2]",		//NREJ_AKCEL_XD_XC_MS2
"Magneto%d%c [Gauss]",		//NREJ_MAGNETO_XD_XC_GAUSS
"TempIMU%d [K]",			//NREJ_TEMP_IMU_XD_K
"Kat kalmIMU%c [rad]",		//NREJ_KAT_KALM_IMU_XC_RAD
"Kat kompIMU%c [rad]",		//NREJ_KAT_KOMP_IMU_XC_RAD
"Kat kwatIMU%c [rad]",		//NREJ_KAT_KWAT_IMU_XC_RAD
"Kat akceIMU%c [rad]",		//NREJ_KAT_AKCE_IMU_XC_RAD
"Kat zyroIMU%c [rad]",		//NREJ_KAT_ZYRO_IMU_XC_RAD
"SzerokoscGeo [rad]",		//NREJ_SZEROKOSC_GEO_RAD
"DlugoscGeo [rad]",			//NREJ_DLUGOSC_GEO_RAD
"WysokoscGNSS [m]",			//NREJ_WYSOKOSC_GNSS_M
"PredWzgZiemi [m/s]",		//NREJ_PREDKOSC_WZGL_ZIEMI_MS
"KursGNSS [rad]",			//NREJ_KURS_GNSS_RAD
"LiczbaSat",				//NREJ_LICZBA_SATELITOW
"VDOP [m]",					//NREJ_VDOP_M
"HDOP [m]",					//NREJ_HDOP_M
"PredGNSS_N [m/s]",			//NREJ_PREDK_GNSS_N_MS
"PredGNSS_E [m/s]",			//NREJ_PREDK_GNSS_E_MS
"OdbiornikRC.kan",			//NREJ_ODBIORNIKRC_KAN
"WyjscieRC.kan",			//NREJ_WYJSCIERC_KAN
"Reg.KataPrze",				//NREJ_REG_KATA_PRZE
"Reg.PredPrze",				//NREJ_REG_PRED_PRZE
"Reg.KataPoch",				//NREJ_REG_KATA_POCH
"Reg.PredPoch",				//NREJ_REG_PRED_POCH
"Reg.KataOdch",				//NREJ_REG_KATA_ODCH
"Reg.PredOdch",				//NREJ_REG_PRED_ODCH
"Reg.Wysokosc",				//NREJ_REG_WYSOKOSCI
"Reg.PrZmiWys",				//NREJ_REG_PR_ZM_WYS
"Reg.KataGeoN",				//NREJ_REG_KATA_GEON
"Reg.PredGeoN",				//NREJ_REG_PRED_GEON
"Reg.KataGeoE",				//NREJ_REG_KATA_GEOE
"Reg.PredGeoE",				//NREJ_REG_PRED_GEOE
"WZadana",					//NREJ_WART_ZADANA
"FilWZad",					//NREJ_FILTR_WZAD
"WWejsci",					//NREJ_WART_WEJSCIOWA
"FilWWej",					//NREJ_FILTR_WWEJ
"FilRozn",					//NREJ_FILTR_ROZN
"WyjP",						//NREJ_WYJ_P
"WyjI",						//NREJ_WYJ_I
"WyjD",						//NREJ_WYJ_D
"WyWyprz",					//NREJ_WYJ_WYPRZ
"Wyjscie",					//NREJ_WYJSCIE
"KalmanWys_",				//NREJ_KALMAN_WYS (X/K/P[n])
"deltaCzasu [s]",			//NREJ_DELTA_CZASU
};

#endif //JEZYK_POLSKI
