/*
 * sys_def_CM7.h
 *
 *  Created on: Nov 10, 2024
 *      Author: PitLab
 */

#ifndef INC_SYS_DEF_CM7_H_
#define INC_SYS_DEF_CM7_H_
#include "stm32h755xx.h"
#include "stm32h7xx_hal.h"
#include "sys_def_wspolny.h"
#include <math.h>
#include "errcode.h"
#include "stdio.h"


#define WER_GLOWNA	0
#define WER_PODRZ	1
#define WER_REPO	291		//numer commitu w repozytorium

//wybór typu wyświetlacza
#define LCD_ILI9488		//https://sklep.msalamon.pl/produkt/wyswietlacz-tft-lcd-35%E2%80%B3-ili9488-320x480/?srsltid=AfmBOopUr_Ot4ZQNoDns7QPYb-sgwqNSRUYaUR1s1TTm1hDWmuxMRWXO
//#define LCD_RPI35B	//https://www.waveshare.com/wiki/3.5inch_RPi_LCD_(B)
//#define LCD_RPI35C	//https://www.waveshare.com/wiki/3.5inch_RPi_LCD_(C) ten niestety jeszcze nie działa

//definicje bitów danych expanderów IO
#define EXP00_TP_INT		0x01	//TP_INT - wejście przerwań panelu dotykowego LCD
#define EXP01_LCD_RESET		0x02	//LCD_RES - reset LCD
#define EXP02_LOG_VSELECT	0x04	//LOG_SD1_VSEL - wybor napięcia IO karty SD: H=3,3V L=1,8V
#define EXP03_CAM_RESET		0x08	//CAM_RES - reset kamery
#define EXP04_LOG_CARD_DET	0x10	//LOG_SD1_CDETECT - wejscie detekcji obecności karty
#define EXP05_USB_HOST_DEV	0x20	//USB_HOST_DEVICE - sposób przedstawiania sie urządzenia po USB jako: 1=Host, 0=Device
#define EXP06_MOD_OPEN_DR1	0x40	//MOD_OSW_IO1 - wyjście Open Drain 1
#define EXP07_MOD_OPOE_DR2	0x80	//MOD_OSW_IO2 - wyjście Open Drain 2

#define EXP10_USB_OVERCURR	0x01	//USB_OVERCURRENT - wejście wygnalizujące przekroczenie poboru prądu przez USB device
#define EXP11_USB_POWER		0x02	//USB_POWER - włącznik zasilania dla zewnętrznego Device
#define EXP12_CAN_STANDBY	0x04	//MODZ_CAN_STBY - włącznie Standby sterownika CAN
#define EXP13_AUDIO_IN_SD	0x08	//AUDIO_IN_SD - włącznika ShutDown mikrofonu
#define EXP14_AUDIO_OUT_SD	0x10	//AUDIO_OUT_SD - włączniek ShutDown wzmacniacza audio
#define EXP15_ETH_RMII_EXER	0x20	//ETH_RMII_EXER - wejście sygnału błędu transmisji ETH
#define EXP16_BMS_I2C_SW	0x40	//BMS_I2C_SW - przełacznik zegara I2C miedzy pakietami
#define EXP17_USB_EN		0x80	//USB_EN - włącznik transmisji USB

#define EXP20_ZASIL_WE1		0x01	//ZASIL_WL_WE1 - wyjście
#define EXP21_ZASIL_WE2		0x02	//ZASIL_WL_WE2 - wyjście
#define EXP22_ZASIL_USB		0x04	//ZASIL_WL_USB - wyjście
#define EXP23_ZASIL_LED		0x08	//ZASIL_LED - wyjście
#define EXP24_WL_WYL		0x10	//WL/WYL - wejście
#define EXP25_LED_NIEB		0x20	//LED niebieski
#define EXP26_LED_ZIEL		0x40	//LED zielony
#define EXP27_LED_CZER		0x80	//LED czerwony

//programowy identyfikator LEDów
#define LED_CZER			0
#define LED_ZIEL			1
#define LED_NIEB			2
#define LICZBA_LED			3


//tryby pracy
#define TP_MENU_GLOWNE		0	//wyświetla ekran menu głównego
#define TP_WROC_DO_MENU		1
#define TP_KALIBRACJE		2	//kalibracja sprzętu
#define TP_POMIARY			3	//wyświetlaj wyniki pomiarów IMU pobrane z CM4
#define TP_NASTAWY			4
#define TP_MG1				5
#define TP_ETHERNET			6
#define TP_IMU_KOSTKA_SYM	7
#define TP_USTAWIENIA		8

#define TP_TESTY			10
#define TP_WITAJ			11
#define TP_MEDIA_AUDIO		13	//menu obsługi audio
#define TP_MEDIA_KAMERA		14	//menu obsługi kamery
#define TP_WYDAJNOSC		15	//menu pomiarów wydajności
#define TP_KARTA_SD			16	//menu obsługi karty SD

#define TP_PODGLAD_IMU		18	//podgląd parametrów IMU podczas kalibracji
#define TP_WYSWIETL_BLAD	19	//wyświetl kod błędu
#define TP_MAGNETOMETR		20	//menu obsługi magnetometru

//polecenia menu Wydajność
#define TP_FRAKTALE			30	//wyświetla benchmark fraktalowy
#define TP_POM_ZAPISU_NOR	31
#define TP_POMIAR_FNOR		32
#define TP_POMIAR_SRAM		33
#define TP_POMIAR_FQSPI		34
#define TP_EMU_MAG_CAN		35
#define TP_IMU_KOSTKA		36
#define TP_W3				37
#define TP_W4				38
#define TP_WROC_DO_WYDAJN	39

//polecenia menu Multimedia
#define TP_MREJ				40	//M.Rej
#define TP_PAPUGA			41
#define TP_MM2				42
#define TP_TEST_TONU		43
#define TP_AUDIO_FFT		44	//FFT sygnału z mikrofonu

#define TP_MM_KOM			47	//generator komunikatów audio
#define TP_WROC_DO_AUDIO	49



//polecenia menu TP_KARTA_SD
#define TPKS_WLACZ_REJ		50
#define TPKS_WYLACZ_REJ		51
#define TPKS_PARAMETRY		52
#define TPKS_POMIAR  		53
#define TPKS_4				54
#define TPKS_5				55
#define TPKS_6				56
#define TPKS_7				57
#define TPKS_8				58
#define TP_WROC_DO_KARTA	59

//polecenia menu TP_KAL_IMU
#define TP_KAL_ZYRO_ZIM		60
#define TP_KAL_ZYRO_POK		61
#define TP_KAL_ZYRO_GOR		62
#define TP_KAL_WZM_ZYROP	63
#define TP_KAL_WZM_ZYROQ	64
#define TP_KAL_WZM_ZYROR	65
#define TP_KAL_AKCEL_2D		66
#define TP_KAL_AKCEL_3D		67
#define TP_KASUJ_DRYFT_ZYRO	68
#define TP_WROC_KAL_IMU		69

#define TP_MAG_KAL1			70
#define TP_MAG_KAL2			71
#define TP_MAG_KAL3			72
#define TP_MAG1				73
#define TP_SPR_PLASKI		74
#define TP_SPR_MAG1			75
#define TP_SPR_MAG2			76
#define TP_SPR_MAG3			77
#define TP_MAG2				78
#define TP_WROC_DO_MAG		79

//Podmenu TP_KALIBRACJE
#define TP_KAL_IMU			80
#define TP_KAL_BARO			81	//kalibruj barometry wedlug wzorcowej zmiany ciśnienia
#define TP_KAL_MAG			82
#define TP_KAL_DOTYK		83	//kalibracja panelu dotykowego
#define TP_KAL_HARD_FAULT	84
#define TP_WROC_DO_KALIBR	89

//podmenu TP_POMIARY
#define TP_POMIARY_IMU		90	//wyświetlaj wyniki pomiarów IMU pobrane z CM4
#define TP_POMIARY_CISN		91
#define TP_POMIARY_RC		92	//dane z odbiornika RC
#define TP_WROC_DO_POMIARY	99

//podmenu TP_NASTAWY
#define TP_NAST_PID_POCH	100
#define TP_NAST_PID_PRZECH	101
#define TP_NAST_PID_ODCH	103
#define TP_NAST_PID_WYSOK	104
#define TP_NAST_PID_NAW_N	105
#define TP_NAST_PID_NAW_E	106
#define TP_NAST_MIKSERA		107
#define TP_WROC_DO_NASTAWY	109

//polecenia menu kamera
#define TP_ZDJECIE			110	//pojedyncze zdjęcie
#define TP_KAMERA			111	//ciagła praca kamery
#define TP_KAM5				112
#define TP_USTAW_KAM_320x240	113
#define TP_USTAW_KAM_480x320	114
#define TP_KAM1				115
#define TP_KAM2				116
#define TP_KAM3				117
#define TP_KAM4				118
#define TP_WROC_DO_KAMERA	119

//polecenia manu ethernet
#define TP_ETH_INFO			120
#define TP_ETH_GADU_GADU	121
#define TP_WROC_DO_ETH		129

//flagi inicjalizacj sprzetu na płytce
#define INIT_FLASH_NOR		0x00000001
#define INIT_FLASH_QSPI		0x00000002
#define INIT_EXPANDER_IO	0x00000004
#define INIT_AUDIO			0x00000008

//flagi inicjalizacj sprzetu poza płytką
#define INIT_MOD_IMU		0x00010000
#define INIT_KAMERA			0x00020000
#define INIT_KARTA_SD		0x00040000
#define INIT_DOTYK			0x00080000
#define INIT_LCD480x320		0x00100000

//semafory sprzętowe

//interfejsy komunikacyjne
#define INTERF_UART			1
#define INTERF_ETH			2
#define INTERF_USB			3
#define ILOSC_INTERF_KOM	3

#define ROZM_BUF16_LCD		320*480
#define STD_OBRAZU_5M 		(2592*1944)		//rozdzielczość przetwornika
#define STD_OBRAZU_4K_N		(2160*1920)
#define STD_OBRAZU_4M		(2592*1520)
#define STD_OBRAZU_5M_N		(1944*1296)
#define STD_OBRAZU_1080P	(1920*1080)		//Full HD
#define STD_OBRAZU_4M_N		(1440*1280)
#define STD_OBRAZU_720P		(1280*720)			//720P
#define STD_OBRAZU_VGA		(640*480)		//1/1 VGA
#define STD_OBRAZU_DVGA		(480*320)		//1/duo VGA
#define STD_OBRAZU_QVGA		(320*240)		//1/quadro VGA
#define STD_OBRAZU_OVGA		(160*120)		//1/octo VGA

//#define ROZM_BUF32_KAM		(2592*1944)		//pełen rozmiar przetwornika nie mieści się w  ext SRAM
#define ROZM_BUF16_KAM		STD_OBRAZU_DVGA

//pamięc flash NOR
#define ADRES_NOR			0x68000000
#define LICZBA_SEKTOROW		256
#define ROZMIAR_SEKTORA		(128*1024)	//128kB
#define SEKTOR_KOM_AUDIO	2		//od tego sektora zaczyna się spis treści komunikatów audio i następujące po nim komunikaty
#define SEKTOR_KONCA_AUDIO
#define LICZBA_SEKT_AUDIO	30		//sektor to 128kB
#define ROZM_WPISU_AUDIO	8		//liczba bajtów zajęta przez wpis spisu komunikatów (4 bajty adres i 4 bajty długości)
#define ADR_SPISU_KOM_AUDIO	(ADRES_NOR + SEKTOR_KOM_AUDIO * ROZMIAR_SEKTORA)
#define ROZM_SPISU_KOM		128		//tyle komunikatów można zapisać w spisie
//#define ADR_POCZATKU_KOM_AUDIO	(ADR_SPISU_KOM_AUDIO + ROZM_SPISU_KOM * ROZM_WPISU_AUDIO)
//#define ADR_KONCA_KOM_AUDIO	(ADR_POCZATKU_KOM_AUDIO + LICZBA_SEKT_AUDIO * ROZMIAR_SEKTORA)
#define ADR_POCZATKU_KOM_AUDIO	0x24000000	//AXI RAM komunikaty mogą znaleźć się od Axi RAM przez domyślnie zewnętrzny flash aż do DRAM
#define ADR_KONCA_KOM_AUDIO		0xC3FFFFFF	//koniec DRAM
#define ADRES_DRAM				0xC0000000


//typy komunikatów głosowych
#define KOMG_WYSOKOSC		1
#define KOMG_NAPIECIE		2
#define KOMG_TEMPERATURA	3
#define KOMG_PREDKOSC		4
#define KOMG_KIERUNEK		5
#define KOMG_MAX_KOMUNIKAT	5

//zakres adresów FLASH bank 1 dostępnych dla CM7
#define POCZATEK_FLASH		0x08000000
#define KONIEC_FLASH		0x080FFFFF



//
#define KWANT_CZASU_TELEMETRII	10	//co tyle milisekund może być wysyłana telemetria


//przesunięcie flagi i maski w zmiennej chStatusPolaczenia
#define STAT_POL_USB	0
#define STAT_POL_UART	2
#define STAT_POL_TCP	4
#define STAT_POL_RTSP	6
#define STAT_POL_NIEAKTYWNY	0
#define STAT_POL_GOTOWY		1
#define STAT_POL_OTWARTY	2
#define STAT_POL_PRZESYLA	3
#define STAT_POL_MASKA		3
#define STAT_POL_MASKA_OTW	1	//pozwala na przejscie z STAT_POL_PRZESYLA na STAT_POL_OTWARTY jednym poleceniem NAND
#define STAT_POL_MASKA_GOT	2	//pozwala na przejscie z STAT_POL_PRZESYLA na STAT_POL_GOTOWY jednym poleceniem NAND

#endif /* INC_SYS_DEF_CM7_H_ */
