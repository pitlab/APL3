/*
 * sys_def_CM7.h
 *
 *  Created on: Nov 10, 2024
 *      Author: PitLab
 */

#ifndef INC_SYS_DEF_CM7_H_
#define INC_SYS_DEF_CM7_H_
#include "stm32h755xx.h"
#include "errcode.h"
#include "stm32h7xx_hal.h"
#include <math.h>

#define WER_GLOWNA	0
#define WER_PODRZ	1
#define WER_REPO	67	//numer commitu w repozytorium

//definicje bitów danych expanderów IO
#define EXP00_TP_INT		0x01	//TP_INT - wejście przerwań panelu dotykowego LCD
#define EXP01_LCD_RESET		0x02	//LCD_RES - reset LCD
#define EXP02_LOG_VSELECT	0x04	//LOG_SD1_VSEL - wybor napięcia IO karty SD
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
#define TP_FRAKTALE			2	//wyświetla benchmark fraktalowy
#define TP_KALIB_DOTYK		3	//kalibracja panelu dotykowego
#define TP_POM_ZAPISU_NOR	4
#define TP_POMIAR_FNOR		5
#define TP_POMIAR_SRAM		6
#define TP_POMIAR_FQSPI		7
#define TP_USTAWIENIA		8
#define TP_KAMERA			9
#define TP_TESTY			10
#define TP_POMIARY_IMU		11	//wyświetlaj wyniki pomiarów IMU pobrane z CM4
#define TP_ZDJECIE			12
#define TP_WITAJ			13
#define TP_MULTIMEDIA		14	//menu multimediow
#define TP_WROC_DO_MMEDIA	15
#define TP_MM1				16
#define TP_MM2				17
#define TP_MM3				18
#define TP_MM_AUDIO_FFT		19	//FFT sygnału z mikrofonu
#define TP_MM_KOM1			20	//komunikat audio 1
#define TP_MM_KOM2			21	//komunikat audio 2
#define TP_MM_KOM3			22	//komunikat audio 3
#define TP_MM_KOM4			23	//komunikat audio 4



//flagi inicjalizacj sprzetu na płytce
#define INIT0_FLASH_NOR		0x00000001
#define INIT0_LCD480x320	0x00000002
#define INIT0_FLASH_QSPI	0x00000004

//flagi inicjalizacj sprzetu poza płytką
#define INIT1_MOD_IMU		0x00000001
#define INIT1_KAMERA		0x00000002

//flagi sprzetu wykrytego przez CM4
#define INITCM4_GNSS		0x00000001


//semafory sprzętowe

//interfejsy komunikacyjne
#define INTERF_UART			1
#define INTERF_ETH			2
#define INTERF_USB			3
#define ILOSC_INTERF_KOM	3

#define ROZM_BUF16_LCD		320*480
#define ROZM_BUF32_KAM		320*480/ 2

#define RAD2DEG				(180/M_PI)
#define DEG2RAD				(M_PI/180)

//pamięc flash NOR
#define ADRES_NOR			0x68000000
#define LICZBA_SEKTOROW		256
#define ROZMIAR_SEKTORA		(128*1024)	//128kB
#define SEKTOR_KOM_AUDIO	2		//od tego sektora zaczyna się spis treści komunikatów audio i następujące po nim komunikaty
#define ROZM_WPISU_AUDIO	8		//liczba bajtów zajęta przez wpis spisu komunikatów (4 bajty adres i 4 bajty długości)
#define ADR_SPISU_KOM_AUDIO	ADRES_NOR + SEKTOR_KOM_AUDIO * ROZMIAR_SEKTORA

#define ADRES_DRAM			0xC0000000

#endif /* INC_SYS_DEF_CM7_H_ */
