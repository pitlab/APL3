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

#define WER_GLOWNA	0
#define WER_PODRZ	1
#define WER_REPO	35	//numer commitu w repozytorium

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
#define EXP25_LED_B			0x20	//LED niebieski
#define EXP26_LED_G			0x40	//LED zielony
#define EXP27_LED_R			0x80	//LED czerwony



//tryby pracy
#define TP_MENU_GLOWNE		0	//wyświetla ekran menu głównego
#define TP_WROC_DO_MENU		1
#define TP_FRAKTALE			2	//wyświetla benchmark fraktalowy
#define TP_KALIB_DOTYK		3	//kalibracja panelu dotykowego
#define TP_MULTITOOL		4
#define TP_POMIAR_FNOR		5
#define TP_POMIAR_SRAM		6
#define TP_POMIAR_FQSPI		7
#define TP_USTAWIENIA		8
#define TP_KAMERA			9
#define TP_TESTY			10
#define TP_POMIARY_IMU		11	//wyświetlaj wyniki pomiarów IMU pobrane z CM4
#define TP_ZDJECIE			12

//flagi inicjalizacj sprzetu na płytce
#define INIT0_FLASH_NOR		0x00000001
#define INIT0_LCD480x320	0x00000002
#define INIT0_FLASH_QSPI	0x00000004

//flagi inicjalizacj sprzetu poza płytką
#define INIT1_MOD_IMU		0x00000001
#define INIT1_KAMERA		0x00000002


//interfejsy komunikacyjne
#define INTERF_UART			1
#define INTERF_ETH			2
#define INTERF_USB			3
#define ILOSC_INTERF_KOM	3

#define ROZM_BUF16_LCD		320*480
#define ROZM_BUF32_KAM		320*480/ 2

#endif /* INC_SYS_DEF_CM7_H_ */
