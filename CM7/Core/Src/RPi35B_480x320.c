//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi wyświetlacza TFT 320x480 RPI 3,5" B https://www.waveshare.com/wiki/3.5inch_RPi_LCD_(B)
//
// Opracowano na podstawie analizy działania systemu z obrazu "RPi LCD Bookworm_32bit only for pi5&pi4" pobranego ze strony waveshare
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "RPi35B_480x320.h"
#include "display.h"
#include "main.h"
#include "moduly_SPI.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "semafory.h"
#include "cmsis_os.h"

// Wyświetlacz pracował na 25MHz ale później zaczął śmiecić na ekranie. Próbuję na 22,2MHz - jest OK


//deklaracje zmiennych
extern SPI_HandleTypeDef hspi5;
extern uint8_t chRysujRaz;
extern uint32_t nZainicjowano[2];		//flagi inicjalizacji sprzętu

uint16_t sBuforLCD[DISP_X_SIZE * DISP_Y_SIZE];
uint8_t chOrient;
uint8_t fch, fcl, bch, bcl;	//kolory czcionki i tła (bajt starszy i młodszy)
uint8_t _transparent;	//flaga określająca czy mamy rysować tło czy rysujemy na istniejącym
struct _current_font
{
	unsigned char* font;
	unsigned char x_size;
	unsigned char y_size;
	unsigned char offset;
	unsigned char numchars;
} cfont;
volatile uint8_t chDMAgotowe;

////////////////////////////////////////////////////////////////////////////////
// Wysyła polecenie do wyświetlacza LCD
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: chDane - polecenie wysłane w jednej ramce ograniczonej CS
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t LCD_write_command16(uint8_t chDane)
{
	HAL_StatusTypeDef chErr;
	uint8_t dane_nadawane[2];

	dane_nadawane[0] = 0x00;
	dane_nadawane[1] = chDane;
	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
		osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
	{
		UstawDekoderZewn(CS_LCD);											//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);	//LCD_RS=0
		chErr = HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_MAX_DELAY);
		UstawDekoderZewn(CS_NIC);											//LCD_CS=1
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
	return chErr;
}

uint8_t LCD_write_command8(uint8_t chDane)
{
	HAL_StatusTypeDef chErr;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
		osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
	{
		UstawDekoderZewn(CS_LCD);											//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);	//LCD_RS=0
		chErr = HAL_SPI_Transmit(&hspi5, &chDane, 1, HAL_MAX_DELAY);
		UstawDekoderZewn(CS_NIC);											//LCD_CS=1
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
	return chErr;
}


////////////////////////////////////////////////////////////////////////////////
// Wysyła do wyświetlacza LCD  bufor danych w jednej ramce ograniczonej CS. Steruje liniami RS i CS
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: *chDane - wskaźnika na bufor z danymi wysłane
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t LCD_WrData(uint8_t* chDane, uint8_t chIlosc)
{
	HAL_StatusTypeDef chErr;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
		osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
	{
		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		chErr = HAL_SPI_Transmit(&hspi5, chDane, chIlosc, HAL_MAX_DELAY);
		UstawDekoderZewn(CS_NIC);										//LCD_CS=1
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
	return chErr;
}


///////////////////////////////////////////////////////////////////////////////
// Wysyła do wyświetlacza LCD przez DMA bufor danych w jednej ramce ograniczonej CS. Steruje liniami RS i CS
// Parametry: *chDane - wskaźnika na bufor z danymi wysłane
// sIlosc - ilość wysyłanych danych
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t LCD_WrDataDMA(uint8_t* chDane, uint16_t sIlosc)
{
	HAL_StatusTypeDef Err;

	UstawDekoderZewn(CS_LCD);										//LCD_CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
	Err = HAL_SPI_Transmit_DMA(&hspi5, chDane, sIlosc);
	UstawDekoderZewn(CS_NIC);										//LCD_CS=1
	return Err;
}


////////////////////////////////////////////////////////////////////////////////
// Wysyła dane do wyświetlacza LCD steruje liniami RS i CS
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: chDane1, chDane2 - dane wysłane w jednej ramce ograniczonej CS
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t LCD_write_data16(uint8_t chDane1, uint8_t chDane2)
{
	HAL_StatusTypeDef chErr;
	uint8_t dane_nadawane[2];

	dane_nadawane[0] = chDane1;
	dane_nadawane[1] = chDane2;
	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
			osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
	{
		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		chErr = HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_MAX_DELAY);
		UstawDekoderZewn(CS_NIC);
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła jedyne dane do wyświetlacza LCD w trybie 16-bitowym, steruje liniami RS i CS na początku i końcu
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: chDane
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t LCD_write_dat_jed16(uint8_t chDane)
{
	HAL_StatusTypeDef chErr;
	uint8_t dane_nadawane[2];

	dane_nadawane[0] = 0x00;
	dane_nadawane[1] = chDane;
	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
			osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
	{
		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		chErr = HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_MAX_DELAY);
		UstawDekoderZewn(CS_NIC);
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
	return chErr;
}

uint8_t LCD_write_dat_jed8(uint8_t chDane)
{
	HAL_StatusTypeDef chErr;
	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
			osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
	{
		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		chErr = HAL_SPI_Transmit(&hspi5, &chDane, 1, HAL_MAX_DELAY);
		UstawDekoderZewn(CS_NIC);
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła pierwsze dane do wyświetlacza LCD w trybie 16-bitowym, steruje liniami RS i CS na początku
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: chDane
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t LCD_write_dat_pie16(uint8_t chDane)
{
	HAL_StatusTypeDef chErr;
	uint8_t dane_nadawane[2];

	dane_nadawane[0] = 0x00;
	dane_nadawane[1] = chDane;


	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
			osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
	{
		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		chErr = HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_MAX_DELAY);
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
	return chErr;
}

uint8_t LCD_write_dat_pie8(uint8_t chDane)
{
	HAL_StatusTypeDef chErr;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
			osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
	{
		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		chErr = HAL_SPI_Transmit(&hspi5, &chDane, 1, HAL_MAX_DELAY);
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła środkowe dane do wyświetlacza LCD w trybie 16-bitowym. Nie steruje liniami CS i RC
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: chDane
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void LCD_write_dat_sro16(uint8_t chDane)
{
	uint8_t dane_nadawane[2];
	HAL_StatusTypeDef chErr;

	dane_nadawane[0] = 0x00;
	dane_nadawane[1] = chDane;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
			osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
		HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_MAX_DELAY);
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
}

void LCD_write_dat_sro8(uint8_t chDane)
{
	HAL_StatusTypeDef chErr;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
			osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
		HAL_SPI_Transmit(&hspi5, &chDane, 1, HAL_MAX_DELAY);
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła dane do wyświetlacza LCD - ostatnie ustawia CS
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: chDane
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void LCD_write_dat_ost16(uint8_t chDane)
{
	uint8_t dane_nadawane[2];
	HAL_StatusTypeDef chErr;

	dane_nadawane[0] = 0x00;
	dane_nadawane[1] = chDane;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
		osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
	{
		HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_MAX_DELAY);
		UstawDekoderZewn(CS_NIC);										//LCD_CS=1
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
}

void LCD_write_dat_ost8(uint8_t chDane)
{
	HAL_StatusTypeDef chErr;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
		osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
	{
		HAL_SPI_Transmit(&hspi5, &chDane, 1, HAL_MAX_DELAY);
		UstawDekoderZewn(CS_NIC);										//LCD_CS=1
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
}



////////////////////////////////////////////////////////////////////////////////
// Czyta dane z wyświetlacza LCD bez sterowania liniami CS i DC
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: *chDane - wskaźnik na zwracane dane
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void LCD_data_read(uint8_t *chDane, uint8_t chIlosc)
{
	HAL_StatusTypeDef chErr;

	while (HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW) != ERR_OK)
			osDelay(1);
	chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
	if (chErr == ERR_OK)
	{
		UstawDekoderZewn(CS_LCD);										//LCD_CS=0
		HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
		HAL_SPI_Receive(&hspi5, chDane, chIlosc, 2);
		UstawDekoderZewn(CS_NIC);										//LCD_CS=1
	}
	HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
}



////////////////////////////////////////////////////////////////////////////////
// Konfiguracja wyświetlacza 3.5inch RPi LCD (B) https://www.waveshare.com/wiki/3.5inch_RPi_LCD_(B)
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLCD_35B_16bit(void)
{
	extern uint8_t chPort_exp_wysylany[LICZBA_EXP_SPI_ZEWN];

	// LCD_RESET 1 - 0 - 1
	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(10);

	chPort_exp_wysylany[0] &= ~EXP01_LCD_RESET;	//RES=0
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

	LCD_write_command16(0xB0);
	LCD_write_data16(0x00, 0x00);
	LCD_write_command16(0x11);

	HAL_Delay(285); 			//285ms przerwy

	LCD_write_command16(0x21);
	LCD_write_command16(0x3A);
	LCD_write_data16(0x00, 0x55);

	LCD_write_command16(0xC2);
	LCD_write_data16(0x00, 0x33);

	LCD_write_command16(0xC5);
	LCD_write_dat_pie16( 0x00);
	LCD_write_dat_sro16(0x1E);
	LCD_write_dat_ost16(0x80);

	LCD_write_command16(0x36);
	LCD_write_data16(0x00, 0x28);

	LCD_write_command16(0xB1);
	LCD_write_data16(0x00, 0xB0);

	LCD_write_command16(0xE0);	//Positive Gamma Correction
	LCD_write_dat_pie16(0x00);
	LCD_write_dat_sro16(0x13);
	LCD_write_dat_sro16(0x18);
	LCD_write_dat_sro16(0x04);
	LCD_write_dat_sro16(0x0F);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x3A);
	LCD_write_dat_sro16(0x56);
	LCD_write_dat_sro16(0x4D);
	LCD_write_dat_sro16(0x03);
	LCD_write_dat_sro16(0x0A);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x30);
	LCD_write_dat_sro16(0x3E);
	LCD_write_dat_ost16(0x0F);

	LCD_write_command16(0xE1);	//Negative Gamma Correction
	LCD_write_dat_pie16(0x00);
	LCD_write_dat_sro16(0x13);
	LCD_write_dat_sro16(0x18);
	LCD_write_dat_sro16(0x01);
	LCD_write_dat_sro16(0x11);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x38);
	LCD_write_dat_sro16(0x34);
	LCD_write_dat_sro16(0x4D);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x0D);
	LCD_write_dat_sro16(0x0B);
	LCD_write_dat_sro16(0x31);
	LCD_write_dat_sro16(0x37);
	LCD_write_dat_ost16(0x0F);

	LCD_write_command16(0x11);
	HAL_Delay(302); 		//302ms

	LCD_write_command16(0x29);
	LCD_write_command16(0x36);	//orientacja ekranu
	LCD_write_data16(0x00, 0xE8);

	LCD_write_command16(0x2A);	//Column Address Set
	LCD_write_dat_pie16(0x00);
	LCD_write_dat_sro16(0x00);
	LCD_write_dat_sro16(0x01);
	LCD_write_dat_ost16(0xDF);

	LCD_write_command16(0x2B);	//Page Address Set
	LCD_write_dat_pie16(0x00);
	LCD_write_dat_sro16(0x00);
	LCD_write_dat_sro16(0x01);
	LCD_write_dat_ost16(0x3F);

	chRysujRaz = 1;
	nZainicjowano[0] |= INIT0_LCD480x320;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Konfiguracja wyświetlacza 3.5inch RPi LCD (C) https://www.waveshare.com/wiki/3.5inch_RPi_LCD_(C)
// Nie działa
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLCD_35C_8bit(void)
{
	extern uint8_t chPort_exp_wysylany[LICZBA_EXP_SPI_ZEWN];

	// LCD_RESET 1 - 0 - 1
	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(10);

	chPort_exp_wysylany[0] &= ~EXP01_LCD_RESET;	//RES=0
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

	LCD_write_command8(0xFF);
	LCD_write_command8(0xFF);
	HAL_Delay(5);

	LCD_write_command8(0xFF);
	LCD_write_command8(0xFF);
	LCD_write_command8(0xFF);
	LCD_write_command8(0xFF);
	HAL_Delay(10);

	LCD_write_command8(0xB0);
	LCD_write_dat_jed8(0x00);

	LCD_write_command8(0xB3);
	LCD_write_dat_pie8(0x02);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_ost8(0x10);

	LCD_write_command8(0xB4);
	LCD_write_dat_jed8(0x11);

	LCD_write_command8(0xC0);
	LCD_write_dat_pie8(0x13);
	LCD_write_dat_sro8(0x3B);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x01);
	LCD_write_dat_sro8(0x00);	//NW
	LCD_write_dat_ost8(0x43);

	LCD_write_command8(0xC1);
	LCD_write_dat_pie8(0x08);
	LCD_write_dat_sro8(0x15);	//CLOCK
	LCD_write_dat_sro8(0x08);
	LCD_write_dat_ost8(0x08);

	LCD_write_command8(0xC4);
	LCD_write_dat_pie8(0x15);
	LCD_write_dat_sro8(0x03);
	LCD_write_dat_sro8(0x03);
	LCD_write_dat_ost8(0x01);

	LCD_write_command8(0xC6);
	LCD_write_dat_jed8(0x02);

	LCD_write_command8(0xC8);
	LCD_write_dat_pie8(0x0C);
	LCD_write_dat_sro8(0x05);
	LCD_write_dat_sro8(0x0A);
	LCD_write_dat_sro8(0x6B);
	LCD_write_dat_sro8(0x04);
	LCD_write_dat_sro8(0x06);
	LCD_write_dat_sro8(0x15);
	LCD_write_dat_sro8(0x10);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x31);
	LCD_write_dat_sro8(0x10);
	LCD_write_dat_sro8(0x15);
	LCD_write_dat_sro8(0x06);
	LCD_write_dat_sro8(0x64);
	LCD_write_dat_sro8(0x0D);
	LCD_write_dat_sro8(0x0A);
	LCD_write_dat_sro8(0x05);
	LCD_write_dat_sro8(0x0C);
	LCD_write_dat_sro8(0x31);
	LCD_write_dat_ost8(0x00);

	LCD_write_command8(0x35);
	LCD_write_dat_jed8(0x00);

	LCD_write_command8(0x0C);
	LCD_write_dat_jed8(0x66);

	LCD_write_command8(0x3A);
	LCD_write_dat_jed8(0x66);

	LCD_write_command8(0x44);
	LCD_write_dat_pie8(0x00);
	LCD_write_dat_ost8(0x01);

	LCD_write_command8(0xD0);
	LCD_write_dat_pie8(0x07);
	LCD_write_dat_sro8(0x07);	//VCI1
	LCD_write_dat_sro8(0x14);	//VRH 0x1D
	LCD_write_dat_ost8(0xA2);	//BT

	LCD_write_command8(0xD1);
	LCD_write_dat_pie8(0x03);
	LCD_write_dat_sro8(0x5A);	//VCM
	LCD_write_dat_ost8(0x10);	//VDV

	LCD_write_command8(0xD2);
	LCD_write_dat_pie8(0x03);
	LCD_write_dat_sro8(0x04);
	LCD_write_dat_ost8(0x04);

	LCD_write_command8(0x11);
	HAL_Delay(150);

	LCD_write_command8(0x2A);
	LCD_write_dat_pie8(0x00);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x01);
	LCD_write_dat_ost8(0x3F);	//320

	LCD_write_command8(0x2B);
	LCD_write_dat_pie8(0x00);
	LCD_write_dat_sro8(0x00);
	LCD_write_dat_sro8(0x01);
	LCD_write_dat_ost8(0xDF);	//480

	//write_SPI_commond(0xB4);
	//write_SPI_data(0x00);
	HAL_Delay(100);
	LCD_write_command8(0x29);

	HAL_Delay(30);
	LCD_write_command8(0x2C);

	chRysujRaz = 1;
	nZainicjowano[0] |= INIT0_LCD480x320;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Konfiguracja wyświetlacza 3.5inch RPi LCD (C) https://www.waveshare.com/wiki/3.5inch_RPi_LCD_(C)
// Na podstawie sterownika NoTro: https://github.com/notro/fbtft/blob/master/fb_ili9486.c
// Nie działa
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLCD_35C_notro(void)
{
	extern uint8_t chPort_exp_wysylany[LICZBA_EXP_SPI_ZEWN];

	// LCD_RESET 1 - 0 - 1
	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(10);

	chPort_exp_wysylany[0] &= ~EXP01_LCD_RESET;	//RES=0
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

	chPort_exp_wysylany[0] |= EXP01_LCD_RESET;	//RES=1
	WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
	HAL_Delay(20);

//	LCD_write_command16(0xFF);
	//LCD_write_command16(0xFF);
	//HAL_Delay(5);		//delay_nms(5);

	//LCD_write_command16(0xFF);
	//LCD_write_command16(0xFF);
	//LCD_write_command16(0xFF);
	//LCD_write_command16(0xFF);
//	HAL_Delay(10);		//delay_nms(10);

	LCD_write_command16(0xB0);	// Interface Mode Control
	LCD_write_dat_jed16(0x00);

	LCD_write_command16(0x11);	// Sleep OUT
	HAL_Delay(250);

	LCD_write_command16(0xB0);	// Power Control 3
	LCD_write_dat_jed16(0x44);

	LCD_write_command16(0xC5);	// VCOM Control 1
	LCD_write_dat_pie16(0x00);
	LCD_write_dat_sro16(0x00);
	LCD_write_dat_sro16(0x00);
	LCD_write_dat_ost16(0x00);

	LCD_write_command16(0xE0);	// PGAMCTRL(Positive Gamma Control)
	LCD_write_dat_pie16(0x0F);
	LCD_write_dat_sro16(0x1F);
	LCD_write_dat_sro16(0x1C);
	LCD_write_dat_sro16(0x0C);
	LCD_write_dat_sro16(0x0F);
	LCD_write_dat_sro16(0x08);
	LCD_write_dat_sro16(0x48);
	LCD_write_dat_sro16(0x98);
	LCD_write_dat_sro16(0x37);
	LCD_write_dat_sro16(0x0A);
	LCD_write_dat_sro16(0x13);
	LCD_write_dat_sro16(0x04);
	LCD_write_dat_sro16(0x11);
	LCD_write_dat_sro16(0x0D);
	LCD_write_dat_ost16(0x00);

	LCD_write_command16(0xE1);	// NGAMCTRL(Negative Gamma Control)
	LCD_write_dat_pie16(0x0F);
	LCD_write_dat_sro16(0x32);
	LCD_write_dat_sro16(0x2E);
	LCD_write_dat_sro16(0x0B);
	LCD_write_dat_sro16(0x0D);
	LCD_write_dat_sro16(0x05);
	LCD_write_dat_sro16(0x47);
	LCD_write_dat_sro16(0x75);
	LCD_write_dat_sro16(0x37);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x10);
	LCD_write_dat_sro16(0x03);
	LCD_write_dat_sro16(0x24);
	LCD_write_dat_sro16(0x20);
	LCD_write_dat_ost16(0x00);

	LCD_write_command16(0xE2);	// Digital Gamma Control 1
	LCD_write_dat_pie16(0x0F);
	LCD_write_dat_sro16(0x32);
	LCD_write_dat_sro16(0x2E);
	LCD_write_dat_sro16(0x0B);
	LCD_write_dat_sro16(0x0D);
	LCD_write_dat_sro16(0x05);
	LCD_write_dat_sro16(0x47);
	LCD_write_dat_sro16(0x75);
	LCD_write_dat_sro16(0x37);
	LCD_write_dat_sro16(0x06);
	LCD_write_dat_sro16(0x10);
	LCD_write_dat_sro16(0x03);
	LCD_write_dat_sro16(0x24);
	LCD_write_dat_sro16(0x20);
	LCD_write_dat_ost16(0x00);

	LCD_write_command16(0x11);	// Sleep OUT
	HAL_Delay(250);

	LCD_write_command16(0x29);		// Display ON
	HAL_Delay(30);

	chRysujRaz = 1;
	nZainicjowano[0] |= INIT0_LCD480x320;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// ustawia orientację ekranu
// Parametry: orient - orientacja
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void LCD_Orient(uint8_t orient)
{
	/*uint8_t chDane[8];

	LCD_write_command16(0x04);
	LCD_data_read(chDane, 8);*/


	LCD_write_command16(0x36); // Memory Access Control
	if (orient)	//PIONOWO
	{
		LCD_write_data16(0x00,
			(0<<2)|	//MH Horizontal Refresh ORDER
			(1<<3)|	//BGR RGB-BGR Order: 0: RGB=RGB; 1:RGB=BGR
			(0<<4)|	//ML Vertical Refresh Order
			(0<<5)|	//MV Row / Column Exchange
			(1<<6)|	//MX Column Address Order
			(0<<7));	//MY Row Address Order
	}
	else	//POZIOMO
	{
		LCD_write_data16(0x00,
			(0<<2)|	//MH Horizontal Refresh ORDER
			(1<<3)|	//BGR RGB-BGR Order: 0: RGB=RGB; 1:RGB=BGR
			(0<<4)|	//ML Vertical Refresh Order
			(1<<5)|	//MV Row / Column Exchange
			(1<<6)|	//MX Column Address Order
			(1<<7));	//MY Row Address Order
	}
	chOrient = orient;
}



////////////////////////////////////////////////////////////////////////////////
// Zapełnij cały ekran kolorem
// Parametry: nic
// Zwraca: nic
// Czas czyszczenia ekranu: 378ms @25MHz
////////////////////////////////////////////////////////////////////////////////
void LCD_clear(uint16_t kolor)
{
	uint32_t y;
	uint8_t x, dane[8];

	//for(y=0; y<DISP_X_SIZE * DISP_Y_SIZE; y++)
		//sBuforLCD[y] = GRAY20;
	if (!kolor)
		kolor = BLACK;

	chRysujRaz = 1;
	setColor(kolor);
	setBackColor(kolor);

	LCD_write_command16(0x2A);	//Column Address Set
	for (x=0; x<8; x++)
		dane[x] = 0;
	dane[5] = 0x01;
	dane[7] = 0xDF;		//479 = 0x1DF
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2B);	//Page Address Set
	dane[5] = 0x01;
	dane[7] = 0x3F;		//319 = 0x13F
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2C);	//Memory Write
	for (x=0; x<4; x++)
	{
		dane[2*x+0] = bch;
		dane[2*x+1] = bcl;
	}
	for(y=0; y<320*480/4; y++)
		LCD_WrData(dane, 8);

	/*/ilość transmitowanych danych jest liczbą 16-bitową, więc całość danych dzielę na mniejsze kawałki
	for (x=0; x<8; x++)
		LCD_WrDataDMA((uint8_t*) sBuforLCD + (x * DISP_X_SIZE * DISP_Y_SIZE / 4), DISP_X_SIZE * DISP_Y_SIZE /4);*/
}



////////////////////////////////////////////////////////////////////////////////
// Rysuj prostokąt wypełniony kolorem
// Parametry:
// Zwraca: nic
// Czas rysowania pełnego ekranu: 372ms @25MHz
////////////////////////////////////////////////////////////////////////////////
void LCD_ProstokatWypelniony(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint16_t kolor)
{
	uint16_t i, j, k;
	uint8_t n, dane[8];

	for (n=0; n<8; n++)
		dane[n] = 0;

	LCD_write_command16(0x2A);	//Column Address Set
	dane[1] = sStartX >> 8;
	dane[3] = sStartX;
	dane[5] = (sStartX + sSzerokosc - 1) >> 8;
	dane[7] = sStartX + sSzerokosc - 1;
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2B);	//Page Address Set
	dane[1] = sStartY >> 8;
	dane[3] = sStartY;
	dane[5] = (sStartY +  sWysokosc - 1) >> 8;
	dane[7] =  sStartY + sWysokosc - 1;
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2C);	//Memory Write
	for(n=0; n<4; n++)
	{
		dane[2*n+0] = kolor >> 8;
		dane[2*n+1] = kolor;
	}

	for(i=0; i<sWysokosc; i++)
	{
		for(j=0; j<sSzerokosc/4; j++)
			LCD_WrData(dane, 8);

		//ponieważ wypełnianie odbywa się paczkami po 4 pixele, może zdarzyć się że nie dopełni wszystkich danych
		k = sSzerokosc - j*4;	//policz czy jest reszta
		if (k)
		{
			for(j=0; j<k; j++)
				LCD_WrData(dane, 2);	//dopełnij reszte
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje poziomą linią
// Parametry: x, y współrzędne początku
// len - długośc linii
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawHLine(int16_t x, int16_t y, int16_t len)
{
	int i;

	if (len < 0)
	{
		len = -len;
		x -= len;
	}
	setXY(x, y, x+len, y);

	for (i=0; i<len+1; i++)
		LCD_write_data16(fch, fcl);
	clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje pionową linię
// Parametry: x, y współrzędne początku
// len - długość linii
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawVLine(int16_t x, int16_t y, int16_t len)
{
	int i;

	if (len < 0)
	{
		len = -len;
		y -= len;
	}
	setXY(x, y, x, y+len);

	for (i=0; i<len+1; i++)
		LCD_write_data16(fch, fcl);
	clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// ustawia parametry pamięci do rysowania linii
// Parametry:
// x1, y1 współrzędne początku obszaru
// x2, y2 współrzędne końca obszaru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t sTemp;
	uint8_t x, dane[8];

	for (x=0; x<8; x++)
		dane[x] = 0;

	if (chOrient == PIONOWO)
	{
		sTemp = x1;
		x1 = y1;
		y1 = sTemp;
		sTemp = x2;
		x2 = y2;
		y2 = sTemp;
		y1=DISP_Y_SIZE - y1;
		y2=DISP_Y_SIZE - y2;
		sTemp = y1;
		y1 = y2;
		y2 = sTemp;
	}

	LCD_write_command16(0x2A);	//Column Address Set
	dane[1] = x1>>8;
	dane[3] = x1;
	dane[5] = x2>>8;
	dane[7] = x2;
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2B);	//Page Address Set
	dane[1] = y1>>8;
	dane[3] = y1;
	dane[5] = y2>>8;
	dane[7] = y2;
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2C);	//Memory Write
}



////////////////////////////////////////////////////////////////////////////////
// zeruje parametry pamięci do rysowania linii
// Parametry:nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void clrXY(void)
{
	if (chOrient == PIONOWO)
		setXY(0, 0, DISP_X_SIZE, DISP_Y_SIZE);
	else
		setXY(0, 0, DISP_Y_SIZE, DISP_X_SIZE);
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor rysowania jako RGB
// Parametry: r, g, b - składowe RGB koloru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setColorRGB(uint8_t r, uint8_t g, uint8_t b)
{
	fch = ((r & 0xF8) | g>>5);
	fcl = ((g & 0x1C)<<3 | b>>3);
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor rysowania jako natywny dla wyświetlacza 5R+6G+5B
// Parametry: color - kolor
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setColor(uint16_t color)
{
	fch = (uint8_t)(color>>8);
	fcl = (uint8_t)(color & 0xFF);
}



////////////////////////////////////////////////////////////////////////////////
// pobiera aktywny kolor
// Parametry: nic
// Zwraca: kolor
////////////////////////////////////////////////////////////////////////////////
uint16_t getColor(void)
{
	return (fch<<8) | fcl;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor tła jako RGB
// Parametry: r, g, b - składowe RGB koloru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setBackColorRGB(uint8_t r, uint8_t g, uint8_t b)
{
	bch=((r&248)|g>>5);
	bcl=((g&28)<<3|b>>3);
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kolor tła jako natywny dla wyświetlacza 5R+6G+5B
// Parametry: color - kolor
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setBackColor(uint16_t color)
{
	if (color == TRANSPARENT)
		_transparent = 1;
	else
	{
		bch = (uint8_t)(color>>8);
		bcl = (uint8_t)(color & 0xFF);
		_transparent = 0;
	}
}



////////////////////////////////////////////////////////////////////////////////
// pobiera aktywny kolor tła
// Parametry: nic
// Zwraca: kolor
////////////////////////////////////////////////////////////////////////////////
uint16_t getBackColor(void)
{
	return (bch<<8) | bcl;
}



////////////////////////////////////////////////////////////////////////////////
// rysuje piksel o współprzędnych x,y we wcześniej zdefiniowanym kolorze
// Parametry: x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawPixel(uint16_t x, uint16_t y)
{
	setXY(x, y, x, y);
	LCD_write_dat_pie16(fch);
	LCD_write_dat_ost16(fcl);
	//clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// rysuj linię o współrzędnych x1,y1, x2,y3 we wcześniej zdefiniowanym kolorze
// Parametry: x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	if (y1==y2)
		drawHLine(x1, y1, x2-x1);
	else if (x1==x2)
		drawVLine(x1, y1, y2-y1);
	else
	{
		uint16_t	dx = (x2 > x1 ? x2 - x1 : x1 - x2);
		int16_t		xstep =  x2 > x1 ? 1 : -1;
		uint16_t	dy = (y2 > y1 ? y2 - y1 : y1 - y2);
		int16_t		ystep =  y2 > y1 ? 1 : -1;
		int16_t	col = x1, row = y1;

		if (dx < dy)
		{
			int16_t t = - (dy >> 1);
			while (1)
			{
				setXY (col, row, col, row);
				//LCD_write_dat_pie(fch);
				//LCD_write_dat_ost(fcl);
				LCD_write_data16(fch, fcl);
				//LCD_write_dat_pie16(fch);
				//LCD_write_dat_ost16(fcl);
				if (row == y2)
					return;
				row += ystep;
				t += dx;
				if (t >= 0)
				{
					col += xstep;
					t   -= dy;
				}
			}
		}
		else
		{
			int16_t t = - (dx >> 1);
			while (1)
			{
				setXY (col, row, col, row);
				//LCD_write_dat_pie(fch);
				//LCD_write_dat_ost(fcl);
				LCD_write_data16(fch, fcl);
				//LCD_write_dat_pie16(fch);
				//LCD_write_dat_ost16(fcl);
				if (col == x2)
					return;
				col += xstep;
				t += dy;
				if (t >= 0)
				{
					row += ystep;
					t   -= dx;
				}
			}
		}
	}
	//clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// rysuj prostokąt o współrzędnych x1,y1, x2,y2 we wcześniej zdefiniowanym kolorze
// Parametry: x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	int nTemp;

	if (x1>x2)
	{
		nTemp = x1;
		x1 = x2;
		x2 = nTemp;
	}
	if (y1>y2)
	{
		nTemp = y1;
		y1 = y2;
		y2 = nTemp;
	}

	drawHLine(x1, y1, x2-x1);
	drawHLine(x1, y2, x2-x1);
	drawVLine(x1, y1, y2-y1);
	drawVLine(x2, y1, y2-y1);
}



////////////////////////////////////////////////////////////////////////////////
// rysuj zaokrąglony prostokąt o współprzędnych x1,y1, x2,y2 we wcześniej zdefiniowanym kolorze
// Parametry: x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t nTemp;

	if (x1>x2)
	{
		nTemp = x1;
		x1 = x2;
		x2 = nTemp;
	}
	if (y1>y2)
	{
		nTemp = y1;
		y1 = y2;
		y2 = nTemp;
	}
	if ((x2-x1)>4 && (y2-y1)>4)
	{
		drawPixel(x1+1,y1+1);
		drawPixel(x2-1,y1+1);
		drawPixel(x1+1,y2-1);
		drawPixel(x2-1,y2-1);
		drawHLine(x1+2, y1, x2-x1-4);
		drawHLine(x1+2, y2, x2-x1-4);
		drawVLine(x1, y1+2, y2-y1-4);
		drawVLine(x2, y1+2, y2-y1-4);
	}
}



////////////////////////////////////////////////////////////////////////////////
// pisze znak na miejscu o podanych współrzędnych
// Parametry: c - znak; x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void printChar(uint8_t c, uint16_t x, uint16_t y)
{
	uint8_t i,ch;
	uint16_t j;
	uint16_t temp;
	uint16_t zz;

	if (!_transparent)
	{
		if (chOrient==POZIOMO)
		{
			setXY(x,y,x+cfont.x_size-1,y+cfont.y_size-1);

			temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
			for(j=0;j<((cfont.x_size/8)*cfont.y_size);j++)
			{
				ch = cfont.font[temp];
				for(i=0;i<8;i++)
				{
					if((ch&(1<<(7-i)))!=0)
						LCD_write_data16(fch, fcl);
					else
						LCD_write_data16(bch, bcl);
				}
				temp++;
			}
		}
		else
		{
			temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;

			for(j=0;j<((cfont.x_size/8)*cfont.y_size);j+=(cfont.x_size/8))
			{
				setXY(x,y+(j/(cfont.x_size/8)),x+cfont.x_size-1,y+(j/(cfont.x_size/8)));
				for (zz=(cfont.x_size/8)-1; zz>=0; zz--)
				{
					ch=cfont.font[temp+zz];
					for(i=0;i<8;i++)
					{
						if((ch&(1<<i))!=0)
							LCD_write_data16(fch, fcl);
						else
							LCD_write_data16(bch, bcl);
					}
				}
				temp+=(cfont.x_size/8);
			}
		}
	}
	else
	{
		temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
		for(j=0;j<cfont.y_size;j++)
		{
			for (zz=0; zz<(cfont.x_size/8); zz++)
			{
				ch = cfont.font[temp+zz];
				for(i=0;i<8;i++)
				{
					if((ch&(1<<(7-i)))!=0)
					{
						setXY(x+i+(zz*8),y+j,x+i+(zz*8)+1,y+j+1);
						LCD_write_data16(fch, fcl);
					}
				}
			}
			temp+=(cfont.x_size/8);
		}
	}
	//clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// ustawia aktualną czcionkę
// Parametry: c - znak; x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void setFont(uint8_t* font)
{
	cfont.font = font;
	cfont.x_size = *(font+0);
	cfont.y_size = *(font+1);
	cfont.offset = *(font+2);
	cfont.numchars = *(font+3);
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera szerokość aktualnej czcionki
// Parametry: nic
// Zwraca: szerokość znaku
////////////////////////////////////////////////////////////////////////////////
uint8_t GetFontX(void)
{
	return cfont.x_size;
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera wysokość aktualnej czcionki
// Parametry: nic
// Zwraca: wysokość znaku
////////////////////////////////////////////////////////////////////////////////
uint8_t GetFontY(void)
{
	return cfont.y_size;
}



////////////////////////////////////////////////////////////////////////////////
// pisze napis na miejscu o podanych współrzędnych
// Parametry:
// *st - ciąg do wypisania
//  x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void print(char *str, uint16_t x, uint16_t y)
{
	int stl;

	stl = strlen((char*)str);

	if (chOrient == POZIOMO)
	{
	if (x == RIGHT)
		x = (DISP_X_SIZE+1)-(stl*cfont.x_size);
	if (x == CENTER)
		x = ((DISP_X_SIZE+1)-(stl*cfont.x_size))/2;
	}
	else	//wersja dla pionowego układu ekranu
	{
	if (x == RIGHT)
		x = (DISP_VX_SIZE+1)-(stl*cfont.x_size);
	if (x == CENTER)
		x = ((DISP_VX_SIZE+1)-(stl*cfont.x_size))/2;
	}

	for (uint16_t i=0; i<stl; i++)
		printChar(*str++, x + (i*(cfont.x_size)), y);
}



////////////////////////////////////////////////////////////////////////////////
// pisze zawijający się napis w ramce o podanych współrzędnych
// Parametry:
// *st - ciąg do wypisania
//  x, y - współrzędne
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void printRamka(char *str, uint16_t x, uint16_t y, uint16_t sx, uint16_t sy)
{
	int dlugoscNapisu, dlugoscWiersza;

	dlugoscNapisu = strlen((char*)str);

	do
	{
		if ((dlugoscNapisu * cfont.x_size) > sx)	//czy napis dłuższy niż szerokość ramki
		{
			//znajdź spację czyli miejsce do złamania napisu zaczynając od ostatniego znaku mieszcząceogo się w ramce
			for (uint16_t n = sx / cfont.x_size; n > 0; n--)
			{
				if (*(str+n) == ' ')
				{
					dlugoscWiersza = n;
					break;
				}
			}
		}
		else
			dlugoscWiersza = dlugoscNapisu;


		//if (chOrient == POZIOMO)		//na razie obsługuję tylko poziomo
		{
			if (x == RIGHT)
				x = (DISP_X_SIZE - sx + 1) - (dlugoscWiersza * cfont.x_size);
			if (x == CENTER)
				x = ((DISP_X_SIZE - sx) / 2)  + (sx - (dlugoscWiersza * cfont.x_size)) / 2;
		}
		for (uint16_t i=0; i<dlugoscWiersza; i++)
			printChar(*str++, x + (i*(cfont.x_size)), y);

		dlugoscNapisu -= dlugoscWiersza;
		y += cfont.y_size;
	} 	while  (dlugoscNapisu && (y < (y + sy)));
}



////////////////////////////////////////////////////////////////////////////////
// rysuje okrąg
// Parametry:
//  x, y - współrzdne środka
//  radius - promień
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawCircle(uint16_t x, uint16_t y, uint16_t radius)
{
	int16_t f = 1 - radius;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * radius;
	int16_t x1 = 0;
	int16_t y1 = radius;

	//cbi(P_CS, B_CS);
	setXY(x, y + radius, x, y + radius);
	LCD_write_data16(fch, fcl);

	setXY(x, y - radius, x, y - radius);
	LCD_write_data16(fch, fcl);

	setXY(x + radius, y, x + radius, y);
	LCD_write_data16(fch, fcl);

	setXY(x - radius, y, x - radius, y);
	LCD_write_data16(fch, fcl);

	while(x1 < y1)
	{
		if(f >= 0)
		{
			y1--;
			ddF_y += 2;
			f += ddF_y;
		}
		x1++;
		ddF_x += 2;
		f += ddF_x;
		setXY(x + x1, y + y1, x + x1, y + y1);
		LCD_write_data16(fch, fcl);

		setXY(x - x1, y + y1, x - x1, y + y1);
		LCD_write_data16(fch, fcl);

		setXY(x + x1, y - y1, x + x1, y - y1);
		LCD_write_data16(fch, fcl);

		setXY(x - x1, y - y1, x - x1, y - y1);
		LCD_write_data16(fch, fcl);

		setXY(x + y1, y + x1, x + y1, y + x1);
		LCD_write_data16(fch, fcl);

		setXY(x - y1, y + x1, x - y1, y + x1);
		LCD_write_data16(fch, fcl);

		setXY(x + y1, y - x1, x + y1, y - x1);
		LCD_write_data16(fch, fcl);

		setXY(x - y1, y - x1, x - y1, y - x1);
		LCD_write_data16(fch, fcl);
	}
	//sbi(P_CS, B_CS);
	//clrXY();
}

////////////////////////////////////////////////////////////////////////////////
// rysuje koło
// Parametry:
//  x, y - współrzdne środka
//  radius - promień
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void fillCircle(uint16_t x, uint16_t y, uint16_t radius)
{
	int16_t y1, x1;

	for(y1=-radius; y1<=0; y1++)
		for(x1=-radius; x1<=0; x1++)
			if(x1*x1+y1*y1 <= radius*radius)
			{
				drawHLine(x+x1, y+y1, 2*(-x1));
				drawHLine(x+x1, y-y1, 2*(-x1));
				break;
			}
}



////////////////////////////////////////////////////////////////////////////////
// wyświetla bitmapę po jednym pikselu
// Parametry:
//  x, y - współrzędne ekranu
//  sx, sy - rozmiar bitmapy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawBitmap(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data)
{
	uint16_t col;
	uint32_t tx, ty, tc;

	if (chOrient == POZIOMO)
	{
		setXY(x, y, x+sx-1, y+sy-1);
		for (tc=0; tc<(sx*sy); tc++)
		{
			col = data[tc];
			LCD_write_data16(col>>8, col);
		}
	}
	else
	{
		for (ty=0; ty<sy; ty++)
		{
			setXY(x, y+ty, x+sx-1, y+ty);
			for (tx=sx-1; tx>=0; tx--)
			{
				col = data[(ty*sx)+tx];
				LCD_write_data16(col>>8, col);
			}
		}
	}
	//clrXY();
}



////////////////////////////////////////////////////////////////////////////////
// wyświetla bitmapę przesyłajac po 16 bajtów bufora - zła kolejność kolorów
// Parametry:
//  x, y - współrzędne ekranu
//  sx, sy - rozmiar bitmapy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawBitmap2(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, uint16_t* data)
{
//	uint16_t col;
	uint32_t tc;

	if (chOrient == POZIOMO)
	{
		setXY(x, y, x+sx-1, y+sy-1);
		for (tc=0; tc<(sx*sy/8); tc++)
		{
			//col = data[tc];
			//LCD_write_data16(col>>8, col & 0xff);
			LCD_WrData((uint8_t*)data + (tc*16), 16);
			//LCD_WrData16(data+(tc*8), 8);
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// wyświetla bitmapę po 4 piksele z rotacją bajtów w wewnętrznym buforze. Dobre kolory i trochę szybciej niż po jednym pikselu
// Parametry:
//  x, y - współrzędne ekranu
//  sx, sy - rozmiar bitmapy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawBitmap3(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data)
{
	uint16_t piksel;
	uint32_t tc;
	uint8_t n, bufor[8];

	if (chOrient == POZIOMO)
	{
		setXY(x, y, x+sx-1, y+sy-1);
		for (tc=0; tc<(sx*sy/4); tc++)
		{
			for (n=0; n<4; n++)
			{
				piksel = data[tc*4+n];
				bufor[n*2+0] = piksel >>8;
				bufor[n*2+1] = piksel;
			}
			LCD_WrData(bufor, 8);
		}
	}

}



////////////////////////////////////////////////////////////////////////////////
// wyświetla bitmapę po 4 piksele przez DMA z rotacją bajtów w wewnętrznym buforze. Dobre kolory i trochę szybciej niż po jednym pikselu
// Parametry:
//  x, y - współrzędne ekranu
//  sx, sy - rozmiar bitmapy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void drawBitmap4(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, const uint16_t* data)
{
	uint16_t piksel;
	uint32_t tc;
	uint8_t n, bufor[16];

	chDMAgotowe = 1;
	if (chOrient == POZIOMO)
	{
		setXY(x, y, x+sx-1, y+sy-1);
		for (tc=0; tc<(sx*sy/8); tc++)
		{
			for (n=0; n<8; n++)
			{
				piksel = data[tc*8+n];
				bufor[n*2+0] = piksel >>8;
				bufor[n*2+1] = piksel;
			}
			do {}
			while (!chDMAgotowe);
			chDMAgotowe = 0;
			LCD_WrDataDMA(bufor, 16);
		}
	}

}




////////////////////////////////////////////////////////////////////////////////
// Odczytuje pamięć obrazu wyświetlacza
// Parametry:
// x1, y1 współrzędne początku obszaru
// x2, y2 współrzędne końca obszaru
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void CzytajPamiecObrazu(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t* bufor)
{
	uint8_t dane[8];

	LCD_write_command16(0x2A);	//Column Address Set
	dane[1] = x1>>8;
	dane[3] = x1;
	dane[5] = x2>>8;
	dane[7] = x2;
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2B);	//Page Address Set
	dane[1] = y1>>8;
	dane[3] = y1;
	dane[5] = y2>>8;
	dane[7] = y2;
	LCD_WrData(dane, 8);

	LCD_write_command16(0x2E);	//Memory Read

	for (uint16_t n=y1; n<y2; n++)
	{
		LCD_data_read(bufor+n*(x2-x1), x2-x1);
	}
}
