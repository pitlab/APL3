//////////////////////////////////////////////////////////////////////////////
//
// Niskopoziomowe funkcje sterujące wyświetlaczem na magistrali SPI
// Niezależne od sterownika wyświetlacza
// Przyjmuję że są zbyt niskopoziomowe aby chronić je semaforami, gdyż byłoby to
// zbyt kosztowne czasowo ze względy na to że funkcje są wywyoływane w w dużych pętlach.
// Semaforami shronione są funkcje ich używajace.
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "LCD/LCD_SPI.h"
#include "Ekran.h"
#include "ModulySPI.h"
#include <math.h>
#include <Main.h>
#include <stdio.h>
#include <string.h>



//deklaracje zmiennych
extern SPI_HandleTypeDef hspi5;



////////////////////////////////////////////////////////////////////////////////
// Wysyła polecenie do wyświetlacza LCD
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: chDane - polecenie wysłane w jednej ramce ograniczonej CS
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t LCD_write_command16(uint8_t chDane)
{
	HAL_StatusTypeDef cBłąd;
	uint8_t dane_nadawane[2];

	dane_nadawane[0] = 0x00;
	dane_nadawane[1] = chDane;
	UstawDekoderZewn(CS_LCD);											//LCD_CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);	//LCD_RS=0
	cBłąd = HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_DELAY_SPI);
	UstawDekoderZewn(CS_NIC);											//LCD_CS=1
	return cBłąd;
}

uint8_t LCD_write_command8(uint8_t chDane)
{
	HAL_StatusTypeDef cBłąd;

	//Ponieważ zegar SPI = 38,46 MHz a wyświetlacz może pracować z prędkością max 20MHz więc przy każdym poleceniu ustaw dzielnik zegara na 2 => 19,23 MHz
	hspi5.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;	//maska preskalera
	hspi5.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_4;

	UstawDekoderZewn(CS_LCD);											//LCD_CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);	//LCD_RS=0
	cBłąd = HAL_SPI_Transmit(&hspi5, &chDane, 1, HAL_DELAY_SPI);
	UstawDekoderZewn(CS_NIC);											//LCD_CS=1
	return cBłąd;
}


////////////////////////////////////////////////////////////////////////////////
// Wysyła do wyświetlacza LCD  bufor danych w jednej ramce ograniczonej CS. Steruje liniami RS i CS
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: *chDane - wskaźnika na bufor z danymi wysłane
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t LCD_WrData(uint8_t* chDane, uint16_t sIlosc)
{
	HAL_StatusTypeDef cBłąd;

	UstawDekoderZewn(CS_LCD);										//LCD_CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
	cBłąd = HAL_SPI_Transmit(&hspi5, chDane, sIlosc, HAL_DELAY_SPI);
	UstawDekoderZewn(CS_NIC);										//LCD_CS=1
	return cBłąd;
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
	HAL_StatusTypeDef cBłąd;
	uint8_t dane_nadawane[2];

	dane_nadawane[0] = chDane1;
	dane_nadawane[1] = chDane2;
	UstawDekoderZewn(CS_LCD);										//LCD_CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
	cBłąd = HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_DELAY_SPI);
	UstawDekoderZewn(CS_NIC);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła jedyne dane do wyświetlacza LCD w trybie 16-bitowym, steruje liniami RS i CS na początku i końcu
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: chDane
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t LCD_write_dat_jed16(uint8_t chDane)
{
	HAL_StatusTypeDef cBłąd;
	uint8_t dane_nadawane[2];

	dane_nadawane[0] = 0x00;
	dane_nadawane[1] = chDane;
	UstawDekoderZewn(CS_LCD);										//LCD_CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
	cBłąd = HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_DELAY_SPI);
	UstawDekoderZewn(CS_NIC);
	return cBłąd;
}


uint8_t LCD_write_dat_jed8(uint8_t chDane)
{
	HAL_StatusTypeDef cBłąd;
	UstawDekoderZewn(CS_LCD);										//LCD_CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
	cBłąd = HAL_SPI_Transmit(&hspi5, &chDane, 1, HAL_DELAY_SPI);
	UstawDekoderZewn(CS_NIC);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła pierwsze dane do wyświetlacza LCD w trybie 16-bitowym, steruje liniami RS i CS na początku
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: chDane
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t LCD_write_dat_pie16(uint8_t chDane)
{
	HAL_StatusTypeDef cBłąd;
	uint8_t dane_nadawane[2];

	dane_nadawane[0] = 0x00;
	dane_nadawane[1] = chDane;

	UstawDekoderZewn(CS_LCD);										//LCD_CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
	cBłąd = HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_DELAY_SPI);
	return cBłąd;
}


uint8_t LCD_write_dat_pie8(uint8_t chDane)
{
	HAL_StatusTypeDef cBłąd;

	UstawDekoderZewn(CS_LCD);										//LCD_CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
	cBłąd = HAL_SPI_Transmit(&hspi5, &chDane, 1, HAL_DELAY_SPI);
	return cBłąd;
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
	dane_nadawane[0] = 0x00;
	dane_nadawane[1] = chDane;

	HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_DELAY_SPI);
}


void LCD_write_dat_sro8(uint8_t chDane)
{
	HAL_SPI_Transmit(&hspi5, &chDane, 1, HAL_DELAY_SPI);
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
	dane_nadawane[0] = 0x00;
	dane_nadawane[1] = chDane;

	HAL_SPI_Transmit(&hspi5, dane_nadawane, 2, HAL_DELAY_SPI);
	UstawDekoderZewn(CS_NIC);										//LCD_CS=1
}


void LCD_write_dat_ost8(uint8_t chDane)
{
	HAL_SPI_Transmit(&hspi5, &chDane, 1, HAL_DELAY_SPI);
	UstawDekoderZewn(CS_NIC);										//LCD_CS=1
}



////////////////////////////////////////////////////////////////////////////////
// Czyta dane z wyświetlacza LCD bez sterowania liniami CS i DC
// Zawiera semafor kontrolujący dostęp do SPI.Funkcja czeka na zwolnienie semafora
// Parametry: *chDane - wskaźnik na zwracane dane
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void LCD_data_read(uint8_t *chDane, uint8_t chIlosc)
{
	UstawDekoderZewn(CS_LCD);										//LCD_CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	//LCD_RS=1
	HAL_SPI_Receive(&hspi5, chDane, chIlosc, 2);
	UstawDekoderZewn(CS_NIC);										//LCD_CS=1
}



