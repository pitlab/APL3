//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa modułów wewnętrznych na magistrali SPI
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "moduly_SPI.h"
#include "errcode.h"
#include "main.h"
#include "sys_def_CM7.h"

//deklaracje zmiennych
uint8_t chPorty_exp_wysylane[LICZBA_EXP_SPI_ZEWN] = {0x00, 0x00, 0xE0};
uint8_t chPorty_exp_odbierane[LICZBA_EXP_SPI_ZEWN];
extern SPI_HandleTypeDef hspi5;
const uint8_t chAdresy_expanderow[LICZBA_EXP_SPI_ZEWN] = {SPI_EXTIO_0, SPI_EXTIO_1, SPI_EXTIO_2};


////////////////////////////////////////////////////////////////////////////////
// Inicjuje układy na magistrali zewnętrznej SPI5. Funkcja najwyższego poziomu, bez parametrów
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujSPIModZewn(void)
{
	uint8_t dane_wysylane[3];
	uint8_t dane_odbierane[3];
	HAL_StatusTypeDef Err;

	dane_wysylane[0] = SPI_EXTIO_0;	//teraz komunikacja z U41

	//ustaw rejestr konfiguracji układu exandera U41
	dane_wysylane[1] = MCP23S08_IOCON;
	dane_wysylane[2] = (1 << 5) |	//bit 5 SEQOP: Sequential Operation mode bit: 1 = Sequential operation disabled, address pointer does not increment, 0 = Sequential operation enabled, address pointer increments.
					   (0 << 4)	|	//bit 4 DISSLW: Slew Rate control bit for SDA output:  1 = Slew rate disabled,  0 = Slew rate enabled.
					   (1 << 3)	|	//bit 3 HAEN: Hardware Address Enable bit (MCP23S08 only): 1 = Enables the MCP23S08 address pins, 0 = Disables the MCP23S08 address pins.
					   (0 << 2)	|	//bit 2 ODR: This bit configures the INT pin as an open-drain output:  1 = Open-drain output (overrides the INTPOL bit), 0 = Active driver output (INTPOL bit sets the polarity).
					   (1 << 1);	//bit 1 INTPOL: This bit sets the polarity of the INT output pin: 1 = Active-high, 0 = Active-low.
	UstawDekoderZewn(CS_IO);
	Err = HAL_SPI_TransmitReceive(&hspi5, dane_wysylane, dane_odbierane, 3, HAL_MAX_DELAY);
	Err |= UstawDekoderZewn(CS_NIC);

	//ustaw rejestr kierunku portów układu exandera U41
	dane_wysylane[1] = MCP23S08_IODIR;	//I/O DIRECTION (IODIR) REGISTER: 1=input, 0=output
	dane_wysylane[2] = (0 << 7) |	//MOD_OSW_IO2
					   (0 << 6) |	//MOD_OSW_IO1
					   (0 << 5) |	//USB_HOST_DEVICE
					   (1 << 4) |	//LOG_SD1_CDETECT - wejscie detekcji obecności karty
					   (0 << 3) |	//CAM_RES
					   (0 << 2) |	//LOG_SD1_VSEL
					   (0 << 1) |	//LCD_RES
					   (1 << 0);	//TP_INT - wejście przerwań panelu dotykowego LCD
	UstawDekoderZewn(CS_IO);
	Err |= HAL_SPI_TransmitReceive(&hspi5, dane_wysylane, dane_odbierane, 3, HAL_MAX_DELAY);
	Err |= UstawDekoderZewn(CS_NIC);


	dane_wysylane[0] = SPI_EXTIO_1;	//teraz komunikacja z U42

	//ustaw rejestr konfiguracji układu exandera U42
	dane_wysylane[1] = MCP23S08_IOCON;
	dane_wysylane[2] = (1 << 5) |	//bit 5 SEQOP: Sequential Operation mode bit: 1 = Sequential operation disabled, address pointer does not increment, 0 = Sequential operation enabled, address pointer increments.
					   (0 << 4)	|	//bit 4 DISSLW: Slew Rate control bit for SDA output:  1 = Slew rate disabled,  0 = Slew rate enabled.
					   (1 << 3)	|	//bit 3 HAEN: Hardware Address Enable bit (MCP23S08 only): 1 = Enables the MCP23S08 address pins, 0 = Disables the MCP23S08 address pins.
					   (0 << 2)	|	//bit 2 ODR: This bit configures the INT pin as an open-drain output:  1 = Open-drain output (overrides the INTPOL bit), 0 = Active driver output (INTPOL bit sets the polarity).
					   (1 << 1);	//bit 1 INTPOL: This bit sets the polarity of the INT output pin: 1 = Active-high, 0 = Active-low.
	UstawDekoderZewn(CS_IO);
	Err |= HAL_SPI_TransmitReceive(&hspi5, dane_wysylane, dane_odbierane, 3, HAL_MAX_DELAY);
	Err |= UstawDekoderZewn(CS_NIC);

	//ustaw rejestr kierunku portów układu exandera U42
	dane_wysylane[1] = MCP23S08_IODIR;	//I/O DIRECTION (IODIR) REGISTER: 1=input, 0=output
	dane_wysylane[2] = (0 << 7) |	//USB_EN - włącznik transmisji USB
					   (0 << 6) |	//BMS_I2C_SW - przełacznik zegara I2C miedzy pakietami
					   (1 << 5) |	//ETH_RMII_EXER - wejście sygnału błędu transmisji ETH
					   (0 << 4) |	//AUDIO_OUT_SD - włączniek ShutDown wzmacniacza audio
					   (0 << 3) |	//AUDIO_IN_SD - włącznika ShutDown mikrofonu
					   (0 << 2) |	//MODZ_CAN_STBY - włącznie Standby sterownika CAN
					   (0 << 1) |	//USB_POWER - wlącznik zasilania dla Device
					   (1 << 0);	//USB_OVERCURRENT - wejście wygnalizujące przekroczenie poboru prądu przez USB device
	UstawDekoderZewn(CS_IO);
	Err |= HAL_SPI_TransmitReceive(&hspi5, dane_wysylane, dane_odbierane, 3, HAL_MAX_DELAY);
	Err |= UstawDekoderZewn(CS_NIC);

	dane_wysylane[0] = SPI_EXTIO_2;	//teraz komunikacja z U43

	//ustaw rejestr konfiguracji układu exandera U43
	dane_wysylane[1] = MCP23S08_IOCON;
	dane_wysylane[2] = (1 << 5) |	//bit 5 SEQOP: Sequential Operation mode bit: 1 = Sequential operation disabled, address pointer does not increment, 0 = Sequential operation enabled, address pointer increments.
					   (0 << 4)	|	//bit 4 DISSLW: Slew Rate control bit for SDA output:  1 = Slew rate disabled,  0 = Slew rate enabled.
					   (1 << 3)	|	//bit 3 HAEN: Hardware Address Enable bit (MCP23S08 only): 1 = Enables the MCP23S08 address pins, 0 = Disables the MCP23S08 address pins.
					   (0 << 2)	|	//bit 2 ODR: This bit configures the INT pin as an open-drain output:  1 = Open-drain output (overrides the INTPOL bit), 0 = Active driver output (INTPOL bit sets the polarity).
					   (1 << 1);	//bit 1 INTPOL: This bit sets the polarity of the INT output pin: 1 = Active-high, 0 = Active-low.
	UstawDekoderZewn(CS_IO);
	Err |= HAL_SPI_TransmitReceive(&hspi5, dane_wysylane, dane_odbierane, 3, HAL_MAX_DELAY);
	Err |= UstawDekoderZewn(CS_NIC);

	//ustaw rejestr kierunku portów układu exandera U43
	dane_wysylane[1] = MCP23S08_IODIR;	//I/O DIRECTION (IODIR) REGISTER: 1=input, 0=output
	dane_wysylane[2] = (0 << 7) |	//LED_R
					   (0 << 6) |	//LED_G
					   (0 << 5) |	//LED_B
					   (1 << 4) |	//WL/WYL - wejście
					   (0 << 3) |	//ZASIL_LED - wyjście
					   (0 << 2) |	//ZASIL_WL_USB - wyjście
					   (0 << 1) |	//ZASIL_WL_WE2 - wyjście
					   (0 << 0);	//ZASIL_WL_WE1 - wyjście
	UstawDekoderZewn(CS_IO);
	Err |= HAL_SPI_TransmitReceive(&hspi5, dane_wysylane, dane_odbierane, 3, HAL_MAX_DELAY);
	Err |= UstawDekoderZewn(CS_NIC);
	return Err;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia sygnały Chip Select dla wybranego układu na magistrali SPI modułów zewnetrznych:
// TP_CS - Panel dotykowy, LCD_CS - wyświetlacz, IO_CS - extendery IO
// Parametry: uklad - identyfikator układu na magistrali
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawDekoderZewn(uint8_t uklad)
{
	uint8_t Err = ERR_OK;

	switch (uklad)
	{
		case CS_TP: 	//Panel dotykowy,
			HAL_GPIO_WritePin(MODZ_ADR0_GPIO_Port, MODZ_ADR0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MODZ_ADR1_GPIO_Port, MODZ_ADR1_Pin, GPIO_PIN_RESET);
			break;

		case CS_LCD:	//wyświetlacz,
			HAL_GPIO_WritePin(MODZ_ADR0_GPIO_Port, MODZ_ADR0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MODZ_ADR1_GPIO_Port, MODZ_ADR1_Pin, GPIO_PIN_RESET);
			break;

		case CS_IO:		//extendery IO
			HAL_GPIO_WritePin(MODZ_ADR0_GPIO_Port, MODZ_ADR0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MODZ_ADR1_GPIO_Port, MODZ_ADR1_Pin, GPIO_PIN_SET);
			break;

		case CS_NIC:	//nic nie jest wybrane
			HAL_GPIO_WritePin(MODZ_ADR0_GPIO_Port, MODZ_ADR0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MODZ_ADR1_GPIO_Port, MODZ_ADR1_Pin, GPIO_PIN_SET);
			break;

		default:	Err = ERR_ZLY_ADRES;
	}

	return Err;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia i pobiera zawartość portu na układzie rozszerzeń podłaczonym do magistrali SPI5 modułów wyjsciowych rdzenia CM7
// Parametry: adres - adres układu rozszerzeń
// 	daneWy - dane wyjściowe
// 	daneWe* - wskaźnika na dane wejściowe
// 	ilosc - liczba przesyłanych danych
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t WyslijDaneExpandera(uint8_t adres, uint8_t daneWy)
{
	HAL_StatusTypeDef Err;
	uint8_t dane_wysylane[3];
	uint8_t dane_odbierane[3];

	//ustaw rejestr kierunku portów układu exandera U43
	dane_wysylane[0] = adres;
	dane_wysylane[1] = MCP23S08_GPIO;
	dane_wysylane[2] = daneWy;
	UstawDekoderZewn(CS_IO);
	Err = HAL_SPI_TransmitReceive(&hspi5, dane_wysylane, dane_odbierane, 3, HAL_MAX_DELAY);
	UstawDekoderZewn(CS_NIC);
	return Err;
}

////////////////////////////////////////////////////////////////////////////////
// Ustawia i pobiera zawartość portu na układzie rozszerzeń podłaczonym do magistrali SPI5 modułów wyjsciowych rdzenia CM7
// Parametry: adres - adres układu rozszerzeń
// 	daneWy - dane wyjściowe
// 	daneWe* - wskaźnika na dane wejściowe
// 	ilosc - liczba przesyłanych danych
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzDaneExpandera(uint8_t adres, uint8_t* daneWe)
{
	HAL_StatusTypeDef Err;
	uint8_t dane_wysylane[3];
	uint8_t dane_odbierane[3];

	//ustaw rejestr kierunku portów układu exandera U43
	dane_wysylane[0] = adres + SPI_EXTIO_RD;
	dane_wysylane[1] = MCP23S08_GPIO;
	dane_wysylane[2] = 0;
	UstawDekoderZewn(CS_IO);
	Err = HAL_SPI_TransmitReceive(&hspi5, dane_wysylane, dane_odbierane, 3, HAL_MAX_DELAY);
	UstawDekoderZewn(CS_NIC);
	*daneWe = dane_odbierane[2];
	return Err;
}

////////////////////////////////////////////////////////////////////////////////
// Ustawia i pobiera zawartość portu na układach rozszerzeń podłaczonych do magistrali SPI5 modułów wyjsciowych rdzenia CM7
// Funkcja najwyższego poziomu do uruchamiania z pętli głównej, pracuje na zmiennych globalnych
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t WymienDaneExpanderow(void)
{
	HAL_StatusTypeDef Err;
	for (uint8_t x=0; x<LICZBA_EXP_SPI_ZEWN; x++)
	{
		Err = WyslijDaneExpandera(chAdresy_expanderow[x], chPorty_exp_wysylane[x]);
		if (Err != ERR_OK)
			return Err;
		Err = PobierzDaneExpandera(chAdresy_expanderow[x], &chPorty_exp_odbierane[x]);
		if (Err != ERR_OK)
			return Err;
	}


	//chPorty_exp_wysylane[2] ^= EXP27_LED_R;	//LED_R
	//chPorty_exp_wysylane[2] ^= EXP26_LED_G;	//LED_G
	chPorty_exp_wysylane[2] ^= EXP25_LED_B;	//LED_B
	return Err;
}

