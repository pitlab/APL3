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
#include "semafory.h"

//deklaracje zmiennych
uint8_t chPort_exp_wysylany[LICZBA_EXP_SPI_ZEWN] = {0x04, 0x00, 0xE0};
uint8_t chPort_exp_odbierany[LICZBA_EXP_SPI_ZEWN];
uint8_t chStanDekoderaSPI;
volatile uint8_t chCzasSwieceniaLED[LICZBA_LED];	//czas świecenia liczony w kwantach 0,1s jest zmniejszany w przerwaniu TIM17_IRQHandler
extern SPI_HandleTypeDef hspi5;
const uint8_t chAdres_expandera[LICZBA_EXP_SPI_ZEWN] = {SPI_EXTIO_0, SPI_EXTIO_1, SPI_EXTIO_2};
extern uint32_t nZainicjowano[2];		//flagi inicjalizacji sprzętu

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
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 100MHz a układ może pracować z prędkością max 10MHz a jest na tej samej magistrali co TFT przy każdym odczytcie przestaw dzielnik zegara z 4 na 16 => 6,25MHz
	nZastanaKonfiguracja_SPI_CFG1 = hspi5.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi5.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;	//maska preskalera
	hspi5.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_16;	//Bits 30:28 MBR[2:0]: master baud rate: 011: SPI master clock/16

	dane_wysylane[0] = SPI_EXTIO_0;	//teraz komunikacja z U41

	//ustaw rejestr konfiguracji układu exandera U41
	dane_wysylane[1] = MCP23S08_IOCON;
	dane_wysylane[2] = (1 << 5) |	//bit 5 SEQOP: Sequential Operation mode bit: 1 = Sequential operation disabled, address pointer does not increment, 0 = Sequential operation enabled, address pointer increments.
					   (0 << 4)	|	//bit 4 DISSLW: Slew Rate control bit for SDA output:  1 = Slew rate disabled,  0 = Slew rate enabled.
					   (1 << 3)	|	//bit 3 HAEN: Hardware Address Enable bit (MCP23S08 only): 1 = Enables the MCP23S08 address pins, 0 = Disables the MCP23S08 address pins.
					   (1 << 2)	|	//bit 2 ODR: This bit configures the INT pin as an open-drain output:  1 = Open-drain output (overrides the INTPOL bit), 0 = Active driver output (INTPOL bit sets the polarity).
					   (0 << 1);	//bit 1 INTPOL: This bit sets the polarity of the INT output pin: 1 = Active-high, 0 = Active-low.
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
					   (1 << 2)	|	//bit 2 ODR: This bit configures the INT pin as an open-drain output:  1 = Open-drain output (overrides the INTPOL bit), 0 = Active driver output (INTPOL bit sets the polarity).
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
					   (1 << 2)	|	//bit 2 ODR: This bit configures the INT pin as an open-drain output:  1 = Open-drain output (overrides the INTPOL bit), 0 = Active driver output (INTPOL bit sets the polarity).
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

	//włącz niebieskiego LEDa sygnalizujacego konfigurację lub kalibracje
	chPort_exp_wysylany[2] |= EXP27_LED_CZER | EXP26_LED_ZIEL;		//wyłącz LED_CZER, wyłącz LED_ZIEL
	chPort_exp_wysylany[2] &= ~EXP25_LED_NIEB;		//włącz LED_NIEB
	Err |= WyslijDaneExpandera(SPI_EXTIO_2, chPort_exp_wysylany[2]);

	nZainicjowano[0] |= INIT0_EXPANDER_IO;
	hspi5.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróć wcześniejszą konfigurację
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
	chStanDekoderaSPI = uklad;
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
// Zwraca ostatnio ustawiony stan dekodera
// Parametry: nic
// Zwraca: stan dekodera
////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzStanDekoderaZewn(void)
{
	return chStanDekoderaSPI;
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
	HAL_StatusTypeDef chErr;
	uint8_t dane_wysylane[3];
	uint32_t nStanSemaforaSPI;

	dane_wysylane[0] = adres;
	dane_wysylane[1] = MCP23S08_GPIO;
	dane_wysylane[2] = daneWy;

	//użyj sprzętowego semafora HSEM_SPI5_WYSW do określenia dostępu do SPI6
	nStanSemaforaSPI = HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW);
	if (!nStanSemaforaSPI)
	{
		chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
		if (chErr == ERR_OK)
		{
			UstawDekoderZewn(CS_IO);
			chErr = HAL_SPI_Transmit(&hspi5, dane_wysylane, 3, HAL_MAX_DELAY);
			UstawDekoderZewn(CS_NIC);
			HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
		}
	}
	else
		chErr = ERR_ZAJETY_SEMAFOR;
	return chErr;
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
	HAL_StatusTypeDef chErr;
	uint8_t dane_wysylane[3];
	uint8_t dane_odbierane[3];
	uint32_t nStanSemaforaSPI;

	dane_wysylane[0] = adres + SPI_EXTIO_RD;
	dane_wysylane[1] = MCP23S08_GPIO;
	dane_wysylane[2] = 0;
	//użyj sprzętowego semafora HSEM_SPI5_WYSW do określenia dostępu do SPI6
	nStanSemaforaSPI = HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW);
	if (!nStanSemaforaSPI)
	{
		chErr = HAL_HSEM_Take(HSEM_SPI5_WYSW, 0);
		if (chErr == ERR_OK)
		{
			UstawDekoderZewn(CS_IO);
			chErr = HAL_SPI_TransmitReceive(&hspi5, dane_wysylane, dane_odbierane, 3, HAL_MAX_DELAY);
			UstawDekoderZewn(CS_NIC);
			HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
		}
	}
	else
		chErr = ERR_ZAJETY_SEMAFOR;
	*daneWe = dane_odbierane[2];
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia i pobiera zawartość portu na układach rozszerzeń podłaczonych do magistrali SPI5 modułów wyjsciowych rdzenia CM7
// Funkcja najwyższego poziomu do uruchamiania z pętli głównej, pracuje na zmiennych globalnych
// Wymaga zewnętrznego objecia odchoną semafora aby uniknąc kolizji dostepu do SPI z wątkiem wyświetlacza
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t WymienDaneExpanderow(void)
{
	HAL_StatusTypeDef chErr;
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 100MHz a układ może pracować z prędkością max 10MHz a jest na tej samej magistrali co TFT przy każdym odczytcie przestaw dzielnik zegara z 4 na 16 => 6,25MHz
	nZastanaKonfiguracja_SPI_CFG1 = hspi5.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi5.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;	//maska preskalera
	hspi5.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_16;	//Bits 30:28 MBR[2:0]: master baud rate: 011: SPI master clock/16

	//ustaw bieżący stan LED-ów
	if (chCzasSwieceniaLED[LED_CZER])
		chPort_exp_wysylany[2] &= ~EXP27_LED_CZER;		//włącz LED_CZER
	else
		chPort_exp_wysylany[2] |= EXP27_LED_CZER;		//wyłącz LED_CZER

	if (chCzasSwieceniaLED[LED_ZIEL])
		chPort_exp_wysylany[2] &= ~EXP26_LED_ZIEL;		//włącz LED_ZIEL
	else
		chPort_exp_wysylany[2] |= EXP26_LED_ZIEL;		//wyłącz LED_ZIEL

	if (chCzasSwieceniaLED[LED_NIEB])
		chPort_exp_wysylany[2] &= ~EXP25_LED_NIEB;		//włącz LED_NIEB
	else
		chPort_exp_wysylany[2] |= EXP25_LED_NIEB;		//wyłącz LED_NIEB

	//wyślij dane do expanderów I/O
	for (uint8_t x=0; x<LICZBA_EXP_SPI_ZEWN; x++)
	{
		chErr = WyslijDaneExpandera(chAdres_expandera[x], chPort_exp_wysylany[x]);
		if (chErr != ERR_OK)
			return chErr;
		chErr = PobierzDaneExpandera(chAdres_expandera[x], &chPort_exp_odbierany[x]);
		if (chErr != ERR_OK)
			return chErr;
	}
	hspi5.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróć wcześniejszą konfigurację
	return chErr;
}

