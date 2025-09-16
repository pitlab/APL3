//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa modułów wewnętrznych
//
// (c) Pit Lab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "moduly_wew.h"
#include "main.h"


extern SPI_HandleTypeDef hspi2;
uint8_t chAdresModulu = 99;	//bieżący adres ustawiony na dekoderze


////////////////////////////////////////////////////////////////////////////////
// Inicjuje układy na magistrali wewnętrznej SPI2. Funkcja najwyższego poziomu, bez parametrów
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujModulyWew(void)
{
	uint8_t dane_wysylane[3];
	HAL_StatusTypeDef chErr;
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 80MHz a układ może pracować z prędkością max 10MHz, przy każdym dostępie przestaw dzielnik zegara na 8
	nZastanaKonfiguracja_SPI_CFG1 = hspi2.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi2.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;		//maska preskalera
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_16;

	UstawDekoderModulow(ADR_EXPIO);			//Ustaw dekoder adresów /CS.

	//ustaw rejestr konfiguracji układu exandera
	dane_wysylane[0] = SPI_EXTIO;	//Adres ukadu IO
	dane_wysylane[1] = MCP23S08_IOCON;
	dane_wysylane[2] = (1 << 5) |	//bit 5 SEQOP: Sequential Operation mode bit: 1 = Sequential operation disabled, address pointer does not increment, 0 = Sequential operation enabled, address pointer increments.
					   (0 << 4)	|	//bit 4 DISSLW: Slew Rate control bit for SDA output:  1 = Slew rate disabled,  0 = Slew rate enabled.
					   (0 << 3)	|	//bit 3 HAEN: Hardware Address Enable bit (MCP23S08 only): 1 = Enables the MCP23S08 address pins, 0 = Disables the MCP23S08 address pins.
					   (0 << 2)	|	//bit 2 ODR: This bit configures the INT pin as an open-drain output:  1 = Open-drain output (overrides the INTPOL bit), 0 = Active driver output (INTPOL bit sets the polarity).
					   (0 << 1);	//bit 1 INTPOL: This bit sets the polarity of the INT output pin: 1 = Active-high, 0 = Active-low.
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	chErr = HAL_SPI_Transmit(&hspi2, dane_wysylane, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1

	//ustaw rejestr kierunku portów układu exandera
	dane_wysylane[1] = MCP23S08_IODIR;	//I/O DIRECTION (IODIR) REGISTER: 1=input, 0=output
	dane_wysylane[2] = (0 << 0) |	//MOD_IO11
					   (0 << 1) |	//MOD_IO12
					   (0 << 2) |	//MOD_IO21
					   (0 << 3) |	//MOD_IO22
					   (0 << 4) |	//MOD_IO31
					   (0 << 5) |	//MOD_IO32
					   (0 << 6) |	//MOD_IO41
					   (0 << 7);	//MOD_IO42
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	chErr |= HAL_SPI_Transmit(&hspi2, dane_wysylane, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	hspi2.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróc poprzednie nastawy
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia zawartość portu na układzie rozszerzeń podłaczonym do magistrali SPI2 modułów wyjsciowych rdzenia CM4
// Parametry: adres - adres układu rozszerzeń
// 	daneWy - dane wyjściowe
// 	daneWe* - wskaźnika na dane wejściowe
// 	ilosc - liczba przesyłanych danych
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t WyslijDaneExpandera(uint8_t daneWy)
{
	HAL_StatusTypeDef chErr;
	uint8_t dane_wysylane[3];
	//uint8_t dane_odbierane[3];
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 80MHz a układ może pracować z prędkością max 10MHz, przy każdym dostępie przestaw dzielnik zegara na 8
	nZastanaKonfiguracja_SPI_CFG1 = hspi2.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi2.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;		//maska preskalera
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_16;

	UstawDekoderModulow(ADR_EXPIO);			//Ustaw dekoder adresów /CS.
	dane_wysylane[0] = SPI_EXTIO + SPI_EXTIO_WR;	//Adres układu IO
	dane_wysylane[1] = MCP23S08_GPIO;				//rejestr
	dane_wysylane[2] = daneWy;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	chErr = HAL_SPI_Transmit(&hspi2, dane_wysylane, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	hspi2.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróc poprzednie nastawy
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera zawartość portu na układzie rozszerzeń podłaczonym do magistrali SPI2 modułów wyjsciowych rdzenia CM4
// Parametry: adres - adres układu rozszerzeń
// 	daneWy - dane wyjściowe
// 	daneWe* - wskaźnika na dane wejściowe
// 	ilosc - liczba przesyłanych danych
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzDaneExpandera(uint8_t* daneWe)
{
	HAL_StatusTypeDef chErr;
	uint8_t dane_wysylane[3];
	uint8_t dane_odbierane[3];
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 80MHz a układ może pracować z prędkością max 10MHz, przy każdym dostępie przestaw dzielnik zegara na 8
	nZastanaKonfiguracja_SPI_CFG1 = hspi2.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi2.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;		//maska preskalera
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_8;

	UstawDekoderModulow(ADR_EXPIO);			//Ustaw dekoder adresów /CS.
	dane_wysylane[0] = SPI_EXTIO + SPI_EXTIO_RD;	//Adres ukadu IO
	dane_wysylane[1] = MCP23S08_GPIO;
	dane_wysylane[2] = 0;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	chErr = HAL_SPI_TransmitReceive(&hspi2, dane_wysylane, dane_odbierane, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	*daneWe = dane_odbierane[2];
	hspi2.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróc poprzednie nastawy
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia sygnału ChipSelect dla modułów i układów pamięci FRAM i Expandera IO
// Adres jesyt ustawiony na portach PK2, PK1 i PI8
// Parametry: modul - adres modułu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawDekoderModulow(uint8_t modul)
{
	uint8_t chErr = BLAD_OK;

	if (modul == chAdresModulu)
		return chErr;		//jest ustawione to co trzeba

	chAdresModulu = modul;	//bieżący adres ustawiony na dekoderze
	switch (modul)
	{
	case ADR_MOD1:	//0
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_RESET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_RESET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_RESET);	//ADR2
		break;

	case ADR_MOD2:	//1
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_SET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_RESET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_RESET);	//ADR2
		break;

	case ADR_MOD3:	//2
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_RESET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_SET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_RESET);	//ADR2
		break;

	case ADR_MOD4:	//3
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_SET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_SET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_RESET);	//ADR2
		break;

	case ADR_NIC:	//4
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_RESET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_RESET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_SET);	//ADR2
		break;

	case ADR_EXPIO:	//5
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_SET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_RESET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_SET);	//ADR2
		break;

	case ADR_FRAM:	//6
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_RESET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_SET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_SET);	//ADR2
		break;

	default:	chErr = ERR_ZLY_ADRES;	break;
	}

	return chErr;
}


////////////////////////////////////////////////////////////////////////////////
// Ustawia adres na liniach ADR0_NA_MOD..ADR1_NA_MOD idących do modułu aby móć zaadresować układy na module
// Adres jest ustawiony na portach PD3 i PD6
// Parametry: modul - adres modułu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawAdresNaModule(uint8_t chAdres)
{
	uint8_t chErr = BLAD_OK;

	chAdres &= 0x03;	//ustawiamy tylko najmłodsze 2 bity

	switch (chAdres)
	{
	case 0:
		HAL_GPIO_WritePin(SUB_MOD_ADR0_GPIO_Port, SUB_MOD_ADR0_Pin, GPIO_PIN_RESET);	//ADR0_NA_MOD
		HAL_GPIO_WritePin(SUB_MOD_ADR1_GPIO_Port, SUB_MOD_ADR1_Pin, GPIO_PIN_RESET);	//ADR1_NA_MOD
		break;

	case 1:
		HAL_GPIO_WritePin(SUB_MOD_ADR0_GPIO_Port, SUB_MOD_ADR0_Pin, GPIO_PIN_SET);		//ADR0_NA_MOD
		HAL_GPIO_WritePin(SUB_MOD_ADR1_GPIO_Port, SUB_MOD_ADR1_Pin, GPIO_PIN_RESET);	//ADR1_NA_MOD
		break;

	case 2:
		HAL_GPIO_WritePin(SUB_MOD_ADR0_GPIO_Port, SUB_MOD_ADR0_Pin, GPIO_PIN_RESET);	//ADR0_NA_MOD
		HAL_GPIO_WritePin(SUB_MOD_ADR1_GPIO_Port, SUB_MOD_ADR1_Pin, GPIO_PIN_SET);		//ADR1_NA_MOD
		break;

	case 3:
		HAL_GPIO_WritePin(SUB_MOD_ADR0_GPIO_Port, SUB_MOD_ADR0_Pin, GPIO_PIN_SET);		//ADR0_NA_MOD
		HAL_GPIO_WritePin(SUB_MOD_ADR1_GPIO_Port, SUB_MOD_ADR1_Pin, GPIO_PIN_SET);		//ADR1_NA_MOD
		break;

		default:	chErr = ERR_ZLY_ADRES;	break;
	}
	return chErr;
}
