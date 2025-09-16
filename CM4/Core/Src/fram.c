//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł obsługi pamięci FRAM FM25V02-G
//
// (c) Pit Lab 2004
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "fram.h"
#include "moduly_wew.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;
extern uint8_t chAdresModulu;	//bieżący adres ustawiony na dekoderze



////////////////////////////////////////////////////////////////////////////////
// Odczytuje rejestr statusu
// Parametry: brak
// Zwraca: odczytany bajt statusu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajStatusFRAM(void)
{
	uint8_t chStatus = FRAM_RDSR;

	if (chAdresModulu != ADR_FRAM)
		UstawDekoderModulow(ADR_FRAM);			//Ustaw dekoder adresów /CS.

	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
    HAL_SPI_Transmit(&hspi2, &chStatus, 1, HAL_MAX_DELAY);					//zapisz opcode
    HAL_SPI_Receive(&hspi2, &chStatus, 1, HAL_MAX_DELAY);					//czytaj dane
    HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);		//CS = 1
    return chStatus;
}



////////////////////////////////////////////////////////////////////////////////
// Zapisuje rejestr statusu
// Parametry: bajt statusu do zapisu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ZapiszStatusFRAM(uint8_t chStatus)
{
	uint8_t chDaneWew[2];

	if (chAdresModulu != ADR_FRAM)
		UstawDekoderModulow(ADR_FRAM);			//Ustaw dekoder adresów /CS.

	chDaneWew[0] = FRAM_WRSR;				//opcode
	chDaneWew[1] = chStatus;				//dane
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
    HAL_SPI_Transmit(&hspi2, chDaneWew, 2, HAL_MAX_DELAY);					//zapisz opcode i dane
    HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);		//CS = 1
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje 1 bajt z pamięci FRAM
// Parametry: sAdress - adres pamięci
// Zwraca: odczytany bajt danych
// Czas wykonania: 8,1us @40MHz
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajFRAM(uint16_t sAdres)
{
	uint8_t chDaneWew[3];
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 40MHz a układ może pracować z prędkością max 40MHz, przy każdym dostępie przestaw dzielnik zegara na 2
	nZastanaKonfiguracja_SPI_CFG1 = hspi2.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi2.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;		//maska preskalera
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_4;		//Bits 30:28 MBR[2:0]: master baud rate: SPI master clock/2

	if (chAdresModulu != ADR_FRAM)
		UstawDekoderModulow(ADR_FRAM);				//Ustaw dekoder adresów /CS.

	chDaneWew[0] = FRAM_READ;					//opcode
	chDaneWew[1] = (uint8_t)(sAdres >>8);		//adres H
	chDaneWew[2] = (uint8_t)(sAdres & 0x00FF);	//adres L

	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
    HAL_SPI_Transmit(&hspi2, chDaneWew, 3, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, chDaneWew, 1, HAL_MAX_DELAY);						//czytaj dane
    HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
    hspi2.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróc poprzednie nastawy
    return chDaneWew[0];
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje wiele bajtów zawartości pamięci FRAM
// Parametry: 
// [i] sAdres - adres pamięci
// [o] *chDaneWyj - wskaźnik na strukturę na odczytanye dane
// [i] chIlosc - ilość danych do odczytu
// Zwraca: nic
// Czas wykonania
////////////////////////////////////////////////////////////////////////////////
void CzytajBuforFRAM(uint16_t sAdres, uint8_t* chDaneWyj, uint16_t sIlosc)
{
	uint8_t chDaneWew[3];
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 40MHz a układ może pracować z prędkością max 40MHz, przy każdym dostępie przestaw dzielnik zegara na 2
	nZastanaKonfiguracja_SPI_CFG1 = hspi2.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi2.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;		//maska preskalera
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_4;		//Bits 30:28 MBR[2:0]: master baud rate: SPI master clock/2

	if (chAdresModulu != ADR_FRAM)
		UstawDekoderModulow(ADR_FRAM);				//Ustaw dekoder adresów /CS.

	chDaneWew[0] = FRAM_READ;					//opcode
	chDaneWew[1] = (uint8_t)(sAdres >>8);		//adres H
	chDaneWew[2] = (uint8_t)(sAdres & 0x00FF);	//adres L

	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
    HAL_SPI_Transmit(&hspi2, chDaneWew, 3, HAL_MAX_DELAY);						//zapisz opcode i adres
    HAL_SPI_Receive(&hspi2, chDaneWyj, sIlosc, HAL_MAX_DELAY);					//czytaj dane
    HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
    hspi2.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróc poprzednie nastawy
}



////////////////////////////////////////////////////////////////////////////////
// Zapisuje 1 bajt do pamięci FRAM
// Parametry: sAdress - adres pamięci
//	      chData - dana do zapisu
// Zwraca: nic
// Czas wykonania: 9,7us @40MHz
////////////////////////////////////////////////////////////////////////////////
void ZapiszFRAM(unsigned short sAdres, uint8_t chDane)
{
	uint8_t chDaneWew[4];
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 40MHz a układ może pracować z prędkością max 40MHz, przy każdym dostępie przestaw dzielnik zegara na 2
	nZastanaKonfiguracja_SPI_CFG1 = hspi2.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi2.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;		//maska preskalera
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_4;		//Bits 30:28 MBR[2:0]: master baud rate: SPI master clock/2

	if (chAdresModulu != ADR_FRAM)
		UstawDekoderModulow(ADR_FRAM);				//Ustaw dekoder adresów /CS.

	chDaneWew[0] = FRAM_WREN;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_Transmit(&hspi2, chDaneWew, 1, HAL_MAX_DELAY);					//zapisz opcode
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);		//CS = 1

	chDaneWew[0] = FRAM_WRITE;					//opcode
	chDaneWew[1] = (uint8_t)(sAdres >>8);		//adres H
	chDaneWew[2] = (uint8_t)(sAdres & 0x00FF);	//adres L
	chDaneWew[3] = chDane;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_Transmit(&hspi2, chDaneWew, 4, HAL_MAX_DELAY);						//zapisz opcode i dane
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	hspi2.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróc poprzednie nastawy
}



////////////////////////////////////////////////////////////////////////////////
// Zapisuje wiele bajtów do pamięci FRAM
// Parametry: 
// [i] sAdress - adres pamięci
// [i] *chData - wskaźnik na dane do zapisu
// [i] chCount - ilość danych do zapisu
// Zwraca: nic
// Czas wykonania: 20,1us dla 16bjatów @40MHz
////////////////////////////////////////////////////////////////////////////////
void ZapiszBuforFRAM(uint16_t sAdres, uint8_t* chDane, uint16_t sIlosc)
{
	uint8_t chDaneWew[3];
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//Ponieważ zegar SPI = 40MHz a układ może pracować z prędkością max 40MHz, przy każdym dostępie przestaw dzielnik zegara na 2
	nZastanaKonfiguracja_SPI_CFG1 = hspi2.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi2.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;		//maska preskalera
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_4;		//Bits 30:28 MBR[2:0]: master baud rate: SPI master clock/2

	if (chAdresModulu != ADR_FRAM)
		UstawDekoderModulow(ADR_FRAM);				//Ustaw dekoder adresów /CS.

	chDaneWew[0] = FRAM_WREN;
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_Transmit(&hspi2, chDaneWew, 1, HAL_MAX_DELAY);						//zapisz opcode
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1

	chDaneWew[0] = FRAM_WRITE;					//opcode
	chDaneWew[1] = (uint8_t)(sAdres >>8);		//adres H
	chDaneWew[2] = (uint8_t)(sAdres & 0x00FF);	//adres L
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_Transmit(&hspi2, chDaneWew, 3, HAL_MAX_DELAY);						//zapisz opcode i adres
	HAL_SPI_Transmit(&hspi2, chDane, sIlosc, HAL_MAX_DELAY);					//zapisz dane
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	hspi2.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróc poprzednie nastawy
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje rejestr ID pamięci
// Parametry:
// [o] *chDaneWyj - wskaźnik na strukturę 9 bajtów z ID
// Zwraca: odczytane ID
// Czas wykonania: 15,5us @10MHz
////////////////////////////////////////////////////////////////////////////////
void CzytajIdFRAM(uint8_t* chDaneID)
{
	uint8_t chID = FRAM_RDID;

	if (chAdresModulu != ADR_FRAM)
		UstawDekoderModulow(ADR_FRAM);			//Ustaw dekoder adresów /CS.

	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
    HAL_SPI_Transmit(&hspi2, &chID, 1, HAL_MAX_DELAY);							//zapisz opcode
    HAL_SPI_Receive(&hspi2, chDaneID, 9, HAL_MAX_DELAY);						//czytaj dane
    HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
}



////////////////////////////////////////////////////////////////////////////////
// Czyta z pamięci 4-bajtową zmienną typu float
// Parametry: sAdres - adres pamięci
// Zwraca: zmienna float
// Czas wykonania ok. 
////////////////////////////////////////////////////////////////////////////////
float CzytajFramFloat(uint16_t sAdres)
{
     typedef union 
    {
    uint8_t array[sizeof(float)];
	float fuData;
    } fUnion;
    fUnion temp;

	CzytajBuforFRAM(sAdres, temp.array, 4);
    return temp.fuData;
}


////////////////////////////////////////////////////////////////////////////////
// Zapisuje do pamięci 4-bajtową zmienną typu float
// Parametry: 
// [i] sAdress - adres pamięci
// [i] fData - dana do zapisu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
//void FramDataWriteFloat(unsigned short sAddress, float fData)
void ZapiszFramFloat(uint16_t sAdres, float fWartosc)
{
    typedef union 
    {
    uint8_t array[sizeof(float)];
	float fuData;
    } fUnion;
    fUnion temp;

    temp.fuData = fWartosc;
    ZapiszBuforFRAM(sAdres, temp.array, 4);
}


////////////////////////////////////////////////////////////////////////////////
// Czyta z pamięci 4-bajtową zmienną typu float i przeprowadza validację
// Parametry: 
// [o] *fWartosc - wskaźnik na zmienną do odczytu
// [i] sAdres - adres pamięci
// [i] fWartMin - wartość minimalna zmiennej
// [i] fWartMax - wartość maksymalna zmiennej
// [i] fWartDomyslna - gdy odczytana wartość nie spełnia warunków walidacji to przyjmuje tą wartość domyślną
// [i] chKodBledu - kod błedu jaki ma zwrócić funkcja w przypadku niespełnienia warunków walidacji
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajFramZWalidacja(uint16_t sAdres, float *fWartosc, float fWartMin, float fWartMax, float fWartDomyslna, uint8_t chKodBledu)
{
	uint8_t chLiczbaPowtorzen = 3;
	uint8_t chErr;

	do
	{
		*fWartosc = CzytajFramFloat(sAdres);
		if ((*fWartosc < fWartMin) || (*fWartosc > fWartMax) || isnan(*fWartosc))
		{
			*fWartosc = fWartDomyslna;    //gdy poza zakresem, to zwróć przekazaną wartość domyślną
			chErr = chKodBledu;
		}
		else
			chErr = BLAD_OK;
	}
	while (chErr && --chLiczbaPowtorzen);	//w przypadku jakiegokolwiek błędu powtórz całość
    return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Czyta z pamięci FRAM 2-bajtową zmienną typu uint16_t
// Parametry: sAdres - adres pamięci
// Zwraca: zmienna uint16_t
// Czas wykonania ok.
////////////////////////////////////////////////////////////////////////////////
uint16_t CzytajFramU16(uint16_t sAdres)
{
	union
	{
	    uint8_t U8[2];
	    uint16_t U16;
	} unia16;

	CzytajBuforFRAM(sAdres, unia16.U8, 2);
    return unia16.U16;
}



////////////////////////////////////////////////////////////////////////////////
// Zapisuje do pamięci FRAM 2-bajtową zmienną typu uint16_t
// Parametry:
// [i] sAdress - adres pamięci
// [i] uint16_t - dana do zapisu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ZapiszFramU16(uint16_t sAdres, uint16_t sWartosc)
{
	union
	{
	    uint8_t U8[2];
	    uint16_t U16;
	} unia16;

	unia16.U16 = sWartosc;
    ZapiszBuforFRAM(sAdres, unia16.U8, 2);
}
