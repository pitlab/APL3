//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika ciśnienia BMP581 na magistrali SPI
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "BMP581.h"
#include "wymiana_CM4.h"
#include "petla_glowna.h"
#include "spi.h"
#include "petla_glowna.h"
#include "main.h"
#include "modul_IiP.h"

// Dopuszczalna prędkość magistrali 1..12MHz
extern SPI_HandleTypeDef hspi2;
extern volatile unia_wymianyCM4_t uDaneCM4;
static uint8_t chProporcjaPomiarow;
static float fP0_BMP581 = 0.0f;	//ciśnienie P0 do obliczeń wysokości [Pa]
static uint8_t chBufBMP581[4];
static uint16_t sLicznikUsrednianiaP0 = 0;			//licznik uśredniania ciśnienia zerowego do obliczeń wysokości

////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z EEPROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujBMP581(void)
{
	uint8_t chDane;

	/*HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	chBufBMP581[0] = PBMP5_CHIP_ID | READ_SPI;
	//chBufBMP581[0] = PBMP5_REV_ID | READ_SPI;
	chBufBMP581[1] = 0;
	HAL_SPI_TransmitReceive(&hspi2, chBufBMP581, chBufBMP581, 2, 5);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	chDane = chBufBMP581[1];*/

	chDane = CzytajSPIu8(PBMP5_CHIP_ID);	//sprawdź obecność układu
	if (chDane != 0x50)
		return ERR_BRAK_BMP581;

	chBufBMP581[0] = PBMP5_DRIVE_CONFIG;
	chBufBMP581[1] = 0x00;
	ZapiszSPIu8(chBufBMP581, 2);

	chDane = CzytajSPIu8(PBMP5_CHIP_STATUS);	//sprawdź status konfiguracji magistrali
	if (chDane != 0x03)							//SPI MODE0 lub MODE3
		return ERR_BRAK_BMP581;

	chBufBMP581[0] = PBMP5_OVERSAMLING_RATE;
	chBufBMP581[1] = (6 << 0)|	//oversampling temperatury: 0=x1, 1=x2, 2=x4, 3=x8, 4=x16, 5=x32, 6=x64, 7=x128
					 (6 << 3)|	//oversampling ciśnienia: 0=x1, 1=x2, 2=x4, 3=x8, 4=x16, 5=x32, 6=x64, 7=x128
					 (1 << 6);	//enable pressure sensor measurements
	ZapiszSPIu8(chBufBMP581, 2);

	chBufBMP581[0] = PBMP5_OUTPUT_DATA_RATE;
	chBufBMP581[1] = (1 << 0)|	//pwr_mode: 0=standby, 1=normal mode in fonfigured ODR grid, 2=forced one time mode mrasurement, 3=non stop mode, measurement wothout further duty cycling
					 (1 << 2)|	//ODR: 0=240Hz, 1=218,5Hz, 2=199,11Hz, 3=179,2Hz, 4=160Hz,, A=100,3Hz
					 (1 << 7);	//deep_dis - disable deep standby
	ZapiszSPIu8(chBufBMP581, 2);

	sLicznikUsrednianiaP0 = LICZBA_PROBEK_USREDNIANIA;	//rozpocznij filtrowanie P0
	uDaneCM4.dane.nZainicjowano |= INIT_BMP581;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika do wywołania w wyższej warstwie
// Wykonuje w pętli 1 pomiar temepratury i 15 pomiarów ciśnienia
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaBMP581(void)
{
	uint8_t chErr = ERR_OK;
	float fCisnienie = 0;

	if ((uDaneCM4.dane.nZainicjowano & INIT_BMP581) != INIT_BMP581)	//jeżeli czujnik nie jest zainicjowany
	{
		chErr = InicjujBMP581();
		if (chErr)
			return chErr;
	}
	else	//czujnik jest zainicjowany
	{
		switch (chProporcjaPomiarow)
		{
		case 0:	uDaneCM4.dane.fTemper[TEMP_BARO2] = (7 * uDaneCM4.dane.fTemper[TEMP_BARO2] + (float)(CzytajSPIs24mp(PBMP5_TEMP_DATA_XLSB) / 65536) + KELVIN) / 8;	break;
		default:
			fCisnienie = (float)(CzytajSPIs24mp(PBMP5_PRESS_DATA_XLSB) / 64);
			uDaneCM4.dane.fCisnie[1] = (7 * uDaneCM4.dane.fCisnie[1] + fCisnienie) / 8;
			break;
		}
		chProporcjaPomiarow++;
		chProporcjaPomiarow &= 0x0F;

		//przygotuj P0
		if (fCisnienie > 0)		//wykonaj tylko dla cykli pomiaru ciśnienia, pomiń cykle pomiary tempertury
		{
			if (sLicznikUsrednianiaP0)	//czy przygotowanie ciśnienia P0 jeszcze trwa
			{
				uDaneCM4.dane.fWysoko[1] = 0.0f;
				fP0_BMP581 = (127 * fP0_BMP581 + fCisnienie) / 128;
				sLicznikUsrednianiaP0--;
				if (sLicznikUsrednianiaP0 == 0)
					uDaneCM4.dane.nZainicjowano |= INIT_P0_BMP851;
			}
			else
				uDaneCM4.dane.fWysoko[1] = WysokoscBarometryczna(uDaneCM4.dane.fCisnie[1], fP0_BMP581, uDaneCM4.dane.fTemper[TEMP_BARO2]);	//P0 gotowe więc oblicz wysokość
		}
	}
	return chErr;
}
