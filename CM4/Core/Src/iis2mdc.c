//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa magnetometru IIS2MDC na magistrali I2C4
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <IIS2MDC.h>
#include "wymiana_CM4.h"
#include "petla_glowna.h"
#include "main.h"


ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM4")))	chDaneMagIIS[8]);	//I2C4 współpracuje z BDMA a on ogarnia tylko SRAM4

extern I2C_HandleTypeDef hi2c4;
extern DMA_HandleTypeDef hdma_i2c4_rx;
extern DMA_HandleTypeDef hdma_i2c4_tx;
extern volatile unia_wymianyCM4_t uDaneCM4;
uint8_t chSekwencjaPomiaruIIS;
extern volatile uint8_t chCzujnikOdczytywanyNaI2CInt;	//identyfikator czujnika obsługiwanego na wewnętrznej magistrali I2C: MAG_MMC lub MAG_IIS



////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika IIS2MDC
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujIIS2MDC(void)
{
	uint8_t chErr = ERR_BRAK_IIS2MDS;

	chDaneMagIIS[0] = PIIS2MDS_WHO_AM_I;
	chErr = HAL_I2C_Master_Transmit(&hi2c4, IIS2MDC_I2C_ADR, chDaneMagIIS, 1, TOUT_I2C4_2B);	//wyślij polecenie odczytu rejestru identyfikacyjnego
	if (!chErr)
	{
		chErr =  HAL_I2C_Master_Receive(&hi2c4, IIS2MDC_I2C_ADR + READ, chDaneMagIIS, 1, TOUT_I2C4_2B);		//odczytaj dane
		if (!chErr)
		{
			if (chDaneMagIIS[0] == 0x40)
			{
				chDaneMagIIS[0] = PIIS2MDS_CFG_REG_A;
				chDaneMagIIS[1] = (0 << 0) |	//MD: Mode of operation of the device: 0=Continuous mode, 1=Single mode, 2 i 3 = Idle mode.
								  (3 << 2) |	//ODR: Output data rate: 0=10Hz, 1=20Hz, 2=50Hz, 3=100Hz
								  (0 << 4) |	//LP: Enables low-power mode: 0=high-resolution mode, 1=low-power mode enabled
								  (0 << 5) |	//SOFT_RST: When this bit is set, the configuration registers and user registers are reset. Flash registers keep their values.
								  (0 << 6) |	//REBOOT: Reboot magnetometer memory content.
								  (1 << 7);		//COMP_TEMP_EN: Enables the magnetometer temperature compensation. For proper operation, this bit must be set to '1'.
				chErr = HAL_I2C_Master_Transmit(&hi2c4, IIS2MDC_I2C_ADR, chDaneMagIIS, 2, TOUT_I2C4_2B);	//wyślij polecenie zapisu konfiguracji

				chDaneMagIIS[0] = PIIS2MDS_CFG_REG_B;
				chDaneMagIIS[1] = (1 << 0) |	//LPF Enables low-pass filter: 0=ODR/2, 1=ODR/4
								  (1 << 1) |	//OFF_CANC Enables offset cancellation
								  (0 << 2) |	//Set_FREQ Selects the frequency of the set pulse: 0=0: set pulse is released every 63 ODR, 1: set pulse is released only at power-on after PD condition)
								  (0 << 3) |	//INT_on_DataOFF If ‘1’, the interrupt block recognition checks data after the hard-iron correction to discover the interrupt
								  (0 << 4);		//OFF_CANC_ONE_SHOT Enables offset cancellation in single measurement mode. The OFF_CANC bit must be set to 1 when enabling offset cancellation in single measurement mode.
				chErr |= HAL_I2C_Master_Transmit(&hi2c4, IIS2MDC_I2C_ADR, chDaneMagIIS, 2, TOUT_I2C4_2B);	//wyślij polecenie zapisu konfiguracji
				if (!chErr)
				{
					uDaneCM4.dane.nZainicjowano |= INIT_IIS2MDC;
					//chErr = StartujOdczytIIS2MDC();
				}
			}
		}
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj jeden element sekwencji potrzebnych do uzyskania pomiaru
// Najlepszy wynik wedlug dokumentacji uzyskuje się wykonując sekwencję: SET, MEASUREMENT, RESET, MEASUREMENT
// Cząstkowe wyniki oprocz natężenia pola zawierają offset. Następnie cząstkowe pomiary należy odjąć od siebie i podzielić przez 2
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania: pełna pętla zajmuje 170ms -> 5,88Hz
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaIIS2MDC(void)
{
	uint8_t chErr = ERR_OK;

	switch (chSekwencjaPomiaruIIS)
	{
	case 0:
		chErr = StartujOdczytIIS2MDC(PIIS2MDS_STATUS_REG);	//polecenie odczytu statusu
		chCzujnikOdczytywanyNaI2CInt = 0;	//nie interpretuj odczytanych danych jako wyniku pomiaru
		break;

	case 1:
		chErr = HAL_I2C_Master_Receive_DMA(&hi2c4, IIS2MDC_I2C_ADR + READ, chDaneMagIIS, 1);		//odczytaj status
		break;

	case 2:
		if (chDaneMagIIS[0] & 0x08)	//3 najmłodsze bity statusu to: new data available dla każdej osi, bit 4 to komplet nowych danych - sprawdzamy komplet
			chErr = StartujOdczytIIS2MDC(PIIS2MDS_OUTX_L_REG);	//polecenie odczytu danych
		else
			chSekwencjaPomiaruIIS -= 3;	//wróć do odczytu statusu
		break;

	case 3:
		chErr = HAL_I2C_Master_Receive_DMA(&hi2c4, IIS2MDC_I2C_ADR + READ, chDaneMagIIS, 8);		//odczytaj dane
		chCzujnikOdczytywanyNaI2CInt = MAG_IIS;	//interpretuj odczytane dane jako pomiar tego magnetometru
		break;

	default: break;
	}
	chSekwencjaPomiaruIIS++;
	chSekwencjaPomiaruIIS &= 0x03;
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjuje odczyt rejestru z magnetometru IIS2MDC
// Parametry: chRejestr - adres rejestru do odczytu
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 760us przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t StartujOdczytIIS2MDC(uint8_t chRejestr)
{
	uint8_t chErr = ERR_BRAK_IIS2MDS;

	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)
	{
		chDaneMagIIS[0] = chRejestr;	//;
		chErr = HAL_I2C_Master_Transmit_DMA(&hi2c4, IIS2MDC_I2C_ADR, chDaneMagIIS, 1);	//wyślij polecenie odczytu wszystkich pomiarów
	}
	else
		chErr = InicjujIIS2MDC();	//wykonaj inicjalizację
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna odczyt danych pomiarowych z IIS2MDC
// Parametry: *dane_wy wskaźnik na dane wychodzące
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 2,2ms przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajIIS2MDC(void)
{
	uint8_t chErr = ERR_BRAK_IIS2MDS;


	if (uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC)
	{
		chCzujnikOdczytywanyNaI2CInt = MAG_IIS;
		chErr = HAL_I2C_Master_Receive_DMA(&hi2c4, IIS2MDC_I2C_ADR + READ, chDaneMagIIS, 8);		//odczytaj dane
	}

	return chErr;
}
