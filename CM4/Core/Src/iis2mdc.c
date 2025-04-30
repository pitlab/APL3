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
ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM4")))	chStatusIIS);	//I2C4 współpracuje z BDMA a on ogarnia tylko SRAM4
ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM4")))	chPolWychMagIIS[2]);	//dane wychodzące aby nie kolidowały z przychodzącymi

extern I2C_HandleTypeDef hi2c4;
extern DMA_HandleTypeDef hdma_i2c4_rx;
extern DMA_HandleTypeDef hdma_i2c4_tx;
extern volatile unia_wymianyCM4_t uDaneCM4;
uint8_t chSekwencjaPomiaruIIS;
extern volatile uint8_t chCzujnikOdczytywanyNaI2CInt;	//identyfikator czujnika obsługiwanego na wewnętrznej magistrali I2C: MAG_MMC lub MAG_IIS
extern volatile uint8_t chCzujnikZapisywanyNaI2CInt;


////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika IIS2MDC
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujIIS2MDC(void)
{
	uint8_t chErr = ERR_BRAK_IIS2MDS;

	chPolWychMagIIS[0] = PIIS2MDS_WHO_AM_I;
	chErr = HAL_I2C_Master_Transmit(&hi2c4, IIS2MDC_I2C_ADR, chPolWychMagIIS, 1, TOUT_I2C4_2B);	//wyślij polecenie odczytu rejestru identyfikacyjnego
	if (!chErr)
	{
		chErr =  HAL_I2C_Master_Receive(&hi2c4, IIS2MDC_I2C_ADR + READ, chDaneMagIIS, 1, TOUT_I2C4_2B);		//odczytaj dane
		if (!chErr)
		{
			if (chDaneMagIIS[0] == 0x40)
			{
				chPolWychMagIIS[0] = PIIS2MDS_CFG_REG_A;
				chPolWychMagIIS[1] = (0 << 0) |	//MD: Mode of operation of the device: 0=Continuous mode, 1=Single mode, 2 i 3 = Idle mode.
								  (3 << 2) |	//ODR: Output data rate: 0=10Hz, 1=20Hz, 2=50Hz, 3=100Hz
								  (0 << 4) |	//LP: Enables low-power mode: 0=high-resolution mode, 1=low-power mode enabled
								  (0 << 5) |	//SOFT_RST: When this bit is set, the configuration registers and user registers are reset. Flash registers keep their values.
								  (0 << 6) |	//REBOOT: Reboot magnetometer memory content.
								  (1 << 7);		//COMP_TEMP_EN: Enables the magnetometer temperature compensation. For proper operation, this bit must be set to '1'.
				chErr = HAL_I2C_Master_Transmit(&hi2c4, IIS2MDC_I2C_ADR, chPolWychMagIIS, 2, TOUT_I2C4_2B);	//wyślij polecenie zapisu konfiguracji

				chPolWychMagIIS[0] = PIIS2MDS_CFG_REG_B;
				chPolWychMagIIS[1] = (0 << 0) |	//LPF Enables low-pass filter: 0=ODR/2, 1=ODR/4
								  (0 << 1) |	//OFF_CANC Enables offset cancellation - nie używam rejestrów offsetu
								  (1 << 2) |	//Set_FREQ Selects the frequency of the set pulse: 0=0: set pulse is released every 63 ODR, 1: set pulse is released only at power-on after PD condition)
								  (0 << 3) |	//INT_on_DataOFF If ‘1’, the interrupt block recognition checks data after the hard-iron correction to discover the interrupt
								  (0 << 4);		//OFF_CANC_ONE_SHOT Enables offset cancellation in single measurement mode. The OFF_CANC bit must be set to 1 when enabling offset cancellation in single measurement mode.
				chErr |= HAL_I2C_Master_Transmit(&hi2c4, IIS2MDC_I2C_ADR, chPolWychMagIIS, 2, TOUT_I2C4_2B);	//wyślij polecenie zapisu konfiguracji
				if (!chErr)
				{
					uDaneCM4.dane.nZainicjowano |= INIT_IIS2MDC;
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

	if ((uDaneCM4.dane.nZainicjowano & INIT_IIS2MDC) != INIT_IIS2MDC)
	{
		chErr = InicjujIIS2MDC();
		return chErr;
	}

	switch (chSekwencjaPomiaruIIS)
	{
	case 0:
		chPolWychMagIIS[0] = PIIS2MDS_STATUS_REG;
		chErr = HAL_I2C_Master_Seq_Transmit_DMA(&hi2c4, IIS2MDC_I2C_ADR, chPolWychMagIIS, 1, I2C_FIRST_FRAME);	//wyślij polecenie odczytu statusu nie kończąc transferu STOP-em
		chCzujnikZapisywanyNaI2CInt = MAG_IIS_STATUS;	//po zapisie wykonaj operację odczytu
		break;

	case 1:
		if (chStatusIIS & 0x08)	//3 najmłodsze bity statusu to: new data available dla każdej osi, bit 4 to komplet nowych danych - sprawdzamy komplet
		{
			chPolWychMagIIS[0] = PIIS2MDS_OUTX_L_REG;	//;
			chErr = HAL_I2C_Master_Seq_Transmit_DMA(&hi2c4, IIS2MDC_I2C_ADR, chPolWychMagIIS, 1, I2C_FIRST_FRAME);	//wyślij polecenie odczytu pomiarów nie kończąc transferu STOP-em
			chCzujnikZapisywanyNaI2CInt = MAG_IIS;	//po zapisie wykonaj operację odczytu
		}
		else
			chSekwencjaPomiaruIIS -= 2;	//wróć do odczytu statusu
		break;

	default: break;
	}
	chSekwencjaPomiaruIIS++;
	chSekwencjaPomiaruIIS &= 0x01;
	return chErr;
}


////////////////////////////////////////////////////////////////////////////////
// Polecenie będące drugą częścią sekwencji podzielonej operacji odczytu statusu.
// Będzie uruchomione w callbacku zakończenia operacji wysłania polecenia odczytu statusu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t MagIIS_CzytajStatus(void)
{
	chCzujnikOdczytywanyNaI2CInt = 0;	//nie interpretuj odczytanych danych jako wyniku pomiaru
	return HAL_I2C_Master_Seq_Receive_DMA(&hi2c4, IIS2MDC_I2C_ADR + READ, &chStatusIIS, 1, I2C_LAST_FRAME);		//odczytaj status i zakończ STOP
}



////////////////////////////////////////////////////////////////////////////////
// Polecenie będące drugą częścią sekwencji podzielonej operacji odczytu dnych.
// Będzie uruchomione w callbacku zakończenia operacji wysłania polecenia odczytu danych
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t MagIIS_CzytajDane(void)
{
	chCzujnikOdczytywanyNaI2CInt = MAG_IIS;		//w callbacku interpretuj odczytane dane jako pomiar magnetometru IIS
	return HAL_I2C_Master_Seq_Receive_DMA(&hi2c4, IIS2MDC_I2C_ADR + READ, chDaneMagIIS, 6, I2C_LAST_FRAME);		//odczytaj dane i zakończ STOP
}
