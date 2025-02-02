//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa magnetometru MMC3416xPJ na magistrali I2C4
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "mmc3416x.h"
#include "wymiana_CM4.h"
#include "petla_glowna.h"
#include "main.h"
//#include "modul_IiP.h"

ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM4")))	chBuforMMC3416x[6]);	//I2C4 współpracuje z BDMA a on ogarnia tylko SRAM4

extern I2C_HandleTypeDef hi2c4;
extern DMA_HandleTypeDef hdma_i2c4_rx;
extern DMA_HandleTypeDef hdma_i2c4_tx;
extern volatile unia_wymianyCM4_t uDaneCM4;

////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika MMC34160PJ
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujMMC3416x(void)
{
	uint8_t chErr;

	chBuforMMC3416x[0] = PMMC3416_PRODUCT_ID;
	chErr = HAL_I2C_Master_Transmit(&hi2c4, MMC34160_I2C_ADR, chBuforMMC3416x, 1, 2);	//wyślij polecenie odczytu rejestru identyfikacyjnego
	if (!chErr)
	{
		chErr =  HAL_I2C_Master_Receive(&hi2c4, MMC34160_I2C_ADR + READ, chBuforMMC3416x, 1, 2);		//odczytaj dane
		if (!chErr)
		{
			if (chBuforMMC3416x[0] == 0x06)
			{
				StartujOdczytMMC3416x();
				uDaneCM4.dane.nZainicjowano |= INIT_MMC34160;
				return ERR_OK;
			}
		}
	}
	uDaneCM4.dane.nZainicjowano &= ~INIT_MMC34160;
	return ERR_BRAK_MMC34160;
}



////////////////////////////////////////////////////////////////////////////////
// Startuje cykliczny pomiar magnetometru MMC3416xPJ. Wykonuje się tylko raz w czasie inicjalizacji
// Parametry: brak
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 760us przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t StartujPomiarMMC3416x(void)
{
	uint8_t chErr = ERR_BRAK_MMC34160;

	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)
	{
		chBuforMMC3416x[0] = PMMC3416_INT_CTRL0;
		chBuforMMC3416x[1] = (1 << 0) |	//TM Take measurement, set ‘1’ will initiate measurement
						 (0 << 1) |	//Continuous Measurement Mode On
						 (0 << 2) |	//CM Freq0..1 How often the chip will take measurements in Continuous Measurement Mode: 0=1,5Hz, 1=13Hz, 2=25Hz, 3=50Hz
						 (0 << 4) |	//No Boost. Setting this bit high will disable the charge pump and cause the storage capacitor to be charged off VDD.
						 (0 << 5) |	//SET
						 (0 << 6) |	//RESET
						 (0 << 7);	//Refill Cap Writing “1” will recharge the capacitor at CAP pin, it is requested to be issued before SET/RESET command.
		chErr = HAL_I2C_Master_Transmit_DMA(&hi2c4, MMC34160_I2C_ADR, chBuforMMC3416x, 2);	//wyślij polecenie wykonania pomiaru
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjuje odczyt danych z magnetometru HMC5883
// Parametry: brak
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 760us przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t StartujOdczytMMC3416x(void)
{
	uint8_t chErr = ERR_BRAK_MMC34160;

	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)
	{
		chBuforMMC3416x[0] = PMMC3416_XOUT_L;
		chErr = HAL_I2C_Master_Transmit_DMA(&hi2c4, MMC34160_I2C_ADR, chBuforMMC3416x, 2);	//wyślij polecenie odczytu wszystkich pomiarów
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna odczyt danych pomiarowych
// Parametry: *dane_wy wskaźnik na dane wychodzące
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 2,2ms przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajMMC3416x(void)
{
	uint8_t chErr = ERR_BRAK_MMC34160;

	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)
		chErr = HAL_I2C_Master_Receive_DMA(&hi2c4, MMC34160_I2C_ADR + READ, chBuforMMC3416x, 6);		//odczytaj dane
	else
		chErr = InicjujMMC3416x();	//wykonaj inicjalizację

	if (chErr == ERR_HAL_BUSY)
	{
		struct I2C_Module* i2c = 0;
		i2c->instance = hi2c4;
		i2c->sclPin = 11;
		i2c->sdaPin = 12;
		i2c->sclPort = GPIOH;
		i2c->sdaPort = GPIOH;
		I2C_ClearBusyFlagErratum(i2c);	//https://electronics.stackexchange.com/questions/267972/i2c-busy-flag-strange-behaviour
	}
	return chErr;
}






void I2C_ClearBusyFlagErratum(struct I2C_Module* i2c)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // 1. Clear PE bit.
  i2c->instance.Instance->CR1 &= ~(0x0001);

  //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C4;
  GPIO_InitStructure.Pull         = GPIO_PULLUP;
  GPIO_InitStructure.Speed        = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 3. Check SCL and SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET);

  //  5. Check SDA Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET);

  //  7. Check SCL Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  // 9. Check SCL High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 11. Check SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
  GPIO_InitStructure.Mode         = GPIO_MODE_AF_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C4;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

  // 13. Set SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 |= 0x8000;

  asm("nop");

  // 14. Clear SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 &= ~0x8000;

  asm("nop");

  // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
  i2c->instance.Instance->CR1 |= 0x0001;

  // Call initialization function.
  HAL_I2C_Init(&(i2c->instance));
}

