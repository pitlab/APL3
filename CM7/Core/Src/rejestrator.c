/*
 * rejestrator.c
 *
 *  Created on: Feb 2, 2025
 *      Author: PitLab
 */

#include "rejestrator.h"
#include "bsp_driver_sd.h"
#include "moduly_SPI.h"



////////////////////////////////////////////////////////////////////////////////
// Zwraca obecność karty w gnieździe. Wymaga wcześniejszego odczytania stanu expanderów I/O, ktore czytane są w każdym obiegu pętli StartDefaultTask()
// Parametry: brak
// Zwraca: obecność karty: SD_PRESENT == 1 lub SD_NOT_PRESENT == 0
////////////////////////////////////////////////////////////////////////////////
uint8_t BSP_SD_IsDetected(void)
{
	uint8_t status = SD_PRESENT;
	extern uint8_t chPorty_exp_odbierane[3];

	if (chPorty_exp_odbierane[0] & EXP04_LOG_CARD_DET)		//styk detekcji karty zwiera do masy gdy karta jest obecna a pulllup wystawia 1 gdy jest nieobecna w gnieździe
	{
		status = SD_NOT_PRESENT;
	}
	return status;
}



////////////////////////////////////////////////////////////////////////////////
// Włącza napiecie 1.8V dla karty
// Parametry: status:  SET - włącz 1,8V
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_SD_DriveTransceiver_1_8V_Callback(FlagStatus status)
{
	extern uint8_t chPorty_exp_wysylane[];
	uint8_t chErr;

	if (status == SET)
		chPorty_exp_wysylane[0] &= ~EXP02_LOG_VSELECT;	//LOG_SD1_VSEL: L=1,8V
	else
		chPorty_exp_wysylane[0] |=  EXP02_LOG_VSELECT;	//LOG_SD1_VSEL: H=3,3V

	//wysyłaj aż dane do ekspandera do skutku
	do
	chErr = WyslijDaneExpandera(SPI_EXTIO_0, chPorty_exp_wysylane[0]);
	while (chErr != ERR_OK);
}
