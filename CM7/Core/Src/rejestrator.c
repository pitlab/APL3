/*
 * rejestrator.c
 *
 *  Created on: Feb 2, 2025
 *      Author: PitLab
 */

#include "rejestrator.h"
#include "bsp_driver_sd.h"

uint8_t BSP_SD_IsDetected(void)
{
	uint8_t status = SD_PRESENT;
	extern uint8_t chPorty_exp_odbierane[];

	if (chPorty_exp_odbierane[0] & EXP04_LOG_CARD_DET)		//styk detekcji karty zwiera do masy gdy karta jest obecna a pulllup wystawia 1 gdy jest nieobecna w gnieździe
	{
	status = SD_NOT_PRESENT;
	}

	return status;
}
