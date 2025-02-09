/*
 * rejestrator.h
 *
 *  Created on: Feb 2, 2025
 *      Author: PitLab
 */

#ifndef INC_REJESTRATOR_H_
#define INC_REJESTRATOR_H_

#include "sys_def_CM7.h"

uint8_t BSP_SD_IsDetected(void);
void HAL_SD_DriveTransceiver_1_8V_Callback(FlagStatus status);
uint8_t MontujFAT(void);

#endif /* INC_REJESTRATOR_H_ */
