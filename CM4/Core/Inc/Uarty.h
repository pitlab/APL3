/*
 * Uarty.h
 *
 *  Created on: 22 maj 2026
 *      Author: PitLab
 */

#ifndef INC_UARTY_H_
#define INC_UARTY_H_

#include "SysDefCM4.h"

#define ROZMIAR_BUF_ODB_UART2	16	//RC1
#define ROZMIAR_BUF_ODB_UART4	16	//RC2
#define ROZMIAR_BUF_ODB_UART8	16	//GNSS / Crossfire

//definicje trybów pracy UART-ów
#define U_GNSS1		1
#define U_GNSS2		2
#define U_CRSF1		3
#define U_CRSF2		4
#define U_SBUS1		5
#define U_SBUS2		6


void WłączOdbiórUART2(void);
void WłączOdbiórUART4(void);
void WłączOdbiórUART8(void);

#endif /* INC_UARTY_H_ */
