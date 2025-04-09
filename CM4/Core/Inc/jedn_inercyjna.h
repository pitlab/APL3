/*
 * jedn_inercyjna.h
 *
 *  Created on: Dec 29, 2024
 *      Author: PitLab
 */

#ifndef INC_JEDN_INERCYJNA_H_
#define INC_JEDN_INERCYJNA_H_

#include "sys_def_CM4.h"



uint8_t InicjujJednostkeInercyjna(void);
void ObliczeniaJednostkiInercujnej(uint8_t chGniazdo);
uint8_t JednostkaInercyjnaKwaterniony(uint8_t chGniazdo);
uint8_t JednostkaInercyjnaMadgwick(void);

#endif /* INC_JEDN_INERCYJNA_H_ */
