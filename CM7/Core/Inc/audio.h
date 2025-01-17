/*
 * audio.h
 *
 *  Created on: Jan 5, 2025
 *      Author: PitLab
 */

#ifndef INC_AUDIO_H_
#define INC_AUDIO_H_

#include "sys_def_CM7.h"

#define ROZMIAR_BUFORA_AUDIO	256
#define SKALA_GLOSNOSCI			100		//rozpiętość głosnośco od zera do tej wartosci

uint8_t InicjujAudio(void);
uint8_t OdtworzProbkeAudio(uint32_t nAdres, uint32_t nRozmiar);
uint8_t OdtworzProbkeAudioZeSpisu(uint8_t chNrKomunikatu);

uint8_t RejestrujAudio(void);
#endif /* INC_AUDIO_H_ */
