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
#define SKALA_GLOSNOSCI_AUDIO	128		//rozpiętość głosnośco od zera do tej wartosci
#define SKALA_GLOSNOSCI_TONU	128		//aby uzyskać finalną amplitud
#define MAX_TON_WARIO			128		//liczba tonów sygnału wario
#define MIN_OKRES_TONU			16		//minimalny rozmiar okresu tonu dla najwyższej częstotliwości
#define SKOK_TONU				5		//zmiana długości okresu między kolejnymi tonami
#define ROZMIAR_BUFORA_TONU		MIN_OKRES_TONU + SKOK_TONU * MAX_TON_WARIO

//obliczenia: Maksymalna finalna amplituda to: (sin(1harm) * AMPLITUDA_1HARM + sin(3harm) * AMPLITUDA_3HARM) * SKALA_GLOSNOSCI_TONU
//Należy tak dobrać amplitudy harmonicznych aby mieściły się w 16 bitach (+-32768)i dawały dodać do komunikatu dźwiękowego bez większych zniekstałceń
#define AMPLITUDA_1HARM			2048	//amplituda pierwszej harmonicznej sygnału
#define AMPLITUDA_3HARM			2048	//amplituda trzeciej harmonicznej sygnału

uint8_t InicjujAudio(void);
uint8_t OdtworzProbkeAudio(uint32_t nAdres, uint32_t nRozmiar);
uint8_t OdtworzProbkeAudioZeSpisu(uint8_t chNrKomunikatu);
void UstawTon(uint8_t chNrTonu, uint8_t chGlosnosc);
uint8_t RejestrujAudio(void);
#endif /* INC_AUDIO_H_ */
