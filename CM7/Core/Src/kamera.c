/*
 * kamera.c
 *
 *  Created on: Dec 14, 2024
 *      Author: PitLab
 */


#include "kamera.h"
uint32_t nBuforKamery[ROZM_BUF32_KAM] __attribute__((section(".ExtSramSection")));
struct st_KonfKam strKonfKam;
