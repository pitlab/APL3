/*
 * KalmanKalibracjiMagnetometrow12X15Z.h
 *
 *  Created on: 28 wrz 2026
 *      Author: PitLab
 */

#ifndef INC_KALMANKALIBRACJIMAGNETOMETROW12X9Z_H_
#define INC_KALMANKALIBRACJIMAGNETOMETROW12X9Z_H_

#include "SysDefCM4.h"
#include "wymiana.h"
#include "arm_math.h"

#define KMSTAN	12	//rozmiar wektora stanu filtra kalibracji magnetometrów
//#define KMPOMR	9	//rozmar wektora pomiaru
#define KMAKTU	3	//rozmiar wektora aktualizacji jednym czujnikiem
#define LICZBA_USREDNIANIA_ZYROSKOPOW	2048
#define WARIANCJA_SZUMU_PROCESU	5e-3
#define WARIANCJA_SZUMU_MAGNETOMETRU1	5e-3
#define WARIANCJA_SZUMU_MAGNETOMETRU2	5e-3
#define WARIANCJA_SZUMU_MAGNETOMETRU3	5e-3

uint8_t InicjujFiltrKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane);
uint8_t PredykcjaFiltraKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaMag11FiltraKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane);

#endif /* INC_KALMANKALIBRACJIMAGNETOMETROW12X9Z_H_ */
