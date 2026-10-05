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
#define KMPMAG	3	//rozmiar wektora aktualizacji jednym czujnikiem

#define WARIANCJA_SZUMU_PROCESU_OBROTU	5e-3
#define WARIANCJA_SZUMU_PROCESU_BIASU	1e-8
#define WARIANCJA_SZUMU_MAGNETOMETRU1	0.094f	//uśredniona wariancja osi: X=0,0865, Y=0,1162, Z=0,0795
#define WARIANCJA_SZUMU_MAGNETOMETRU2	0.081f	//uśredniona wariancja osi: X=0,1485, Y=0,0403, Z=0,0548
#define WARIANCJA_SZUMU_MAGNETOMETRU3	5e-1

uint8_t InicjujFiltrKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane);
uint8_t PredykcjaFiltraKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaMag1FiltraKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane);
uint8_t AktulizacjaMag2FiltraKalmanaKalibracjiMagnetometrów12X9Z(stWymianyCM4_t *dane);

#endif /* INC_KALMANKALIBRACJIMAGNETOMETROW12X9Z_H_ */
