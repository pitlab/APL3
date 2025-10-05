/*
 * rejestrator.h
 *
 *  Created on: Feb 2, 2025
 *      Author: PitLab
 */

#ifndef INC_REJESTRATOR_H_
#define INC_REJESTRATOR_H_

#include "sys_def_CM7.h"

#define DATA_SIZE              ((uint32_t)0x0640000U)
#define BUFFER_SIZE            ((uint32_t)_MAX_SS)
#define NB_BUFFER              DATA_SIZE / BUFFER_SIZE
#define NB_BLOCK_BUFFER        BUFFER_SIZE / BLOCKSIZE /* Number of Block (512o) by Buffer */
#define SD_TIMEOUT             ((uint32_t)0x00100000U)
#define ADDRESS                ((uint32_t)0x00004000U) /* SD Address to write/read data */
#define DATA_PATTERN           ((uint32_t)0xB5F3A5F3U) /* Data pattern to write */

#define ROZMIAR_BUFORA_LOGU		(2*512)
#define MAX_ROZMIAR_WPISU_LOGU	20
#define WPISOW_NA_SYNC			100		//po tylu zapisach linii robiony jest SYNC na karcie

//definicja znaczenia bitów rejestratora
#define STATREJ_FAT_GOTOWY		0x01
#define STATREJ_ZAPISZ_NAGLOWEK	0x02
#define STATREJ_WLACZONY		0x04
#define STATREJ_OTWARTY_PLIK	0x08
#define STATREJ_ZAMKNIJ_PLIK	0x10
#define STATREJ_BYL_OTWARTY		0x20


//definicje bitów konfiguracji logera
#define KLOG1_CZAS      0x00000001	    //czas
#define KLOG1_PRES1     0x00000002	    //ciśnienie atmosferyczne z czujnika ciśnienia 1
#define KLOG1_PRES2		0x00000004	    //ciśnienie atmosferyczne z czujnika ciśnienia 2
#define KLOG1_AMSL1    	0x00000008	    //wysokość barometryczna z czujnika ciśnienia 1
#define KLOG1_AMSL2    	0x00000010	    //wysokość barometryczna z czujnika ciśnienia 2

#define KLOG1_IAS1      0x00000020	    //prędkość wzgledem powietrza z czujnika różnicowego 1
#define KLOG1_IAS2      0x00000040	    //prędkość wzgledem powietrza z czujnika różnicowego 2
#define KLOG1_VARIO     0x00000080	    //wskazania wariometru

#define KLOG1_ZYRO1P    0x00000100	    //skaliborwana prędkość obrotowa P żyroskopu 1
#define KLOG1_ZYRO1Q    0x00000200	    //skaliborwana prędkość obrotowa Q żyroskopu 1
#define KLOG1_ZYRO1R    0x00000400	    //skaliborwana prędkość obrotowa R żyroskopu 1
#define KLOG1_ZYRO2P    0x00000800	    //skaliborwana prędkość obrotowa P żyroskopu 2
#define KLOG1_ZYRO2Q    0x00001000	    //skaliborwana prędkość obrotowa Q żyroskopu 2
#define KLOG1_ZYRO2R    0x00002000	    //skaliborwana prędkość obrotowa R żyroskopu 2

#define KLOG1_AKCEL1X   0x00004000	    //przyspieszenie w osi X akcelerometru 1
#define KLOG1_AKCEL1Y   0x00008000	    //przyspieszenie w osi Y akcelerometru 1
#define KLOG1_AKCEL1Z   0x00010000	    //przyspieszenie w osi Z akcelerometru 1
#define KLOG1_AKCEL2X   0x00020000	    //przyspieszenie w osi X akcelerometru 2
#define KLOG1_AKCEL2Y   0x00040000	    //przyspieszenie w osi Y akcelerometru 2
#define KLOG1_AKCEL2Z   0x00080000	    //przyspieszenie w osi Z akcelerometru 2

#define KLOG1_MAG1X     0x00100000	    //składowa magnetyczna w osi X magnetometru 1
#define KLOG1_MAG1Y     0x00200000	    //składowa magnetyczna w osi Y magnetometru 1
#define KLOG1_MAG1Z     0x00400000	    //składowa magnetyczna w osi Z magnetometru 1
#define KLOG1_MAG2X     0x00800000	    //składowa magnetyczna w osi X magnetometru 2
#define KLOG1_MAG2Y     0x01000000	    //składowa magnetyczna w osi Y magnetometru 2
#define KLOG1_MAG2Z     0x02000000	    //składowa magnetyczna w osi Z magnetometru 2
#define KLOG1_MAG3X     0x04000000	    //składowa magnetyczna w osi X magnetometru 3
#define KLOG1_MAG3Y     0x08000000	    //składowa magnetyczna w osi Y magnetometru 3
#define KLOG1_MAG3Z     0x10000000	    //składowa magnetyczna w osi Z magnetometru 3

#define KLOG1_TEMPBARO1	0x20000000	    //temperatura czujnika ciśnienia 1
#define KLOG1_TEMPIMU1	0x40000000	    //temperatura IMU1
#define KLOG1_TEMPIMU2	0x80000000	    //temperatura IMU2

//drugie słowo konfiguracji logera
#define KLOG2_GLONG     0x00000001	    //szerokość geograficzna z GPS
#define KLOG2_GLATI     0x00000002	    //długość geograficzna z GPS
#define KLOG2_GALTI     0x00000004	    //wysokość n.p.m. z GPS
#define KLOG2_GSPED     0x00000008	    //prędkość wzgledem ziemi z GPS
#define KLOG2_GCURS     0x00000010	    //kurs względem ziemi z GPS
#define KLOG2_GSATS     0x00000020	    //liczba widocznych satelitów
#define KLOG2_GVDOP     0x00000040	    //Vertical Dilution of Precision
#define KLOG2_GHDOP     0x00000080	    //Horizontal Dilution of Precision

#define KLOG2_GSPD_E    0x00000100	    //niefiltrowana prędkość z GPS w kierunku wschodnim
#define KLOG2_GSPD_N    0x00000200	    //niefiltrowana prędkość z GPS w kierunku północnym

#define KLOG2_TEMPCISR1	0x00400000	    //temperatura czujnika ciśnienia różnicowego 1
#define KLOG2_TEMPCISR2	0x00800000	    //temperatura czujnika ciśnienia różnicowego 2

#define KLOG2_ZYROSUR1P 0x01000000	    //surowa prędkość obrotowa P żyroskopu 1
#define KLOG2_ZYROSUR1Q 0x02000000	    //surowa prędkość obrotowa Q żyroskopu 1
#define KLOG2_ZYROSUR1R 0x04000000	    //surowa prędkość obrotowa R żyroskopu 1
#define KLOG2_ZYROSUR2P 0x08000000	    //surowa prędkość obrotowa P żyroskopu 2
#define KLOG2_ZYROSUR2Q 0x10000000	    //surowa prędkość obrotowa Q żyroskopu 2
#define KLOG2_ZYROSUR2R 0x20000000	    //surowa prędkość obrotowa R żyroskopu 2

#define KLOG2_CISROZ1	0x40000000		//ciśnienie czujnika różnicowego 1
#define KLOG2_CISROZ2	0x80000000		//ciśnienie czujnika różnicowego 2


//trzecie słowo konfiguracji logera
#define KLOG3_KATPHI    0x00000001	    //kąt phi wektora inercji
#define KLOG3_KATTHE    0x00000002	    //kąt theta wektora inercji
#define KLOG3_KATPSI    0x00000004	    //kąt psi wektora inercji
#define KLOG3_KATPHIA   0x00000008      //kąt phi obliczony na podstawie danych z akcelerometru
#define KLOG3_KATTHEA   0x00000010	    //kąt theta obliczony na podstawie danych z akcelerometru
#define KLOG3_KATPSIM   0x00000020	    //kąt psi obliczony na podstawie danych z magnetometru
#define KLOG3_KATPHIZ   0x00000040	    //kąt phi obliczony jako całka prędkości P z żyroskopu
#define KLOG3_KATTHEZ   0x00000080	    //kąt theta obliczony jako całka prędkości Q z żyroskopu
#define KLOG3_KATPSIZ   0x00000100	    //kąt psi obliczony jako całka prędkości R z żyroskopu

/*
#define LOG1CONF_VOLT1      0x00004000	    //napięcie z sondy pomiaru mocy 1
#define LOG1CONF_CURR1      0x00008000	    //prąd z sondy pomiaru mocy 1

#define LOG1CONF_VOLT2      0x00010000	    //napięcie z sondy pomiaru mocy 2
#define LOG1CONF_CURR2      0x00020000	    //prąd z sondy pomiaru mocy 2
#define LOG1CONF_TEMP1      0x00040000	    //temperatura z termopary 1
#define LOG1CONF_TEMP2      0x00080000	    //temperatura z termopary 2
#define LOG1CONF_TEMP3      0x00100000	    //temperatura z LM335 1
#define LOG1CONF_TEMP4      0x00200000	    //temperatura z LM335 2
#define LOG1CONF_REFT       0x00400000	    //temperatura odniesienia termopary
#define LOG1CONF_MCURS      0x00800000	    //wyliczony kurs magnetyczny

#define LOG1CONF_CURR       0x01000000	    //prąd pobierany przez serwa
#define LOG1CONF_VIN        0x02000000	    //napięcie zasilajace za przetwornicą
#define LOG1CONF_RPM1       0x04000000	    //prędkość obrotowa 1
#define LOG1CONF_RPM2       0x08000000	    //prędkość obrotowa 2
#define LOG1CONF_RCHEALTH   0x10000000	    //poziom szumu w sygnale RC
#define LOG1CONF_RSSI       0x20000000	    //RSSI odbiornika RC
#define LOG1CONF_CARD_BUF   0x40000000	    //rozmiar zajetego bufora na karcie SD
#define LOG1CONF_ERROR      0x80000000	    //logowane błędy


#define LOG2CONF_           0x00000800      //
#define LOG2CONF_SERO09     0x00001000	    //serwo wyjściowe na kanale 9
#define LOG2CONF_SERO10     0x00002000	    //serwo wyjściowe na kanale 10
#define LOG2CONF_SERO11     0x00004000	    //serwo wyjściowe na kanale 11
#define LOG2CONF_SERO12     0x00008000	    //serwo wyjściowe na kanale 12

#define LOG2CONF_SERI1      0x00010000	    //serwo wejściowe na kanale 1
#define LOG2CONF_SERI2      0x00020000	    //serwo wejściowe na kanale 2
#define LOG2CONF_SERI3      0x00040000	    //serwo wejściowe na kanale 3
#define LOG2CONF_SERI4      0x00080000	    //serwo wejściowe na kanale 4
#define LOG2CONF_SERI5      0x00100000	    //serwo wejściowe na kanale 5
#define LOG2CONF_SERI6      0x00200000	    //serwo wejściowe na kanale 6
#define LOG2CONF_SERI7      0x00400000	    //serwo wejściowe na kanale 7
#define LOG2CONF_SERI8      0x00800000	    //serwo wejściowe na kanale 8

#define LOG2CONF_SERO01     0x01000000     //serwo wyjściowe na kanale 1
#define LOG2CONF_SERO02     0x02000000     //serwo wyjściowe na kanale 2
#define LOG2CONF_SERO03     0x04000000     //serwo wyjściowe na kanale 3
#define LOG2CONF_SERO04     0x08000000     //serwo wyjściowe na kanale 4
#define LOG2CONF_SERO05     0x10000000     //serwo wyjściowe na kanale 5
#define LOG2CONF_SERO06     0x20000000     //serwo wyjściowe na kanale 6
#define LOG2CONF_SERO07     0x40000000     //serwo wyjściowe na kanale 7
#define LOG2CONF_SERO08     0x80000000     //serwo wyjściowe na kanale 8


#define LOG3CONF_FALTI      0x00000200	    //wysokość przefiltrowana filtrem Kalmana
#define LOG3CONF_FVARIO     0x00000400	    //wario przefiltrowane filtrem Kalmana
#define LOG3CONF_FACCZ      0x00000800	    //składowa Z przyspieszenia po transformacji napędzajaca filtr Kalmana
#define LOG3CONF_PDIFV      0x00001000	    //napięcie z różnicowego czujnika ciśnienia
#define LOG3CONF_PABSV      0x00002000	    //napięcie z bezwzględnego czujnika ciśnienia
#define LOG3CONF_ENERGY1    0x00004000	    //energia pobrana z pakietu 1
#define LOG3CONF_ENERGY2    0x00008000	    //energia pobrana z pakietu 2

#define LOG3CONF_QUATW      0x00010000	    //quaternion w
#define LOG3CONF_QUATX      0x00020000	    //quaternion x
#define LOG3CONF_QUATY      0x00040000	    //quaternion y
#define LOG3CONF_QUATZ      0x00080000	    //quaternion z

#define LOG3CONF_GYRO1_TEMP 0x00100000	    //temperatura żyroskopu 1
#define LOG3CONF_GYRO2_TEMP 0x00200000	    //temperatura żyroskopu 2
#define LOG3CONF_SONAR_DIST 0x00400000	    //wysokość odczytana z sonaru
#define LOG3CONF_ALTI_DRIFT 0x00800000	    //dryft wysokości czujnika barometrycznego

#define LOG3CONF_MIX_SUM    0x01000000	    //średnia wszystkich kanałów miksera, odpowiada za ciąg koptera
#define LOG3CONF_2          0x02000000	    //
#define LOG3CONF_3          0x04000000	    //
#define LOG3CONF_4          0x08000000	    //
#define LOG3CONF_AZI2BAS    0x10000000	    //azymut do bazy - zapis w celu weryfikacji obliczeń
#define LOG3CONF_DIR2BAS    0x20000000	    //kurs strzałki do bazy - zapis w celu weryfikacji obliczeń
#define LOG3CONF_BASELON    0x40000000	    //współrzędne bazy
#define LOG3CONF_BASELAT    0x80000000	    //współrzędne bazy

//czwarte słowo konfguracyjne logera - równoległa rejestracja danych z modułu All-In-One
#define LOG4CONF_AIOACC1_X  0x00000001      //składowa przyspieszenia w osi X akcelerometru
#define LOG4CONF_AIOACC1_Y  0x00000002      //składowa przyspieszenia w osi Y akcelerometru
#define LOG4CONF_AIOACC1_Z  0x00000004      //składowa przyspieszenia w osi Z akcelerometru

#define LOG4CONF_AIOGYRO_P  0x00000008      //składowa predkości kątowej wokół osi X
#define LOG4CONF_AIOGYRO_Q  0x00000010      //składowa predkości kątowej wokół osi Y
#define LOG4CONF_AIOGYRO_R  0x00000020      //składowa predkości kątowej wokół osi Z

#define LOG4CONF_AIOMAGN_X  0x00000040      //składowa magnetyczna w osi X
#define LOG4CONF_AIOMAGN_Y  0x00000080      //składowa magnetyczna w osi Y
#define LOG4CONF_AIOMAGN_Z  0x00000100      //składowa magnetyczna w osi Z

#define LOG4CONF_AIOALTI1   0x00000200      //wysokość barometryczna z czujnika 1
#define LOG4CONF_AIOVARI1   0x00000400      //prędkość pionowa vario z czujnika 1
#define LOG4CONF_AIOALTI2   0x00000800      //wysokość barometryczna z czujnika 2
#define LOG4CONF_AIOVARI2   0x00001000      //prędkość pionowa vario z czujnika 2
#define LOG4CONF_AIOTEMPPR  0x00002000      //temperatura czujnika ciśnienia


#define LOG4CONF_ESTWAYX    0x00100000	    //droga w osi X uzyskana w wyniku podwójnego całkownia akcelerometru [m]
#define LOG4CONF_ESTWAYY    0x00200000	    //droga w osi Y uzyskana w wyniku podwójnego całkownia akcelerometru [m]
#define LOG4CONF_ESTWAYZ    0x00400000	    //droga w osi Z uzyskana w wyniku podwójnego całkownia akcelerometru [m]

#define LOG4CONF_ESTSPEEDX  0x00800000	    //prędkość w osi X uzyskana w wyniku całkownia akcelerometru [m/s]
#define LOG4CONF_ESTSPEEDY  0x01000000	    //prędkość w osi Y uzyskana w wyniku całkownia akcelerometru [m/s]
#define LOG4CONF_ESTSPEEDZ  0x02000000	    //prędkość w osi Z uzyskana w wyniku całkownia akcelerometru [m/s]

#define LOG4CONF_ESTACCX    0x04000000	    //przyspieszenie w osi X uzyskana w wyniku estymacji [m/s^2]
#define LOG4CONF_ESTACCY    0x08000000	    //przyspieszenie w osi Y uzyskana w wyniku estymacji [m/s^2]
#define LOG4CONF_ESTACCZ    0x10000000	    //przyspieszenie w osi Z uzyskana w wyniku estymacji [m/s^2]

#define LOG4CONF_EST_ACCX   0x20000000	    //estymowane przyspieszenie w x
#define LOG4CONF_EST_ACCY   0x40000000	    //estymowane przyspieszenie w y
#define LOG4CONF_EST_ACCZ   0x80000000	    //estymowane przyspieszenie w z */




uint8_t BSP_SD_IsDetected(void);
void HAL_SD_DriveTransceiver_1_8V_Callback(FlagStatus status);
uint8_t ObslugaPetliRejestratora(void);
uint8_t Wait_SDCARD_Ready(void);


#endif /* INC_REJESTRATOR_H_ */
