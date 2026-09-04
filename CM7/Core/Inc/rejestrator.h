/*
 * rejestrator.h
 *
 *  Created on: Feb 2, 2025
 *      Author: PitLab
 */

#ifndef INC_REJESTRATOR_H_
#define INC_REJESTRATOR_H_

#include "SysDefCM7.h"

#define DATA_SIZE              ((uint32_t)0x0640000U)
#define BUFFER_SIZE            ((uint32_t)_MAX_SS)
#define NB_BUFFER              DATA_SIZE / BUFFER_SIZE
#define NB_BLOCK_BUFFER        BUFFER_SIZE / BLOCKSIZE /* Number of Block (512o) by Buffer */
#define SD_TIMEOUT             ((uint32_t)0x00100000U)
#define ADDRESS                ((uint32_t)0x00004000U) /* SD Address to write/read data */
#define DATA_PATTERN           ((uint32_t)0xB5F3A5F3U) /* Data pattern to write */

#define ROZMIAR_BUFORA_LOGU		(7*512)
#define MAX_ROZMIAR_WPISU_LOGU	22
#define WPISOW_NA_SYNC			100		//po tylu zapisach linii robiony jest SYNC na karcie

//nadpisz timeouty gdyż wartosci domyślne wynoszą 0xFFFFFFFF i przy braku karty proces praktycznie się zawiesza
//#define SDMMC_DATATIMEOUT                  ((uint32_t)0xFFU)
//#define SDMMC_SWDATATIMEOUT                ((uint32_t)0xFFU)


//definicja znaczenia bitów rejestratora
#define STATREJ_FAT_GOTOWY		0x01
#define STATREJ_ZAPISZ_NAGLOWEK	0x02
#define STATREJ_WLACZONY		0x04
#define STATREJ_OTWARTY_PLIK	0x08
#define STATREJ_ZAMKNIJ_PLIK	0x10
#define STATREJ_BYL_OTWARTY		0x20
#define STATREJ_ZAPISZ_BMP		0x40
#define STATREJ_ZAPISZ_JPG		0x80
#define STATREJ_KARTA_OBECNA	0x8000

//definicje bitów konfiguracji logera
#define KLOG1_CZAS      	0x00000001	    //czas
#define KLOG1_DELTA_CZASU	0x00000002		//czas obsługi pętli głównej autopilota
#define KLOG1_PRES1    		0x00000004	    //ciśnienie atmosferyczne z czujnika ciśnienia 1
#define KLOG1_PRES2			0x00000008	    //ciśnienie atmosferyczne z czujnika ciśnienia 2
#define KLOG1_AMSL1    		0x00000010	    //wysokość barometryczna bezwzględna z czujnika ciśnienia 1
#define KLOG1_AMSL2    		0x00000020	    //wysokość barometryczna bezwzględna z czujnika ciśnienia 2
#define KLOG1_AGL1    		0x00000040	    //wysokość barometryczna względna z czujnika ciśnienia 1
#define KLOG1_AGL2    		0x00000080	    //wysokość barometryczna względna z czujnika ciśnienia 2

#define KLOG1_WARIO1    	0x00000100	    //wskazania wariometru 1
#define KLOG1_WARIO2    	0x00000200	    //wskazania wariometru 2
#define KLOG1_CISROZ1		0x00000400		//ciśnienie czujnika różnicowego 1
#define KLOG1_CISROZ2		0x00000800		//ciśnienie czujnika różnicowego 2
#define KLOG1_IAS1     		0x00001000	    //prędkość wzgledem powietrza z czujnika różnicowego 1
#define KLOG1_IAS2     	 	0x00002000	    //prędkość wzgledem powietrza z czujnika różnicowego 2
#define KLOG1_TEMPBARO1		0x00004000	    //temperatura czujnika ciśnienia 1
#define KLOG1_TEMPBARO2		0x00008000	    //temperatura czujnika ciśnienia 2
#define KLOG1_TEMPCISR1		0x00010000	    //temperatura czujnika ciśnienia różnicowego 1
#define KLOG1_TEMPCISR2		0x00020000	    //temperatura czujnika ciśnienia różnicowego 2

#define KLOG1_BAT1_NAP		0x00040000		//napięcie baterii 1
#define KLOG1_BAT1_PRAD		0x00080000		//prąd baterii 1
#define KLOG1_BAT1_ENER		0x00100000		//energia baterii 1
#define KLOG1_ZAS1_NAP		0x00200000		//napięcie wejściowe zasilania 1
#define KLOG1_BAT2_NAP		0x00400000		//napięcie baterii 2
#define KLOG1_BAT2_PRAD		0x00800000		//prąd baterii 2
#define KLOG1_BAT2_ENER		0x01000000		//energia baterii 2
#define KLOG1_ZAS2_NAP		0x02000000		//napięcie wejściowe zasilania 12

#define KLOG1_ADC1_1		0x04000000		//wejście analogowe 1, kanał 1
#define KLOG1_ADC1_2		0x08000000		//wejście analogowe 1, kanał 2
#define KLOG1_ADC2_1		0x10000000		//wejście analogowe 2, kanał 1
#define KLOG1_ADC2_2		0x20000000		//wejście analogowe 2, kanał 2
#define KLOG1_TEMP_CPU		0x40000000		//temperatura CPU
#define KLOG1_NAP_SERW		0x80000000		//napięcie magistrali serw



//drugie słowo konfiguracji logera
#define KLOG2_ZYROSUR1P 	0x00000001	    //surowa prędkość obrotowa P żyroskopu 1
#define KLOG2_ZYROSUR1Q 	0x00000002	    //surowa prędkość obrotowa Q żyroskopu 1
#define KLOG2_ZYROSUR1R 	0x00000004	    //surowa prędkość obrotowa R żyroskopu 1
#define KLOG2_ZYROSUR2P 	0x00000008	    //surowa prędkość obrotowa P żyroskopu 2
#define KLOG2_ZYROSUR2Q 	0x00000010	    //surowa prędkość obrotowa Q żyroskopu 2
#define KLOG2_ZYROSUR2R 	0x00000020	    //surowa prędkość obrotowa R żyroskopu 2

#define KLOG2_ZYRO1P    	0x00000040	    //skalibrowana prędkość obrotowa P żyroskopu 1
#define KLOG2_ZYRO1Q    	0x00000080	    //skalibrowana prędkość obrotowa Q żyroskopu 1
#define KLOG2_ZYRO1R    	0x00000100	    //skalibrowana prędkość obrotowa R żyroskopu 1
#define KLOG2_ZYRO2P    	0x00000200	    //skalibrowana prędkość obrotowa P żyroskopu 2
#define KLOG2_ZYRO2Q    	0x00000400	    //skalibrowana prędkość obrotowa Q żyroskopu 2
#define KLOG2_ZYRO2R    	0x00000800	    //skalibrowana prędkość obrotowa R żyroskopu 2

#define KLOG2_AKCEL1X   	0x00001000	    //przyspieszenie w osi X akcelerometru 1
#define KLOG2_AKCEL1Y   	0x00002000	    //przyspieszenie w osi Y akcelerometru 1
#define KLOG2_AKCEL1Z   	0x00004000	    //przyspieszenie w osi Z akcelerometru 1
#define KLOG2_AKCEL2X   	0x00008000	    //przyspieszenie w osi X akcelerometru 2
#define KLOG2_AKCEL2Y   	0x00010000	    //przyspieszenie w osi Y akcelerometru 2
#define KLOG2_AKCEL2Z   	0x00020000	    //przyspieszenie w osi Z akcelerometru 2

#define KLOG2_MAG1X     	0x00040000	    //składowa magnetyczna w osi X magnetometru 1
#define KLOG2_MAG1Y     	0x00080000	    //składowa magnetyczna w osi Y magnetometru 1
#define KLOG2_MAG1Z     	0x00100000	    //składowa magnetyczna w osi Z magnetometru 1
#define KLOG2_MAG2X     	0x00200000	    //składowa magnetyczna w osi X magnetometru 2
#define KLOG2_MAG2Y     	0x00400000	    //składowa magnetyczna w osi Y magnetometru 2
#define KLOG2_MAG2Z     	0x00800000	    //składowa magnetyczna w osi Z magnetometru 2
#define KLOG2_MAG3X     	0x01000000	    //składowa magnetyczna w osi X magnetometru 3
#define KLOG2_MAG3Y     	0x02000000	    //składowa magnetyczna w osi Y magnetometru 3
#define KLOG2_MAG3Z     	0x04000000	    //składowa magnetyczna w osi Z magnetometru 3

#define KLOG2_TEMPIMU1		0x08000000	    //temperatura IMU1
#define KLOG2_TEMPIMU2		0x10000000	    //temperatura IMU2


//trzecie słowo konfiguracji logera
#define KLOG3_BSP_IMUX  	0x00000001	    //kąt phi wektora inercji BSP (po filtrze Kalmana)
#define KLOG3_BSP_IMUY  	0x00000002	    //kąt theta wektora inercji
#define KLOG3_BSP_IMUZ  	0x00000004	    //kąt psi wektora inercji

#define KLOG3_KOMP_IMUX 	0x00000008	    //kąt phi wektora inercji uzyskany z filtra komplementarnego
#define KLOG3_KOMP_IMUY 	0x00000010	    //kąt theta wektora inercji
#define KLOG3_KOMP_IMUZ 	0x00000020	    //kąt psi wektora inercji

#define KLOG3_KWAT_IMUX 	0x00000040	    //kąt phi wektora inercji obliczone na kwaternionach
#define KLOG3_KWAT_IMUY 	0x00000080	    //kąt theta wektora inercji
#define KLOG3_KWAT_IMUZ 	0x00000100	    //kąt psi wektora inercji

#define KLOG3_AKC_IMUX  	0x00000200      //kąt phi obliczony na podstawie danych z akcelerometru
#define KLOG3_AKC_IMUY  	0x00000400	    //kąt theta obliczony na podstawie danych z akcelerometru
#define KLOG3_AKC_IMUZ  	0x00000800	    //kąt psi obliczony na podstawie danych z magnetometru
#define KLOG3_ZYR_IMUX 		0x00001000	    //kąt phi obliczony jako całka prędkości P z żyroskopu
#define KLOG3_ZYR_IMUY 	 	0x00002000	    //kąt theta obliczony jako całka prędkości Q z żyroskopu
#define KLOG3_ZYR_IMUZ 	 	0x00004000	    //kąt psi obliczony jako całka prędkości R z żyroskopu

#define KLOG3_GLONG     	0x00010000	    //szerokość geograficzna z GPS
#define KLOG3_GLATI     	0x00020000	    //długość geograficzna z GPS
#define KLOG3_GALTI     	0x00040000	    //wysokość n.p.m. z GPS
#define KLOG3_GSPED     	0x00080000	    //prędkość wzgledem ziemi z GPS
#define KLOG3_GCURS     	0x00100000	    //kurs względem ziemi z GPS
#define KLOG3_GSATS     	0x00200000	    //liczba widocznych satelitów
#define KLOG3_GVDOP     	0x00400000	    //Vertical Dilution of Precision
#define KLOG3_GHDOP     	0x00800000	    //Horizontal Dilution of Precision

#define KLOG3_GSPD_E    	0x01000000	    //niefiltrowana prędkość z GPS w kierunku wschodnim
#define KLOG3_GSPD_N    	0x02000000	    //niefiltrowana prędkość z GPS w kierunku północnym

#define KLOG3_PRES3			0x04000000	    //ciśnienie atmosferyczne z czujnika ciśnienia 3
#define KLOG3_AMSL3    		0x08000000	    //wysokość barometryczna bezwzględna z czujnika ciśnienia 3
#define KLOG3_WARIO3    	0x10000000	    //wskazania wariometru 3



//czwarte słowo konfiguracji logera
#define KLOG4_ODBRC_K1		0x00000001
#define KLOG4_ODBRC_K2		0x00000002
#define KLOG4_ODBRC_K3		0x00000004
#define KLOG4_ODBRC_K4		0x00000008
#define KLOG4_ODBRC_K5		0x00000010
#define KLOG4_ODBRC_K6		0x00000020
#define KLOG4_ODBRC_K7		0x00000040
#define KLOG4_ODBRC_K8		0x00000080
#define KLOG4_ODBRC_K9		0x00000100
#define KLOG4_ODBRC_K10		0x00000200
#define KLOG4_ODBRC_K11		0x00000400
#define KLOG4_ODBRC_K12		0x00000800
#define KLOG4_ODBRC_K13		0x00001000
#define KLOG4_ODBRC_K14		0x00002000
#define KLOG4_ODBRC_K15		0x00004000
#define KLOG4_ODBRC_K16		0x00008000
#define KLOG4_WYJRC_K1		0x00010000
#define KLOG4_WYJRC_K2		0x00020000
#define KLOG4_WYJRC_K3		0x00040000
#define KLOG4_WYJRC_K4		0x00080000
#define KLOG4_WYJRC_K5		0x00100000
#define KLOG4_WYJRC_K6		0x00200000
#define KLOG4_WYJRC_K7		0x00400000
#define KLOG4_WYJRC_K8		0x00800000
#define KLOG4_WYJRC_K9		0x01000000
#define KLOG4_WYJRC_K10		0x02000000
#define KLOG4_WYJRC_K11		0x04000000
#define KLOG4_WYJRC_K12		0x08000000
#define KLOG4_WYJRC_K13		0x10000000
#define KLOG4_WYJRC_K14		0x20000000
#define KLOG4_WYJRC_K15		0x40000000
#define KLOG4_WYJRC_K16		0x80000000

//piąte słowo konfiguracji logera
#define KLOG5_PID_PRZE_WZAD		0x00000001	//wartość zadana regulatora sterowania przechyleniem
#define KLOG5_PID_PRZE_FWEJ		0x00000002	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define KLOG5_PID_PRZE_FROZ		0x00000004	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define KLOG5_PID_PRZE_WY_P		0x00000008	//wyjście członu P
#define KLOG5_PID_PRZE_WY_I		0x00000010	//wyjście członu I
#define KLOG5_PID_PRZE_WY_D		0x00000020	//wyjście członu D
#define KLOG5_PID_PRZE_WYPRZ	0x00000040	//wyjście członu wyprzedzającego
#define KLOG5_PID_PRZE_WYJ		0x00000080	//wyjście regulatora sterowania przechyleniem
#define KLOG5_PID_PK_PRZE_WZAD	0x00000100	//wartość zadana regulatora sterowania prędkością kątową przechylenia
#define KLOG5_PID_PK_PRZE_FZAD	0x00000200	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
#define KLOG5_PID_PK_PRZE_FWEJ	0x00000400	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define KLOG5_PID_PK_PRZE_FROZ	0x00000800	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define KLOG5_PID_PK_PRZE_WY_P	0x00001000	//wyjście członu P
#define KLOG5_PID_PK_PRZE_WY_D	0x00002000	//wyjście członu D
#define KLOG5_PID_PK_PRZE_WYPRZ	0x00004000	//wyjście członu wyprzedzającego
#define KLOG5_PID_PK_PRZE_WYJ	0x00008000	//wyjście regulatora sterowania prędkością kątową przechylenia

#define KLOG5_PID_POCH_WZAD		0x00010000	//wartość zadana regulatora sterowania pochyleniem
#define KLOG5_PID_POCH_FWEJ		0x00020000	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define KLOG5_PID_POCH_FROZ		0x00040000	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define KLOG5_PID_POCH_WY_P		0x00080000	//wyjście członu P
#define KLOG5_PID_POCH_WY_I		0x00100000	//wyjście członu I
#define KLOG5_PID_POCH_WY_D		0x00200000	//wyjście członu D
#define KLOG5_PID_POCH_WYPRZ	0x00400000	//wyjście członu wyprzedzającego
#define KLOG5_PID_POCH_WYJ		0x00800000	//wyjście regulatora sterowania pochyleniem
#define KLOG5_PID_PK_POCH_WZAD	0x01000000	//wartość zadana regulatora sterowania prędkością kątową pochylenia
#define KLOG5_PID_PK_POCH_FZAD	0x02000000	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
#define KLOG5_PID_PK_POCH_FWEJ	0x04000000	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define KLOG5_PID_PK_POCH_FROZ	0x08000000	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define KLOG5_PID_PK_POCH_WY_P	0x10000000	//wyjście członu P
#define KLOG5_PID_PK_POCH_WY_D	0x20000000	//wyjście członu D
#define KLOG5_PID_PK_POCH_WYPRZ	0x40000000	//wyjście członu wyprzedzającego
#define KLOG5_PID_PK_POCH_WYJ	0x80000000	//wyjście regulatora sterowania prędkością kątową pochylenia

//szóste słowo konfiguracji logera
#define KLOG6_PID_ODCH_WZAD		0x00000001	//wartość zadana regulatora sterowania odchyleniem
#define KLOG6_PID_ODCH_FWEJ		0x00000002	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define KLOG6_PID_ODCH_FROZ		0x00000004	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define KLOG6_PID_ODCH_WY_P		0x00000008	//wyjście członu P
#define KLOG6_PID_ODCH_WY_I		0x00000010	//wyjście członu I
#define KLOG6_PID_ODCH_WY_D		0x00000020	//wyjście członu D
#define KLOG6_PID_ODCH_WYPRZ	0x00000040	//wyjście członu wyprzedzającego
#define KLOG6_PID_ODCH_WYJ		0x00000080	//wyjście regulatora sterowania odchyleniem
#define KLOG6_PID_PK_ODCH_WZAD	0x00000100	//wartość zadana regulatora sterowania prędkością kątową odchylenia
#define KLOG6_PID_PK_ODCH_FZAD	0x00000200	//przefiltrowana wartość zadana do liczenia wartosci wyprzedzającej
#define KLOG6_PID_PK_ODCH_FWEJ	0x00000400	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define KLOG6_PID_PK_ODCH_FROZ	0x00000800	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define KLOG6_PID_PK_ODCH_WY_P	0x00001000	//wyjście członu P
#define KLOG6_PID_PK_ODCH_WY_D	0x00002000	//wyjście członu D
#define KLOG6_PID_PK_ODCH_WYPRZ	0x00004000	//wyjście członu wyprzedzającego
#define KLOG6_PID_PK_ODCH_WYJ	0x00008000	//wyjście regulatora sterowania prędkością kątową odchylenia

#define KLOG6_PID_WYSO_WZAD		0x00010000	//wartość zadana regulatora sterowania wysokością
#define KLOG6_PID_WYSO_FWEJ		0x00020000	//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
#define KLOG6_PID_WYSO_FROZ		0x00040000	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define KLOG6_PID_WYSO_WY_P		0x00080000	//wyjście członu P
#define KLOG6_PID_WYSO_WY_I		0x00100000	//wyjście członu I
#define KLOG6_PID_WYSO_WY_D		0x00200000	//wyjście członu D
#define KLOG6_PID_WYSO_WYPRZ	0x00400000	//wyjście członu wyprzedzającego
#define KLOG6_PID_WYSO_WYJ		0x00800000	//wyjście regulatora sterowania odchyleniem
#define KLOG6_PID_PR_WYSO_WZAD	0x01000000	//wartość zadana regulatora prędkości zmiany wysokości
#define KLOG6_PID_PR_WYSO_FWEJ	0x02000000	//przefiltrowana wartość wejściowa do liczenia akcji różniczkującej
#define KLOG6_PID_PR_WYSO_FZAD	0x04000000	//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
#define KLOG6_PID_PK_WYSO_FROZ	0x08000000	//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
#define KLOG6_PID_PR_WYSO_WY_P	0x10000000	//wyjście członu P
#define KLOG6_PID_PR_WYSO_WY_D	0x20000000	//wyjście członu D
#define KLOG6_PID_PR_WYSO_WYPRZ	0x40000000	//wyjście członu wyprzedzającego
#define KLOG6_PID_PR_WYSO_WYJ	0x80000000	//wyjście regulatora sterowania prędkością zmiany wysokości

//siódme słowo konfiguracji logera - regultory pozycji geograficznej
#define KLOG7_PID_POZNAWN_WZAD	0x00000001
#define KLOG7_PID_POZNAWN_FWEJ	0x00000002
#define KLOG7_PID_POZNAWN_FROZ	0x00000004
#define KLOG7_PID_POZNAWN_WY_P	0x00000008
#define KLOG7_PID_POZNAWN_WY_I	0x00000010
#define KLOG7_PID_POZNAWN_WY_D	0x00000020
#define KLOG7_PID_POZNAWN_WYPRZ	0x00000040
#define KLOG7_PID_POZNAWN_WYJ	0x00000080
#define KLOG7_PID_PRENAWN_WZAD	0x00000100
#define KLOG7_PID_PRENAWN_FWEJ	0x00000200
#define KLOG7_PID_PRENAWN_FZAD	0x00000400
#define KLOG7_PID_PRENAWN_FROZ	0x00000800
#define KLOG7_PID_PRENAWN_WY_P	0x00001000
#define KLOG7_PID_PRENAWN_WY_D	0x00002000
#define KLOG7_PID_PRENAWN_WYPRZ	0x00004000
#define KLOG7_PID_PRENAWN_WYJ	0x00008000

#define KLOG7_PID_POZNAWE_WZAD	0x00010000
#define KLOG7_PID_POZNAWE_FWEJ	0x00020000
#define KLOG7_PID_POZNAWE_FROZ	0x00040000
#define KLOG7_PID_POZNAWE_WY_P	0x00080000
#define KLOG7_PID_POZNAWE_WY_I	0x00100000
#define KLOG7_PID_POZNAWE_WY_D	0x00200000
#define KLOG7_PID_POZNAWE_WYPRZ	0x00400000
#define KLOG7_PID_POZNAWE_WYJ	0x00800000
#define KLOG7_PID_PRENAWE_WZAD	0x01000000
#define KLOG7_PID_PRENAWE_FWEJ	0x02000000
#define KLOG7_PID_PRENAWE_FZAD	0x04000000
#define KLOG7_PID_PRENAWE_FROZ	0x08000000
#define KLOG7_PID_PRENAWE_WY_P	0x10000000
#define KLOG7_PID_PRENAWE_WY_D	0x20000000
#define KLOG7_PID_PRENAWE_WYPRZ	0x40000000
#define KLOG7_PID_PRENAWE_WYJ	0x80000000

//ósme słowo konfiguracji logera - filtry Kalmana
#define KLOG8_KALWYS_X0			0x00000001
#define KLOG8_KALWYS_X1			0x00000002
#define KLOG8_KALWYS_X2			0x00000004
#define KLOG8_KALWYS_X3			0x00000008
#define KLOG8_KALWYS_X4			0x00000010

#define KLOG8_KALWYS_K0			0x00000020
#define KLOG8_KALWYS_K1			0x00000040
#define KLOG8_KALWYS_K2			0x00000080
#define KLOG8_KALWYS_K3			0x00000100
#define KLOG8_KALWYS_K4			0x00000200
#define KLOG8_KALWYS_K5			0x00000400

#define KLOG8_KALWYS_P0			0x00000800
#define KLOG8_KALWYS_P1			0x00001000
#define KLOG8_KALWYS_P2			0x00002000
#define KLOG8_KALWYS_P3			0x00004000
#define KLOG8_KALWYS_P4			0x00008000



#define LICZBA_SLOW_REJESTRATORA	8
/*
#define LOG1CONF_RPM1       0x04000000	    //prędkość obrotowa 1
#define LOG1CONF_RPM2       0x08000000	    //prędkość obrotowa 2
#define LOG1CONF_RCHEALTH   0x10000000	    //poziom szumu w sygnale RC
#define LOG1CONF_RSSI       0x20000000	    //RSSI odbiornika RC
#define LOG1CONF_CARD_BUF   0x40000000	    //rozmiar zajetego bufora na karcie SD
#define LOG1CONF_ERROR      0x80000000	    //logowane błędy

#define LOG3CONF_FALTI      0x00000200	    //wysokość przefiltrowana filtrem Kalmana
#define LOG3CONF_FVARIO     0x00000400	    //wario przefiltrowane filtrem Kalmana
#define LOG3CONF_FACCZ      0x00000800	    //składowa Z przyspieszenia po transformacji napędzajaca filtr Kalmana

#define LOG3CONF_SONAR_DIST 0x00400000	    //wysokość odczytana z sonaru
#define LOG3CONF_ALTI_DRIFT 0x00800000	    //dryft wysokości czujnika barometrycznego
#define LOG3CONF_MIX_SUM    0x01000000	    //średnia wszystkich kanałów miksera, odpowiada za ciąg koptera

#define LOG3CONF_AZI2BAS    0x10000000	    //azymut do bazy - zapis w celu weryfikacji obliczeń
#define LOG3CONF_DIR2BAS    0x20000000	    //kurs strzałki do bazy - zapis w celu weryfikacji obliczeń
#define LOG3CONF_BASELON    0x40000000	    //współrzędne bazy
#define LOG3CONF_BASELAT    0x80000000	    //współrzędne bazy

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



void WatekRejestratora(void *argument);
uint8_t BSP_SD_IsDetected(void);
void HAL_SD_DriveTransceiver_1_8V_Callback(FlagStatus status);
uint8_t ObslugaPetliRejestratora(void);
uint8_t Wait_SDCARD_Ready(void);
void ObslugaZapisuJpeg(void);

#endif /* INC_REJESTRATOR_H_ */
