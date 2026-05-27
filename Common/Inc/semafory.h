/*
 * semafory.h
 *
 *  Created on: Dec 29, 2024
 *      Author: PitLab
 */

#ifndef INC_SEMAFORY_H_
#define INC_SEMAFORY_H_

//definicje procesów obsługiwanych przez semafory
#define HSEM_LCD	1	//semafor HSEM_SPI5_WYSW przejęty przez wyświetlacz
#define HSEM_DOTYK	2	//semafor HSEM_SPI5_WYSW przejęty przez panel dotykowy
#define HSEM_EXP	3	//semafor HSEM_SPI5_WYSW przejęty przez extender IO
#define HSEM_CM4	4	//semafory HSEM_CM7_TO_CM4 lub HSEM_CM4_TO_CM7 przejęte przez rdzeń CM4
#define HSEM_CM7	7	//semafory HSEM_CM7_TO_CM4 lub HSEM_CM4_TO_CM7 przejęte przez rdzeń CM7

//semafory do obsługi bufora wymiany
#ifndef HSEM_CM4_TO_CM7
#define HSEM_CM4_TO_CM7 (1U)
#endif

#ifndef HSEM_CM7_TO_CM4
#define HSEM_CM7_TO_CM4 (2U)
#endif

//semafor do współdzielenia SPI5 między wyświetlacz działajacy w wątku wyświetlacza i panel dotykowy oraz extender IO działajacą w pętli głównej
#ifndef HSEM_SPI5
#define HSEM_SPI5 (3U)
#endif



#endif /* INC_SEMAFORY_H_ */
