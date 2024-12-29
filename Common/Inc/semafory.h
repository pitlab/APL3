/*
 * semafory.h
 *
 *  Created on: Dec 29, 2024
 *      Author: PitLab
 */

#ifndef INC_SEMAFORY_H_
#define INC_SEMAFORY_H_

//semafory do obsługi bufora wymiany
#ifndef HSEM_CM4_TO_CM7
#define HSEM_CM4_TO_CM7 (1U)
#endif

#ifndef HSEM_CM7_TO_CM4
#define HSEM_CM7_TO_CM4 (2U)
#endif

//semafor do współdzielenia SPI5 między wyświetlacz działajacy w wątku wyświetlacza i resztę działajacą w pętli głównej
#ifndef HSEM_SPI6_WYSW
#define HSEM_SPI6_WYSW (3U)
#endif



#endif /* INC_SEMAFORY_H_ */
