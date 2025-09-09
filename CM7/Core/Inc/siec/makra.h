/*
 * Makrodefinicje zaczerpniete z ksiażki Marcina Peczarskiego "Mikrokontrolery STM32 w sieci Ethernet"
 *
 *
 *  Created on: Sep 8, 2025
 *      Author: PitLab
 */

#ifndef INC_SIEC_MAKRA_H_
#define INC_SIEC_MAKRA_H_

/* Definiujemy trzy priorytety przerwań. */
#define HIGH_IRQ_PRIO 1
#define LWIP_IRQ_PRIO 2
#define LOW_IRQ_PRIO  3
#define PREEMPTION_PRIORITY_BITS 2

/* Makro IRQ_DECL_PROTECT(prv) deklaruje zmienną prv, która
   przechowuje informację o blokowanych przerwaniach. Makro
   IRQ_PROTECT(prv, lvl) zapisuje w zmiennej prv stan aktualnie
   blokowanych przerwań i blokuje przerwania o priorytecie niższym
   lub równym lvl. Makro IRQ_UNPROTECT(prv) przywraca stan blokowania
   przerwań zapisany w zmiennej prv. */
#if defined __GNUC__
  #define IRQ_DECL_PROTECT(prv)                             \
    u32_t prv
  #define IRQ_PROTECT(prv, lvl) ({                          \
    u32_t tmp;                                              \
    asm volatile (                                          \
      "mrs %0, BASEPRI\n\t"                                 \
      "movs %1, %2\n\t"                                     \
      "msr BASEPRI, %1" :                                   \
      "=r" (prv), "=r" (tmp) :                              \
      "i" ((lvl) << (8 - PREEMPTION_PRIORITY_BITS))         \
    );                                                      \
  })
  #define IRQ_UNPROTECT(prv) ({                             \
    asm volatile ("msr BASEPRI, %0" : : "r" (prv));         \
  })
#else
  #define IRQ_DECL_PROTECT(prv)                             \
    u32_t prv
  #define IRQ_PROTECT(prv, lvl)                             \
    (prv = __get_BASEPRI(),                                 \
     __set_BASEPRI((lvl) << (8 - PREEMPTION_PRIORITY_BITS)))
  #define IRQ_UNPROTECT(prv)                                \
    __set_BASEPRI(prv)
#endif

#endif /* INC_SIEC_MAKRA_H_ */
