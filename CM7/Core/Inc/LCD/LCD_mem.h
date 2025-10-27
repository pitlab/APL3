

#ifndef INC_LCD_MEM_H_
#define INC_LCD_MEM_H_
#include "stm32h7xx_hal.h"


void WypelnijBuforEkranu(uint8_t *chBufor, uint32_t nKolorARGB);
void RysujProstokatWypelnionywBuforze(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *chBufor, uint32_t nKolorARGB);
void RysujLiniewBuforze(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t *chBufor, uint32_t nKolorARGB);
void RysujLiniePoziomawBuforze(uint16_t x1, uint16_t x2, uint16_t y, uint8_t *chBufor, uint32_t nKolorARGB);
void RysujLiniePionowawBuforze(uint16_t x, uint16_t y1, uint16_t y2, uint8_t *chBufor, uint32_t nKolorARGB);
void RysujOkragwBuforze(uint16_t x, uint16_t y, uint16_t promien, uint8_t *chBufor, uint32_t nKolorARGB);
void RysujZnakwBuforze(uint8_t chZnak, uint16_t x, uint16_t y, uint8_t *chBufor, uint32_t nKolorARGB, uint32_t nTloARGB, uint8_t chPrzezroczystosc);
void RysujNapiswBuforze(char *chNapis, uint16_t x, uint16_t y, uint8_t *chBufor, uint32_t nKolorARGB, uint32_t nTloARGB, uint8_t chPrzezroczystosc);

#endif /* INC_LCD_MEM_H_ */
