

#ifndef INC_LCD_MEM_H_
#define INC_LCD_MEM_H_
#include "stm32h7xx_hal.h"

//Rozmiar bufora okna
#define SZER_BUFORA		DISP_X_SIZE
#define WYS_BUFORA		DISP_Y_SIZE
#define NAPIS_Z_TLEM	0	//napis ma tło w kolorze tła
#define NAPIS_PRZEZR	1	//napis jest rysowany bez tła

//void PostawPikselwBuforze(uint16_t x, uint16_t y, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru);
void PostawPikselwBuforze(uint16_t x, uint16_t y, uint16_t sSzerokosc, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru);
//void WypelnijEkranwBuforze1(uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru);
void WypelnijEkranwBuforze(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *chBufor, uint16_t sKolor);
void RysujProstokatWypelnionywBuforze(uint16_t sStartX, uint16_t sStartY, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru);
void RysujLiniewBuforze(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t sSzerokosc, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru);
void RysujLiniePoziomawBuforze(uint16_t x1, uint16_t x2, uint16_t y, uint16_t sSzerokosc, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru);
void RysujLiniePionowawBuforze(uint16_t x, uint16_t y1, uint16_t y2, uint16_t sSzerokosc, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru);
void RysujOkragwBuforze(uint16_t x, uint16_t y, uint16_t promien, uint16_t sSzerokosc, uint8_t *chBufor, uint8_t *chKolor, uint8_t chRozmKoloru);
void RysujZnakwBuforze(uint8_t chZnak, uint16_t x, uint16_t y, uint16_t sSzerokosc, uint8_t *chBufor, uint8_t *chKolor, uint8_t *chTlo, uint8_t chRozmKoloru);
void RysujNapiswBuforze(char *chNapis, uint16_t x, uint16_t y, uint16_t sSzerokosc, uint8_t *chBufor, uint8_t *chKolor, uint8_t *chTlo, uint8_t chRozmKoloru);
void RysujHistogramOSD_RGB32(uint8_t *chOSD, uint8_t *histR, uint8_t *histG, uint8_t *histB);

#endif /* INC_LCD_MEM_H_ */
