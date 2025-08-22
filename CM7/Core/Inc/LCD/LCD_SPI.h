/*
 * LCD_SPI.h
 *
 *  Created on: Aug 21, 2025
 *      Author: PitLab
 */

#ifndef INC_LCD_LCD_SPI_H_
#define INC_LCD_LCD_SPI_H_
#include "stm32h7xx_hal.h"


uint8_t LCD_write_command8(uint8_t chDane);
uint8_t LCD_write_command16(uint8_t chDane);
uint8_t LCD_WrData(uint8_t* chDane, uint16_t sIlosc);
uint8_t LCD_WrDataDMA(uint8_t* chDane, uint16_t sIlosc);
uint8_t LCD_write_data16(uint8_t chDane1, uint8_t chDane2);
uint8_t LCD_write_dat_jed8(uint8_t chDane);
uint8_t LCD_write_dat_jed16(uint8_t chDane);
uint8_t LCD_write_dat_pie8(uint8_t chDane);
uint8_t LCD_write_dat_pie16(uint8_t chDane);
void LCD_write_dat_sro8(uint8_t chDane);
void LCD_write_dat_sro16(uint8_t chDane);
void LCD_write_dat_ost8(uint8_t chDane);
void LCD_write_dat_ost16(uint8_t chDane);
void LCD_data_read(uint8_t *chDane, uint8_t chIlosc);

#endif /* INC_LCD_LCD_SPI_H_ */
