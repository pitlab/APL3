/*
 * moduly_SPI.h
 *
 *  Created on: Nov 8, 2024
 *      Author: PitLab
 */

#include "sys_def_CM7.h"


#ifndef INC_MODULY_SPI_H_
#define INC_MODULY_SPI_H_

//definicje adresów dekodera zewnętrznych modułów SPI
#define CS_TP 	0	//Panel dotykowy,
#define CS_LCD 	1	//wyświetlacz,
#define CS_IO 	2	//extendery IO
#define CS_NIC 	3	//nic nie jest wybrane

//Adresy SPI układów extenderów IO
#define SPI_EXTIO_0		0x42	//U41
#define SPI_EXTIO_1		0x40	//U42
#define SPI_EXTIO_2		0x44	//U43
#define SPI_EXTIO_RD	1		//odczyt
#define SPI_EXTIO_WR	0		//zapis

#define LICZBA_EXP_SPI_ZEWN	3		//liczba ekspanderów IO na magistrali SPI modułów zewnętrznych

//rejestry układu MCP23S08
#define MCP23S08_IODIR		0x00	//I/O DIRECTION (IODIR) REGISTER: 1=input, 0=output
#define MCP23S08_IOPL		0x01	//INPUT POLARITY (IPOL) REGISTER: 1 = GPIO register bit will reflect the opposite logic state of the input pin, 0= the same
#define MCP23S08_GPINTEN	0x02	//INTERRUPT-ON-CHANGE CONTROL (GPINTEN) REGISTER: 1 = Enable GPIO input pin for interrupt-on-change event, 0=disable
#define MCP23S08_DEFVAL		0x03	//DEFAULT COMPARE (DEFVAL) REGISTER FOR INTERRUPT-ONCHANGE: DEF7:DEF0: These bits set the compare value for pins configured for interrupt-on-change from defaults <7:0>. Refer to INTCON. If the associated pin level is the opposite from the register bit, an interrupt occurs
#define MCP23S08_INTCON		0x04	//INTERRUPT CONTROL (INTCON) REGISTER: = Controls how the associated pin value is compared for interrupt-on-change, 0 = Pin value is compared against the previous pin value.
#define MCP23S08_IOCON		0x05	//CONFIGURATION (IOCON) REGISTER:
#define MCP23S08_GPPU		0x06	//PULL-UP RESISTOR CONFIGURATION (GPPU) REGISTER: 1 = Pull-up enabled, 0 = Pull-up disabled.
#define MCP23S08_INTF		0x07	//NTERRUPT FLAG (INTF) REGISTER (RO): 1 = Pin caused interrupt. 0 = Interrupt not pending
#define MCP23S08_INTCAP		0x08	//INTERRUPT CAPTURE (INTCAP) REGISTER: 1 = Logic-high, 0 = Logic-low.
#define MCP23S08_GPIO		0x09	//PORT (GPIO) REGISTER: GP7:GP0: These bits reflect the logic level on the pins <7:0>: 1 = Logic-high, 0 = Logic-low.
#define MCP23S08_OLAT		0x0A	//OUTPUT LATCH REGISTER (OLAT): OL7:OL0: These bits reflect the logic level on the output latch <7:0>: 1 = Logic-high, 0 = Logic-low.

uint8_t InicjujSPIModZewn(void);
uint8_t UstawDekoderZewn(uint8_t uklad);
uint8_t PobierzStanDekoderaZewn(void);
uint8_t WyslijDaneExpandera(uint8_t adres, uint8_t daneWy);
uint8_t PobierzDaneExpandera(uint8_t adres, uint8_t* daneWe);
uint8_t WymienDaneExpanderow(void);

#endif /* INC_MODULY_SPI_H_ */
