/*
 * moduly_wew.h
 *
 *  Created on: Dec 4, 2024
 *      Author: PitLab
 */

#ifndef INC_MODULY_WEW_H_
#define INC_MODULY_WEW_H_

#include "sys_def_CM4.h"


#define SPI_EXTIO		0x40	//Adres SPI układu extendera IO
#define SPI_EXTIO_RD	1		//odczyt
#define SPI_EXTIO_WR	0		//zapis

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


uint8_t InicjujModulyWew(void);
uint8_t UstawDekoderModulow(uint8_t modul);
uint8_t UstawAdresNaModule(uint8_t chAdres);
uint8_t WyslijDaneExpandera(uint8_t daneWy);
uint8_t PobierzDaneExpandera(uint8_t* daneWe);


#endif /* INC_MODULY_WEW_H_ */
