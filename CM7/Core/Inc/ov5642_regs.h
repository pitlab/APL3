/*
 * ov5642_regs.h
 *
 *  Created on: Jul 12, 2025
 *      Author: PitLab
 */

#ifndef INC_OV5642_REGS_H_
#define INC_OV5642_REGS_H_
#include "sys_def_CM7.h"


struct sensor_reg {
	uint16_t reg;
	uint8_t val;
};

#define KAM_ROZDZIELCZOSC_X		2592	//natywna rozdzielczość przetwornika obrazu
#define KAM_ROZDZIELCZOSC_Y		1944
#define KAM_SZEROKOSC_OBRAZU	480		//ustawiona wyjściowa rozdzielczosć obrazu
#define KAM_WYSOKOSC_OBRAZU		320
#define	KAM_ZOOM_CYFROWY		3
#define KAM_POCZATEK_OKNA_X		((KAM_ROZDZIELCZOSC_X - KAM_ZOOM_CYFROWY * KAM_SZEROKOSC_OBRAZU)/2)
#define	KAM_POCZATEK_OKNA_Y		((KAM_ROZDZIELCZOSC_Y - KAM_ZOOM_CYFROWY * KAM_WYSOKOSC_OBRAZU)/2)



#endif /* INC_OV5642_REGS_H_ */
