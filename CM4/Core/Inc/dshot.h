/*
 * dshot.h
 *
 *  Created on: Jul 29, 2025
 *      Author: Piotr
 */

#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_
#include "SysDefCM4.h"
#include "Wymiana.h"
#include "KonfigFram.h"


//definicje timingu
#define DS150_ZEGAR			6000000
#define DS300_ZEGAR			12000000
#define DS600_ZEGAR			24000000
#define DS1200_ZEGAR		48000000

//definicje protokołu DShot
#define DS_BITOW_GAZU		11
#define DS_BITOW_TELE		1
#define DS_BITOW_CRC		4
#define DS_BITOW_DANYCH		16
#define DS_BITOW_PRZERWY	9
#define DS_BITOW_LACZNIE	(DS_BITOW_DANYCH + DS_BITOW_PRZERWY)
#define DS_OFFSET_DSHOT		48		//wartości przesyłane protokołem są powiększone o taką wartość
#define DS_MAX_DANE			2048	//maksymalna wartość danych uwzgledniana przez protokół

//definicje protokołów
#define PROTOKOL_DSHOT150	1
#define PROTOKOL_DSHOT300	2
#define PROTOKOL_DSHOT600	3
#define PROTOKOL_DSHOT1200	4


//definicje poleceń DShot na podstawie: https://betaflight.com/docs/development/API/Dshot
#define DSHOT_CMD_MOTOR_STOP		0	//Currently not implemented
#define DSHOT_CMD_BEEP1				1	//Wait at least length of beep (260ms) before next command
#define DSHOT_CMD_BEEP2				2	//Wait at least length of beep (260ms) before next command
#define DSHOT_CMD_BEEP3				3	//Wait at least length of beep (260ms) before next command
#define DSHOT_CMD_BEEP4				4	//Wait at least length of beep (260ms) before next command
#define DSHOT_CMD_BEEP5				5	//Wait at least length of beep (260ms) before next command
#define DSHOT_CMD_ESC_INFO			6	//Wait at least 12ms before next command
#define DSHOT_CMD_SPIN_DIRECTION_1	7	//Need 6x
#define DSHOT_CMD_SPIN_DIRECTION_2	8	//Need 6x
#define DSHOT_CMD_3D_MODE_OFF		9	//Need 6x
#define DSHOT_CMD_3D_MODE_ON		10	//Need 6x
#define DSHOT_CMD_SETTINGS_REQUEST	11	//Currently not implemented
#define DSHOT_CMD_SAVE_SETTINGS		12	//Need 6x, wait at least 35ms before next command
#define DSHOT_EXTENDED_TELEMETRY_ENABLE	13	//Need 6x (only on EDT enabled firmware)
#define DSHOT_EXTENDED_TELEMETRY_DISABLE	14	//Need 6x (only on EDT enabled firmware)
//15..19	-	Not yet assigned
#define DSHOT_CMD_SPIN_DIRECTION_NORMAL		20	//Need 6x
#define DSHOT_CMD_SPIN_DIRECTION_REVERSED	21	//Need 6x
#define DSHOT_CMD_LED0_ON			22
#define DSHOT_CMD_LED1_ON			23
#define DSHOT_CMD_LED2_ON			24
#define DSHOT_CMD_LED3_ON			25
#define DSHOT_CMD_LED0_OFF			26
#define DSHOT_CMD_LED1_OFF			27
#define DSHOT_CMD_LED2_OFF			28
#define DSHOT_CMD_LED3_OFF			29
//#define Audio_Stream_mode_on/Off	30	//Currently not implemented
//#define Silent_Mode_on/Off			31	//Currently not implemented
#define DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE		32	//Need 6x. Disables commands 42 to 47
#define DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE		33	//Need 6x. Enables commands 42 to 47
#define DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY		34	//Need 6x. Enables commands 42 to 47 and sends eRPM if normal DShot frame
#define DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY	35	//Need 6x. Enables commands 42 to 47 and sends eRPM period if normal DShot frame
//36..41	-	Not yet assigned
#define DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY		42	//1°C per LSB
#define DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY			43	//10mV per LSB, 40.95V max
#define DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY			44	//100mA per LSB, 409.5A max
#define DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY		45	//10mAh per LSB, 40.95Ah max
#define DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY			46	//100erpm per LSB, 409500erpm max
#define DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY		47	//16µs per LSB, 65520µs max TBD
#define DSHOT_CMD_NORMALNA_PRACA						48


typedef struct
{
	uint32_t nT0H;
	uint32_t nT1H;
	uint32_t nBit;
} stDShot_t;


//deklaraje funkcji
uint8_t UstawTrybDShot(uint8_t chProtokol, uint8_t chKanal);
//uint8_t AktualizujDShotDMA(uint16_t sWysterowanie, uint8_t chKanal);
uint8_t AktualizujDShotDMA(uint16_t sWysterowanie, uint8_t cPolecenie, uint8_t chKanal);

#endif /* INC_DSHOT_H_ */
