#ifndef SRC_NMEA_H_
#define SRC_NMEA_H_

#include "SysDefCM4.h"
#include "wymiana.h"

#define ST_ERR				0   //błąd w dekodowaniu, szukaj dalej
#define ST_NAGLOWEK1		0   //wykryj znak $
#define ST_NAGLOWEK2		1   //wykryj znaki GP
#define ST_MSGID_GP			2
#define ST_MSGID_GN			3
#define ST_MSGID_MTK		4


#define ST_GGA_TIME			10
#define ST_GGA_LATITUDE		11
#define ST_GGA_LAT_NS		12
#define ST_GGA_LONGITUD		13
#define ST_GGA_LON_WE		14
#define ST_GGA_FIX_IND		15
#define ST_GGA_SAT_USE		16
#define ST_GGA_HDOP			17
#define ST_GGA_ALTITUDE 	18

#define ST_GLL_LAT			20

#define ST_GSA_MODE1		30
#define ST_GSA_MODE2		31
#define ST_GSA_SAT_USED		32
#define ST_GSA_PDOP			33
#define ST_GSA_HDOP			34
#define ST_GSA_VDOP			35
#define ST_GSA_MODE1_GS 	36
#define ST_GSA_MODE2_GS 	37
#define ST_GSA_SAT_USED_GS 	38
#define ST_GSA_PDOP_GS 		39  //ciag dlaszy dla GSA od nr 45


#define	ST_GSV_NOMSG		40
#define ST_GSA_HDOP_GS  	45
#define ST_GSA_VDOP_GS  	46

#define ST_MSS_SIGSTR		50

#define ST_PMTK011			58
#define ST_PMTK010			59


#define ST_RMC_UTC			60
#define ST_RMC_STATUS		61
#define ST_RMC_LATITUDE		62
#define ST_RMC_LAT_NS		63
#define ST_RMC_LONGITUD		64
#define ST_RMC_LON_WE		65
#define ST_RMC_SPEED		66
#define ST_RMC_COURSE		67
#define ST_RMC_DATE			68
#define ST_RMC_MAG_VAR		69

#define ST_RMC_UTC_GS       70
#define ST_RMC_STATUS_GS    71
#define ST_RMC_LATITUDE_GS  72
#define ST_RMC_LAT_NS_GS    73
#define ST_RMC_LONGITUD_GS  74
#define ST_RMC_LON_WE_GS    75
#define ST_RMC_SPEED_GS	    76
#define ST_RMC_COURSE_GS    77
#define ST_RMC_DATE_GS	    78
#define ST_RMC_MAG_VAR_GS   79

#define ST_VTG_COURSE		80
#define ST_ZDA_UTC			90

#define ST_TXT				100
#define ST_TXT_OPIS			101
#define ST_TXT_ANTSUPERV	102
#define ST_TXT_MOD			103

#define BUF_STAN_SIZE   16


//definicje funkcji
uint8_t DekodujNMEA(uint8_t cDaneIn, stGnss_t *stGnss);
uint8_t Asci2UChar(uint8_t *cDaneIn, uint8_t cLiczbaZnakow);
uint16_t Asci2UShort(uint8_t *cDaneIn, uint8_t cLiczbaZnakow);
uint32_t Asci2ULong(uint8_t *cDaneIn, uint8_t cLiczbaZnakow);
uint8_t DecodeLonLat(uint8_t *cDaneIn, uint8_t cLiczbaZnakow, double *dStopnie);
//uint8_t PrepareGPSInitFrame(uint8_t chFrameNr, int8_t *chDane);
uint8_t DecodeFloat(uint8_t *cDaneIn, uint8_t cLiczbaZnakow, float *fResult);
uint8_t InitGPS(void);

//deklaracje zmienych
extern uint16_t sInitFlag;   //zawiera flagi inicjalizacji zasobów sprzętowych


//deklaracje funkcji
extern void PutData0 (uint8_t *data, uint8_t data_size);
extern void SetUartSpeed(uint8_t cPortNum, uint32_t nBaud);
extern uint32_t CountTime(uint32_t *nLastTim);
extern void InitUBlox(void);
extern void SpeedUBlox(void);


#endif /* SRC_NMEA_H_ */
