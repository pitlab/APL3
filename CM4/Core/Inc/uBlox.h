#ifndef SRC_UBLOX_H_
#define SRC_UBLOX_H_

//Struktura ramki UBX
//0xB5 - preambuła 1
//0x62 - preambuła 2
//Klasa
//ID
//Długość L
//Długość H
//Payload
//CK_A - suma kontrolna L liczona od Klasy do końca payloadu
//CK_B - suma kontrolna H



//Definicje poleceń konfiguracyjnych dla odbiorników uBlox

// --- CFG-DAT --- - ustawienie formatu daty
#define CFG_DAT_LEN 10
const uint8_t CFG_DAT[CFG_DAT_LEN] = {0xB5, 0x62, 0x06, 0x06, 0x02, 0x00, 0x00, 0x00, 0x0E, 0x4A}; // CFG-DAT (ustaw standard daty)

// --- CFG-MSG --- - Ustawienia protokołów
// format "'PRRAMBUłA' 'KOMENDA'"
#define CFG_MSG_PREAMB_LEN 6
const uint8_t CFG_MSG_PREAMB[CFG_MSG_PREAMB_LEN] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00};

// komendy
#define CFG_MSG_LEN 10
#define CFG_MSG_COMMANDS 47
const uint8_t CFG_MSG_LIST[CFG_MSG_COMMANDS][CFG_MSG_LEN] = {
    { 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0xB2 }, // NAV-POSECEF
    { 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9 }, // NAV-POSLLH
    { 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0 }, // NAV-STATUS
    { 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC7 }, // NAV-DOP
    { 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xD5 }, // NAV-SOL
    { 0x01, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x22 }, // NAV-VELECEF
    { 0x01, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x29 }, // NAV-VELNED
    { 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x8B }, // NAV-TIMEGPS
    { 0x01, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x92 }, // NAV-TIMEUTC
    { 0x01, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x99 }, // NAV-CLOCK
    { 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xFB }, // NAV-SVINFO
    { 0x01, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x02 }, // NAV-DGPS
    { 0x01, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x09 }, // NAV-SBAS
    { 0x02, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x93 }, // RXM-SVIN
    { 0x02, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0xA8 }, // RXM-RCT5
    { 0x0A, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B, 0x01 }, // MON-IO
    { 0x0A, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x16 }, // MON-EXCEPT
    { 0x0A, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1D }, // MON-MSGPP
    { 0x0A, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x24 }, // MON-RXBUF
    { 0x0A, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x2B }, // MON-TXBUF
    { 0x0A, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x32 }, // MON-HW
    { 0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x39 }, // MON-USB
    { 0x0A, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0xD3 }, // MON-PT
    { 0x0A, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0xDA }, // MON-RXR
    { 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0xFB }, // AID-REQ
    { 0x0B, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1E }, // AID-MAPMATCH
    { 0x0B, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4A, 0x4B }, // AID-ALM
    { 0x0B, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x52 }, // AID-EPH
    { 0x0B, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x59 }, // AID-ALPSERV
    { 0x0B, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0x60 }, // AID-AOP
    { 0x0D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1D, 0x12 }, // TIM-TP
    { 0x0D, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x20 }, // TIM-TM
    { 0x0D, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x35 }, // TIM-TM2
    // NMEA
    { 0xF0, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x02, 0x2E }, // NMEA GxGGA /2
    { 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B }, // NMEA GxGLL
    { 0xF0, 0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x01, 0x07, 0x4B }, // NMEA GxGSA /5
    { 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 }, // NMEA GxGSV
    { 0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x05, 0x45 }, // NMEA GxRMC /1
    { 0xF0, 0x05, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x79 }, // NMEA GxVTG /10
    { 0xF0, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x4D }, // NMEA GxGRS
    { 0xF0, 0x07, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x10, 0x86 }, // NMEA GxGST /10 - dokładność GPS
    { 0xF0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x5B }, // NMEA GxZDA
    { 0xF0, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x62 }, // NMEA GxGBS
    { 0xF0, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x69 }, // NMEA GxDTM
    { 0xF1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x47 }, // NMEA PUBX,00 - ma dokładne info o błędach
    { 0xF1, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40 }, // NMEA PUBX,03
    { 0xF1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x47 }, // NMEA PUBX,04
};

// --- CFG-SBAS ---
#define CFG_SBAS_LEN 16
const uint8_t CFG_SBAS[CFG_SBAS_LEN] = { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x01, 0x03, 0x03, 0x00, 0x51, 0x62, 0x06, 0x00, 0xE4, 0x2F};

// --- CFG-NAV ---
#define CFG_NAV_LEN 37
const uint8_t CFG_NAV[CFG_NAV_LEN] = { 0xB5, 0x62, 0x06, 0x03, 0x1C, 0x00, 0x06, 0x03, 0x10, 0x18, 0x14, 0x05, 0x00, 0x3C, 0x3C, 0x14, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x17, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x0F, 0x00, 0x00, 0x00, 0x91, 0x54};

// --- CFG-NAV2 ---
#define CFG_NAV2_LEN 48
const uint8_t CFG_NAV2[CFG_NAV2_LEN] = { 0xB5, 0x62, 0x06, 0x1A, 0x28, 0x00, 0x06, 0x00, 0x00, 0x00, 0x03, 0x04, 0x10, 0x02, 0x50, 0xC3, 0x00, 0x00, 0x18, 0x14, 0x05, 0x3C, 0x00, 0x03, 0x00, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x0D};

// --- CFG-NAV5 ---
#define CFG_NAV5_LEN 44
const uint8_t CFG_NAV5[CFG_NAV5_LEN] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xFE};

// --- CFG-NMEA --- - ustawienie wersji protokołu
#define CFG_NMEA_LEN 19
const uint8_t CFG_NMEA[CFG_NMEA_LEN] = { 0xB5, 0x62, 0x06, 0x17, 0x04, 0x00, 0x00, 0x23, 0x00, 0x02, 0x46, 0x54, 0x62, 0x06, 0x17, 0x00, 0x00, 0x1D, 0x5D };

// --- CFG-RATE --- - pomiar 5 Hz
#define CFG_RATE_LEN 14
const uint8_t CFG_RATE[CFG_RATE_LEN] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};

// --- CFG-RXM --- - Performance Mode#define CFG-SBAS-LEN
#define CFG_RXM_LEN 10
const uint8_t CFG_RXM[CFG_RXM_LEN] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};

// --- CFG-PRT --- - konfiguracja portów
#define CFG_PRT_LEN 28
#define CFG_PRT_COMMANDS 5
const uint8_t CFG_PRT_LIST[CFG_PRT_COMMANDS][CFG_PRT_LEN] = {
    { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x48 },	// I2C off
    { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x02, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x99, 0x7F },	// UART2 off
    { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1D, 0x84 }, // USB off
    { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x86 },	// SPI OFF
    { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x92, 0x8A }	// UART1 38400 in=(uUBX+NMEA+RTCM) out=(NMEA)
};

#endif /* SRC_UBLOX_H_ */
