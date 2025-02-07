
#include "sys_def_CM4.h"

#define ADR_MS5611   0xEE

//definicje Poleceń czujnika MS
#define PMS_RESET			0x1E
#define PMS_CONV_D1_OSR256	0x40
#define PMS_CONV_D1_OSR512	0x42
#define PMS_CONV_D1_OSR1024	0x44
#define PMS_CONV_D1_OSR2048	0x46
#define PMS_CONV_D1_OSR4096	0x48
#define PMS_CONV_D2_OSR256	0x50
#define PMS_CONV_D2_OSR512	0x52
#define PMS_CONV_D2_OSR1024	0x54
#define PMS_CONV_D2_OSR2048	0x56
#define PMS_CONV_D2_OSR4096	0x58
#define PMS_ADC_READ		0x00
#define PMS_PROM_READ_C1	0xA2
#define PMS_PROM_READ_C2	0xA4
#define PMS_PROM_READ_C3	0xA6
#define PMS_PROM_READ_C4	0xA8
#define PMS_PROM_READ_C5	0xAA
#define PMS_PROM_READ_C6	0xAC



uint8_t InicjujMS5611(void);
uint16_t CzytajKonfiguracjeMS5611(uint8_t chAdres);
uint8_t StartKonwersjiMS5611(uint8_t chTyp);
uint32_t CzytajWynikKonwersjiMS5611(void);
float MS5611_LiczTemperature(uint32_t nKonwersja, int32_t* ndTemp);
float MS5611_LiczCisnienie(uint32_t nKonwersja, int32_t ndTemp);
uint8_t ObslugaMS5611(void);
