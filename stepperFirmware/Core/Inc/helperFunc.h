#include <stdint.h>  // Needed for int32_t and uint16_t
#include "string.h"
void initStruct(void); // init the stepper struct on startup
void initEEprom(void); // init the eeprom on startup and set the statistics for the struct
int32_t read_int32(uint16_t addr_hi, uint16_t addr_lo);
int getBatteryState(void);
uint32_t rememberRecordEEPROM(void);

