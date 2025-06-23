#include "helperFunc.h"
#include "main.h"
#include "eeprom_emul_types.h"

// two ints and 6 floats
#define ADDR_INT1_H 0x0001	   // hour
#define ADDR_INT1_L 0x0002	   // hour
#define ADDR_INT2_H 0x0003	   // record
#define ADDR_INT2_L 0x0004	   // record
#define ADDR_FLOAT_BASE 0x0005 // Float1 uses 0x0005 & 0x0006, etc.


// MATT
uint32_t a_VarDataTab[8] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
#define FRACTIONAL_SCALE 10000 // 4 dec places
__IO uint32_t ErasingOnGoing = 0;
extern myVal;

void initStruct(){
    stepper.steps = 0;
	stepper.hour = 0;
//	stepper.record = 0;
	stepper.pace = IDLE;
	stepper.screen = HOMESCREEN;
	stepper.lastStepTime = 0;

	stepper.showButtons = 0;
	stepper.lastButtonPress = 9999999;
}

int getBatteryState(void) {
	uint16_t level = readADCharrsha(BATTMONchannel); // Ensure BATTMONchannel is defined
	    if (level < 2180) {
	        return 1;
	    } else if (level <= 2450) { // Include 2450 in nominal
	        return 2;
	    } else { // level > 2450
	        return 3;
	    }
}

void outputBatteryLevel(){
	static uint32_t lastSampleTime = 0;

	if (HAL_GetTick() - lastSampleTime >= 3000) {  // sample every 5s
	  lastSampleTime = HAL_GetTick();
	  BATTERY newState = getBatteryState();

	  if (newState == FULL){
		  printf("Battery full\r\n");
	  }
	  else if (newState == NORMAL){
		  printf("Nominal\r\n");
	  }
	  else{
		  printf("Critical\r\n");
	  }
	}
}

// EEPROM READ & WRITE CODE HERE
void resetEEPROMVariables() {
#ifndef eeprom
	uint32_t val = 1;
	eepromHeader();
	EE_WriteVariable32bits(1, val);
	EE_WriteVariable32bits(2, val);
	EE_WriteVariable32bits(3, val);
	EE_WriteVariable32bits(4, val);
	EE_WriteVariable32bits(5, val);
	EE_WriteVariable32bits(6, val);
	EE_WriteVariable32bits(7, val);
	EE_Status ee_status = EE_WriteVariable32bits(8, val);
	eepromFooter(ee_status);
#else
#endif

}

void updateRecordEE()
{ // this function updates the statistics in EEprom
#ifndef eeprom
	uint32_t record = rememberRecordEEPROM();
	if (record < stepper.record) {
		eepromHeader();
		uint32_t val = stepper.record;
		EE_Status ee_status = EE_WriteVariable32bits(2, val);
		eepromFooter(ee_status);
	}
#else
#endif
}

void resetRecordEE()
{ // this function updates the statistics in EEprom
#ifndef eeprom
	eepromHeader();
	uint32_t val = 0;
	EE_Status ee_status = EE_WriteVariable32bits(2, val);
	eepromFooter(ee_status);
#else
#endif
}

uint32_t rememberRecordEEPROM() {
#ifndef eeprom
	uint32_t record;
	EE_Status ee_status;
	ee_status = EE_ReadVariable32bits(2, &a_VarDataTab[2]);
	record = a_VarDataTab[2];
	myVal = record;
	return record;
#else
#endif
}

uint32_t getWhole(float value) {
	float integer_part = floorf(value); // Get whole number part
	return (uint32_t)integer_part;
}

uint32_t getDecimal(float value) {
	float integer_part = floorf(value); // Get whole number part
	float fractional_part = value - integer_part; // Get fractional part
	return (uint32_t)(fractional_part * FRACTIONAL_SCALE); // Scale fractional part (e.g., 0.456 -> 4560)
}

void readCaliEE(cali3D *caliStruct)
{
#ifndef eeprom
	EE_Status ee_status;
	cali3D recordings;
	uint32_t xSensitivity;
	uint32_t xOffsetDecimal;
	uint32_t yOffsetWhole;
	uint32_t yOffsetDecimal;
	uint32_t zOffsetWhole;
	uint32_t zOffsetDecimal;
	ee_status = EE_ReadVariable32bits(3, &a_VarDataTab[3]); // read this part at this address
	recordings.xOffset = a_VarDataTab[3];
	ee_status |= EE_ReadVariable32bits(4, &a_VarDataTab[4]); // read this part at this address
	recordings.xSensitivity = a_VarDataTab[4];

	if (ee_status != EE_OK) {
		errorState();
	}

	ee_status = EE_ReadVariable32bits(5, &a_VarDataTab[5]); // read this part at this address
	recordings.yOffset = a_VarDataTab[5];
	ee_status |= EE_ReadVariable32bits(6, &a_VarDataTab[6]); // read this part at this address
	recordings.ySensitivity = a_VarDataTab[6];

	if (ee_status != EE_OK) {
		errorState();
	}

	ee_status = EE_ReadVariable32bits(7, &a_VarDataTab[7]); // read this part at this address
	recordings.zOffset = a_VarDataTab[7];
	ee_status |= EE_ReadVariable32bits(8, &a_VarDataTab[8]); // read this part at this address
	recordings.zSensitivity = a_VarDataTab[8];

	if (ee_status != EE_OK) {
		errorState();
	}
	caliStruct->xOffset = recordings.xOffset;
	caliStruct->xSensitivity = recordings.xSensitivity;
	caliStruct->yOffset = recordings.yOffset;
	caliStruct->ySensitivity = recordings.ySensitivity;
	caliStruct->zOffset = recordings.zOffset;
	caliStruct->zSensitivity = recordings.zSensitivity;
	//return recordings;
#else
#endif
}

void writeCaliEE(float* xOffset, float* yOffset, float* zOffset, float* xSensitivity, float* ySensitivity, float* zSensitivity)
{
#ifndef eeprom
	eepromHeader();
	EE_Status ee_status;
	uint32_t xOffsetWhole = getWhole(*xOffset);
	uint32_t xSensitivityWhole = getWhole(*xSensitivity);
	uint32_t yOffsetWhole = getWhole(*yOffset);
	uint32_t ySensitivityWhole = getWhole(*ySensitivity);
	uint32_t zOffsetWhole = getWhole(*zOffset);
	uint32_t zSensitivityWhole = getWhole(*zSensitivity);

	ee_status = EE_WriteVariable32bits(3, xOffsetWhole);
	if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 1;ee_status|= EE_CleanUp_IT();}
	if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {errorState();}

	ee_status = EE_WriteVariable32bits(4, xSensitivityWhole);
	if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 1;ee_status|= EE_CleanUp_IT();}
	if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {errorState();}

	ee_status = EE_WriteVariable32bits(5, yOffsetWhole);
	if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 1;ee_status|= EE_CleanUp_IT();}
	if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {errorState();}

	ee_status = EE_WriteVariable32bits(6, ySensitivityWhole);
	if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 1;ee_status|= EE_CleanUp_IT();}
	if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {errorState();}

	ee_status = EE_WriteVariable32bits(7, zOffsetWhole);
	if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 1;ee_status|= EE_CleanUp_IT();}
	if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {errorState();}

	ee_status = EE_WriteVariable32bits(8, zSensitivityWhole);
	if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 1;ee_status|= EE_CleanUp_IT();}
	if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {errorState();}

	eepromFooter(ee_status);
#else
#endif
}

void eepromHeader() {
  if (HAL_FLASH_Unlock() != HAL_OK) errorState();  // Unlock only
  while (ErasingOnGoing == 1) {}  // Wait for ongoing cleanup
}

void eepromFooter(EE_Status ee_status) {
	if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {
	        ErasingOnGoing = 1;
	        ee_status |= EE_CleanUp_IT();
	    }
	if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {
		errorState();
	}
	HAL_FLASH_Lock();
}

void errorState() {
	HAL_FLASH_Lock();
	while (1) {
		stepper.screen = SYS_ERROR;
		manageDisp();
	}
}

