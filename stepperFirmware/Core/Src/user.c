#include "user.h"

// 1 - UP
// 0 - ENTER
// 2 - DOWN
// 3 - BACK

void inputActioner(){
    if (stepper.pace == IDLE){
        if (stepper.screen == HOMESCREEN && stepper.showButtons && isButtonPressed(2)){
        	// go to reset screen where stats can be selectively reset
            stepper.screen = SYS_RESET;
        } else if (stepper.screen == HOMESCREEN && stepper.showButtons && isButtonPressed(1)) {
        	// calibrate
        	stepper.screen = CALIBRATION;
        	calibrateAllAxes();
        } else if (stepper.screen == SYS_ERROR && isButtonPressed(3)) {
        	// retry due to error
        	stepper.screen = HOMESCREEN;
        } else if (stepper.screen == SYS_RESET && isButtonPressed(1)) {
        	// reset all time steps
        	stepper.screen = HOMESCREEN;
        	stepper.record = 0;
        	resetRecordEE();
        } else if (stepper.screen == SYS_RESET && isButtonPressed(2)) {
        	// reset current steps? Or hour steps if we get that working
        	stepper.screen = HOMESCREEN;
        	stepper.hour - 0;
        } else if (stepper.screen == SYS_RESET && isButtonPressed(3)) {
        	// Go back
        	stepper.screen = HOMESCREEN;
        }

        uint8_t enterButton = HAL_GPIO_ReadPin(ENT_BUTT_GPIO_Port, ENT_BUTT_Pin);
        uint8_t upButton = HAL_GPIO_ReadPin(UP_BUTT_GPIO_Port, UP_BUTT_Pin);
        uint8_t backButton = HAL_GPIO_ReadPin(BCK_BUTT_GPIO_Port, BCK_BUTT_Pin);
        uint8_t downButton = HAL_GPIO_ReadPin(DWN_BUTT_GPIO_Port, DWN_BUTT_Pin);

		if ((enterButton == GPIO_PIN_SET) || (upButton == GPIO_PIN_SET) || (backButton == GPIO_PIN_SET) || (downButton == GPIO_PIN_SET))
		{
			stepper.lastButtonPress = HAL_GetTick(); // Update the last button press time
		}

    }
}
