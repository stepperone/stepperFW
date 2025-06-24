#include "buttons.h"
#include "main.h"   // this contains the GPIO pin definitions found within each project

int isButtonPressed(int butt)
{
	static uint32_t last_time[4] = {0, 0, 0, 0}; // Separate timers per button
	static uint8_t last_state[4] = {GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET};
	static uint8_t buttonWasPressed[4] = {0, 0, 0, 0};
	uint8_t current_state = GPIO_PIN_RESET;
	int hasButtonPressed = 0;

	if (HAL_GPIO_ReadPin(ENT_BUTT_GPIO_Port, ENT_BUTT_Pin) || HAL_GPIO_ReadPin(UP_BUTT_GPIO_Port, UP_BUTT_Pin) || HAL_GPIO_ReadPin(BCK_BUTT_GPIO_Port, BCK_BUTT_Pin) || HAL_GPIO_ReadPin(DWN_BUTT_GPIO_Port, DWN_BUTT_Pin))
	{
		stepper.lastButtonPress = HAL_GetTick(); // Update the last button press time regardless of which button is pressed
	}

	// Read the correct button
	if (butt == 0) {
		current_state = HAL_GPIO_ReadPin(ENT_BUTT_GPIO_Port, ENT_BUTT_Pin);
	}
	else if (butt == 1) {
		current_state = HAL_GPIO_ReadPin(UP_BUTT_GPIO_Port, UP_BUTT_Pin);
	}
	else if (butt==2){
		current_state = HAL_GPIO_ReadPin(BCK_BUTT_GPIO_Port, BCK_BUTT_Pin);
	}
	else if(butt==3){
		current_state = HAL_GPIO_ReadPin(DWN_BUTT_GPIO_Port, DWN_BUTT_Pin);
	}

	// Debounce logic for this specific button
	if (current_state != last_state[butt]) {
		last_time[butt] = HAL_GetTick();
	}

	if ((HAL_GetTick() - last_time[butt]) > 25)
	{
		if (current_state == GPIO_PIN_SET && !buttonWasPressed[butt]) {
			buttonWasPressed[butt] = 1;
			hasButtonPressed = 1;
			stepper.lastButtonPress = HAL_GetTick(); // Update the last button press time
		}
		else if (current_state == GPIO_PIN_RESET) {
			buttonWasPressed[butt] = 0;
		}
	}

	last_state[butt] = current_state;
	return hasButtonPressed;
}

void testButtons(void)
{
	int buttonsPressed = 0;
	// set all LEDs on the board to on
	HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);

	int ent = 0;
	int walk = 0;
	int idle = 0;
	int hb = 0;

	// Test each button, Once all buttons are pressed, this test will be complete
	while (buttonsPressed < 4) {
		if ((HAL_GPIO_ReadPin(ENT_BUTT_GPIO_Port, ENT_BUTT_Pin) == GPIO_PIN_SET) && (ent == 0)) {
			HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
			buttonsPressed++;
			ent = 1;
			// HAL_Delay(50); // simple 50ms delay after any button press
		}
		if ((HAL_GPIO_ReadPin(UP_BUTT_GPIO_Port, UP_BUTT_Pin) == GPIO_PIN_SET) && (walk == 0)) {
			HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
			buttonsPressed++;
			walk = 1;
			// HAL_Delay(50); // simple 50ms delay after any button press
		}
		if ((HAL_GPIO_ReadPin(BCK_BUTT_GPIO_Port, BCK_BUTT_Pin) == GPIO_PIN_SET) && (idle == 0)) {
			HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
			buttonsPressed++;
			idle = 1;
			// HAL_Delay(50); // simple 50ms delay after any button press
		}
		if ((HAL_GPIO_ReadPin(DWN_BUTT_GPIO_Port, DWN_BUTT_Pin) == GPIO_PIN_SET) && (hb == 0)) {
			HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
			buttonsPressed++;
			hb = 1;
			// HAL_Delay(50); // simple 50ms delay after any button press
		}
	}

}

void findLEDS(void) {
    // Turn all LEDs on initially
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);

    int led = 0;

    while (led < 1) {
        if (HAL_GPIO_ReadPin(ENT_BUTT_GPIO_Port, ENT_BUTT_Pin) == GPIO_PIN_SET) {
            switch (led) {
                case 0:
                    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
                    break;
                case 1:
                    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
                    break;
                case 2:
                    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
                    break;
                case 3:
        			HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
        			HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
        			HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);
        			HAL_Delay(500);
        			HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
        			HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
        			HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
        			break;
            }
            led++;
            HAL_Delay(300);  // Debounce delay
        }
    }
}


