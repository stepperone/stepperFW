#include "leds.h"
#include "main.h"         // this contains the GPIO pin definitions found within each project

extern struct stepperStruct stepper; // reference to stepper struct

void ledManager()
{
    static uint32_t lastBlink = 0; // stores last toggle time for blinking

    // If system is in error state, blink the WALK LED
    if (stepper.screen == SYS_ERROR)
    {
        if (HAL_GetTick() - lastBlink > 250)
        {
            HAL_GPIO_TogglePin(WALK_LED_GPIO_Port, WALK_LED_Pin);
            lastBlink = HAL_GetTick();
        }
        // Turn off other LEDs during error state
        HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
        return; // Skip the rest
    }

    // If not in error, show one active LED based on current pace
    switch (stepper.pace)
    {
    case RUNNING:
        HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
        break;
    case WALKING:
        HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
        break;
    case IDLE:
    default:
        HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
        break;
    }
}

void singleBlink(void) {
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
}

void doubleBlink(void) {
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
}

void fastDoubleBlink(void) {
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
}

void tripleBlink(void) {
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(166);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
    HAL_Delay(166);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(166);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(166);
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WALK_LED_GPIO_Port, WALK_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, GPIO_PIN_RESET);
}
