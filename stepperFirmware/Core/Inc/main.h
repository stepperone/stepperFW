/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "accelerometer.h"
#include "buttons.h"
#include "helperFunc.h"
#include "leds.h"
#include "user.h"
#include "DEV_Config.h"
#include "screens.h"
#include "eeprom_emul.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BATT_REG_Pin GPIO_PIN_0
#define BATT_REG_GPIO_Port GPIOA
#define IDLE_LED_Pin GPIO_PIN_4
#define IDLE_LED_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define DIN_Pin GPIO_PIN_7
#define DIN_GPIO_Port GPIOA
#define Z_IN_Pin GPIO_PIN_1
#define Z_IN_GPIO_Port GPIOB
#define Y_IN_Pin GPIO_PIN_2
#define Y_IN_GPIO_Port GPIOB
#define X_IN_Pin GPIO_PIN_10
#define X_IN_GPIO_Port GPIOB
#define SPI_CS_Pin GPIO_PIN_13
#define SPI_CS_GPIO_Port GPIOB
#define DC_Pin GPIO_PIN_14
#define DC_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_15
#define RST_GPIO_Port GPIOB
#define BUSY_Pin GPIO_PIN_8
#define BUSY_GPIO_Port GPIOA
#define ST_Pin GPIO_PIN_9
#define ST_GPIO_Port GPIOA
#define WALK_LED_Pin GPIO_PIN_6
#define WALK_LED_GPIO_Port GPIOC
#define RUN_LED_Pin GPIO_PIN_7
#define RUN_LED_GPIO_Port GPIOC
#define BCK_BUTT_Pin GPIO_PIN_9
#define BCK_BUTT_GPIO_Port GPIOD
#define DWN_BUTT_Pin GPIO_PIN_10
#define DWN_BUTT_GPIO_Port GPIOA
#define ENT_BUTT_Pin GPIO_PIN_0
#define ENT_BUTT_GPIO_Port GPIOD
#define UP_BUTT_Pin GPIO_PIN_1
#define UP_BUTT_GPIO_Port GPIOD
#define E2_Pin GPIO_PIN_4
#define E2_GPIO_Port GPIOB
#define E1_Pin GPIO_PIN_5
#define E1_GPIO_Port GPIOB
#define E0_Pin GPIO_PIN_6
#define E0_GPIO_Port GPIOB
#define PWR_Pin GPIO_PIN_7
#define PWR_GPIO_Port GPIOB
#define WC_Pin GPIO_PIN_10
#define WC_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define Xchannel ADC_CHANNEL_11
#define Ychannel ADC_CHANNEL_10
#define Zchannel ADC_CHANNEL_9
#define BATTMONchannel ADC_CHANNEL_0

#define eeprom // for i2c eeprom
// states the system can be in (mainly for the display)
typedef enum
{
	PWR_OFF,     		// 0
    HOMESCREEN,  		// 1
    SYS_ERROR,   		// 2
    CALIBRATION, 		// 3
	CALIBRATE_X_UP,		// 4
	CALIBRATE_X_DOWN,	// 5
	CALIBRATE_Y_UP,		// 6
	CALIBRATE_Y_DOWN,	// 7
	CALIBRATE_Z_UP,		// 8
	CALIBRATE_Z_DOWN,	// 9
    SYS_RESET   		// 10
} SCREEN;

// Battery
typedef enum
{
	CRITICAL,
	NORMAL,
	FULL
} BATTERY;


// identified paces of the user
typedef enum
{
    IDLE,    // 0
    WALKING, // 1
    RUNNING  // 2
} PACE;

typedef struct
{
	float xOffset;
	float yOffset;
	float zOffset;
	float xSensitivity;
	float ySensitivity;
	float zSensitivity;
} cali3D;

struct stepperStruct
{
    int steps;     // number of steps since device was powered on
    int hour;      // number of steps within the last hour
    int record;    // max number of steps in any session device has been on
    PACE pace;     // current pace detected of the user
    SCREEN screen; // what screen should be displayed
    uint32_t lastStepTime;

    // system variables
    int showButtons;               // show the button labels on the display (only accept button input on homescreen if labels are showing)
    unsigned long lastButtonPress; // timestamp a button was last pressed
};

extern struct stepperStruct stepper; // global variable to hold stepper information
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
