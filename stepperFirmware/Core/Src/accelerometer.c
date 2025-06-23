#include "accelerometer.h"
#include "main.h" // this contains the GPIO pin definitions found within each project
#include "buttons.h"
#include "leds.h"
#include "screens.h"

extern float sensZ;
extern float pos;
extern float neg;

// ---- Module Variables ----
extern ADC_HandleTypeDef hadc1; // Declare if needed
static ADC_HandleTypeDef *hadc = &hadc1;


//extern float offsetX = 0;
//extern float offsetY = 0;
//extern float offsetZ = 0;

 static StepDetector_t step_detector = {
    .weights = STEP_WEIGHTS,
	.weight_sum = 1.0f
 };

 uint32_t stepTimestamps[MAX_STEPS] = {0};
 int stepIndex = 0;

 extern struct stepperStruct stepper; // reference to stepper struct

 extern uint32_t lastStepTime = 0;


 // For pace (interval) averaging:
 static uint32_t paceIntervals[MAX_PACES] = {0};
 static int      paceIntervalIndex = 0;
 uint32_t minimumWait;


 void initialiseStepDetector(void) {
	uint32_t now = HAL_GetTick();
	step_detector.last_step_time = 0;
	step_detector.oldStep = 0;
	step_detector.newStep = 0;
	minimumWait = now + MIN_STARTUP_WAIT;
 }

 // check if the sensor is present and
 int selfTestHarrsha(void) {

	uint16_t baselineX = readADCharrsha(Xchannel);  // X axis
	uint16_t baselineY = readADCharrsha(Ychannel);  // Y axis
	uint16_t baselineZ = readADCharrsha(Zchannel);  // Z axis
    HAL_GPIO_WritePin(ST_GPIO_Port, ST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
	uint16_t stX = readADCharrsha(Xchannel);  // X axis
	uint16_t stY = readADCharrsha(Ychannel);  // Y axis
	uint16_t stZ = readADCharrsha(Zchannel);  // Z axis
    HAL_GPIO_WritePin(ST_GPIO_Port, ST_Pin, GPIO_PIN_RESET);
    float dx = stX - baselineX;
    float dy = stY - baselineY;
    float dz = stZ - baselineZ;

//	bool x_ok = (dx < 0);
//	bool y_ok = (dy >  0);
//	bool z_ok = (dz >  0);

    // BELOW IS FOR DEBUGGING ONLY
    // I believe our adxl is not wired to the mcu correctly
    // this is not an issue, it just means that our xyz is wired in the order of zyx pretty sure. can change this later if needed
	bool x_ok = (dx > 0);
	bool y_ok = (dy >  0);
	bool z_ok = (dz <  0);

    return (x_ok && y_ok && z_ok) ? 0 : 1;
 }

 // this is when a step is detected
 void stepIdentified(void)
 {
     uint32_t now = HAL_GetTick();

     // Shift steps for interval checking
     step_detector.oldStep = step_detector.newStep;
     step_detector.newStep = now; // this should be the current timestamp

     // Store timestamp for spm (optional)
     stepTimestamps[stepIndex] = now;
     stepIndex = (stepIndex + 1) % MAX_STEPS;
     stepper.steps++;
     stepper.record++;
     stepper.hour++;

     step_detector.last_step_time = now;
 }


 int getStepsPerMinute(void) {
     uint32_t current_time = HAL_GetTick();
     int steps = 0;

     for (int i = 0; i < MAX_STEPS; i++) {
         uint32_t ts = stepTimestamps[i];

         // Ignore uninitialized timestamps (assuming 0 means unused)
         if (ts == 0) continue;

         if ((current_time - ts) <= ONE_MINUTE_MS) {
             // finds valid timestamps within the last minute
             // and counts them as steps
             steps++;
         }
     }
     return steps;
 }

 // observe detected steps and determine pace
 void paceDetection(void) {
     uint32_t now = HAL_GetTick();

     // only proceed if we have valid timestamps
     if ((step_detector.newStep != 0) && (step_detector.oldStep != 0) && (now > minimumWait)) {
         uint32_t interval = step_detector.newStep - step_detector.oldStep;

         if (interval <= MIN_RUNNING_INTERVAL_MS) {
             stepper.pace = RUNNING;
             printf("Pace: RUNNING\n");
         } else if (interval <= MIN_WALKING_INTERVAL_MS) {
             stepper.pace = WALKING;
             printf("Pace: WALKING\n");
         }

         if ((now - step_detector.newStep) > MIN_IDLE_INTERVAL_MS) {
             stepper.pace = IDLE;
             printf("Pace: IDLE (timeout)\n");
         }
     }

     lastStepTime = now;
 }



//-- OLD (but def works) --//
// static bool wasBelow = true;
// void observeSensor(void) {
//     float w   = convert_adc_to_gharrsha(readADCharrsha(Zchannel), offsetZ, sensitivityZ);
//     sensZ = w;
//     uint32_t now = HAL_GetTick();
//     bool above   = (w >= MIN_ACCELERATION) && (w <= MAX_ACCELERATION);
//     bool canFire = (now - step_detector.last_step_time) >= MIN_STEP_INTERVAL;
//     bool willStep = canFire && wasBelow && above;
//
//     if (willStep) {
//         step_detector.last_step_time = now;
//         stepper.lastStepTime = now;
//         stepIdentified();
//     }
//     wasBelow = !above;
// }

#define MIN_ACCELERATION   1.3f   // g
#define MAX_ACCELERATION   3.5f   // g
#define MIN_STEP_INTERVAL  0.03f  // seconds
#define MAX_STEP_INTERVAL  0.25f  // seconds
#define MIN_STEP_GAP_MS    250    // debounce in ms

uint32_t accelTime = 0;        // ms
uint32_t lastAccelTime = 0;    // ms
bool validAccelDetected = false;

void observeSensor(void) {
                                                         float xAccel = convert_adc_to_gharrsha(readADCharrsha(Xchannel), offsetX, sensitivityX);
    float yAccel = convert_adc_to_gharrsha(readADCharrsha(Ychannel), offsetY, sensitivityY);
    float zAccel = convert_adc_to_gharrsha(readADCharrsha(Zchannel), offsetZ, sensitivityZ);
    float accelMagnitude = sqrtf(xAccel*xAccel + yAccel*yAccel + zAccel*zAccel);

    bool withinMin = (accelMagnitude >= MIN_ACCELERATION);
    bool withinMax = (accelMagnitude <= MAX_ACCELERATION);
    bool inValidRange = withinMin && withinMax;

    uint32_t now = HAL_GetTick();

    if (inValidRange) {
        if (!validAccelDetected) {
            lastAccelTime = now;
            accelTime = 0;
        } else {
            accelTime += (now - lastAccelTime);
            lastAccelTime = now;
        }
        validAccelDetected = true;
    } else if (validAccelDetected) {
        // We just exited the valid acceleration window
        if (accelTime >= MIN_STEP_INTERVAL * 1000 && accelTime <= MAX_STEP_INTERVAL * 1000) {
            if (now - stepper.lastStepTime > MIN_STEP_GAP_MS) {
                stepIdentified();
                stepper.lastStepTime = now;
            }
        }
        // Reset state
        validAccelDetected = false;
        accelTime = 0;
    }
}


//-- NEW -- //
 // Step detection thresholds
// #define IMPACT_MIN_THRESHOLD    1.3f
// #define IMPACT_MAX_THRESHOLD    3.5f
// #define MIN_STEP_INTERVAL_SEC   0.03f
// #define MAX_STEP_INTERVAL_SEC   0.25f
// #define MS_TO_SECONDS_FACTOR    1000.0f
//
// typedef struct {
//     float currentZ;
//     float impact;
//     float stepTimeSeconds;
//     bool impactInRange;
//     bool timeInRange;
//     bool aboveThreshold;
// } StepAnalysis;
//
// /**
//  * Reads and converts Z-axis accelerometer data
//  * @return Converted Z-axis acceleration value
//  */
// static float readZAcceleration(void) {
//     return convert_adc_to_gharrsha(
//         readADCharrsha(Zchannel),
//         offsetZ,
//         sensitivityZ
//     );
// }
//
// /**
//  * Calculates time elapsed since last step in seconds
//  * @param now Current timestamp in milliseconds
//  * @return Time elapsed in seconds
//  */
// static float getStepIntervalSeconds(uint32_t now) {
//     uint32_t intervalMs = now - step_detector.last_step_time;
//     return intervalMs / MS_TO_SECONDS_FACTOR;
// }
//
// /**
//  * Analyzes current sensor reading for step detection
//  * @param currentZ Current Z-axis acceleration
//  * @param stepTimeSeconds Time since last step
//  * @return Analysis results structure
//  */
// static StepAnalysis analyzeStepConditions(float currentZ, float stepTimeSeconds) {
//     StepAnalysis analysis = {0};
//
//     analysis.currentZ = currentZ;
//     analysis.impact = fabsf(currentZ);
//     analysis.stepTimeSeconds = stepTimeSeconds;
//
//     analysis.impactInRange = (analysis.impact >= IMPACT_MIN_THRESHOLD) &&
//                             (analysis.impact <= IMPACT_MAX_THRESHOLD);
//
//     analysis.timeInRange = (stepTimeSeconds >= MIN_STEP_INTERVAL_SEC) &&
//                           (stepTimeSeconds <= MAX_STEP_INTERVAL_SEC);
//
//     analysis.aboveThreshold = (currentZ >= MIN_ACCELERATION) &&
//                              (currentZ <= MAX_ACCELERATION);
//
//     return analysis;
// }
//
// /**
//  * Records a detected step and updates timestamps
//  * @param now Current timestamp
//  */
// static void recordStep(uint32_t now) {
//     step_detector.last_step_time = now;
//     stepper.lastStepTime = now;
//     stepIdentified();
// }
//
// /**
//  * Main step detection function - analyzes accelerometer data to detect steps
//  * Uses state machine approach: detects transition from below to above threshold
//  * with additional validation for impact magnitude and timing constraints
//  */
// void observeSensor(void) {
//     static bool wasBelowThreshold = true;
//
//     // Read current sensor data
//     float currentZ = readZAcceleration();
//
//     uint32_t currentTime = HAL_GetTick();
//     float stepInterval = getStepIntervalSeconds(currentTime);
//
//     // Analyze current conditions
//     StepAnalysis analysis = analyzeStepConditions(currentZ, stepInterval);
//
//     // Step detection logic: transition from below to above threshold
//     // with additional validation criteria
//     bool stepDetected = wasBelowThreshold &&
//                        analysis.aboveThreshold &&
//                        analysis.impactInRange &&
//                        analysis.timeInRange;
//
//     if (stepDetected) {
//         recordStep(currentTime);
//     }
//
//     // Update state for next iteration
//     wasBelowThreshold = !analysis.aboveThreshold;
// }


 void calibrateAllAxes(void) {
	stepper.screen = CALIBRATION;
	manageDisp();
	HAL_Delay(500);

	stepper.screen = CALIBRATE_Y_UP;
	manageDisp();
	calibrateAxis(Ychannel, &offsetY, &sensitivityY, "Y");

	stepper.screen = CALIBRATE_Z_UP;
	manageDisp();
	calibrateAxis(Zchannel,  &offsetZ, &sensitivityZ, "Z");

	stepper.screen = CALIBRATE_X_UP;
	manageDisp();
	calibrateAxis(Xchannel, &offsetX, &sensitivityX, "X");

	writeCaliEE(&offsetX, &offsetY, &offsetZ, &sensitivityX, &sensitivityY, &sensitivityZ);

	stepper.screen = HOMESCREEN;
	manageDisp();
 }

 int takeSample(const char* axisName){
	    static uint32_t xStartTime = 0;
	    static uint32_t yStartTime = 0;
	    static uint32_t zStartTime = 0;


	    if (axisName == "X") {

	        int diffXY = readADCharrsha(Xchannel) - readADCharrsha(Ychannel);
	        int diffYX = readADCharrsha(Ychannel) - readADCharrsha(Xchannel);

	        if (diffXY > 350) {
	            if (xStartTime == 0) xStartTime = HAL_GetTick();
	            if (HAL_GetTick() - xStartTime >= 3000)  {
				    stepper.screen = CALIBRATE_X_DOWN;
				    manageDisp();
	            	return 1;
	            }
	        } else if (diffYX > 300) {
	            if (xStartTime == 0) xStartTime = HAL_GetTick();
	            if (HAL_GetTick() - xStartTime >= 3000) {
	            	return 2;
	            }
	        } else {
	            xStartTime = 0; // Reset if condition breaks
	        }
	    }

	    else if (axisName == "Y") {
//		    stepper.screen = CALIBRATE_Y_UP;
//		    manageDisp();
	        int diffYZ = readADCharrsha(Ychannel) - readADCharrsha(Zchannel);
	        int diffZY = readADCharrsha(Zchannel) - readADCharrsha(Ychannel);

	        if (diffYZ > 300) {
	            if (yStartTime == 0) yStartTime = HAL_GetTick();
	            if (HAL_GetTick() - yStartTime >= 3000) {
				    stepper.screen = CALIBRATE_Y_DOWN;
				    manageDisp();
	            	return 1;
	            }
	        } else if (diffZY > 300) {
	            if (yStartTime == 0) yStartTime = HAL_GetTick();
	            if (HAL_GetTick() - yStartTime >= 3000) return 2;

	        } else {
	            yStartTime = 0;
	        }
	    }

	    else if (axisName == "Z") {
//		    stepper.screen = CALIBRATE_Z_UP;
//		    manageDisp();
	        int diffZX = readADCharrsha(Zchannel) - readADCharrsha(Xchannel);
	        int diffXZ = readADCharrsha(Xchannel) - readADCharrsha(Zchannel);

	        if (diffZX > 300) {
	            if (zStartTime == 0) zStartTime = HAL_GetTick();
	            if (HAL_GetTick() - zStartTime >= 3000) {
	    		    stepper.screen = CALIBRATE_Z_DOWN;
	    		    manageDisp();
	            	return 1;
	            }
	        } else if (diffXZ > 300) {
	            if (zStartTime == 0) zStartTime = HAL_GetTick();
	            if (HAL_GetTick() - zStartTime >= 3000) return 2;
	        } else {
	            zStartTime = 0;
	        }
	    }

	    return 0; // Default case: no condition met or not yet 3s
 }

 void calibrateAxis(uint32_t channel, float* offset, float* sensitivity, const char* axisName) {
     uint16_t tempP = 0, tempN = 0;
     int calibStage = 0;

     while (calibStage < 3) {
         uint16_t val = readADCharrsha(channel);  // Live read

         if (calibStage == 0) {
             printf("Click button for positive %s: %u\r\n", axisName, val);
             if (takeSample(axisName)==1) {
                 tempP = readADCharrsha(channel);  // +axis
                 pos = tempP;
                 calibStage++;
                 fastDoubleBlink();
             }
         }
         else if (calibStage == 1) {
             printf("Click button for negative %s: %u\r\n", axisName, val);
             if (takeSample(axisName)==2) {
                 tempN = readADCharrsha(channel);  // -axis
                 neg = tempN;
                 calibStage++;
                 fastDoubleBlink();
             }
         }
         else if (calibStage == 2) {
             *sensitivity = (tempP - tempN) / 2.0f;
             *offset = (tempP + tempN) / 2.0f;
             printf("+%s: %u     -%s: %u\r\n", axisName, tempP, axisName, tempN);
             calibStage++;
//             doubleBlink();
             calibStage++;
         }
     }
 }


 uint16_t readADCharrsha(uint32_t channel) {
     ADC_ChannelConfTypeDef sConfig = {0};
     HAL_ADC_Stop(&hadc1);
     sConfig.Channel = channel;
     sConfig.Rank = ADC_REGULAR_RANK_1;
     sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

     HAL_ADC_ConfigChannel(&hadc1, &sConfig);
     HAL_ADC_Start(&hadc1);
     HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
     uint16_t value = HAL_ADC_GetValue(&hadc1);
     HAL_ADC_Stop(&hadc1);
     return value;
 }

 float convert_adc_to_gharrsha(uint32_t adc_value, float offset, float sensitivity) {
     return (adc_value - offset) / sensitivity;
 }


 // Process raw accelerometer data
 static AccelData_t process_accelerometer_data(const uint32_t raw_values[]) {
     return (AccelData_t){
         .x = convert_adc_to_gharrsha(raw_values[0], offsetX, sensitivityX),
         .y = convert_adc_to_gharrsha(raw_values[1], offsetY, sensitivityY),
         .z = convert_adc_to_gharrsha(raw_values[2], offsetZ, sensitivityZ)
     };
 }

 // Calculate weighted average of acceleration
 static float computeMagnitude(const AccelData_t *data, const float weights[]) {
     return (data->x * weights[0] + data->y * weights[1] + data->z * weights[2])
            / step_detector.weight_sum;
 }


