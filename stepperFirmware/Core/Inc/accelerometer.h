#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdint.h>
#include <stdbool.h>


// ---- Constants ----
#define ACCEL_AXES          3
#define ADC_REF_VOLTAGE     3.3f
#define ADC_MAX_VALUE       4095.0f  // For 12-bit ADC
#define ACCEL_RANGE         2.0f     // ±2g
#define MIN_STEP_INTERVAL   250      // ms
//#define MIN_STEP_INTERVAL   50     // ms

//#define MIN_ACCELERATION    0.3f   // g
#define MIN_ACCELERATION 	1.3f     // g
#define MAX_ACCELERATION    3.5f     // g
#define STEP_WEIGHTS        {0.0f, 0.0f, 1.0f}  // X, Y, Z weights

#define MAX_STEPS   		5
#define ONE_MINUTE_MS       60000
#define minRunningPace 		2.25
#define minWalkingPace 		1

#define MIN_RUNNING_INTERVAL_MS   500    // 120 spm
#define MIN_WALKING_INTERVAL_MS   800    // 75 spm
#define MIN_IDLE_INTERVAL_MS	  1000   // No step taken within the last second

#define MAX_PACES 			5
#define MIN_STARTUP_WAIT   1000

// ---- Type Definitions ----
typedef struct {
   float x;
   float y;
   float z;
} AccelData_t;

typedef struct {
   uint32_t last_step_time;
   uint32_t newStep;
   uint32_t oldStep;
   float weights[ACCEL_AXES];
   float weight_sum;
} StepDetector_t;


// ---- Function Prototypes ----
int selfTestHarrsha(void);
void observeSensor(void);
void stepIdentified(void);
void paceDetection(void);
uint16_t readADCharrsha(uint32_t channel);
float convert_adc_to_gharrsha(uint32_t adc_value, float offset, float sensitivity);
int getStepsPerMinute(void);
void testADC(void);
void testShakeZ(void);
static AccelData_t process_accelerometer_data(const uint32_t raw_values[]);
static float getWeightedAccel(void);
void testG(void);
void calibrateAllAxes(void);
void calibrateAxis(uint32_t channel, float* offset, float* sensitivity, const char* axisName);
void initialiseStepDetector(void);
// ---- External Global Variables ----
extern uint32_t stepTimestamps[MAX_STEPS];
extern int stepIndex;
extern float sensitivityX, sensitivityY, sensitivityZ, offsetX, offsetY, offsetZ;
extern uint32_t lastStepTime;
extern uint32_t minimumWait;

#endif // ACCELEROMETER_H
