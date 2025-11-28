#include <stdint.h>


#include "Globals.h"
#include "Sensors.h"

File logFile;


float RunDuration = 3*1/frequency;     // Controller active 5 seconds
float RestDuration = 10.0f;    // Rest 3 seconds


uint32_t RunEndTime = 0;
uint32_t RestEndTime = 0;

uint32_t Iteration = 0;
uint32_t FrameCounter = 0;