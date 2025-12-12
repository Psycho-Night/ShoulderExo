#include <stdint.h>


#include "Globals.h"
#include "Sensors.h"

File logFile;


float RunDuration = 3*1/frequency;      // Controller active 3*period of the sin wave
float RestDuration = 10.0f;             // Rest 10 seconds before moving to nex iteration


uint32_t RunEndTime = 0;
uint32_t RestEndTime = 0;

uint32_t Iteration = 0;
uint32_t FrameCounter = 0;
int SaveCounter = 0;
int PrevFrameCount = 0;