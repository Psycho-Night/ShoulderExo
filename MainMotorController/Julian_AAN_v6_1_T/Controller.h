#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "Globals.h"
#include "Sensors.h"
#include "Comunication.h"
float signum();
// float EncoderAngle();
// float 
void SaveFrame(Frame &frame);
bool FetchFrame(Frame &outFrame, uint32_t index);
void RestStep();
void AAN();
void CloseLogFile();

#endif