#include <Arduino.h>

#include "Sensors.h"
#include "Constants.h"
#include "Globals.h"
#include "Controller.h"

float EncoderAngle(){
  // Read encoder 
  int sensorValue = analogRead(AngPin);
  // Calculate the angle
  float readAngle = -0.149f*sensorValue + 188.66f+5.0f;
  // float readAngle = sensorValue; // Change to this line for calibration
  return readAngle;
}

float ESCONCurrent(){
  // Read data
  int EsconOut = analogRead(EsconOutPin);
  // Calculate Voltage
  float voltage = (EsconOut/4095.0f) * 3.3f;
  //  Calculate Current
  float Current = (voltage - 1.65f) / 1.65f * 5.00f;
  return Current;
}