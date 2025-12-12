#include "core_pins.h"
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

#include "Comunication.h"
#include "Constants.h"
#include "Globals.h"
#include "Sensors.h"
#include "Controller.h"

void ProccesCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("SET")){
    if (!controllerActive) {
      if (sscanf(cmd.c_str(), "SET %f %f %f %f %f %f", &frequency, &amplitude, &offset, &phaseShift, &alpha, &a) == 6) {
      Serial.println("PARAMS UPDATED: ");
      Serial.print("Frequency ");
      Serial.print(frequency);
      Serial.print(", ");
      Serial.print("Amplitude ");
      Serial.print(amplitude);
      Serial.print(", ");
      Serial.print("Offset ");
      Serial.print(offset);
      Serial.print(", ");
      Serial.print("Phase Shift ");
      Serial.print(phaseShift);
      Serial.print(", ");
      Serial.print("Alpha ");
      Serial.print(alpha);
      Serial.print(", ");
      Serial.print("a ");
      Serial.println(a);

      // State = 2;
      } else {
        Serial.println("UPDATE FAILED");
      }
      
    }
    else {
      Serial.println("STOP THE MOTOR BEFORE SET");
    }     
  } else if (cmd.startsWith("START")){
    digitalWrite(MotorEnablePin, HIGH);
    controllerActive = true;
    Iteration = 1;
    CloseLogFile();
    State = 1;
    FrameCounter = 0;

    previousTime = micros();
    T_ff = 0;
    T_ff_prev = 0;
    error = 0;
    error_prev = 0;
    vel_error = 0;
    currentAngle = EncoderAngle();
    currentAngleRad = currentAngle * deg2rad;
    tickCount = 0;
    tickFlag = false;
    StartTime = micros();
    RunEndTime = StartTime + (uint32_t)(RunDuration * 1e6);

    char filename[32];
    sprintf(filename, "AAN_%04d.bin", Iteration);
    logFile = SD.open(filename, FILE_WRITE);

    if (!logFile) {
      Serial.println("SD OPEN FAIL");
      State = 0;
      return;

    } else {
      Serial.print("LOG FILE: ");
      Serial.println(filename);
    }


    

    Serial.println("START OK");
  } else if (cmd.startsWith("STOP")) {
    controllerActive = false;
    digitalWrite(MotorEnablePin, LOW);
    analogWrite(MotorTorquePin,0);
    if (logFile) logFile.close();
    State = 0;

    Serial.println("STOP OK");
  }
}
// Data frame packed ensures exactly 40bytes of data per frame
struct __attribute__((packed)) Frame {              
  uint16_t header;                // Always 0xAA55
  uint32_t t;                     // timestamp [microseconds]
  float targetA;                  // Target angle [deg]
  float currentA;                 // Current angle [deg]
  // float torque;                   // Torque [Nm]
  float t_ff;                     // T_FF [Nm]
  float t_fb;                     // T_FB [Nm]
  // float desiredI;                 // Desired output [A]
  float actualI;                  // Received input [A]
  float freq;                     // System Frequency [Hz]
  uint16_t padding;                // Always 0xBB66
};

// Send frame Sending frame to PC for GUI

void SendFrame(struct Frame &frame) {
  Serial.write((uint8_t *)&frame, sizeof(frame));
}

