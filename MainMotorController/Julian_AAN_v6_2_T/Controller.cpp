#include <stddef.h>
#include "FS.h"
#include "avr/pgmspace.h"
#include <stdint.h>
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

#include "Comunication.h"
#include "Constants.h"
#include "Globals.h"
#include "Sensors.h"
#include "Controller.h"

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

struct __attribute__((packed)) T_ff_buffer {
  float Tff;
};

Frame FrameBufer[Max_Frame_Num];

float TffNextBuf[Max_Frame_Num];
float T_ff_init[Max_Frame_Num];

bool FrameBuffFetch(float &T_ff, uint32_t frameIndex) {
  if (frameIndex >= Max_Frame_Num) return false;

  T_ff = FrameBufer[frameIndex].t_ff+alpha*(FrameBufer[frameIndex].targetA-FrameBufer[frameIndex].currentA);

  return true;
}

float signum(float sig_input) {
  if (sig_input > 0) {
    float sig_out = 1.0f;
    return sig_out;
  } else if (sig_input < 0) {
    float sig_out = -1.0f;
    return sig_out;
  } else {
    float sig_out = 0.0f;
    return sig_out;
  }
}


void SaveFrame(Frame &frame) {
  if (!logFile) return;

  logFile.write((const uint8_t *)&frame, sizeof(frame));
}


void saveTrialToSD(uint16_t iter) {

    char filename[32];
    sprintf(filename, "AAN_%04d.bin", iter);

    if (SD.exists(filename)) SD.remove(filename);

    File f = SD.open(filename, FILE_WRITE);

    if (!f) {
        Serial.print("SAVE FAIL: ");
        Serial.println(filename);
        return;
    }

    size_t bytes = FrameCounter * sizeof(Frame);
    f.write((uint8_t*)FrameBufer, bytes);
    f.flush();
    f.close();

    Serial.print("Saved ");
    Serial.print(FrameCounter);
    Serial.print(" frames to ");
    Serial.println(filename);
}





bool FetchFrame(Frame &outFrame, uint16_t iterNum, uint32_t frameIndex) {
  char filename[32];
  sprintf(filename, "AAN_%04d.bin", iterNum);

  File f = SD.open(filename, FILE_READ);
  
  if (!f){
    Serial.print("Fetch: failed open ");
    Serial.println(filename);
    return false;
  } 

  uint32_t pos = frameIndex * sizeof(Frame);
  uint32_t fsize = (uint32_t)f.size();
  if (pos + sizeof(Frame) > fsize){
    f.close();
    return false;
  } 

  f.seek(pos);
  size_t n = f.read((uint8_t *)&outFrame, sizeof(outFrame));
  f.close();  
  return n == sizeof(outFrame);
}

void AAN(){

  


  if (micros() >= RunEndTime) {
    // Stop motor
    analogWrite(MotorTorquePin, 0);
    digitalWrite(MotorDirectionPin, LOW);
    digitalWrite(MotorEnablePin, LOW);

    // CloseLogFile();

    // Prepare rest period
    RestEndTime = micros() + (uint32_t)(RestDuration * 1e6);

    State = 2;  // switch to REST state
    Serial.print("ITERATION ");
    Serial.print(Iteration);
    Serial.println(" COMPLETE — RESTING");

    return;
  }

  // Encoder
  float rawAngle = EncoderAngle();
  currentAngle = 0.9f * currentAngle + 0.1f * rawAngle; // low-pass filter
  currentAngleRad = currentAngle*deg2rad;

  // Time
  currentTime = micros();
  if (StartTime == 0) {
    StartTime = currentTime;
  }
  
  float deltaTime = (currentTime - previousTime)/1000000.0f;
  float SysFreq = 1.0f/deltaTime;
  float elapsedTime = (currentTime - StartTime)/1000000.0f; // seconds
  if (elapsedTime < InitDelay) {
    targetAngleRad = currentAngleRad;
  } else {
    // Sin wave trajectory
    float SinTime = elapsedTime - InitDelay;
    float sinWave = offset + amplitude * sin(2.0f*PI*frequency*SinTime + phaseShift);
    float ramp = constrain((SinTime/RampTime),0.0f,1.0f);  // 1s ramp
    targetAngle = (1.0f - ramp) * currentAngle + ramp * sinWave;
    targetAngleRad = targetAngle*deg2rad;
  }
  
  // Error terms for controller
  error = targetAngleRad - currentAngleRad;
  error_vel = (error-error_prev)/deltaTime;
  vel_error = error_vel - error_vel_prev;

  //------------- AAN --------------------------------------------------------------
  // Feed forward torque T_ff
  // Frame prevFrame;

  float prevTff = 0.0f;
 
  if (Iteration == 1) {
    T_ff = 0.0f;
  } else {
    if (FrameCounter<Max_Frame_Num) {
      // FrameBuffFetch(prevTff, FrameCounter);
      T_ff = T_ff_init[FrameCounter];
      // float T_ff_next = T_ff + alpha *error;
      // TffBuf[FrameCounter].Tff = T_ff_next;
      
    } else {
      // error_received = 0;
      T_ff = 0.0f;
    }
  }

  // Feedback torque T_fb
  epsilon = error + beta *vel_error;
  betta = a / (1.0f + b*epsilon*epsilon);
  f = epsilon/betta;

  K_P = f * error;
  K_D = f * vel_error;

  T_fb = constrain(K_P*error + K_D*vel_error, -MaxTorque, MaxTorque);

  // Filter
  T_fb = 0.9f*T_fb_prev + 0.1f*T_fb; 
  
  // Desired torque
  Torque_old = T_ff + T_fb;                                 // Combined torque from feedback and feed forward
  Torque = alpha_out*Torque_old + signum(Torque_old)*bound; // Deadzone mitigation
  
  // Convert torque to current
  desiredCurrent = abs(Torque) / gearRatio /Kt;

  // Check for direction rotation
  if (Torque >=0) {
    digitalWrite(MotorDirectionPin, HIGH);
  } else {
    digitalWrite(MotorDirectionPin, LOW);
  }
  // Calculate and send PWM
  DesiredPWM = constrain((desiredCurrent/maxCurrent)*maxPWM, 0, maxPWM);      
  analogWrite(MotorTorquePin,DesiredPWM);
  
  // Escon feedback
  AverageCurrent = ESCONCurrent();

  error_prev = error;
  error_vel_prev = error_vel;
  previousTime = currentTime;
  T_fb_prev = T_fb;
  SaveCounter++;

  

  // Frame frame;
  if (SaveCounter >= 10){
    SaveCounter = 0;
    Frame &frame = FrameBufer[FrameCounter];

    frame.header = 0xAA55;
    frame.t = currentTime;
    frame.targetA = targetAngle;
    frame.currentA = currentAngle;
    frame.t_ff = T_ff;
    frame.t_fb = T_fb;
    frame.actualI = AverageCurrent;
    frame.freq = SysFreq;
    frame.padding = 0xBB66;

    float T_ff_next = T_ff + alpha *error;
    TffNextBuf[FrameCounter] = T_ff_next;

    FrameCounter++;
  }

  if (GUIcounter == Send2Gui) {
    // SendFrame(frame);
    GUIcounter = 0;
  }

  GUIcounter++;
}



void RestStep(){
  // Motor stays off
  analogWrite(MotorTorquePin, 0);
  digitalWrite(MotorDirectionPin, LOW);
  digitalWrite(MotorEnablePin, LOW);
  // saveTrialToSD(Iteration);

  static bool savedThisRest = false;

    // Only save once
    if (!savedThisRest) {
        saveTrialToSD(Iteration);
        savedThisRest = true;
    }
  
  
  if (micros() >= RestEndTime) {
      // Start next iteration
      
      if (Iteration < FinalIteration) {
        Iteration++;
        PrevFrameCount = FrameCounter;

        for (uint32_t k = 0; k < PrevFrameCount; k++) {
          T_ff_init[k] = TffNextBuf[k];
        }

        FrameCounter = 0;
        SaveCounter = 0;
        T_ff_prev = 0;
        error_prev = 0;
        error_vel_prev = 0;

        StartTime = micros();
        RunEndTime = StartTime + (uint32_t)(RunDuration * 1e6);

      } else {
        State = 0; // finished all runs
        Serial.println("ALL ITERATIONS COMPLETE");
        return;
      }
      
      savedThisRest = false;

      
      // Reset controller values if needed

            
  }
}

void CloseLogFile() {
  if (logFile) {
    logFile.flush();
    logFile.close();
  }
}

