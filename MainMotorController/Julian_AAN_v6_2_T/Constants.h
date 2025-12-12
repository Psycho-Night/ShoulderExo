#ifndef CONSTANTS_HCONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>


// ===== SYSYEM FREQUENCY =====
// 1000  = 1000Hz;
// 2000  =  500Hz;
// 2222  =  450Hz; 
// 2500  =  400Hz;
// 2857  =  350Hz; 
// 3333  =  300Hz;
// 4000  =  250Hz;
// 5000  =  200Hz;
// 6667  =  150Hz;
// 10000 =  100Hz;
// 20000 =   50Hz;
// for now 500Hz is stable but only in first itteration
// 100 is decent 
const int TimerTickNo = 20000;
const int Send2Gui = 50;    // Counter to send data to GUI  

// ===== AAN parameters =====
const float beta = 0.05f; // Kept cosntant at 0.05
const float b = 5.0f;     // Kept cosntant at 5.0
const int FinalIteration = 7;

const int Max_Frame_Num = 8000;


// ===== PIN SETUP =====
// -------- ESCON outputs ---------------
const int MotorEnablePin = 1;     // Enable pin (Digital Input 2, HIGH = Enabled, LOW = Disabled)
const int MotorDirectionPin = 2;  // Direction pin (Digital Input 4, HIGH = CW, LOW = CCW)
const int MotorTorquePin = 24;    // PWM set value (Analog Input 1), 12-bit resolution

// ------------------- Sensors --------------------------
const int AngPin = A8;          // Shaft Encoder Pin use only A8 unless PCB board is changed. Teensy can handle max 3.3v
const int EsconOutPin = A9;     // Average current output from ESCON

// ------------------------- Parameters --------------------------------------------
const float V_max = 3.3f;        // Max input or output voltage [V]
const float maxPWM = 4095;      // 12-bit PWM range
const float maxCurrent = 4.24f;  // Maximum allowable current for motor [A]
const float Kt = 21.3f/1000.0f;     // Torque constant [Nm/A]
const float MaxTorque = 6.32;   // Maximum allowable Tourque from motor[Nm]

// =========================== OTHER PARAMETERS ====================================
const float deg2rad = PI/180.0f;   // convertion from degrees to radians for reverse convertion it's 1/deg2rad
const int gearRatio = 70;       // Gear ratio between motor and output shaft motor 35:1 and bevel gear 2:1 combined 70:1
const float alpha_out = 1.0f;      // Filter for deadzone
const float bound = 0.001f;       // upper/lower bound for deadzone   

#endif
