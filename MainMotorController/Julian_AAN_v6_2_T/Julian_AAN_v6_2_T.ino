// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// AAN controller - Maxon Current controller v6.1 Written for Teensy 4.1
//
// AAN coded on Teensy works at 500Hz and sends vaulues to GUI at 10Hz for debugging
// Each Iteration is firstly saved in RAM memory and then moved to SDcard.
// Each Iteration is saved on SDcard as .bin files. Has to be decoded later.
// State machine implemented for easier workflow of Teensy 
// 
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

#include "Constants.h"
#include "Globals.h"
#include "Comunication.h"
#include "Sensors.h"
#include "Controller.h"

// =========================== SYSYEM FREQUENCY ====================================
IntervalTimer sysTimer;
volatile bool tickFlag = false;   // Flag to run the loop
volatile uint32_t tickCount = 0;  // CLock counter
int GUIcounter = 0;


// =========================== STATE MACHINE ======================================
int State = 0;

// ======================= AAN =====================================================
float alpha = 0.01;      // Tuning parameter can be changed in GUI

float a = 2.0;            // Tuning parameter can be changed in GUI

float error = 0;          // Position error 
float error_prev = 0;     // Previous position error
float error_vel = 0;      // Velocity of the error -> error_dot(t)
float error_vel_prev = 0;  // Previous velocity of the error -> error_dot(t-1)
float vel_error = 0;      // Error of velocity -> error_dot(t) - error_dot(t-1)
float error_received = 0; // Error from previous iteration

float epsilon = 0;
float betta = 0;
float f = 0;
float K_P = 0;            // P gain value
float K_D = 0;            // D gain value

float T_ff = 0;           // Feed forward tourqe
float T_ff_prev = 0;      // Feed forward term from previous iteration 
float T_fb = 0;           // Feedback term 
float T_fb_prev = 0;      // Previous feedback term
float Torque = 0;         // Torque output from AAN after deadzone mitigation
float Torque_old = 0;     // Torque output from AAN before deadzone mitigation

// ========================= SIN WAVE ==============================================
float frequency = 0.2;             // Frequecy of sin trajectory in Hz
float amplitude = 35.0;             // Amplitude of sin trajectory in deg
float offset = 35;                  // Offset of sin trajectory in deg
float phaseShift = -PI/2;           // Phase shift for graduate rise of sin trajectory in rad
unsigned long RampTime = 2;         // Time of ramp signal
unsigned long StartTime = 0;        // Time in which start controller starts

//  ======================== TIME ==================================================
unsigned long currentTime = 0;      // Current time [microseconds]
unsigned long previousTime = 0;     // For time calculation [microseconds]
unsigned long InitDelay = 2;        // Delay for the controller to start

//  ======================== PIN SETUP =============================================
// ------------------------- Parameters --------------------------------------------
float desiredCurrent = 0;       // Current output from AAN controller [A]

// =========================== OTHER PARAMETERS ====================================
float targetAngle = 0;          // Target Angle [deg]
float targetAngleRad = 0;       // Target Angle [rad]
float currentAngle = 0;         // Current Angle [deg] read from sensor
float currentAngleRad = 0;      // Current Angle [rad] read from sensor
float AverageCurrent = 0;       // Current read from Escon [A]
int DesiredPWM = 0;             // PWM send from Teensy
bool controllerActive = false;  // Activation of the control loop

// static int lastState = -1;

// =========================== Functions ===========================================
// --------------------------- System Frequency ------------------------------------
void sysTickISR() {
  tickFlag = true;
  tickCount++;
}

void setup() {
  Serial.begin(1000000);

  // Pins setup
  pinMode(MotorEnablePin, OUTPUT);
  pinMode(MotorDirectionPin, OUTPUT);
  pinMode(MotorTorquePin, OUTPUT);
  pinMode(AngPin, INPUT);
  pinMode(EsconOutPin, INPUT);

  digitalWrite(MotorEnablePin, LOW);
  digitalWrite(MotorDirectionPin, LOW);
  analogWrite(MotorTorquePin, 0);

  analogReadResolution(12);
  analogWriteResolution(12);

  targetAngle = EncoderAngle();
  targetAngleRad = targetAngle*deg2rad;
  currentAngle = EncoderAngle();
  currentAngleRad = currentAngle*deg2rad;

  error = 0;
  error_prev = 0;
  vel_error = 0;
  T_ff_prev = 0;

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD FAIL");
  } else {
    Serial.println("SD OK");
  }


  sysTimer.begin(sysTickISR, TimerTickNo);
  
  Serial.println("Teensy READY");
}

void loop() {
  if (Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    ProccesCommand(cmd);
  }

  while (tickFlag) {
    tickFlag = false;

    switch (State) {
      case 1:
        digitalWrite(MotorEnablePin, HIGH);
        AAN();
        break;
      case 2:
        RestStep();
        break;
      default:
        break;
    }
    

  } // End of TickFlag
}

