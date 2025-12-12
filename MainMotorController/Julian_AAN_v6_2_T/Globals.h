#include <stdint.h>
#ifndef GLOBALS_H
#define GLOBALS_H
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// ====================== SYSYEM FREQUENCY ========================================
extern volatile bool tickFlag;       // Flag to run the loop
extern volatile uint32_t tickCount;  // CLock counter
extern int GUIcounter;

// =========================== STATE MACHINE ===============================
extern int State;

// ======================= AAN =====================================================
extern float alpha;           // Tuning parameter can be changed in GUI

extern float a;               // Tuning parameter can be changed in GUI

extern float error;           // Position error
extern float error_prev;      // Previous position error
extern float error_vel;       // Velocity of the error -> error_dot(t)
extern float error_vel_prev;  // Previous velocity of the error -> error_dot(t-1)
extern float vel_error;       // Error of velocity -> error_dot(t) - error_dot(t-1)
extern float error_received;  // Error from previous iteration

extern float epsilon;
extern float betta;
extern float f;
extern float K_P;             // P gain value
extern float K_D;             // D gain value

extern float T_ff;            // Feed forward tourqe
extern float T_ff_prev;       // Feed forward term from previous iteration
extern float T_fb;            // Feedback term
extern float T_fb_prev;       // Previous feedback term
extern float Torque;          // Torque output from AAN after deadzone mitigation
extern float Torque_old;      // Torque output from AAN before deadzone mitigation


// ========================= SIN WAVE ==============================================
extern float frequency;          // Frequecy of sin trajectory in Hz
extern float amplitude;          // Amplitude of sin trajectory in deg
extern float offset;             // Offset of sin trajectory in deg
extern float phaseShift;         // Phase shift for graduate rise of sin trajectory in rad
extern unsigned long RampTime;   // Time of ramp signal
extern unsigned long StartTime;  // Time in which start controller starts

//  ======================== TIME ==================================================
extern unsigned long currentTime;   // Current time [microseconds]
extern unsigned long previousTime;  // For time calculation [microseconds]
extern unsigned long InitDelay;     // Delay for the controller to start
extern float RunDuration;   // seconds
extern float RestDuration;          // seconds

extern uint32_t RunEndTime;         // micros()
extern uint32_t RestEndTime;        // micros()

//  ======================== PIN SETUP =============================================
// ------------------------- Parameters --------------------------------------------
extern float desiredCurrent;  // Current output from AAN controller [A]

// =========================== OTHER PARAMETERS ====================================
extern float targetAngle;      // Target Angle [deg]
extern float targetAngleRad;   // Target Angle [rad]
extern float currentAngle;     // Current Angle [deg] read from sensor
extern float currentAngleRad;  // Current Angle [rad] read from sensor
extern float AverageCurrent;   // Current read from Escon [A]
extern int DesiredPWM;         // PWM send from Teensy
extern bool controllerActive;  // Activation of the control loop

// =========================== SD CARD ====================================

extern File logFile;
extern uint32_t Iteration;
extern uint32_t FrameCounter;
extern int SaveCounter;
extern int PrevFrameCount;

#endif