// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// AAN controller - Maxon Current controller v3.0 Written for Teensy 4.1
// Soft start, Time in micro seconds 1 kHz system
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ------------------- 1 kHz SYSTEM SETUP -----------------------------------------------
#include <Arduino.h>

IntervalTimer sysTimer;

volatile bool ticFlag = false;
volatile uint32_t tickCount = 0;

void sysTickISR() {
  ticFlag = true;
  tickCount++;
}

//--------------------- AAN ------------------------------------------------------------

const float alpha = 0.006;  // working 0.006
const float beta = 0.0;  // Keep it 0.05


// Tune those and keep changes
const float a = 1.5;      // for now 1.5 but might have to be decreased
const float b = 5.0;      // Keep it 5.0

float error = 0;          // Position error
float error_prev = 0;     // Previous position error
float error_vel = 0;      // Velocity error

float epsilon = 0;
float betta = 0;
float f = 0;

float T_ff = 0;           // Feed forward torque
float T_ff_prev = 0;      // Previous feed forward torque 
float T_fb = 0;           // Feedback torque

//----------- SIN WAVE ---------------------------------------------------------------

const float frequency = 0.05;             // Frequency in Hz (adjust as needed)
const float amplitude = 35.0;             // Amplitude (max deviation from midpoint)
const float offset = 35.0;                // Offset/midpoint of the sine wave
const long InitialisationDelay = 3000000; // Delay used for initialisation
const float phaseShift = -PI/2;           // Phase shift for graduate raise from 0 deg -PI/2 if sin wave and 0 if step

//------------ TIME ---------------------------------------------------------------

unsigned long previousTime = 0; // For time calcualtion
long timeOff = 0;

//------------ ESCON ---------------------------------------------------------------
// Motor control
const int motorEnablePin = 1;       // Enable (Digital Input 2, High Active)
const int motorDirectionPin = 2;    // Direction (Digital Input 4, High Active)
const int motorTourqePin = 24;      // PWM Set Value (Analog Input 1)

// Sensors
const int ANG_pin = A8;             // Shaft encoder pin
const int ESCON_OUT_pin = A9;       // ESCON speed output

const float V_max = 3.3;            // Max command voltage (e.g. driver input 0-5V)
const int maxPWM = 4095;            // 12-bit PWM range
const int PWM_deadZone = 33;
float desiredCurrent = 0;           // Desired current in A
const float maxCurrent = 4.24;      // Max allowed current for driver
const float Kt = 21.3/1000;         // Torque constany [Nm/A]

//------------- Parameters -----------------------------------------------------------

float targetAngle = 0;
float targetPWM = 0;
const float deg2rad = PI/180;               // Degrees to rad
float targetAngleRad = targetAngle*deg2rad; // Target anlge in rads

float currentAngle = 0;
float currentAngleRad = 0;
float currentAngle_prev = 0;
float currentAngleRad_prev = 0;

float ActualCurrent = 0;


const int gearRatio = 70;

const float mass = 1.88; // kg
const float J = 0.02; // kg*m^2
const float L = 0.0866; // m
const float g = 9.81; // m/s^2

int DesiredPWM = 0;
float desiredVoltage = 0;
float maxSpeed = 10600;
float alpha_out = 1;

float Torque = 0;
float Torque_old = 0;

float up_bound = 0.01;    // upper bound for deadzone
float down_bound = -0.01; // lower bound for deadzone

bool controllerActive = false;  // Prevents immediate torque output

// ------------------ Binary frame struct ----------------------------

struct Frame {
  uint32_t t;         // timestamp [µs]
  float target;       // target angle
  float current;      // current angle
  float torque;       // torque
  float T_ff;         // feedforward torque
  float T_fb;         // feedback torque
  float desiredI;     // desired current
  float actualI;      // measured current
  float freq;         // system frequency
};

void sendFrame(const Frame &frame) {
  Serial.write((uint8_t *)&frame, sizeof(frame));
}

void setup() {
    Serial.begin(115200);  // Serial Monitor

  // Pins setup
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorTourqePin, OUTPUT);
  pinMode(ANG_pin, INPUT);
  pinMode(ESCON_OUT_pin, INPUT);

  digitalWrite(motorEnablePin, LOW);
  digitalWrite(motorDirectionPin, LOW);
  analogWrite(motorTourqePin, 0);

  analogReadResolution(12);
  analogWriteResolution(12);

  targetAngle = EncoderAngle();
  targetAngleRad = targetAngle*deg2rad;
  currentAngle = EncoderAngle();
  currentAngleRad = currentAngle*deg2rad;


  // error = targetAngle-currentAngle;

  error = 0;
  error_prev = 0;
  error_vel = 0;
  T_ff_prev = 0;

  sysTimer.begin(sysTickISR, 1000);
}

void loop() {
  if (ticFlag) {
  ticFlag = false;
  long currentTime = micros();
  float deltaTime = (currentTime - previousTime) / 1000000.0f;
  float Frequency = 1.0f / deltaTime; // Frequency of the system [Hz]

  // --- Target angle (sinusoidal after init delay) ---
  if (currentTime < InitialisationDelay) {
    targetAngle = EncoderAngle();
    targetAngleRad = targetAngle * deg2rad;
  } else {
    long SinTime = currentTime - InitialisationDelay;
    float sineValue = offset + amplitude * sin(2.0f * PI * frequency * (SinTime / 1000000.0) + phaseShift);
    
    // Smooth blend between init pos and sine wave start
    float ramp = constrain((currentTime - InitialisationDelay) / 1000000.0, 0.0, 1.0);  // 1s ramp
    targetAngle = (1.0 - ramp) * currentAngle + ramp * sineValue;
    targetAngleRad = targetAngle * deg2rad;

    // Enable controller only after delay
    controllerActive = true;
  }
   // --- Encoder filtering ---
  float rawAngle = EncoderAngle();
  currentAngle = 0.1f * rawAngle + 0.9f * currentAngle;
  currentAngleRad = currentAngle * deg2rad;

  // --- Error terms ---
  error = targetAngleRad - currentAngleRad;
  float raw_derivative = (error - error_prev) / deltaTime;
  error_vel = 0.9f * error_vel + 0.1f * raw_derivative;

  // float Torque = 0;

  if (controllerActive) {
    // --------- AAN Controller -----------

    // Feedforward torque
    T_ff = T_ff_prev + alpha * error_prev;

    // Feedback torque
    epsilon = error + beta * error_vel;
    betta   = a / (1 + b * epsilon * epsilon);
    f = epsilon / betta;

    float K_p = f * error;
    float K_d = f * error_vel;

    T_fb = K_p * error + K_d * error_vel;

    // Desired torque
    float Torque_old = T_ff + T_fb;

    // Deadzone mitigation
    Torque = alpha_out * Torque_old + signum(Torque_old) * up_bound;

    // Convert torque to current
    desiredCurrent = (abs(Torque) / gearRatio) / Kt;

    if (Torque >= 0) digitalWrite(motorDirectionPin, HIGH);
    else digitalWrite(motorDirectionPin, LOW);

    DesiredPWM = constrain((desiredCurrent / maxCurrent) * maxPWM, 0, maxPWM);

    analogWrite(motorTourqePin, DesiredPWM);
    digitalWrite(motorEnablePin, HIGH);

    T_ff_prev = T_ff; // update for next step
  } else {
    // Controller inactive → zero torque
    analogWrite(motorTourqePin, 0);
    digitalWrite(motorEnablePin, LOW);
  }

  // --- ESCON feedback ---
  ActualCurrent = analogRead(ESCON_OUT_pin);
  ActualCurrent = ActualCurrent / 4095.0f * 4.24f;

  error_prev = error;
  previousTime = currentTime;

  // --- Serial plot output ---
  // String output = "TT: " + String(currentTime) + 
  //                 ", TA: " + String(targetAngle) +
  //                 ", CA: " + String(currentAngle) +
  //                 ", T: " + String(Torque) +
  //                 ", T_FF: " + String(T_ff)+
  //                 ", T_FB: " + String(T_fb)+
  //                 ", TC: " + String(desiredCurrent) +
  //                 ", AC: " + String(ActualCurrent)+
  //                 ", F: " + String(Frequency);
  // Serial.println(output);

  // --- Binary frame output ---
  Frame frame;
  frame.t = currentTime;
  frame.target = targetAngle;
  frame.current = currentAngle;
  frame.torque = Torque;
  frame.T_ff = T_ff;
  frame.T_fb = T_fb;
  frame.desiredI = desiredCurrent;
  frame.actualI = ActualCurrent;
  frame.freq = Frequency;

  sendFrame(frame);
  
  }
}

float signum(float sig_input){
  if (sig_input>0) {
  float sig_out = 1.0;
  return sig_out;
  }
  else if (sig_input<0) {
  float sig_out = -1.0;
  return sig_out;
  }
  else {
  float sig_out = 0;
  return sig_out;
  }
  
}

float EncoderAngle(){
  // Read encoder 
  int sensorValue = analogRead(ANG_pin);
  // Calculate the angle
  // float readAngle = -0.3315*sensorValue + 200.9-20;
  float readAngle = -0.0845f*sensorValue + 181.16f;
  // float readAngle = sensorValue; // Change to this line for calibration
  return readAngle;
}
