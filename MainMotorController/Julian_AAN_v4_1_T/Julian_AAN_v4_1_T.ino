// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// AAN controller - Maxon Current controller v4.1 Written for Teensy 4.1
// Soft start, Time in micro seconds 1 kHz system,
// Additional binary computer communication and new GUI(rewritten)

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#include <Arduino.h>

// ================ 1 kHz SYSTEM ===================================================
IntervalTimer sysTimer;
volatile bool tickFlag = false;
volatile uint32_t tickCount = 0;


// ======================= AAN =====================================================
float alpha = 0.006;      // Tuning parameter can be changed in GUI
const float beta = 0.05;  // Kept cosntant at 0.05

float a = 5.0;            // Tuning parameter can be changed in GUI
const float b = 5.0;      // Kept cosntant at 5.0

float error = 0;          // Position error
float error_prev = 0;     // Previous position error
float vel_error = 0;      // Velocity error

float epsilon = 0;
float betta = 0;
float f = 0;
float K_P = 0;            // P gain value
float K_D = 0;            // D gain value

float T_ff = 0;           // Feed forward tourqe
float T_ff_prev = 0;      // Previous feed forward term
float T_fb = 0;           // Feedback term 
float T_fb_prev = 0;      // Previous feedback term
float Torque = 0;         // Torque output from AAN after deadzone mitigation
float Torque_old = 0;     // Torque output from AAN before deadzone mitigation

// ========================= SIN WAVE ==============================================
float frequency = 0.05;             // Frequecy of sin trajectory in Hz
float amplitude = 35.0;             // Amplitude of sin trajectory in deg
float offset = 35;                  // Offset of sin trajectory in deg
float phaseShift = -PI/2;           // Phase shift for graduate rise of sin trajectory in rad
unsigned long RampTime = 5*1000000; // Time of ramp signal
unsigned long StarTime = 0;         // Time in which start controller starts

//  ======================== TIME ==================================================
unsigned long currentTime = 0;      // Current time [microseconds]
unsigned long previousTime = 0;     // For time calculation [microseconds]
unsigned long InitDelay = 2*1000000;// Delay for the controller to start

//  ======================== PIN SETUP =============================================
// ------------------------- ESCON outputs -----------------------------------------
const int MotorEnablePin = 1;     // Enable pin (Digital Input 2, HIGH = Enabled, LOW = Disabled)
const int MotorDirectionPin = 2;  // Direction pin (Digital Input 4, HIGH = CW, LOW = CCW)
const int MotorTorquePin = 24;    // PWM set value (Analog Input 1), 12-bit resolution

// ------------------------- Sensors -----------------------------------------------
const int AngPin = A8;          // Shaft Encoder Pin use only A8 unless PCB board is changed. Teensy can handle max 3.3v
const int EsconOutPin = A9;     // Average current output from ESCON

// ------------------------- Parameters --------------------------------------------
const float V_max = 3.3;        // Max input or output voltage [V]
const float maxPWM = 4095;      // 12-bit PWM range
float desiredCurrent = 0;       // Current output from AAN controller [A]
const float maxCurrent = 4.24;  // Maximum allowable current for motor [A]
const float Kt = 21.3/1000;     // Torque constant [Nm/A]

// =========================== OTHER PARAMETERS ====================================
const float deg2rad = PI/180;   // convertion from degrees to radians for reverse convertion it's 1/deg2rad
float targetAngle = 0;          // Target Angle [deg]
float targetAngleRad = 0;       // Target Angle [rad]
float currentAngle = 0;         // Current Angle [deg] read from sensor
float currentAngleRad = 0;      // Current Angle [rad] read from sensor
float AverageCurrent = 0;       // Current read from Escon [A]
const int gearRatio = 70;       // Gear ratio between motor and output shaft motor 35:1 and bevel gear 2:1 combined 70:1
int DesiredPWM = 0;             // PWM send from Teensy
const float alpha_out = 1;      // Filter for deadzone
const float bound = 0.05;       // upper/lower bound for deadzone   
bool controllerActive = false;  // Activation of the control loop

// =========================== COMPUTER COMUNICATION ===============================
struct Frame {
  uint16_t header;  // Always 0xAA55
  uint32_t t;       // timestamp [microseconds]
  float targetA;    // Target angle [deg]
  float currentA;   // Current angle [deg]
  float torque;     // Torque [Nm]
  float t_ff;       // T_FF [Nm]
  float t_fb;       // T_FB [Nm]
  float desiredI;   // Desired output [A]
  float actualI;    // Received input [A]
  float freq;       // System Frequency [Hz]
};

// =========================== Functions ===========================================
// --------------------------- System Frequency ------------------------------------
void sysTickISR() {
  tickFlag = true;
  tickCount++;
}

// --------------------------- Comunication ----------------------------------------
void SendFrame(struct Frame &frame) {
  Serial.write((uint8_t *)&frame, sizeof(frame));
}

void ProccesCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("SET")){
    if (!controllerActive) {
      if (sscanf(cmd.c_str(), "SET %f %f %f %f %f %f", &frequency, &amplitude, &offset, &phaseShift, &alpha, &a) == 6) {
      // sscanf(cmd.c_str(), "SET %f %f %f %f %f %f", &frequency, &amplitude, &offset, &phaseShift, &alpha, &a);
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
    StarTime = 0;

    Serial.println("START OK");
  } else if (cmd.startsWith("STOP")) {
    controllerActive = false;
    digitalWrite(MotorEnablePin, LOW);
    analogWrite(MotorTorquePin,0);

    Serial.println("STOP OK");
  }
}
// --------------------------- Sensor Read -----------------------------------------
float EncoderAngle(){
  // Read encoder 
  int sensorValue = analogRead(AngPin);
  // Calculate the angle
  // float readAngle = -0.3315*sensorValue + 200.9-20;
  float readAngle = -0.149f*sensorValue + 188.66f;
  // float readAngle = sensorValue; // Change to this line for calibration
  return readAngle;
}

float EsconCurrent(){
  // Read data
  int EsconOut = analogRead(EsconOutPin);
  //  Calculate Current
  float Current = EsconOut / 4095.0f * 4.24f;
  return Current;
}
// --------------------------- Other -----------------------------------------------
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

// --------------------------- System Frequency ------------------------------------

void setup() {
  Serial.begin(1000000);  // Serial Monitor
  // Serial.begin(115200);

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
  

  sysTimer.begin(sysTickISR, 1000);
  Serial.println("READY");

}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    ProccesCommand(cmd);
  }
  
  if (tickFlag) {
    tickFlag = false;
    if (controllerActive) {
      // Time
      currentTime = micros();

      if (StarTime == 0) {
      StarTime = currentTime;
      }
      
      float deltaTime = (currentTime - previousTime)/1000000.0f;
      float SysFreq = 1.0f/deltaTime;

      if (currentTime-StarTime < InitDelay) {
        targetAngle = EncoderAngle();
        targetAngleRad = targetAngle * deg2rad;
      } else {
        // Sin wave trajectory
        float SinTime = currentTime - StarTime - InitDelay;
        float sinWave = offset + amplitude * sin(2.0f*PI*frequency*(SinTime/1000000.0f) + phaseShift);
        float ramp = constrain((currentTime - RampTime)/1000000.0f,0.0f,1.0f);  // 1s ramp
        targetAngle = (1.0f - ramp) * currentAngle + ramp * sinWave;
        targetAngleRad = targetAngle*deg2rad;
      }
      // Encoder
      float rawAngle = EncoderAngle();
      currentAngle = 0.9f * currentAngle + 0.1f * rawAngle; // low-pass filter
      currentAngleRad = currentAngle*deg2rad;

      // Error terms for controller
      error = targetAngleRad - currentAngleRad;
      vel_error = (error-error_prev)/deltaTime;

      //------------- AAN --------------------------------------------------------------
      // Feed forward torque T_ff
      T_ff = T_ff_prev + alpha*error_prev;

      // Feedback torque T_fb
      epsilon = error + beta *vel_error;
      betta = a / (1.0f + b*epsilon*epsilon);
      f = epsilon/betta;

      K_P = f * error;
      K_D = f * error;

      T_fb = K_P*error + K_D*vel_error;
      // Filter
      T_fb = 0.8f*T_fb_prev + 0.2f*T_fb; 
      

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
      AverageCurrent = EsconCurrent();

      // Record current values as previous
      error_prev = error;
      previousTime = currentTime;
      T_ff_prev = T_ff;
      T_fb_prev = T_fb;

      // Prepare and send data to PC
      Frame frame;
      frame.header = 0xAA55;
      frame.t = currentTime;
      frame.targetA = targetAngle;
      frame.currentA = currentAngle;
      frame.torque = Torque;
      frame.t_ff = T_ff;
      frame.t_fb = T_fb;
      frame.desiredI = desiredCurrent;
      frame.actualI = AverageCurrent;
      frame.freq = SysFreq;
      SendFrame(frame);
      
      //----------------- Signal send test ----------------------
      // frame.targetA = 2222.2f;
      // frame.currentA = 3333.3f;
      // frame.torque = 4444.4f;
      // frame.t_ff = 5555.5f;
      // frame.t_fb = 6666.6f;
      // frame.desiredI = 7777.7f;
      // frame.actualI = 8888.8f;
      // frame.freq = 9999.9f;


      // Sensor Calibration
      // Serial.println(currentAngle);
      
    } // Controller active
  } // Tick active
}
