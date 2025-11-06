// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// AAN controller - Maxon Current controller v5.0 Written for Teensy 4.1
//
// Teensy works only to tranfer data between PC and Exoskeleton
// Teensy receives tourqe command and sends PWM to ESCON
// Teensy send to PC current angle and average current received from ESCON
//
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#include <Arduino.h>
// ================ 500 Hz SYSTEM ===================================================
IntervalTimer sysTimer;
volatile bool tickFlag = false;   // Flag to run the loop
volatile uint32_t tickCount = 0;  // CLock counter
const int TimerTickNo = 2000;         // 1000 = 1kHz system; 2000 = 500Hz system;

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
float Torque = 0;        // Received data from PC
float Torque_prev = 0;   // Previous tourqe
const float maxCurrent = 4.24;  // Maximum allowable current for motor [A]
const float Kt = 21.3/1000;     // Torque constant [Nm/A]
const float MaxTorque = 6.32;   // Maximum allowable Tourque from motor[Nm]

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
const float bound = 0.001;       // upper/lower bound for deadzone   
bool controllerActive = false;  // Activation of the control loop

// =========================== COMPUTER COMUNICATION ===============================
struct Frame {
  uint16_t header;                // Always 0xAA55
  float currentA;                 // Current angle [deg]
  float actualI;                  // Received input [A]
  float receivedTau;
};

struct InputPacket {
  uint16_t header;                // always 0x55AA
  float Tau;                // feed-forward term from previous iteration
};

InputPacket incomingPacket;       
bool newPacketAvailable = false;  // Flag for new packet
// =========================== Functions ===========================================
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
  if (cmd.startsWith("START")){
    digitalWrite(MotorEnablePin, HIGH);
    controllerActive = true;
    Serial.println("START OK");

  } else if (cmd.startsWith("STOP")) {
    controllerActive = false;
    digitalWrite(MotorEnablePin, LOW);
    analogWrite(MotorTorquePin,0);
    Serial.println("STOP OK");

  }

}

bool ReceiveBinaryPacket(float &tauOut) {
  static uint8_t buffer[sizeof(InputPacket)];
  static size_t index = 0;

  while (Serial.available() > 0) {
    uint8_t incoming = Serial.read();
    buffer[index++] = incoming;

    if (index == sizeof(InputPacket)) {
      InputPacket temp;
      memcpy(&temp, buffer, sizeof(InputPacket));
      index = 0;

      // Validate header
      if (temp.header == 0x55AA) {
        tauOut = temp.Tau;
        return true;
      }
    }
  }
  return false;  // No valid packet yet
}

// --------------------------- Sensor Read -----------------------------------------
float EncoderAngle(){
  // Read encoder 
  int sensorValue = analogRead(AngPin);
  // Calculate the angle
  // float readAngle = -0.3315*sensorValue + 200.9-20;
  float readAngle = -0.149f*sensorValue + 188.66f+5.0f;
  // float readAngle = sensorValue; // Change to this line for calibration
  return readAngle;
}

float EsconCurrent(){
  // Read data
  int EsconOut = analogRead(EsconOutPin);
  // Calculate Voltage
  float voltage = (EsconOut/4095.0f) * 3.3f;
  //  Calculate Current
  float Current = (voltage - 1.65f) / 1.65f * 5.00f;
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




void setup() {
  Serial.begin(1000000);  // Serial Monitor

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
  
  sysTimer.begin(sysTickISR, TimerTickNo);
  Serial.println("READY");

}

void loop() {
  if (Serial.available() > 0) {
    if (Serial.peek() != 0x55){
      String cmd = Serial.readStringUntil('\n');
      ProccesCommand(cmd);
    }    
  }
  if (tickFlag){
    tickFlag = false;
    // Encoder
    float rawAngle = EncoderAngle();
    currentAngle = 0.9f * currentAngle + 0.1f * rawAngle; // low-pass filter
    currentAngleRad = currentAngle*deg2rad;

    // ESCON
    AverageCurrent = EsconCurrent();


    if (controllerActive){
      float receivedTau = Torque_prev;
      if (ReceiveBinaryPacket(receivedTau)) {
        Torque = receivedTau;
        Torque_prev = Torque;
      } else {
        Torque = Torque_prev; // use last known value
    }
    
    // Convert torque to current
    desiredCurrent = abs(Torque) / gearRatio /Kt;

    if (Torque >=0) {
      digitalWrite(MotorDirectionPin, HIGH);
    } else {
      digitalWrite(MotorDirectionPin, LOW);
    }

    // Calculate and send PWM
    DesiredPWM = constrain((desiredCurrent/maxCurrent)*maxPWM, 0, maxPWM);      
    analogWrite(MotorTorquePin,DesiredPWM);

    } // end of controllerActive

    Frame frame;
    frame.header = 0xAA55;
    frame.currentA = currentAngle;
    frame.actualI = AverageCurrent;
    frame.receivedTau = Torque;

    SendFrame(frame);
      
  } // end of tickFlag
} // end of loop
