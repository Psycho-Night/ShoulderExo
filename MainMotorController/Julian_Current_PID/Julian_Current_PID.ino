// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Maxon Motor - Torque PID v1.0 - Wrote Teensy 4.1

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//----------- PID ---------------------------------------------------------------
// Keep track of changes

const float K_p = 1.0;
const float K_i = 0.0;
const float K_d = 0.0;

float error = 0.0;
float error_d = 0.0;
float error_prev = 0.0;
float error_int = 0.0;

float PID_out = 0.0;

//----------- TIME --------------------------------------------------------------

long previousTime = 0; // For time calcualtion
long timeOff = 0;

//----------- ESCON -------------------------------------------------------------
// Motor control
const int motorEnablePin = 1;       // Enable (Digital Input 2, High Active)
const int motorDirectionPin = 2;    // Direction (Digital Input 4, High Active)
const int motorTourqePin = 24;      // PWM Set Value (Analog Input 1) 12-bit

// Sensors
const int ESCON_OUT_pin = A8;       // Read Current from ESCON (Analog Output 1) 12-bit


//----------- PARAMETERS --------------------------------------------------------

const float V_max = 3.3;            // Max command voltage (e.g. driver input 0-5V)
const int maxPWM = 4095;            // 12-bit PWM range
const int PWM_deadZone = 33;
float desiredCurrent = 0;           // Desired current in A
const float maxCurrent = 4.24;      // Max allowed current for driver
const float Kt = 21.3/1000;         // Torque constany [Nm/A]

float targetTourque = 0;
float targetCurrent = 0;
float targetPWM = 0;
const float deg2rad = PI/180;               // Degrees to rad

float currentTorque = 0;
float currentTorque_prev = 0;

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



//----------- SIN WAVE ----------------------------------------------------------

const float maxAngleRad = 70*deg2rad; // Keep at max 90 deg
const float maxTorque   = L*mass*g*sin(maxAngleRad); // 
const float frequency = 0.05;            // Frequency in Hz (adjust as needed)
const float amplitude = maxTorque/2;           // Amplitude (max deviation from midpoint)
const float offset = maxTorque/2;              // Offset/midpoint of the sine wave
const long InitialisationDelay = 2000;  // Delay used for initialisation
const float phaseShift = -PI/2;          // Phase shift for graduate raise from 0 deg -PI/2 if sin wave and 0 if step (frequency = 0)

void setup() {
  // Pins setup
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorTourqePin, OUTPUT);
  pinMode(ESCON_OUT_pin, INPUT);

  analogReadResolution(12);
  analogWriteResolution(12);

  digitalWrite(motorEnablePin, HIGH);
  digitalWrite(motorDirectionPin, LOW);
  analogWrite(motorTourqePin, 0);


}

void loop() {
  // Time
  long currentTime = micros(); // [ms]
  float deltaTime = (currentTime - previousTime) / 1000000.0; // [s]
  float Frequency = 1.0 / deltaTime; // Frequency of the system [Hz]

  if (currentTime<InitialisationDelay) { // Modify to start faster
    targetTourque = 0; //
   }
  else {
    long SinTime = currentTime - InitialisationDelay;
    targetTourque = offset + amplitude * sin(2.0 * PI * frequency * (SinTime/1000.0) + phaseShift); // Sine wave
  }
  
  // Read values from ESCON
  ActualCurrent = analogRead(ESCON_OUT_pin);
  ActualCurrent = ActualCurrent/4095*4.24;

  targetCurrent = (abs(targetTourque)/gearRatio) / Kt; // Convertion from Torque to Current [A]

  error = targetCurrent - ActualCurrent;
  // Serial.println(error);

  error_d = (error-error_prev)/deltaTime;
  // Serial.println(error_d);
  error_int += error;
  // Serial.println(error_int);
  PID_out = K_p*error+K_d*error_d+K_i*error_int;

  // Serial.println(PID_out);



  

  if (PID_out>=0){
    digitalWrite(motorDirectionPin,HIGH);
  }
  else {
    digitalWrite(motorDirectionPin,LOW);
  }
  
  DesiredPWM = constrain((abs(PID_out)/maxCurrent) * maxPWM, 0, maxPWM);

  analogWrite(motorTourqePin, DesiredPWM);
  digitalWrite(motorEnablePin, HIGH);

  previousTime = currentTime;
  error_prev = error;

  Serial.print(targetCurrent);
  Serial.print(" ");
  Serial.print(ActualCurrent);
  Serial.print(" ");
  Serial.print(DesiredPWM);
  Serial.print(" ");
  Serial.println(Frequency);



  

}


