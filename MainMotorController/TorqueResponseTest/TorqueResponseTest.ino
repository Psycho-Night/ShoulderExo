// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Tourqe Reference - Maxon Current controller v1.0

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//----------- SIN WAVE ---------------------------------------------------------------

const float frequency = 0.05;            // Frequency in Hz (adjust as needed)
const float amplitude = 0.7930404;           // Amplitude (max deviation from midpoint)
const float offset = 0.7930404;              // Offset/midpoint of the sine wave
const long InitialisationDelay = 2000;  // Delay used for initialisation
const float phaseShift = -PI/2;          // Phase shift for graduate raise from 0 deg -PI/2 if sin wave and 0 if step

//------------ TIME ---------------------------------------------------------------

long previousTime = 0; // For time calcualtion
long timeOff = 0;

//------------ ESCON ---------------------------------------------------------------

const int motorEnablePin = 1;       // Enable (Digital Input 2, High Active)
const int motorDirectionPin = 2;    // Direction (Digital Input 4, High Active)
const int motorTourqePin = 24;       // PWM Set Value (Analog Input 1)


const int ANG_pin = A0;             // Shaft encoder pin def A0
const int AnOut = A1;               // ESCON speed output

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


const int gearRatio = 70;

const float mass = 1.88; // kg
const float J = 0.02; // kg*m^2
const float L = 0.0866; // m
const float g = 9.81; // m/s^2

int DesiredPWM = 0;
float desiredVoltage = 0;
float maxSpeed = 10600;
float alpha_out = 1;

float up_bound = 0.01;    // upper bound for deadzone
float down_bound = -0.01; // lower bound for deadzone



void setup() {
  Serial.begin(115200);  // Serial Monitor

  // Pins setup
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorTourqePin, OUTPUT);
  pinMode(ANG_pin, INPUT);
  pinMode(AnOut, INPUT);

  digitalWrite(motorEnablePin, HIGH);
  digitalWrite(motorDirectionPin, LOW);
  analogWrite(motorTourqePin, 0);

  analogWriteResolution(12);

  targetAngle = EncoderAngle();
  targetAngleRad = targetAngle*deg2rad;
  currentAngle = EncoderAngle();
  currentAngleRad = currentAngle*deg2rad;
  // error = targetAngle-currentAngle;

}

void loop() {
  // Time
  long currentTime = millis(); // [ms]
  float deltaTime = (currentTime - previousTime) / 1000.0; // [s]
  float Frequency = 1.0 / deltaTime; // Frequency of the system [Hz]

  // Target
  if (currentTime<InitialisationDelay) { // Modify to start faster
    targetAngle = 0; //
    targetAngleRad = targetAngle * deg2rad;
   }
  else {
    long SinTime = currentTime - InitialisationDelay;
    targetAngle = offset + amplitude * sin(2.0 * PI * frequency * (SinTime/1000.0) + phaseShift); // Sine wave
    targetAngleRad = targetAngle * deg2rad;
  }

  // Read and filter angle read
  float rawAngle = EncoderAngle(); // [deg]
  currentAngle = 0.1 * rawAngle + 0.9 * currentAngle;  // low-pass filter

  float Torque = targetAngle;

  desiredCurrent = (abs(Torque)/gearRatio) / Kt; // Convertion from Torque to Current [A]

  if (Torque>=0){
    digitalWrite(motorDirectionPin,HIGH);
  }
  else {
    digitalWrite(motorDirectionPin,LOW);
  }
  
  DesiredPWM = constrain((desiredCurrent/maxCurrent) * maxPWM, 0, maxPWM);

  analogWrite(motorTourqePin, DesiredPWM);
  digitalWrite(motorEnablePin, HIGH);

  previousTime = currentTime;

  // ------------------- Send COM Message ----------------------------------
  
  // COM message for plotting 
  String output = "Time: " + String(currentTime) + 
                "ms, Target Angle: " + String(desiredCurrent) +
                ", Current Angle: " + String(currentAngle) +
                ", Torque: " + String(Torque)+
                ", PWM: " + String(DesiredPWM)+
                ", sampling Frequency: " + String(Frequency);
                // ", PID: " + String(ESCON_out);
                // ", Observer: " + String(u);
              //  ", kp: " + String(Kp);
              //  ", kd: " + String(Kd);
               
  Serial.println(output);

}


float EncoderAngle(){
  // Read encoder 
  int sensorValue = analogRead(ANG_pin);
  // Calculate the angle
  float readAngle = -0.3315*sensorValue + 200.9-20;
  // float readAngle = sensorValue; // Change to this line for calibration
  return readAngle;
}