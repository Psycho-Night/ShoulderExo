// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// PID controller - Maxon Speed controller v1.0

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//--------------------- PID ------------------------------------------------------------
// RECORD CHANGES !!!!!

float kf = 0.0;       // Keep zero
float kp = 2.0;       // 0.2Hz - 1.0; 0.1Hz - 1.2; 0.05Hz - 1.8
float kd = 0.0;      // 0.2Hz - 0.0; 0.1Hz - 0.01; 0.05Hz - 0.05
float ki = 0.00;       // 0.2Hz - 0.0; 0.1Hz - 0.0; 0.05Hz - 0.0

float error = 0;
float error_prev = 0;
float error_vel = 0;
float error_int = 0;
float raw_integral = 0;

//----------- SIN WAVE ---------------------------------------------------------------

const float frequency = 0.05;  // Frequency in Hz (adjust as needed)
const float amplitude = 35.0; // Amplitude (max deviation from midpoint)
const float offset = 35.0;    // Offset/midpoint of the sine wave

const long InitialisationDelay = 2000; // Delay used for initialisation
//const float phaseShift = asin(-offset/amplitude); // Phase shift for sin wave
const float phaseShift = -PI/2;

//------------ TIME ---------------------------------------------------------------

long previousTime = 0; // For time calcualtion
long timeOff = 0;

//------------ ESCON ---------------------------------------------------------------

const int motorEnablePin = 2;     // Enable (Digital Input 2, High Active)
const int motorDirectionPin = 4;  // Direction (Digital Input 4, High Active)
const int motorTourqePin = 6;   // PWM Set Value (Analog Input 1)

const int ANG_pin = A0;   // Shaft encoder pin def A0

const float V_max = 3.3;           // Max command voltage (e.g. driver input 0-5V)
const int maxPWM = 4095;            // 12-bit PWM range
const int PWM_deadZone = 33;
float desiredCurrent = 0;  // Desired current in A
const float maxCurrent = 4.24;      // Max allowed current for driver
const float Kt = 21.3/1000; // Nm/A

//------------- Parameters -----------------------------------------------------------

float targetAngle = 0;
float targetPWM = 0;
const float deg2rad = PI/180;               // Degrees to rad
float targetAngleRad = targetAngle*deg2rad; // Target anlge in rads

float currentAngle = 0;
float currentAngleRad = 0;
float currentAngle_prev = 0;
float currentAngleRad_prev = 0;

float MotorInputT_prev = 0;

float u_prev = 0;

float direction = 0;

const int gearRatio = 70;

const float mass = 1.88; // kg
const float J = 0.02; // kg*m^2
const float L = 0.0866; // m
const float g = 9.81; // m/s^2

int DesiredPWM = 0;
float desiredVoltage = 0;

void setup() {
  Serial.begin(115200);  // Serial Monitor

  // Pins setup
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorTourqePin, OUTPUT);
  pinMode(ANG_pin, INPUT);

  digitalWrite(motorEnablePin, HIGH);
  digitalWrite(motorDirectionPin, LOW);
  analogWrite(motorTourqePin, 0);

  analogWriteResolution(12);

  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  targetAngle = EncoderAngle();
  targetAngleRad = targetAngle*deg2rad;
  currentAngle = EncoderAngle();
  currentAngleRad = currentAngle*deg2rad;
  error = targetAngle-currentAngle;

 

}

void loop() {
  // Time
  unsigned long currentTime = millis(); // Read time from internal clock
  
  float deltaTime = (currentTime - previousTime) / 1000.0; // Calculate time differenve

  float Frequency = 1.0 / deltaTime; // Calculate frequency of the system

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

  // Read and filter encoder
  float rawAngle = EncoderAngle();

  currentAngle = 0.1 * rawAngle + 0.9 * currentAngle;  // low-pass filter
  currentAngleRad = currentAngle * deg2rad;

  // Error, filtered derivative and filtered integral
  error = targetAngleRad - currentAngleRad;

  float raw_derivative = (error - error_prev) / deltaTime;
  error_vel = 0.9 * error_vel + 0.1 * raw_derivative;  // smoothed derivative

  raw_integral += 0.5*(error+error_prev)*deltaTime;
  error_int = 0.9 *error_int + 0.1 * raw_integral; // smoothed integral

  // ------------------- Controller ----------------------------------

  float GravityComp = kf*sin(targetAngleRad); // Feed forward
  float PID_out = kp*error + ki*error_int + kd*error_vel; // Feedback


  float MotorInputT = GravityComp + PID_out; // Desired torque signal for motor
  
  desiredVoltage = abs(MotorInputT);


  if (MotorInputT>=0){
    digitalWrite(motorDirectionPin,HIGH);
  }
  else {
    digitalWrite(motorDirectionPin,LOW);
  }

  DesiredPWM = constrain((desiredVoltage / V_max) * maxPWM, 0, maxPWM);
  if (DesiredPWM > PWM_deadZone-10){
    DesiredPWM = constrain(DesiredPWM, PWM_deadZone+5, maxPWM);
  }
  
  analogWrite(motorTourqePin,DesiredPWM);
  digitalWrite(motorEnablePin, HIGH);
  

  // Update prevoious values
  error_prev = error;
  previousTime = currentTime;
  u_prev = MotorInputT;
  MotorInputT_prev = MotorInputT;

// COM message for plotting 
  String output = "Time: " + String(currentTime) + 
                "ms, Target Angle: " + String(targetAngle) +
                ", Current Angle: " + String(currentAngle) +
                ", Torque: " + String(DesiredPWM)+
                ", PWM: " + String(DesiredPWM)+
                ", sampling Frequency: " + String(Frequency)+
                ", Gravity Comp: " + String(GravityComp)+
                ", PID: " + String(PID_out);
                // ", Observer: " + String(u);
              //  ", kp: " + String(Kp);
              //  ", kd: " + String(Kd);
               
Serial.println(output);

// DEBUG

// Serial.print(GravityComp);
//   Serial.print(" ");
//   Serial.print(PID_out);
//   Serial.print(" ");
//   Serial.println(u);
//   Serial.print(" ");
  // Serial.println(error);
  // Serial.println(speed);
  // Serial.println(pwmValue3);
  // Serial.println(V/V_max);
  // Serial.print(I);
  // Serial.print(";");
  // Serial.println(MotorInputT);
  // Serial.println(gearRatio);
  // Serial.println(ratio_tor);
  // Serial.println(Tfb);
  // Serial.println(Tff);
  // Serial.println(Kp);
  // Serial.println(Kd);
  // Serial.println(f);
  // Serial.println(beeta);
  // Serial.println(deltaTime);
  // Serial.println(f);
  // Serial.println(MotorInputT);
  // Serial.println(MotorInputT);
  // Serial.println(MotorInputT);
  // Serial.println(MotorInputT);
  // Serial.println(direction);

}


float EncoderAngle(){
  // Read encoder 
  int sensorValue = analogRead(ANG_pin);
  // Calculate the angle
  float readAngle = -0.3315*sensorValue + 200.9-20;
  // float readAngle = sensorValue; // Change to this line for calibration
  return readAngle;
}
