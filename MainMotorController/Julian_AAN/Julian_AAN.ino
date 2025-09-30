// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// AAN controller - Maxon Current controller v1.0 Written for Arduino Due

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//--------------------- AAN ------------------------------------------------------------

const float alpha = 2.0;  // woriking 2.0 or 3.5 if Freqency is 100Hz
const float beta = 0.05;  // Keep it 0.05


// Tune those and keep changes
const float a = 2.0;      // working 2.0 or 3.5 if Freqency is 100Hz
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

const float frequency = 0.05;            // Frequency in Hz (adjust as needed)
const float amplitude = 35.0;           // Amplitude (max deviation from midpoint)
const float offset = 35.0;              // Offset/midpoint of the sine wave
const long InitialisationDelay = 2000;  // Delay used for initialisation
const float phaseShift = -PI/2;          // Phase shift for graduate raise from 0 deg -PI/2 if sin wave and 0 if step

//------------ TIME ---------------------------------------------------------------

long previousTime = 0; // For time calcualtion
long timeOff = 0;

//------------ ESCON ---------------------------------------------------------------
// Motor control
const int motorEnablePin = 2;       // Enable (Digital Input 2, High Active)
const int motorDirectionPin = 4;    // Direction (Digital Input 4, High Active)
const int motorTourqePin = 6;       // PWM Set Value (Analog Input 1)

// Sensors
const int ANG_pin = A0;             // Shaft encoder pin def A0
const int ESCON_OUT_pin = A8;               // ESCON speed output

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

void setup() {
  Serial.begin(115200);  // Serial Monitor

  // Pins setup
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorTourqePin, OUTPUT);
  pinMode(ANG_pin, INPUT);
  pinMode(ESCON_OUT_pin, INPUT);

  digitalWrite(motorEnablePin, HIGH);
  digitalWrite(motorDirectionPin, LOW);
  analogWrite(motorTourqePin, 0);

  analogReadResolution(12);
  analogWriteResolution(12);

  targetAngle = EncoderAngle();
  targetAngleRad = targetAngle*deg2rad;
  currentAngle = EncoderAngle();
  currentAngleRad = currentAngle*deg2rad;
  error = targetAngle-currentAngle;

}

void loop() {

  // Time
  long currentTime = millis(); // [ms]
  float deltaTime = (currentTime - previousTime) / 1000.0; // [s]
  float Frequency = 1.0 / deltaTime; // Frequency of the system [Hz]

  // Target
  if (currentTime<InitialisationDelay) { // Modify to start faster
    targetAngle = EncoderAngle();
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
  currentAngleRad = currentAngle * deg2rad; // [rad]

  // Filter derivative
  error = targetAngleRad - currentAngleRad; // [rad]

  float raw_derivative = (error - error_prev) / deltaTime; // [rad/s]
  error_vel = 0.9 * error_vel + 0.1 * raw_derivative;  // smoothed derivative

  // ------------------- AAN Controller ----------------------------------
  
  //  Torque - Feed forward (FF)
  T_ff = T_ff_prev + alpha*error_prev;
  
  //  Torque - Feedback (FB)

  epsilon = error + beta*error_vel;
  betta   = a/(1+b*epsilon*epsilon);
  f = epsilon/betta;

  // Gain calculation
  float K_p = f*error;
  float K_d = f*error_vel;

  // Torque calculation
  T_fb = K_p*error+K_d*error_vel;

  // Deadzone mitigation
  Torque_old = (T_ff+T_fb); // Desired torque [Nm]
  Torque = alpha_out*Torque_old+signum(Torque_old)*up_bound; // Filtered torque [Nm]

  // ------------------- Send Value ----------------------------------


  desiredCurrent = (abs(Torque)/gearRatio) / Kt; // Convertion from Torque to Current [A]

  if (Torque>=0){
    digitalWrite(motorDirectionPin,HIGH);
  }
  else {
    digitalWrite(motorDirectionPin,LOW);
  }
  
  DesiredPWM= constrain((desiredCurrent/maxCurrent) * maxPWM, 0, maxPWM);

  analogWrite(motorTourqePin, DesiredPWM);
  digitalWrite(motorEnablePin, HIGH);

  // Read values from ESCON
  ActualCurrent = analogRead(ESCON_OUT_pin);
  ActualCurrent = ActualCurrent/4095*4.24;
  // ActualCurrent = ActualCurrent/1023*4.24;

  error_prev = error;
  previousTime = currentTime;

  // ------------------- Send COM Message ----------------------------------
  
  // COM message for plotting 
  String output = "Time: " + String(currentTime) + 
                "ms, Target Angle: " + String(targetAngle) +
                ", Current Angle: " + String(currentAngle) +
                ", T_FF: " + String(T_ff)+
                ", T_FB: " + String(T_fb)+
                ", Torque: " + String(Torque)+
                ", Target Current: " + String(desiredCurrent) +
                ", Current: " + String(ActualCurrent) +
                ", PWM: " + String(DesiredPWM)+
                ", sampling Frequency: " + String(Frequency);
                // ", PID: " + String(ESCON_out);
                // ", Observer: " + String(u);
              //  ", kp: " + String(Kp);
              //  ", kd: " + String(Kd);
               
  Serial.println(output);

  // --------------- DEBUG ---------------------------------------------------
  // Serial.print(T_ff);
  // Serial.print(" ");
  // Serial.print(T_ff_prev);
  // Serial.print(" ");
  // Serial.print(T_fb);
  // Serial.print(" ");
  // Serial.println(Torque);
  // Serial.print(" ");




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
  float readAngle = -0.0845*sensorValue + 181.16;
  // float readAngle = sensorValue; // Change to this line for calibration
  return readAngle;
}
