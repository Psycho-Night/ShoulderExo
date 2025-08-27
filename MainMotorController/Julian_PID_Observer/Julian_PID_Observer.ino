// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// PID controller with observer v1

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//--------------------- PID ------------------------------------------------------------
// RECORD CHANGES !!!!!

float kf = 1.75;      //  sin- 4.0 ; 35 - 4.5;
float kp = 1.15;      // 1.15 Good ->0.95 0.75 sin- 2.75; 35 - 0.6;
float kd = 0.08;       //  sin- 0.25; 35 - 0.25;
float ki = 0.55;      // 0.75 sin- 0.70; 35 - 0.22;

float error = 0;
float error_prev = 0;
float error_vel = 0;
float error_int = 0;
float raw_integral = 0;

//----------- SIN WAVE ---------------------------------------------------------------

const float frequency = 0.05*4;  // Frequency in Hz (adjust as needed)
const float amplitude = 35.0; // Amplitude (max deviation from midpoint)
const float offset = 35.0;    // Offset/midpoint of the sine wave

//------------ TIME ---------------------------------------------------------------

long previousTime = 0; // For time calcualtion
long timeOff = 0;

//------------ ESCON ---------------------------------------------------------------

const int motorEnablePin = 2;     // Enable (Digital Input 2, High Active)
const int motorDirectionPin = 4;  // Direction (Digital Input 4, High Active)
const int motorTourqePin = 6;   // PWM Set Value (Analog Input 1)

const int ANG_pin = A0;   // Shaft encoder pin def A0

const float V_max = 5.0;           // Max command voltage (e.g. driver input 0-5V)
const int maxPWM = 4095;            // 12-bit PWM range
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

const int Num_Samples = 2;    // Number of sumples for filter
float T_Samples[Num_Samples];  // Store last 50 Motor inputs
float T_Sum = 0;               // Sum of Tourques
int T_index = 0;                 // Current index for the buffer
float u_filtered = 0; // Filtered output

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

   // Initialize all samples to zero
  for (int i = 0; i < Num_Samples; i++) {
    T_Samples[i] = 0;
  }
}


void loop() {
  // Time
  long currentTime = millis(); // Read time from internal clock
  
  float deltaTime = (currentTime - previousTime) / 1000.0; // Calculate time differenve

  float Frequency = 1.0 / deltaTime; // Calculate frequency of the system


  // Target
  if (currentTime/1000<2) { // Modify to start faster
    targetAngle = 0; //
    targetAngleRad = targetAngle * deg2rad;
   }
  if (currentTime/1000<5 && currentTime>=2){
    targetAngle = 0; //
    targetAngleRad = targetAngle * deg2rad;
  }
  else {
    targetAngle = offset + amplitude * sin(2 * PI * frequency * currentTime/1000+PI/4); // Sine wave
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
  float u = L*mass*g*sin(currentAngleRad) + u_prev; // Observer
  
  
  T_Sum -= T_Samples[T_index];      // Subtract oldest value from sum

  T_Samples[T_index] = u; // Store new value

  T_Sum += T_Samples[T_index];        // Add new value to sum

  T_index++;

  if (T_index >= Num_Samples){
    T_index = 0; // Wrap around buffer 
  }


  u_filtered = T_Sum/Num_Samples; // Calculate the average 

  float MotorInputT = GravityComp + PID_out - u_filtered*0; // Desired torque signal for motor

  
  desiredCurrent = abs((MotorInputT/gearRatio) / Kt);

  //  Calcualte the direftion of Motor based on the direction of torque
  if (MotorInputT>=0){
    DesiredPWM = constrain((desiredCurrent / maxCurrent) * maxPWM, 0, maxPWM);
    digitalWrite(motorDirectionPin,HIGH);
    analogWrite(motorTourqePin,DesiredPWM);
    digitalWrite(motorEnablePin, HIGH);
  }
  else {
    DesiredPWM = constrain((desiredCurrent / maxCurrent) * maxPWM, 0, maxPWM);
    digitalWrite(motorDirectionPin,LOW);
    analogWrite(motorTourqePin,DesiredPWM);
    digitalWrite(motorEnablePin, HIGH);

  }
  // Update prevoious values
  error_prev = error;
  previousTime = currentTime;
  u_prev = MotorInputT;
  MotorInputT_prev = MotorInputT;

// COM message for plotting 
  String output = "Time: " + String(currentTime) + 
                "ms, Target Angle: " + String(targetAngle) +
                ", Current Angle: " + String(currentAngle) +
                ", Torque: " + String(MotorInputT)+
                ", PWM: " + String(DesiredPWM)+
                ", sampling Frequency: " + String(Frequency)+
                ", Gravity Comp: " + String(GravityComp)+
                ", PID: " + String(PID_out)+
                ", Observer: " + String(u);
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
  float readAngle = -0.5087*sensorValue + 190.35;
  // float readAngle = sensorValue; // Change to this line for calibration
  return readAngle;
}
