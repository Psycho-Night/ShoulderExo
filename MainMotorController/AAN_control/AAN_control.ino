// AAN

const float alpha = 0.5;
const float a = 0.00001f;
const float b = 100.0;
const float beta = 9.0;

float error = 0;
float error_prev = 0;
float error_vel = 0;

long Tff = 0;
long Tff_prev = 0;
long Tfb = 0;

long Kcn_fn = 0;

float Kp = 0;
float Kd = 0;
float f = 0;
float beeta = 0;
float eps = 0;

float input_angle;

float Gains[2] = {0.0, 0.0};

// SIN WAVE

const float frequency = 0.2;  // Frequency in Hz (adjust as needed)
const float amplitude = 20.0; // Amplitude (max deviation from midpoint)
const float offset = 20.0;    // Offset/midpoint of the sine wave

// TIME

long previousTime = 0; // For time calcualtion
long timeOff = 0;

// ESCON

const int motorEnablePin = 2;     // Enable (Digital Input 2, High Active)
const int motorDirectionPin = 4;  // Direction (Digital Input 4, High Active)
const int motorSpeedPin = 6;   // PWM Set Value (Analog Input 1)

// Parameters
const float V_max = 5.0;           // Max command voltage (e.g. driver input 0-5V)
const int maxPWM = 255;            // 8-bit PWM range
const float desiredCurrent = 0;  // Desired current in A
const float maxCurrent = 4.24;      // Max allowed current for driver

const int ANG_pin = A0;   // Shaft encoder pin def A0
const int hallPins1[] = { A3, A4, A5 };



  // CONVERTION
float targetAngle = 0;
const float deg2rad = PI/180;               // Degrees to rad
float targetAngleRad = targetAngle*deg2rad; // Target anlge in rads
const int gearBox = 5.4;
const int wormGear = 50;
const int gearRatio = 2454;

float currentAngle = 0;
float currentAngleRad = 0;
float currentAngle_prev = 0;
float currentAngleRad_prev = 0;

float direction = 0;




void setup() {
  Serial.begin(115200);  // Serial Monitor

  // Set pin modes
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT);
  pinMode(ANG_pin, INPUT);

  digitalWrite(motorEnablePin, HIGH);
  digitalWrite(motorDirectionPin, LOW);
  analogWrite(motorSpeedPin, LOW);

  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  Serial.println("Motor Controller Ready.");
  Serial.println("Enter target position in degrees.");

  // Attach interrupts for motor 1 sensors
  // for (int pin : hallPins) attachInterrupt(digitalPinToInterrupt(pin), stateChangedMotor1, CHANGE);

  targetAngle = EncoderAngle();
  targetAngleRad = targetAngle*deg2rad;
  currentAngle = EncoderAngle();
  currentAngleRad = currentAngle*deg2rad;
  error = targetAngle-currentAngle;

}

void loop() {
  
    // Time

  //long currentTime = millis();
  //long deltaTime = ((float) (currentTime-previousTime))/1000;

  long currentTime = millis(); 
  float deltaTime = (currentTime - previousTime) / 1000.0;

 
  // Check if any data is available
  //if (Serial.available() > 0) {
    // Read the value as a float
    //targetAngle = Serial.parseFloat();
    //Serial.print("Received target angle: ");
    //Serial.println(targetAngle, 2);
  //}


if (Serial.available() > 0) {
    // input_angle = Serial.parseInt(); 
    // targetAngle = input_angle;
    String input_angle = Serial.readStringUntil('\n');
    targetAngle = input_angle.toFloat();
    //Serial.println(received);
  }

  // targetAngle = input_angle;//offset + amplitude * sin(2 * PI * frequency * currentTime/1000); // Sine wave
  //targetAngle = offset + amplitude * sin(2 * PI * frequency * currentTime/1000); // Sine wave
  targetAngleRad = targetAngle * deg2rad;



  currentAngle = EncoderAngle();

  long currentAngleRad = currentAngle*deg2rad;

  error = targetAngleRad-currentAngleRad;          // Current error angle
  error_vel = (error-error_prev)/deltaTime;  // Derivative error

 

////////////////////////////////////////////////////

  //long eps = error+beta*error_vel;
  //long beeta = a/(1+b*eps*eps);
  //long f = eps/beeta;
  //Kp = f*error;
  //Kd = f*error_vel;

//////////////////////////////////////////////////////

  Tff = Tff_fcn(Tff_prev, alpha, error_prev);
  Tff_prev = Tff;

  //Kcn_fn = K_fcn(error, error_vel, a, b, beta, &Kp, &Kd);

   K_fcn(error, error_vel, a, b, beta, &Kp, &Kd);

  Tfb = Tfb_fcn(error_vel, error, Kp, Kd);

  long MotorInputT = Tff+Tfb;
  //long MotorInputT = 2+Tfb;

  float Kt = 21.3/1000;           // Nm/A (example value)
  float V_max = 24.0;        // Max supply voltage
  float R = 0.465 ; //resistance in ohms
  float maxPWM = 255.0;
  float ratio_tor = MotorInputT/gearRatio;
  float I = (MotorInputT/gearRatio) / Kt;          // Required current in A
  long V = I * R;          // Simplified: back-calculate V as if R = 1
  float I_max = maxCurrent;

  float desiredCurrent = I;
  if (I > maxCurrent) desiredCurrent = maxCurrent;
  if (I < -maxCurrent) desiredCurrent = -maxCurrent;



digitalWrite(motorEnablePin, HIGH);
float error2 = targetAngle - currentAngle;


if (error2 < 0) { // 1 degree tolerance
digitalWrite(motorDirectionPin, LOW);
//sendCurrentCommand(desiredCurrent);
currentToPWM(desiredCurrent, I_max, maxPWM);
//analogWrite(motorSpeedPin, 255);
} else {
digitalWrite(motorDirectionPin, HIGH);
//sendCurrentCommand(desiredCurrent);
currentToPWM(desiredCurrent, I_max, maxPWM);
 //analogWrite(motorSpeedPin, 255);
}



  error_prev = error;
  previousTime = currentTime;

String output = "Time: " + String(currentTime) + 
                "ms, Target Angle: " + String(targetAngle) +
                ", Current Angle: " + String(currentAngle) +
                ", Torque: " + String(MotorInputT);

Serial.println(output);



  // Serial.print(currentTime);
  // Serial.print(";");
  // Serial.print(targetAngle);
  // Serial.print(";");
  // Serial.print(currentAngle);
  // Serial.print(";");
  //Serial.println(error);
  //Serial.println(pwmValue);
  //Serial.println(pwmValue3);
  //Serial.println(V/V_max);
  //Serial.print(I);
  // Serial.print(";");
  // Serial.println(MotorInputT);
  //Serial.println(gearRatio);
  //Serial.println(ratio_tor);
  //Serial.println(Tfb);
  //Serial.println(Tff);
  //Serial.println(Kp);
  //Serial.println(Kd);
  //Serial.println(f);
  //Serial.println(beeta);
  //Serial.println(deltaTime);
  //Serial.println(f);
  //Serial.println(MotorInputT);
  //Serial.println(MotorInputT);
  //Serial.println(MotorInputT);
   //Serial.println(MotorInputT);
  //Serial.println(direction);
   //delay (100);


}



float EncoderAngle(){
  // Read encoder on joint and calculate the angle
  int sensorValue = analogRead(ANG_pin);
  // float targetAngle = -0.2341*sensorValue + 280.94;
  float readAngle = -0.3315*sensorValue + 200.9;
  //float targetAngle = sensorValue;
  return readAngle;
}


float Tff_fcn(float Tff_prev, float alpha, float error_prev) {
  return Tff_prev + alpha * error_prev;
}

void K_fcn(float error, float error_vel, float a, float b, float beta, float* Kp, float* Kd){
  float eps = error+beta*error_vel;
  float beeta = a/(1+b*eps*eps);
  float f = eps/beeta;
  *Kp = f*error;
  *Kd = f*error_vel;
}

float Tfb_fcn(float error_vel, float error, float Kp, float Kd){
  float Tfb = error*Kp+error_vel*Kd;
  return Tfb;
}
// Function to send current command as PWM
//void sendCurrentCommand(float current) {
  // Clamp current to max
  //current = constrain(current, 0, maxCurrent);

  // Map current to voltage
 //float V_cmd = (current / maxCurrent) * V_max;

  // Map voltage to PWM
  //int pwmValue = V_cmd / V_max * maxPWM;
  //pwmValue = constrain(pwmValue, 0, maxPWM);

 // analogWrite(motorEnablePin, pwmValue);
//}

// Function to send current command as PWM
// ---- Function ----
float currentToPWM(float desiredCurrent, float I_max, int maxPWM) {
  return (desiredCurrent / I_max) * maxPWM;
}