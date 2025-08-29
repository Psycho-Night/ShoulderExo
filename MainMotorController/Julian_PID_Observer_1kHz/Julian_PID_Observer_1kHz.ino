// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// PID Controller with Observer v2 (1ms Sampling)
// Optimized for Arduino Due using Hardware Timer Interrupt
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//--------------------- PID ------------------------------------------------------------
float kf = 1.75;
float kp = 1.3;
float kd = 0.5;
float ki = 0.8;

float error = 0;
float error_prev = 0;
float error_vel = 0;
float error_int = 0;
float raw_integral = 0;

//----------- SIN WAVE ---------------------------------------------------------------
const float frequency = 0.25;
const float amplitude = 20.0;
const float offset = 20.0;

const float frequency_T = 0.5;
const float amplitude_T = 7.2 / 10;
const float offset_T = 7.2 / 10;

//------------ ESCON ---------------------------------------------------------------
const int motorEnablePin = 2;
const int motorDirectionPin = 4;
const int motorTourqePin = 6;

const int ANG_pin = A0;

const float V_max = 5.0;
const int maxPWM = 4095;
float desiredCurrent = 0;
const float maxCurrent = 4.24;
const float Kt = 21.3 / 1000;

//------------- Parameters -----------------------------------------------------------
float targetAngle = 0;
float targetPWM = 0;
const float deg2rad = PI / 180;
float targetAngleRad = targetAngle * deg2rad;

float currentAngle = 0;
float currentAngleRad = 0;
float currentAngle_prev = 0;

float MotorInputT_prev = 0;
float u_prev = 0;
float direction = 0;

const int gearRatio = 70;
const float mass = 1.88;
const float J = 0.02;
const float L = 0.0866;
const float g = 9.81;

int DesiredPWM = 0;

const int Num_Samples = 2;
float T_Samples[Num_Samples];
float T_Sum = 0;
int T_index = 0;
float u_filtered = 0;

// --------- CHIRP SIGNAL -------------------
const float k = 1.025;
const float T = 1.0 / 1.0;
const float f_0 = 0.01;

//------------ TIME CONTROL -----------------
volatile bool controlFlag = false;
volatile unsigned long currentTime = 0;   // ms timestamp from timer

// ***************** TIMER INTERRUPT HANDLER *****************
void TC0_Handler() {
  TC_GetStatus(TC0, 0);   // Clear interrupt flag
  controlFlag = true;     // Signal control loop
  currentTime++;          // Increment ms counter
}

// ***************** TIMER SETUP FUNCTION *****************
void setupTimer1ms() {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);

  // Configure timer on channel 0
  TC_Configure(TC0, 0,
               TC_CMR_TCCLKS_TIMER_CLOCK4 | // MCK/128
               TC_CMR_WAVE |                // Waveform mode
               TC_CMR_WAVSEL_UP_RC);        // Count up to RC

  uint32_t rc = VARIANT_MCK / 128 / 1000;   // 1ms interrupt rate
  TC_SetRC(TC0, 0, rc);

  TC_Start(TC0, 0);
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;  // Enable interrupt
  NVIC_EnableIRQ(TC0_IRQn);
}

// ***************** SETUP *****************
void setup() {
  Serial.begin(115200);

  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorTourqePin, OUTPUT);
  pinMode(ANG_pin, INPUT);

  digitalWrite(motorEnablePin, HIGH);
  digitalWrite(motorDirectionPin, LOW);
  analogWrite(motorTourqePin, 0);

  analogWriteResolution(12);

  for (int i = 0; i < Num_Samples; i++) {
    T_Samples[i] = 0;
  }

  targetAngle = EncoderAngle();
  targetAngleRad = targetAngle * deg2rad;
  currentAngle = EncoderAngle();
  currentAngleRad = currentAngle * deg2rad;
  error = targetAngle - currentAngle;

  setupTimer1ms();
}

// ***************** MAIN LOOP *****************
void loop() {
  if (controlFlag) {
    controlFlag = false;   // Reset flag

    float deltaTime = 0.001; // Always 1ms now, no drift
    float Frequency = 1.0 / deltaTime;  // Will always be 1000 Hz

    // Generate target trajectory
    if (currentTime < 2000) {
      targetAngle = 0;
      targetAngleRad = targetAngle * deg2rad;
    } else {
      targetAngle = offset + amplitude * sin(2.0 * PI * frequency * (currentTime / 1000.0) - (3.0 * PI / 4.0));
      targetAngleRad = targetAngle * deg2rad;
    }

    // Read encoder and filter
    float rawAngle = EncoderAngle();
    currentAngle = 0.1 * rawAngle + 0.9 * currentAngle;
    currentAngleRad = currentAngle * deg2rad;

    // Compute error and derivatives
    error = targetAngleRad - currentAngleRad;
    float raw_derivative = (error - error_prev) / deltaTime;
    error_vel = 0.9 * error_vel + 0.1 * raw_derivative;
    raw_integral += 0.5 * (error + error_prev) * deltaTime;
    error_int = 0.9 * error_int + 0.1 * raw_integral;

    // ------------------- PID + OBSERVER -------------------
    float GravityComp = kf * sin(targetAngleRad);
    float PID_out = kp * error + ki * error_int + kd * error_vel;
    float u = L * mass * g * sin(currentAngleRad) + u_prev;

    // Filter observer torque
    T_Sum -= T_Samples[T_index];
    T_Samples[T_index] = u;
    T_Sum += T_Samples[T_index];
    T_index++;
    if (T_index >= Num_Samples) T_index = 0;
    u_filtered = T_Sum / Num_Samples;

    // Generate motor torque command (chirp example)
    float MotorInputT = offset_T + amplitude_T * sin(2.0 * PI * f_0 * ((T * pow(k, ((currentTime / 1000.0) - 1) / log(k)))));
    desiredCurrent = abs((MotorInputT / gearRatio) / Kt);

    // Control motor direction and PWM
    DesiredPWM = constrain((desiredCurrent / maxCurrent) * maxPWM, 0, maxPWM);
    if (MotorInputT >= 0) {
      digitalWrite(motorDirectionPin, HIGH);
    } else {
      digitalWrite(motorDirectionPin, LOW);
    }
    analogWrite(motorTourqePin, DesiredPWM);
    digitalWrite(motorEnablePin, HIGH);

    // Save states
    error_prev = error;
    u_prev = MotorInputT;
    MotorInputT_prev = MotorInputT;

    // Output data for logging — non-blocking
    Serial.print("t:"); Serial.print(currentTime);
    Serial.print(", Target:"); Serial.print(targetAngle);
    Serial.print(", Current:"); Serial.print(currentAngle);
    Serial.print(", Torque:"); Serial.print(MotorInputT);
    Serial.print(", PWM:"); Serial.print(DesiredPWM);
    Serial.print(", Freq:"); Serial.print(Frequency);
    Serial.print(", PID:"); Serial.print(PID_out);
    Serial.print(", Observer:"); Serial.println(u);
  }
}

// ***************** ENCODER FUNCTION *****************
float EncoderAngle() {
  int sensorValue = analogRead(ANG_pin);
  float readAngle = -0.5087 * sensorValue + 190.35;
  return readAngle;
}
