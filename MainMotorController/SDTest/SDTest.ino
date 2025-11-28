// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Test for use of SD card in a project + signal generation + logging
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// ================ 500 Hz SYSTEM ===================================================
IntervalTimer sysTimer;
volatile bool tickFlag = false;
volatile uint32_t tickCount = 0;
const int TimerTickNo = 2000;   // 500 Hz

// ========================= SIN WAVE ==============================================
float frequency   = 0.05;
float amplitude   = 35.0;
float offset      = 35.0;
float phaseShift  = -PI/2;
unsigned long RampTime = 2;
unsigned long StartTime = 0;

// ======================== TIME ====================================================
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long InitDelay = 2;

// ------------------------- Sensors (dummy for now) -------------------------------
float EncoderAngle() { return 33.33f; }  
float currentAngle = 33.33f;
float targetAngle = 0.0f;
float targetAngleRad = 0.0f;
const float deg2rad = PI/180.0f;

// =========================== DATA LOGGING CONTROL ================================
File logFile;
const char *fileName = "log.bin";

// Stop after N seconds of signal generation:
const float LOG_DURATION = 60.0f;   // <<< change if needed
bool loggingFinished = false;

// =========================== COMPUTER COMMUNICATION ===============================
struct __attribute__((packed)) Frame {
  uint16_t header;          
  uint32_t t;               
  float targetA;
  float currentA;
  float torque;
  float t_ff;
  float t_fb;
  float desiredI;
  float actualI;
  float freq;
  uint16_t padding;
};

// =========================== Functions ===========================================
void sysTickISR() {
  tickFlag = true;
  tickCount++;
}



// ================================================================================
//                                   SETUP
// ================================================================================
void setup() {
  Serial.begin(1000000);
  delay(500);

  // ------------------ SD card ---------------------
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD INIT FAIL!");
    while (1);
  }
  Serial.println("SD OK");

  // Open logging file (overwrite)
  logFile = SD.open(fileName, FILE_WRITE);
  if (!logFile) {
    Serial.println("ERROR OPENING LOG FILE!");
    while (1);
  } else {
    // Clear file
    logFile.truncate(0);
  }

  sysTimer.begin(sysTickISR, TimerTickNo);
  Serial.println("READY");
}

// ================================================================================
//                                   MAIN LOOP
// ================================================================================
void loop() {

  if (loggingFinished) return;

  if (tickFlag) {
    tickFlag = false;
    currentTime = micros();

    if (StartTime == 0) StartTime = currentTime;

    float deltaTime = (currentTime - previousTime) / 1000000.0f;
    // float SysFreq = 1.0f / deltaTime;
    float SysFreq = (deltaTime > 0) ? (1.0f / deltaTime) : 0.0f;
    float elapsedTime = (currentTime - StartTime) / 1000000.0f;

    // ------------------ Trajectory ------------------
    if (elapsedTime < InitDelay) {
      targetAngle = EncoderAngle();
    } else {
      float SinTime = elapsedTime - InitDelay;
      float sinWave = offset + amplitude * sin(2.0f * PI * frequency * SinTime + phaseShift);
      float ramp = constrain(SinTime / RampTime, 0.0f, 1.0f);
      targetAngle = (1.0f - ramp) * currentAngle + ramp * sinWave;
    }

    // ------------------ Fill Frame ------------------
    Frame frame;
    frame.header = 0xAA55;
    frame.t = currentTime;
    frame.targetA = targetAngle;
    frame.currentA = 3.3333;
    frame.torque = 4.4444;
    frame.t_ff = 5.5555;
    frame.t_fb = 6.6666;
    frame.desiredI = 7.7777;
    frame.actualI = 8.8888;
    frame.freq = SysFreq;
    frame.padding = 0xBB66;

    // ------------------ Save to SD ------------------
    logFile.write((uint8_t*)&frame, sizeof(Frame));

    previousTime = currentTime;

    // ------------------ Stop Condition ------------------
    if (elapsedTime >= LOG_DURATION) {

      loggingFinished = true;
      logFile.close();

      Serial.println("\nLOGGING COMPLETE. READING BACK:\n");

      // ------------------ Read back & print ------------------
      File f = SD.open(fileName, FILE_READ);
      if (!f) {
        Serial.println("ERROR RE-OPENING LOG FILE!");
        return;
      }

      Frame rd;
      while (f.read((uint8_t*)&rd, sizeof(Frame)) == sizeof(Frame)) {
        Serial.print("t=");
        Serial.print(rd.t);
        Serial.print("  target=");
        Serial.print(rd.targetA);
        Serial.print(" Test value");
        Serial.print(rd.desiredI);
        Serial.print(" padding");
        Serial.print(rd.padding);
        Serial.print("  freq=");
        Serial.println(rd.freq);
    
      }

      f.close();
      Serial.println("\nDONE.");
    }
  }
}

