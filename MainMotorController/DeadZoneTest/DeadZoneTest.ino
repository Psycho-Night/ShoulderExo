// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Maxon Motor - Dead Zone Test v2.0 - Rewrote for Teensy 4.1

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//------------ ESCON ---------------------------------------------------------------

const int motorEnablePin = 1;     // Enable (Digital Input 2, High Active)
const int motorDirectionPin = 2;  // Direction (Digital Input 4, High Active)
const int motorTourqePin = 24;   // PWM Set Value (Analog Input 1)
// const int motorTourqePin = DAC1;   // PWM Set Value (Analog Input 1)

int DesiredPWM = 0;
const int MaxPWM = 4095;
int NewPWM = 0;

const int freq = 5000.0; //


void setup() {
  Serial.begin(115200);  // Serial Monitor

  // Pins setup
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorTourqePin, OUTPUT);

  analogWriteResolution(12);
  // analogWriteFrequency(motorTourqePin, freq);

  digitalWrite(motorEnablePin, HIGH);
  digitalWrite(motorDirectionPin, HIGH);
  analogWrite(motorTourqePin, 0);
  // analogWriteFrequency(uint8_t pin, float frequency)


  

  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

 

}

void loop() {
  
  if (Serial.available() > 0) {
  String input = Serial.readStringUntil('\n');  
  NewPWM = input.toInt();           

  if (0 <= NewPWM && NewPWM <= 4095) {
    DesiredPWM = NewPWM;
    analogWrite(motorTourqePin, DesiredPWM);

    Serial.print("PWM set to: ");
    Serial.println(DesiredPWM);
  } else {
    Serial.println("Invalid PWM! Enter a value between 0 and 4095.");
  }
}

}
