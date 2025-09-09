// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Maxon Motor - Dead Zone Test v1.0

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//------------ ESCON ---------------------------------------------------------------

const int motorEnablePin = 2;     // Enable (Digital Input 2, High Active)
const int motorDirectionPin = 4;  // Direction (Digital Input 4, High Active)
const int motorTourqePin = 6;   // PWM Set Value (Analog Input 1)
// const int motorTourqePin = DAC1;   // PWM Set Value (Analog Input 1)

int DesiredPWM = 0;
const int MaxPWM = 4095;
int NewPWM = 0;


void setup() {
  Serial.begin(115200);  // Serial Monitor

  // Pins setup
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorTourqePin, OUTPUT);


  digitalWrite(motorEnablePin, HIGH);
  digitalWrite(motorDirectionPin, HIGH);
  analogWrite(motorTourqePin, 0);

  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  analogWriteResolution(12);

}

void loop() {
  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read input until newline
    NewPWM = input.toInt();           // Convert input to a float
      if (0 <= NewPWM && NewPWM <= 4095) { // Validate the input range
        DesiredPWM = NewPWM;
        analogWrite(motorTourqePin, DesiredPWM);
        Serial.print("PWM set to: ");
        Serial.println(DesiredPWM);
        // if (DesiredPWM == 0) {
        //   // Force output to absolute 0 V when stopped
        //   digitalWrite(motorTourqePin, LOW);
        // } else {
        //   analogWrite(motorTourqePin, DesiredPWM);
        // }
      }
  
      else {
        Serial.println("Invalid PWM! Enter a value between 0 and 4095.");
  }}

}
