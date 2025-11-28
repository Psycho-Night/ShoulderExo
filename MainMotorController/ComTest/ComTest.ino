#include <Arduino.h>

struct Frame {
  uint16_t header;   // Always 0xAA55
  float currentA;    // Example: angle [deg]
  float actualI;     // Example: current [A]
  float receivedTau; // Example: torque or whatever
};

void SendFrame(const Frame &frame) {
  Serial.write((uint8_t *)&frame, sizeof(frame));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for USB serial connection
  }
  Serial.println("Teensy ready");
}

void loop() {
  const uint16_t HEADER_FROM_PC = 0x55AA;
  const uint16_t HEADER_TO_PC   = 0xAA55;

  // Wait for frame from PC (6 bytes)
  if (Serial.available() >= 6) {
    uint8_t buf[6];
    Serial.readBytes(buf, 6);

    uint16_t header = buf[0] | (buf[1] << 8);
    if (header == HEADER_FROM_PC) {
      float inputVal;
      memcpy(&inputVal, &buf[2], sizeof(float));

      // Create a response frame
      Frame frame;
      frame.header = HEADER_TO_PC;
      frame.currentA = inputVal;   // Example math
      frame.actualI = 2.222;            // Echo the received float
      frame.receivedTau = 3.3333; // Another arbitrary calculation

      SendFrame(frame);
    }
  }
}

