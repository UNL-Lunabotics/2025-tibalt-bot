#include "RoboClaw.h"

#define ROBOCLAW_ADDRESS 0x80

SoftwareSerial serialTest(18, 19);  // TX=18, RX=19
RoboClaw roboclaw(&serialTest, 10000);

bool runLoop = false;

void setup() {
  serialTest.begin(9600);
  roboclaw.begin(38400);
  Serial.begin(115200);  // Serial from Jetson or PC

  // Wait until we get a "start" signal over Serial
  while (!runLoop) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input == "start") {
        runLoop = true;
        Serial.println("Starting autonomous loop.");
        delay(100); // Optional short delay before starting
      }
    }
    delay(100);
  }
}

void loop() {
  if (runLoop) {
    roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS, 127);  // Full forward
    delay(100); // Adjust delay to control loop rate if needed
  }
}
