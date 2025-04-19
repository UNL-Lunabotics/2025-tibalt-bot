#include "RoboClaw.h"

#define ROBOCLAW_ADDRESS 0x80

//Commands:
// screen /dev/ttyACM0 115200
// start

SoftwareSerial serialDt(18, 19);  //TODO
RoboClaw rcDt(&serialDt, 10000);

SoftwareSerial serialHopLift(20, 21); //TODO
RoboClaw rcHopLift(&serialHopLift, 10000);

//TODO: hopper servo latch

bool runLoop = false;

void setup() {
  Serial.begin(9600);

  rcDt.begin(38400);
  rcDt.begin(38400);


  //TODO: just here for testing things idk
  // rcDt.ForwardBackwardM1(ROBOCLAW_ADDRESS, 70);
  // rcDt.ForwardBackwardM2(ROBOCLAW_ADDRESS, 48);
  // delay(1000);
  // rcDt.ForwardBackwardM1(ROBOCLAW_ADDRESS, 64);
  // rcDt.ForwardBackwardM2(ROBOCLAW_ADDRESS, 64);
  // delay(1000);
  // rcHopLift.ForwardBackwardM1(ROBOCLAW_ADDRESS, 100);
  // delay(1000);
  // //TODO: open latch
  // delay(1000);
  // rcHopLift.ForwardBackwardM1(ROBOCLAW_ADDRESS, 28);
  // delay(1000);
  // //TODO: close latch
  // delay(1000);
  // rcHopLift.ForwardBackwardM1(ROBOCLAW_ADDRESS, 100);
  // delay(1000);
  // //TODO: open latch
  // delay(1000);
  // rcHopLift.ForwardBackwardM1(ROBOCLAW_ADDRESS, 28);
  // delay(1000);
  // //TODO: close latch
  // delay(1000);
  // rcHopLift.ForwardBackwardM1(ROBOCLAW_ADDRESS, 64);


  // Wait until we get a "start" signal over Serial
  while (!runLoop) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      Serial.println(input);

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
    rcDt.ForwardBackwardM1(ROBOCLAW_ADDRESS, 64);
    rcDt.ForwardBackwardM2(ROBOCLAW_ADDRESS, 64);
    rcHopLift.ForwardBackwardM1(ROBOCLAW_ADDRESS, 64);
  }
}
