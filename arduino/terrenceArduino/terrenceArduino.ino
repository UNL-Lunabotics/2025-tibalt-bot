#include "RoboClaw.h"
#include <Servo.h>

#define ROBOCLAW_ADDRESS 0x80

//Commands:
// screen /dev/ttyACM0 115200
// start

Servo dt[4];

RoboClaw rcHopLift(&Serial2, 10000);

//TODO: hopper servo latch

bool runLoop = false;

void setup() {
  Serial.begin(9600);


  dt[0].attach(2);
  dt[1].attach(4);
  dt[2].attach(6);
  dt[3].attach(8);
  int dtForward = 50 * 5 + 1500;
  int dtBackward = -50 * 5 + 1500;
  int dtStop = 0 * 5 + 1500;
  rcHopLift.begin(38400);


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
  dt[0].writeMicroseconds(dtBackward);
  dt[1].writeMicroseconds(dtBackward);
  dt[2].writeMicroseconds(dtForward);
  dt[3].writeMicroseconds(dtForward);
  delay(10000);
  dt[0].writeMicroseconds(dtForward);
  dt[1].writeMicroseconds(dtForward);
  dt[2].writeMicroseconds(dtBackward);
  dt[3].writeMicroseconds(dtBackward);
  delay(10000);
  dt[0].writeMicroseconds(dtStop);
  dt[1].writeMicroseconds(dtStop);
  dt[2].writeMicroseconds(dtStop);
  dt[3].writeMicroseconds(dtStop);
  delay(1000);
  rcHopLift.ForwardBackwardM2(ROBOCLAW_ADDRESS, 0);
  delay(30000);
  //TODO: open latch
  // delay(1000);
  rcHopLift.ForwardBackwardM2(ROBOCLAW_ADDRESS, 127);
  delay(30000);
}
