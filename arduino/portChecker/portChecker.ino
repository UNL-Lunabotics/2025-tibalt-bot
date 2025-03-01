// sudo chmod 777 /dev/ttyACM0
// if you get a port permission denied

// 0 <= x < 64 -> back drive
// 64 -> stop drive
// 64 < x <= 127 -> forward drive 

// #include <ros.h>
// #include <std_msgs/Int16MultiArray.h>
// #include <std_msgs/Int16.h>
// #include <Servo.h>
// #include <Encoder.h>
#include "RoboClaw.h"

// TODO: make sure below addresses are the config of the roboclaws in terms of who is attached to who
#define addresssRc 0x80 // 128

//// serial port for drivetrain data
// serial port for drivetrain data
SoftwareSerial serialDt(25,24);
RoboClaw rcDt(&serialDt, 10000);

SoftwareSerial serialLeftLinAct(8,9);
RoboClaw rcExcLeft(&serialLeftLinAct, 10000);

SoftwareSerial serialRightLinAct(10,11);
RoboClaw rcExcRight(&serialRightLinAct, 10000);

SoftwareSerial serialExDrive(22,23);
RoboClaw rcExcDrive(&serialExDrive, 10000);

SoftwareSerial serialHopper(12,13);
RoboClaw rcHopper(&serialHopper, 10000);

SoftwareSerial serialVibe(7,6);
RoboClaw rcVibe(&serialVibe, 10000);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // assigns which motors are on which pins of the arduino
  rcDt.begin(38400);
  rcExcLeft.begin(38400);
  rcExcRight.begin(38400);
  rcExcDrive.begin(38400);
  rcHopper.begin(38400);
  rcVibe.begin(38400);

  // start all the motors as not moving at the start
  // left drivetrain motor
  rcDt.ForwardBackwardM1(addresssRc, 64);
  // right drivetrain motor
  rcDt.ForwardBackwardM2(addresssRc, 64);
  // left bullet linear actuator
  rcExcLeft.ForwardBackwardM1(addresssRc, 64);
  // right bullet linear actuator
  rcExcRight.ForwardBackwardM1(addresssRc, 64);
  // left industrial linear actuator
  rcExcLeft.ForwardBackwardM2(addresssRc, 64);
  // right industrial linear actuator
  rcExcRight.ForwardBackwardM2(addresssRc, 64);
  // left exc drive motor
  rcExcDrive.ForwardBackwardM1(addresssRc, 64);
  // right exc drive motor
  rcExcDrive.ForwardBackwardM2(addresssRc, 64);
  // hopper open/close actuator
  rcHopper.ForwardBackwardM2(addresssRc, 64);
  // hopper vibe
  rcVibe.ForwardBackwardM2(addresssRc, 64);

}

void loop() {
  // put your main code here, to run repeatedly:
  // node_handle.spinOnce();
  // delay(100); 

  // left drivetrain motor
  rcDt.ForwardBackwardM1(addresssRc, 64); // NOTE WORKS: higher runs backwards
  // right drivetrain motor
  rcDt.ForwardBackwardM2(addresssRc, 64); // NOTE WORKS: believe higher runs backwards
  // left bullet linear actuator
  rcExcLeft.ForwardBackwardM1(addresssRc, 64); // NOTE: higher value retracts the LEFT INDUSTRIAL LIN ACT
  // right bullet linear actuator
  rcExcRight.ForwardBackwardM1(addresssRc, 64); // NOTE: higher values extend right industrial actuator
  // left industrial linear actuator
  rcExcLeft.ForwardBackwardM2(addresssRc, 64); // lower values extend the left bullet actuator
  // right industrial linear actuator
  rcExcRight.ForwardBackwardM2(addresssRc, 64); // higher values extend the right bullet
  // left exc drive motor
  rcExcDrive.ForwardBackwardM1(addresssRc, 64); // higher values rotate in right direction
  // right exc drive motor
  rcExcDrive.ForwardBackwardM2(addresssRc, 64); // lower values rotate in right direction
  // hopper open/close actuator
  rcHopper.ForwardBackwardM2(addresssRc, 64);
  // hopper vibe
  rcVibe.ForwardBackwardM2(addresssRc, 64); 
}
