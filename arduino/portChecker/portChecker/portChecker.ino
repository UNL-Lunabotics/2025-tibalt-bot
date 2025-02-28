// sudo chmod 777 /dev/ttyACM0
// if you get a port permission denied

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

Servo FRWheel; //GPIO
Servo FLWheel; //GPIO
Servo BRWheel; //GPIO
Servo BLWheel; //GPIO
Servo DigMotor; //GPIO

// motor offsets
//TODO: test offsets
int FRWheelOff = 5;
int FLWheelOff = 0;
int BRWheelOff = 5;
int BLWheelOff = 0;
int DigMotorOff = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  FRWheel.attach(9); 
  FLWheel.attach(10); 
  BRWheel.attach(7);
  BLWheel.attach(8);
  // TODO: add dig motor pin
  
  // 30 -> back drive
  // 90 -> stop drive
  // 150 -> forward drive 
  FRWheel.write(120 + FRWheelOff);
  FLWheel.write(120 + FLWheelOff);
  BRWheel.write(120 + BRWheelOff);
  BLWheel.write(120 + BLWheelOff);
  DigMotor.write(120 + DigMotorOff);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100); 
}
