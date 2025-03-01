// sudo chmod 777 /dev/ttyACM0
// if you get a port permission denied

// 0 <= x < 64 -> back drive
// 64 -> stop drive
// 64 < x <= 127 -> forward drive 

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <Servo.h>
#include <Encoder.h>
#include "RoboClaw.h"

// TODO: make sure below addresses are the config of the roboclaws in terms of who is attached to who
#define addresssRc 0x80 // 128
#define IRSensor A0
#define pressureSensor A1

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

ros::NodeHandle node_handle;
std_msgs::Int16MultiArray joy_arr;
std_msgs::Int16MultiArray enc_arr;
std_msgs::Int16MultiArray sensor_arr;


ros::Publisher encoderChatter("enc_pipe", &enc_arr);
ros::Publisher sensorChatter("sensor_pipe", &sensor_arr);
// long excDriveLeftEncoder, excDriveRightEncoder;

/*
 * motor offsets - sometimes some motors move forward/backward 
 * when they're supposed to stay still when that happens, the offset can be 
 * adjusted as needed to get it working
*/
// TODO: update below offsets once the robot is together, just using trial and error


/*
  speeds array will need
  left drivetrain speed, right drivetrain speed
  left bullet actuator, right bullet actuator, left industrial actuator, right industrial actuator, left drive, right drive
  hopper actuator, hopper vibe motor

  optional additions (add later if needed)
  camera arm and camera arm rotate
*/

void joyCallback(const std_msgs::Int16MultiArray& joy_arr) {

  // assign values to the motors based on the joystick array data
  // left drivetrain motor
  rcDt.ForwardBackwardM1(addresssRc,joy_arr.data[0]);
  // right drivetrain motor
  rcDt.ForwardBackwardM2(addresssRc,joy_arr.data[1]);
  
  // left bullet linear actuator
  rcExcLeft.ForwardBackwardM1(addresssRc,joy_arr.data[2]);
  // right bullet linear actuator
  rcExcRight.ForwardBackwardM1(addresssRc,joy_arr.data[4]);
  // left industrial linear actuator
  rcExcLeft.ForwardBackwardM2(addresssRc,joy_arr.data[3]);
  // right industrial linear actuator
  rcExcRight.ForwardBackwardM2(addresssRc,joy_arr.data[5]);
  // left exc drive motor
  rcExcDrive.ForwardBackwardM1(addresssRc,joy_arr.data[6]);
  // right exc drive motor
  rcExcDrive.ForwardBackwardM2(addresssRc,joy_arr.data[7]);

  // vibrational motor
  rcVibe.ForwardBackwardM2(addresssRc, joy_arr.data[8]);
  // hopper actuator motor
  rcHopper.ForwardBackwardM2(addresssRc,joy_arr.data[9]);
}

ros::Subscriber<std_msgs::Int16MultiArray> joy_subscriber("TMP", &joyCallback);

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

  node_handle.initNode();
  node_handle.subscribe(joy_subscriber);
  node_handle.advertise(encoderChatter);
  node_handle.advertise(sensorChatter)

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
  node_handle.spinOnce();
  delay(100); 
    
  
  float volts = analogRead(IRSensor)*0.0048828125;  // value from sensor * (5/1024)
  int distance = 13*pow(volts, -1); // worked out from datasheet graph
  
  int pressureReading = analogRead(pressureSensor);

  int dataLength = 2;
  sensor_arr.data_length = dataLength;
  short value[dataLength] = {distance, pressureReading};
  sensor_arr.data = value;
  sensorChatter.publish(&sensor_arr);

  // TODO: add switch code if the meches add them to their systems
  // switch states
  // bool hopperSS = analogRead(hopperTop) > 0;

  // Encoders
  // excDriveLeftEncoder = roboclaw.ReadEncM1(address4ExcDrive);
  // excDriveRightEncoder = roboclaw.ReadEncM2(address4ExcDrive);

  // TODO: CJ says the large industrial actuators have encoders? If so, add that to this code

  // int dataLength = 2;
  // 

  node_handle.spinOnce();
}
