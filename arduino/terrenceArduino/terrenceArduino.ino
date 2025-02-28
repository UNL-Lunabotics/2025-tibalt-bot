// sudo chmod 777 /dev/ttyACM0
// if you get a port permission denied

// 30 -> back drive
// 90 -> stop drive
// 150 -> forward drive 

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

// All GPIO
Servo FRWheel;
Servo FLWheel;
Servo BRWheel;
Servo BLWheel;
Servo DigMotor;

// 
ros::NodeHandle node_handle;
std_msgs::Int16 joy_num;
std_msgs::Int16MultiArray joy_arr;

// motor offsets
int FRWheelOff = 5;
int FLWheelOff = 0;
int BRWheelOff = 5;
int BLWheelOff = 0;
int DigMotorOff = 0;


void motor_update(const std_msgs::Int16MultiArray& joystick_arr) {
  FRWheel.write(joy_arr.data[0] + FRWheelOff);
  FLWheel.write(joy_arr.data[1] + FLWheelOff);
  BRWheel.write(joy_arr.data[2] + BRWheelOff);
  BLWheel.write(joy_arr.data[3] + BLWheelOff);
  DigMotor.write(joy_arr.data[4] + DigMotorOff);
}

ros::Subscriber<std_msgs::Int16MultiArray> joy_subscriber("motor_speeds", &motor_update);

/*
This code will run once to initialize everything.
*/
void setup() {
  Serial.begin(9600);

  FRWheel.attach(9); 
  FLWheel.attach(10); 
  BRWheel.attach(7);
  BLWheel.attach(8); 
  // TODO: add dig motor pin

  node_handle.initNode();
  node_handle.subscribe(joy_subscriber);

  FRWheel.write(90 + FRWheelOff);
  FLWheel.write(90 + FLWheelOff);
  BRWheel.write(90 + BRWheelOff);
  BLWheel.write(90 + BLWheelOff);
  DigMotor.write(90 + DigMotorOff);
}

void loop() {
  // put your main code here, to run repeatedly:
  node_handle.spinOnce();
  delay(100); 
}
