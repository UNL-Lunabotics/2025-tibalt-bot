#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16_multi_array.h>
// #include <std_msgs/Int16.h>
#include <micro_ros_arduino.h>
#include <RoboClaw.h>
#include <Servo.h>

#define ROBOCLAW_ADDRESS 0x80
#define ROBOCLAW_BAUD 38400

// RoboClaw definitions
RoboClaw roboclaw1(&Serial1, 10000); // M1 = Hopper Linear, M2 = Excavation Motor | Teensy Tx = 1, Rx = 0
RoboClaw roboclaw2(&Serial2, 10000); // M1 = Excavation Linear                    | Teensy Tx = 8, Rx = 7

// Servo definitions
Servo dtRight;
Servo dtLeft;
Servo hopperServo;

// Pin Assignments
int hopperServoPin = 9;
int dtRightPin = 4;
int dtLeftPin = 5;

// drivetrain params
const int pwmStop = 1500;
const int pwmMin = 1000;
const int pwmMax = 2000;

// rcl definitions
rcl_subscription_t subscriber;
std_msgs__msg__Int16MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
}


// MOTOR UPDATE CALLBACK FUNCTION
void motor_update(const std_msgs::Int16MultiArray& joystick_arr) {
  const std_msgs__msg__Int16MultiArray * motor_msg = (const std_msgs__msg__Int16MultiArray *)msgin;

  // drivetrain motor speed augmentation to produce a value from 1000 to 2000
  int dtLeftPwm = motor_msg->data.data[0] * 5 + pwmStop; // expects data[0] to be in range of -100 to 100
  int dtRightPwm = motor_msg->data.data[1] * 5 + pwmStop; // expects data[1] to be in range of -100 to 100

  // constrain speeds to between 1000 and 2000 (min and max pwm values for drivetrain motor speeds)
  dtLeftPwm = constrain(dtLeftPwm, pwmMin, pwmMax);
  dtRightPwm = constrain(dtRightPwm, pwmMin, pwmMax);

  // TODO: Increment drivetrain motors here (slowly increase or decrease speed)

  // drivetrain motors
  dtLeft.writeMicroseconds(dtLeftPwm);
  dtRight.writeMicroseconds(dtRightPwm);

  // excavation and hopper motors
  roboclaw1.ForwardBackwardM2(ROBOCLAW_ADDRESS, motor_msg->data.data[2]); // Excavation drive
  roboclaw2.ForwardBackwardM1(ROBOCLAW_ADDRESS, motor_msg->data.data[3]); // Excavation linear actuator
  roboclaw1.ForwardBackwardM1(ROBOCLAW_ADDRESS, motor_msg->data.data[4]); // Hopper linear actuator
  
  // possibilty of button debouncing screwing this up. In that case, increment position value to target position (with constraints)
  // ensure hopper servo is in standard mode, not rotational mode
  hopperServo.write(motor_msg->data.data[5]); // Hopper servo
}

// SETUP FUNCTION
void setup() {
  //TODO: This line may not be needed
  Serial.begin(9600);

  // setup roboclaws (excavation lin actuator and motor, hopper lin acuator)
  roboclaw2.begin(ROBOCLAW_BAUD);
  roboclaw1.begin(ROBOCLAW_BAUD);

  // setup drivetrain motors
  dtLeft.attach(dtLeftPin);
  dtRight.attach(dtRightPin);

  // setup hopper servo
  hopperServo.attach(hopperServoPin);

  // Make sure the motors aren't moving
  hopperServo.write(0);
  roboclaw1.ForwardBackwardM2(ROBOCLAW_ADDRESS, 64); // excavation drive
  roboclaw1.ForwardBackwardM1(ROBOCLAW_ADDRESS, 64); // hopper linear actuator
  dtLeft.writeMicroseconds(pwmStop);
  dtRight.writeMicroseconds(pwmStop);

  // Initialize data array (receiving data from motor_speeds topic)
  msg.data.data = (int16_t *) malloc(sizeof(int16_t) * 10); // Reserve space for 10 int16_t values
  msg.data.size = 0;
  msg.data.capacity = 10;

  // Start up micro ros
  set_microros_transports();

  // Set up ros2 stuff
  allocator = rcl_get_default_allocator();
   
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "roboclaw_node", "", &support);
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "motor_speeds");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &motor_callback, ON_NEW_DATA);
}

void loop() {
  delay(100); //TODO: don't know if this delay is necessary
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
