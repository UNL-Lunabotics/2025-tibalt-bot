#include <micro_ros_arduino.h>
#include <RoboClaw.h>
#include <Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16_multi_array.h>
// #include <std_msgs/Int16.h>

#define ROBOCLAW_ADDRESS 0x80
#define ROBOCLAW_BAUD 38400

RoboClaw roboclaw1(&Serial1, 10000); // M1: hopper actuator M2: excavation drive
RoboClaw roboclaw2(&Serial2, 10000); // M1: excavation actuator

Servo dtRight;
Servo dtLeft;
Servo hopperLatch;

int dtRightPin = 4;
int dtLeftPin = 5;
int hopperServoPin = 9;

const int pwmStop = 1500;

rcl_subscription_t subscriber;
std_msgs__msg__Int16MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


//TODO: check topic indices
void motor_update(const void * msgin) {
  const std_msgs__msg__Int16MultiArray * motor_msg = (const std_msgs__msg__Int16MultiArray *)msgin;
  int dtLeftPwm = motor_msg->data.data[0] * 5 + pwmStop;
  int dtRightPwm = motor_msg->data.data[1] * 5 + pwmStop;
  //TODO: Increment drivetrain motors here (slowly increase or decrease speed)
  dtLeft.writeMicroseconds(dtLeftPwm);
  dtRight.writeMicroseconds(dtRightPwm);
  roboclaw1.ForwardBackwardM2(ROBOCLAW_ADDRESS, motor_msg->data.data[2]); // Excavation drive
  roboclaw2.ForwardBackwardM1(ROBOCLAW_ADDRESS, motor_msg->data.data[3]); // Excavation actuator
  roboclaw1.ForwardBackwardM1(ROBOCLAW_ADDRESS, motor_msg->data.data[4]); // Hopper actuator
  hopperLatch.write(motor_msg->data.data[5]);
}


void setup() {
  //TODO: This line may not be needed
  Serial.begin(9600);

  roboclaw1.begin(ROBOCLAW_BAUD);
  roboclaw2.begin(ROBOCLAW_BAUD);

  dtRight.attach(dtRightPin);
  dtLeft.attach(dtLeftPin);

  hopperLatch.attach(hopperServoPin);

  // Make sure the motors aren't moving
  roboclaw1.ForwardBackwardM1(ROBOCLAW_ADDRESS, 64);
  roboclaw1.ForwardBackwardM2(ROBOCLAW_ADDRESS, 64);
  roboclaw2.ForwardBackwardM1(ROBOCLAW_ADDRESS, 64);
  dtRight.writeMicroseconds(pwmStop);
  dtLeft.writeMicroseconds(pwmStop);

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
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &motor_update, ON_NEW_DATA);
}

void loop() {
  delay(100); //TODO: don't know if this delay is necessary
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
