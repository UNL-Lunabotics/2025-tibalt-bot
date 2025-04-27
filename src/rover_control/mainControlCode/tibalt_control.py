# This is the Python ROS2 library
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# This is the Python ROS library that communicates with the joystick controller
from sensor_msgs.msg import Joy

# The Roboclaw library we use to control the motors
from std_msgs.msg import Int16 # Roboclaw requires Int16 types
from std_msgs.msg import Int16MultiArray

'''CONSTANT DECLARATION'''
# Motor speed constants to help make the code more readable
# They range from 0-127, with 0 as full power backward, 64 as stop, and 127 as full power forward
MOTOR_FORWARDS = 127
MOTOR_BACKWARDS = 0
MOTOR_STOP = 64

# Servo constants to help make the code more readable
# It ranges from 0-180 degrees
SERVO_FULL_CLOSE = 0
SERVO_FULL_OPEN = 180

# The motor speed increment so the excavation system doesn't go from 0 to 100 speed at once
EXCAV_MOTOR_INCREMENT = 10

'''CONTROL SCHEME'''
EXCAV_DIG_B = 0
EXCAV_RETRACT_B = 0
EXCAV_EXTEND_B = 0
HOPPER_LATCH_B = 0
HOPPER_RETRACT_B = 0
HOPPER_EXTEND_B = 0

class HOPPER_STATE(enumerate):
  """Enum to keep track of the current hopper state."""
  RESTING = 0
  EXTENDING = 1
  RETRACTING = 2

class EXCAV_STATE(enumerate):
  """Enum to keep track of the excavation states."""
  RESTING = 0
  EXTENDING = 1
  RETRACTING = 2

class Tibalt(Node):
  """The node class for all the main Tibalt logic. This contains the functionality
  for drive train and autonomy."""

  def __init__(self):
    super().__init__('tibalt') # Initializes this as a node with the name 'tibalt'

    # Hopper class variables
    self.hopper_state = HOPPER_STATE.RESTING
    self.hopper_vibration_motor = MOTOR_STOP
    self.hopper_actuator_motor = MOTOR_STOP
    self.hopper_latch_servo = SERVO_FULL_CLOSE
    self.hopper_is_latched = True

    # Drivetrain class variables
    self.dt_left_motors = MOTOR_STOP
    self.dt_right_motors = MOTOR_STOP

    # Excavation motor variables
    self.excav_state = EXCAV_STATE.RESTING
    self.excav_dig_motor = MOTOR_STOP
    self.excav_actuator_motor = MOTOR_STOP
    self.excav_is_digging = False

    # This creates a Rate object that we will use to sleep the system at the specified frequency (in Hz)
    # We need this to make sure the node's don't publish data too quickly
    self.rate = self.create_rate(
      frequency=10,
      clock=self.get_clock()
    )

    # This node will publish the motor speeds we want the Arduino to set things to
    self.publisher = self.create_publisher(
      msg_type=Int16MultiArray,
      topic='motor_speeds'
    )

    # This node subscribes to 'joystick_data' (published from joystick_pipeline)
    # This Joy object gives the current axis and button input from the joystick
    self.subscription = self.create_subscription(
      msg_type=Joy,
      topic='joystick_data',
      callback=self.control_callback  # This function gets triggered every time
                                      # a message is received from the topic
    )
  
  def control_callback(self, joystick: Joy) -> None:
    """This contains the main driver control code."""

    # Button check for excavation
    if joystick.buttons[EXCAV_EXTEND_B] == 1:    self.excav_state = EXCAV_STATE.EXTENDING
    elif joystick.buttons[EXCAV_RETRACT_B] == 1: self.excav_state = EXCAV_STATE.RETRACTING
    else:                                        self.excav_state = EXCAV_STATE.RESTING

    # The dig motor follows a toggle control scheme
    if joystick.buttons[EXCAV_DIG_B] == 1: self.excav_is_digging = not self.excav_is_digging

    # Button check for hopper
    if joystick.buttons[HOPPER_EXTEND_B] == 1:    self.hopper_state = HOPPER_STATE.EXTENDING
    elif joystick.buttons[HOPPER_RETRACT_B] == 1: self.hopper_state = HOPPER_STATE.RETRACTING
    else:                                         self.hopper_state = HOPPER_STATE.RESTING

    # The latch motor is a toggle
    if joystick.buttons[HOPPER_LATCH_B] == 1: self.hopper_is_latched = not self.hopper_is_latched

    drivetrain_logic(self=self, joystick=joystick)
    excavation_logic(self=self)
    hopper_logic(self=self)

    motor_speeds = Int16MultiArray()
    # [dtLeft, dtRight, exTurn, exActuator, hopperActuator, hopperHatch, hopperVibe] (variable names can change but this is the order for the motors)
    motor_speeds.data = [
      self.dt_left_motors,
      self.dt_right_motors,
      self.excav_dig_motor,
      self.excav_actuator_motor,
      self.hopper_actuator_motor,
      self.hopper_latch_servo,
      self.hopper_vibration_motor
      ]
    
    self.publisher.publish(motor_speeds)

def drivetrain_logic(self, joystick: Joy) -> None:
  """The logic for computing the motor speeds for the drivetrain.
  
  Parameters:
    self
    joystick, the Joy object representing joystick input from the driver
  Returns: None, it sets relevant class variables instead
  """

  # Don't understand the math? Me neither, accept that it works
  x_axis: float = joystick.axes[0]  # pos<-right, neg<-left
  y_axis: float = joystick.axes[1]  # pos<-down, neg<-up

  forward: int = int(x_axis * 63)
  turn: int = int(y_axis * 63)

  DtLeft: int = 64 + forward + turn
  DtRight: int = 64 + forward - turn

  self.dt_left_motors = DtLeft
  self.dt_right_motors = DtRight

def excavation_logic(self) -> None:
  """The logic for computing the motor speeds for the excavation system.
  It contains the state machine that will do the necessary calculations.
  
  Parameters:
    self
    joystick, the Joy object representing joystick input from the driver
  Returns: None, it sets relevant class variables instead
  """
  # These are helper functions to help incrementally change motor speeds
  def incrementMotor(motorSpeed, max) -> int:
    if (motorSpeed + EXCAV_MOTOR_INCREMENT > max):
      return max
    else:
      return motorSpeed + EXCAV_MOTOR_INCREMENT
    
  def decrementMotor(motorSpeed, min) -> int:
    if (motorSpeed - EXCAV_MOTOR_INCREMENT < min):
      return min
    else:
      return motorSpeed - EXCAV_MOTOR_INCREMENT
  
  # The digging motor is handled independently of the excavation state
  if (self.excav_is_digging):
    self.excav_dig_motor = MOTOR_FORWARDS
  else:
    self.excav_dig_motor = MOTOR_STOP

  # In the resting state, the linear actuator motors is not moving.
  # Check if the extend button is pressed
  if self.excav_state == EXCAV_STATE.RESTING:
    self.excav_actuator_motor = MOTOR_STOP

  # In the extending state, the actuator is extending while the excavator is spinning.
  # The motors are in this state as long as the extending button remains pressed.
  # check if the button is released
  elif self.excav_state == EXCAV_STATE.EXTENDING:

    # incase the actuator is still moving backwards
    if (self.excav_actuator_motor < MOTOR_STOP):
      #sets it to the increment value so that it will be at MOTOR_STOP once incrementMotor() is called
      self.excav_actuator_motor = -EXCAV_MOTOR_INCREMENT

    self.excav_actuator_motor = incrementMotor(self.excavation_actuator_motor, MOTOR_FORWARDS)

  # In the retracting state, the excavator is still spinning while the
  # actuator is retracting. Stays in the retracting state for a set
  # amount of time.
  elif self.excav_state == EXCAV_STATE.RETRACTING:

    # incase the actuator is still moving forward
    if (self.excav_actuator_motor > MOTOR_STOP):
      #sets it to the increment value so that it will be at MOTOR_STOP once decrementMotor() is called
      self.excav_actuator_motor = EXCAV_MOTOR_INCREMENT

    self.excav_actuator_motor = decrementMotor(self.excavation_actuator_motor, MOTOR_BACKWARDS)

def hopper_logic(self) -> None:
  """The logic for computing the motor speeds for the hopper system.
  It contains the state machine that will do the necessary calculations.
  
  Parameters:
    self
    joystick, the Joy object representing joystick input from the driver
  Returns: None, it sets relevant class variables instead
  """

  # TODO figure out vibration motor if we decide to actually use it

  # The latch servo is independent of the hopper state
  if (self.hopper_is_latched):
    self.hopper_latch_servo = SERVO_FULL_CLOSE
  else:
    self.hopper_latch_servo = SERVO_FULL_OPEN

  # If the extend button is currently being pressed
  if self.hopper_state == HOPPER_STATE.EXTENDING:
    self.hopper_actuator_motor = MOTOR_FORWARDS

  # If the retracting button is currently being pressed
  elif self.hopper_state == HOPPER_STATE.RETRACTING:
    self.hopper_actuator_motor = MOTOR_BACKWARDS
  
  # If no hopper buttons are being pressed, do nothing
  elif self.hopper_state == HOPPER_STATE.RESTING:
    self.hopper_actuator_motor = MOTOR_STOP

def main(args=None) -> None:
  """Initializes Tibalt and starts the control loop."""

  try:
    with rclpy.init(args=args):
      tibalt = Tibalt()
      
      while rclpy.ok(): # Ensures that the code runs continuously until shutdown

        # This will allow the node to process pending callback requests once before
        # continuing to run this loop. This allows us to control the callback rate
        rclpy.spin_once(tibalt)  

        # Sleep the node for 10Hz without blocking the entire system
        tibalt.loop_rate.sleep()
  except (KeyboardInterrupt, ExternalShutdownException):
    # Shuts down if a KeyboardInterrupt or ExternalShutdownException is detected
    # i.e. if Ctrl+C is pressed or if ROS2 is shutdown externally

    # This command cleans up ROS2 resources to shutdown gracefully
    # TODO: make sure we test this with disconnects, closing terminal, etc
    rclpy.shutdown()

if __name__ == '__main__':
  main()